"""Standalone CNN-guidance trainer (any machine with a GPU).

Memory-frugal: streams ONE shard at a time (never loads the whole dataset into
RAM) and uses a small 2-level U-Net + mixed precision. Mirrors
ml_planner/train/train_guidance.ipynb. torch/onnx are imported inside main() so
the planner never depends on them.

Usage:
  python ml_planner/train/train_guidance.py --data-dir ml_planner/data \
         --out ml_planner/models/guidance.onnx \
         [--epochs 60 --base 16 --lr 2e-3 --batch 8 --patience 10]

Expects guidance_dataset*.npz shards (from ml_planner.build_dataset) in
--data-dir, each with channels (n,4,G,G), label (n,G,G) meters, mask (n,G,G),
affine (n,4). G must equal ml_planner.config.GRID_RES.
"""
import argparse
import glob
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import ml_planner.config as mlcfg


def shard_paths(data_dir):
    paths = sorted(glob.glob(os.path.join(data_dir, "guidance_dataset*.npz")))
    if not paths:
        raise SystemExit(f"no guidance_dataset*.npz found in {data_dir}")
    return paths


def load_shard(path):
    """Load one shard and normalize labels by each sample's crop diagonal
    (ranking-invariant). Returns (channels, label_norm, mask) as float32."""
    d = np.load(path)
    ch = d['channels'].astype('float32')
    af = d['affine']
    diag = (np.sqrt(2.0) * (af[:, 3] / af[:, 2])).astype('float32')   # crop diagonal (m)
    lan = (d['label'].astype('float32') / diag[:, None, None])
    ms = d['mask'].astype('float32')
    return ch, lan, ms


def build_model(base):
    """Small 2-level U-Net; output adds the dist-to-goal channel (residual over
    Euclid) so it only learns the detour correction."""
    import torch
    import torch.nn as nn

    class DoubleConv(nn.Module):
        def __init__(self, ci, co):
            super().__init__()
            self.net = nn.Sequential(
                nn.Conv2d(ci, co, 3, padding=1), nn.BatchNorm2d(co), nn.ReLU(inplace=True),
                nn.Conv2d(co, co, 3, padding=1), nn.BatchNorm2d(co), nn.ReLU(inplace=True))

        def forward(self, x):
            return self.net(x)

    class UNet(nn.Module):
        def __init__(self, cin=4, base=16):
            super().__init__()
            self.d1 = DoubleConv(cin, base);      self.p1 = nn.MaxPool2d(2)
            self.d2 = DoubleConv(base, base * 2); self.p2 = nn.MaxPool2d(2)
            self.mid = DoubleConv(base * 2, base * 4)
            self.u2 = nn.ConvTranspose2d(base * 4, base * 2, 2, stride=2); self.c2 = DoubleConv(base * 4, base * 2)
            self.u1 = nn.ConvTranspose2d(base * 2, base, 2, stride=2);     self.c1 = DoubleConv(base * 2, base)
            self.head = nn.Conv2d(base, 1, 1)

        def forward(self, x):
            x1 = self.d1(x); x2 = self.d2(self.p1(x1)); m = self.mid(self.p2(x2))
            y = self.c2(torch.cat([self.u2(m), x2], 1))
            y = self.c1(torch.cat([self.u1(y), x1], 1))
            return self.head(y) + x[:, 2:3]           # residual over normalized dist-to-goal

    return UNet(base=base)


def masked_mse(pred, y, m):
    m = m > 0
    return (((pred - y) ** 2) * m).sum() / m.sum().clamp(min=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', default=os.path.join(os.path.dirname(__file__), '..', 'data'))
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'models', 'guidance.onnx'))
    ap.add_argument('--epochs', type=int, default=60)
    ap.add_argument('--base', type=int, default=16)
    ap.add_argument('--lr', type=float, default=2e-3)
    ap.add_argument('--batch', type=int, default=8)
    ap.add_argument('--patience', type=int, default=10, help="early-stop after N epochs w/o val improvement")
    args = ap.parse_args()

    import torch
    from torch.cuda.amp import autocast, GradScaler
    import onnx

    G = mlcfg.GRID_RES
    paths = shard_paths(args.data_dir)
    # Hold out the last shard for validation; stream the rest for training. With
    # a single shard, split its samples 80/20.
    if len(paths) >= 2:
        train_paths, val_paths = paths[:-1], paths[-1:]
        val = [load_shard(p) for p in val_paths]
    else:
        ch, lan, ms = load_shard(paths[0])
        nval = max(1, len(ch) // 5)
        val = [(ch[:nval], lan[:nval], ms[:nval])]
        train_paths = paths  # streamed; a tiny val overlap on one shard is acceptable
    print(f"{len(paths)} shard(s): train~{len(train_paths)} shard(s), val {sum(len(v[0]) for v in val)} samples | grid {G}")

    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    use_amp = dev == 'cuda'
    model = build_model(args.base).to(dev)
    print(f"device {dev} | params {sum(p.numel() for p in model.parameters())} | amp {use_amp}")
    opt = torch.optim.Adam(model.parameters(), args.lr)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)
    scaler = GradScaler(enabled=use_amp)
    bs = args.batch

    def val_loss():
        model.eval()
        tot, cnt = 0.0, 0
        with torch.no_grad():
            for ch, lan, ms in val:
                for i in range(0, len(ch), bs):
                    xb = torch.tensor(ch[i:i+bs]).to(dev)
                    yb = torch.tensor(lan[i:i+bs]).to(dev)
                    mb = torch.tensor(ms[i:i+bs]).to(dev)
                    with autocast(enabled=use_amp):
                        l = masked_mse(model(xb)[:, 0], yb, mb)
                    tot += float(l) * len(xb); cnt += len(xb)
        return tot / max(1, cnt)

    best, best_state, since = 1e9, None, 0
    for epoch in range(args.epochs):
        model.train()
        order = list(train_paths); random.shuffle(order)
        for p in order:
            ch, lan, ms = load_shard(p)
            idx = np.random.permutation(len(ch))
            for i in range(0, len(idx), bs):
                j = idx[i:i+bs]
                xb = torch.tensor(ch[j]).to(dev); yb = torch.tensor(lan[j]).to(dev); mb = torch.tensor(ms[j]).to(dev)
                opt.zero_grad()
                with autocast(enabled=use_amp):
                    loss = masked_mse(model(xb)[:, 0], yb, mb)
                scaler.scale(loss).backward(); scaler.step(opt); scaler.update()
            del ch, lan, ms                                   # free the shard before the next
        sched.step()
        vl = val_loss()
        if vl < best - 1e-9:
            best, best_state, since = vl, {k: v.cpu().clone() for k, v in model.state_dict().items()}, 0
        else:
            since += 1
        if epoch % 5 == 0 or since == 0:
            print(f"epoch {epoch:3d}  val masked-MSE {vl:.6f}  best {best:.6f}")
        if since >= args.patience:
            print(f"early stop at epoch {epoch} (no val improvement for {args.patience})")
            break
    print("best val masked-MSE", best)
    model.load_state_dict(best_state)

    # Export a SINGLE self-contained ONNX (channels -> cost_to_go, GxG).
    model.eval().cpu()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    dummy = torch.zeros(1, 4, G, G)
    torch.onnx.export(
        model, dummy, args.out,
        input_names=['channels'], output_names=['cost_to_go'], opset_version=13,
        dynamic_axes={'channels': {0: 'batch'}, 'cost_to_go': {0: 'batch'}})
    onnx.save_model(onnx.load(args.out), args.out, save_as_external_data=False)
    print(f"exported single-file ONNX -> {args.out}")


if __name__ == '__main__':
    main()
