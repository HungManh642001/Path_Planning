"""Standalone CNN-guidance trainer (any machine with a GPU).

Mirrors ml_planner/train/train_guidance.ipynb as a CLI so you can train off
Colab. torch/onnx are imported inside main() so the planner never depends on
them. The U-Net + training loop are intentionally duplicated from the notebook
(the two are parallel delivery formats of the same pipeline).

Usage:
  python ml_planner/train/train_guidance.py --data-dir ml_planner/data \
         --out ml_planner/models/guidance.onnx [--epochs 120 --base 48 --lr 2e-3 --batch 8]

Expects one or more guidance_dataset*.npz shards (from ml_planner.build_dataset)
in --data-dir, each with channels (N,4,G,G), label (N,G,G) meters, mask (N,G,G),
affine (N,4). G must equal ml_planner.config.GRID_RES.
"""
import argparse
import glob
import os
import sys

import numpy as np

# Repo root on sys.path so `import config` / `ml_planner.config` work when run
# as a script from anywhere.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import ml_planner.config as mlcfg


def load_shards(data_dir):
    paths = sorted(glob.glob(os.path.join(data_dir, "guidance_dataset*.npz")))
    if not paths:
        raise SystemExit(f"no guidance_dataset*.npz found in {data_dir}")
    ch, la, ms, af = [], [], [], []
    for p in paths:
        d = np.load(p)
        ch.append(d['channels']); la.append(d['label']); ms.append(d['mask']); af.append(d['affine'])
    channels = np.concatenate(ch).astype('float32')
    label = np.concatenate(la).astype('float32')
    mask = np.concatenate(ms).astype('float32')
    affine = np.concatenate(af)
    G = mlcfg.GRID_RES
    assert channels.shape[1:] == (4, G, G), f"shape {channels.shape} != (*,4,{G},{G}); check GRID_RES"
    print(f"loaded {len(paths)} shard(s): {len(channels)} samples, {int(mask.sum())} labeled cells")
    return channels, label, mask, affine


def build_model(base):
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
        def __init__(self, cin=4, base=48):
            super().__init__()
            self.d1 = DoubleConv(cin, base);       self.p1 = nn.MaxPool2d(2)
            self.d2 = DoubleConv(base, base * 2);  self.p2 = nn.MaxPool2d(2)
            self.d3 = DoubleConv(base * 2, base * 4); self.p3 = nn.MaxPool2d(2)
            self.mid = DoubleConv(base * 4, base * 8)
            self.u3 = nn.ConvTranspose2d(base * 8, base * 4, 2, stride=2); self.c3 = DoubleConv(base * 8, base * 4)
            self.u2 = nn.ConvTranspose2d(base * 4, base * 2, 2, stride=2); self.c2 = DoubleConv(base * 4, base * 2)
            self.u1 = nn.ConvTranspose2d(base * 2, base, 2, stride=2);     self.c1 = DoubleConv(base * 2, base)
            self.head = nn.Conv2d(base, 1, 1)

        def forward(self, x):
            import torch
            x1 = self.d1(x); x2 = self.d2(self.p1(x1)); x3 = self.d3(self.p2(x2)); m = self.mid(self.p3(x3))
            y = self.c3(torch.cat([self.u3(m), x3], 1))
            y = self.c2(torch.cat([self.u2(y), x2], 1))
            y = self.c1(torch.cat([self.u1(y), x1], 1))
            return self.head(y) + x[:, 2:3]          # residual over normalized dist-to-goal

    return UNet(base=base)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', default=os.path.join(os.path.dirname(__file__), '..', 'data'))
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'models', 'guidance.onnx'))
    ap.add_argument('--epochs', type=int, default=120)
    ap.add_argument('--base', type=int, default=48)
    ap.add_argument('--lr', type=float, default=2e-3)
    ap.add_argument('--batch', type=int, default=8)
    args = ap.parse_args()

    import torch
    import onnx

    channels, label, mask, affine = load_shards(args.data_dir)
    G = mlcfg.GRID_RES

    # Normalize cost-to-go by each sample's crop diagonal (ranking-invariant).
    side = affine[:, 3] / affine[:, 2]
    diag = (np.sqrt(2.0) * side).astype('float32')
    label_n = label / diag[:, None, None]

    rng = np.random.default_rng(0)
    idx = rng.permutation(len(channels))
    nval = max(1, len(channels) // 5)
    val_idx, tr_idx = idx[:nval], idx[nval:]

    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = build_model(args.base).to(dev)
    print(f"device {dev} | params {sum(p.numel() for p in model.parameters())}")

    Xtr = torch.tensor(channels[tr_idx]); Ytr = torch.tensor(label_n[tr_idx]); Mtr = torch.tensor(mask[tr_idx])
    Xva = torch.tensor(channels[val_idx]); Yva = torch.tensor(label_n[val_idx]); Mva = torch.tensor(mask[val_idx])
    opt = torch.optim.Adam(model.parameters(), args.lr)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)

    def masked_mse(pred, y, m):
        m = m > 0
        return (((pred - y) ** 2) * m).sum() / m.sum().clamp(min=1)

    best, best_state = 1e9, None
    bs = args.batch
    for epoch in range(args.epochs):
        model.train(); perm = torch.randperm(len(Xtr))
        for i in range(0, len(Xtr), bs):
            j = perm[i:i + bs]
            loss = masked_mse(model(Xtr[j].to(dev))[:, 0], Ytr[j].to(dev), Mtr[j].to(dev))
            opt.zero_grad(); loss.backward(); opt.step()
        sched.step()
        model.eval()
        with torch.no_grad():
            vp = torch.cat([model(Xva[k:k + bs].to(dev))[:, 0].cpu() for k in range(0, len(Xva), bs)])
            vloss = float(masked_mse(vp, Yva, Mva))
        if vloss < best:
            best = vloss; best_state = {k: v.cpu().clone() for k, v in model.state_dict().items()}
        if epoch % 10 == 0:
            print(f"epoch {epoch:3d}  val masked-MSE {vloss:.6f}")
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
