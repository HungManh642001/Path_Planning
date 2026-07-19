"""Standalone GPU trainer for the tangent-graph GNN guidance (spec §6).

Consumes graph_dataset_*.npz shards (ml_planner.graph_dataset), trains a small
message-passing net that predicts the residual-over-Euclid cost-to-go per
node, and saves weights + meta to a plain .npz consumed by the numpy
inference in ml_planner/graph_guidance.py.

  python ml_planner/train/train_graph.py \
      --data-dir ml_planner/data --out ml_planner/models/graph_guidance.npz
"""
import argparse
import os
import sys

import numpy as np
import torch
import torch.nn as nn

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from ml_planner.graph_dataset import load_shards          # noqa: E402


class MPNN(nn.Module):
    def __init__(self, node_dim=7, edge_dim=2, hidden=64, rounds=4):
        super().__init__()
        self.rounds = rounds
        self.enc = nn.Linear(node_dim, hidden)
        self.msg = nn.Sequential(nn.Linear(2 * hidden + edge_dim, hidden),
                                 nn.ReLU(), nn.Linear(hidden, hidden))
        self.upd = nn.GRUCell(hidden, hidden)
        self.dec = nn.Sequential(nn.Linear(hidden, hidden),
                                 nn.ReLU(), nn.Linear(hidden, 1))

    def forward(self, x, edge_index, edge_attr):
        """x (M,node_dim); edge_index (2,2E) BOTH directions; edge_attr (2E,edge_dim)."""
        h = torch.relu(self.enc(x))
        src, dst = edge_index[0], edge_index[1]
        for _ in range(self.rounds):
            m = self.msg(torch.cat([h[src], h[dst], edge_attr], dim=1))
            agg = torch.zeros_like(h).index_add_(0, dst, m)
            h = self.upd(agg, h)
        return nn.functional.softplus(self.dec(h).squeeze(-1))


def _to_batch(graphs, device):
    """Concatenate graphs; offset LOCAL edge indices; duplicate edges both ways."""
    xs, eis, eas, tgt, msk = [], [], [], [], []
    off = 0
    for g in graphs:
        m = g['node_feat'].shape[0]
        xs.append(torch.tensor(g['node_feat']))
        e = g['edges'].astype(np.int64) + off
        ei = np.concatenate([e.T, e.T[::-1]], axis=1)
        eis.append(torch.tensor(ei, dtype=torch.long))
        ea = np.concatenate([g['edge_feat'], g['edge_feat']], axis=0)
        eas.append(torch.tensor(ea))
        d = float(g['scale'])
        r_t = np.clip(g['label'] / d - g['node_feat'][:, 0], 0.0, None)
        tgt.append(torch.tensor(r_t.astype(np.float32)))
        msk.append(torch.tensor(g['mask']))
        off += m
    return (torch.cat(xs).to(device),
            torch.cat(eis, dim=1).to(device),
            torch.cat(eas).to(device),
            torch.cat(tgt).to(device),
            torch.cat(msk).to(device))


def save_weights(model, out, hidden, rounds, node_dim=7, edge_dim=2):
    arrays = {k: v.detach().cpu().numpy() for k, v in model.state_dict().items()}
    np.savez(out, __meta__=np.asarray([hidden, rounds, node_dim, edge_dim],
                                      dtype=np.int64), **arrays)


def train(graphs, out, epochs=150, hidden=64, rounds=4, lr=1e-3,
          batch_graphs=32, val_frac=0.1, device='auto', seed=0):
    """Returns (first_epoch_train_loss, last_epoch_train_loss)."""
    if device == 'auto':
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(graphs))
    n_val = int(len(graphs) * val_frac)
    val_idx, tr_idx = idx[:n_val], idx[n_val:]
    model = MPNN(hidden=hidden, rounds=rounds).to(device)
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    huber = nn.HuberLoss(reduction='none')
    first = last = None
    for ep in range(epochs):
        model.train()
        rng.shuffle(tr_idx)
        losses = []
        for b0 in range(0, len(tr_idx), batch_graphs):
            batch = [graphs[i] for i in tr_idx[b0:b0 + batch_graphs]]
            x, ei, ea, tgt, msk = _to_batch(batch, device)
            r = model(x, ei, ea)
            loss = (huber(r, tgt) * msk).sum() / msk.sum().clamp(min=1.0)
            opt.zero_grad()
            loss.backward()
            opt.step()
            losses.append(float(loss))
        tr_loss = float(np.mean(losses))
        first = tr_loss if first is None else first
        last = tr_loss
        if val_idx.size and (ep % 10 == 0 or ep == epochs - 1):
            model.eval()
            with torch.no_grad():
                x, ei, ea, tgt, msk = _to_batch([graphs[i] for i in val_idx], device)
                r = model(x, ei, ea)
                v = float((huber(r, tgt) * msk).sum() / msk.sum().clamp(min=1.0))
            print(f"epoch {ep:4d}  train {tr_loss:.5f}  val {v:.5f}", flush=True)
        elif ep % 10 == 0:
            print(f"epoch {ep:4d}  train {tr_loss:.5f}", flush=True)
    save_weights(model, out, hidden, rounds)
    print(f"saved -> {out}", flush=True)
    return first, last


def main():
    ap = argparse.ArgumentParser(description="Train the tangent-graph GNN guidance.")
    ap.add_argument('--data-dir', default=os.path.join(
        os.path.dirname(__file__), "..", "data"))
    ap.add_argument('--out', default=os.path.join(
        os.path.dirname(__file__), "..", "models", "graph_guidance.npz"))
    ap.add_argument('--epochs', type=int, default=150)
    ap.add_argument('--hidden', type=int, default=64)
    ap.add_argument('--rounds', type=int, default=4)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--val-frac', type=float, default=0.1)
    ap.add_argument('--device', default='auto')
    args = ap.parse_args()
    graphs = load_shards(args.data_dir)
    if not graphs:
        raise SystemExit(f"no graph_dataset_*.npz shards in {args.data_dir}")
    print(f"{len(graphs)} graphs loaded", flush=True)
    train(graphs, args.out, epochs=args.epochs, hidden=args.hidden,
          rounds=args.rounds, lr=args.lr, val_frac=args.val_frac,
          device=args.device)


if __name__ == '__main__':
    main()
