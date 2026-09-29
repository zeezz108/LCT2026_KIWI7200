"""doubleT_obstacle: foreground vs median background, 3D euclidean clustering, per-frame cluster table + key-frame plots."""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from bagio import Bag

OUT = Path(__file__).parent / "plots"
bg = np.load(Path(r"D:\Python Projects\ЛЦТ 2026\data") / "obstacle_bg_median.npy")
bag = Bag("doubleT_obstacle")
n = len(bag)


def cluster(p, eps=0.35, min_pts=8):
    if len(p) == 0:
        return []
    tree = cKDTree(p)
    pairs = tree.query_pairs(eps, output_type="ndarray")
    m = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(len(p), len(p)))
    nc, lab = connected_components(m, directed=False)
    out = []
    for k in range(nc):
        idx = np.nonzero(lab == k)[0]
        if len(idx) >= min_pts:
            out.append(idx)
    return sorted(out, key=lambda i: -len(i))


rows = []
key = {20, 30, 40, 60, 138, 150, 160, 180, 200}
fg_store = {}
for i in range(n):
    a, s, _ = bag.points(i)
    g = a.reshape(7200, 128)
    xyz = np.stack([g["x"], g["y"], g["z"]], -1).astype(np.float64)
    r = np.linalg.norm(xyz, axis=-1)
    thr = np.maximum(0.3, 0.03 * np.nan_to_num(bg, nan=0))
    fg = (r > 0) & ((np.isfinite(bg) & (r < bg - thr)) | ~np.isfinite(bg))
    p = xyz[fg]
    cl = cluster(p)
    for idx in cl[:6]:
        q = p[idx]
        lo, hi = q.min(0), q.max(0)
        rows.append((i, s, len(idx), *np.median(q, 0), *(hi - lo), np.linalg.norm(np.median(q, 0)[:2])))
    if i in key:
        fg_store[i] = (xyz[r > 0], p, [p[idx] for idx in cl[:6]])

rows = np.array(rows)
np.save(Path(r"D:\Python Projects\ЛЦТ 2026\data") / "obstacle_clusters.npy", rows)
t0 = rows[0, 1] if len(rows) else 0
print("frame  t     npts   x     fwd    z     dx    dy    dz   dist")
for rrow in rows:
    i, s, npts, x, y, z, dx, dy, dz, d = rrow
    if npts >= 20:
        print(f"{int(i):4d} {s - t0:5.1f} {int(npts):5d} {x:6.2f} {-y:6.2f} {z:5.2f} {dx:5.2f} {dy:5.2f} {dz:5.2f} {d:6.1f}")

for i, (allp, fgp, cls) in fg_store.items():
    fig, axs = plt.subplots(1, 3, figsize=(24, 6))
    ax = axs[0]
    s = (-allp[:, 1] > -5) & (-allp[:, 1] < 70)
    ax.scatter(-allp[s, 1], allp[s, 0], s=0.1, c="lightgray")
    for k, q in enumerate(cls):
        ax.scatter(-q[:, 1], q[:, 0], s=3, label=f"c{k} n={len(q)}")
    ax.set_xlim(-5, 70); ax.set_ylim(-4, 8); ax.set_title(f"frame {i} BEV (fwd vs x)"); ax.legend(fontsize=7)
    ax = axs[1]
    ax.scatter(-allp[s, 1], allp[s, 2], s=0.1, c="lightgray")
    for q in cls:
        ax.scatter(-q[:, 1], q[:, 2], s=3)
    ax.set_xlim(-5, 70); ax.set_title("side (fwd vs z)")
    ax = axs[2]
    if cls:
        c = np.median(cls[0], 0)
        s2 = np.abs(-allp[:, 1] - (-c[1])) < 1.0
        ax.scatter(allp[s2, 0], allp[s2, 2], s=0.5, c="lightgray")
        for q in cls:
            q2 = q[np.abs(-q[:, 1] - (-c[1])) < 1.5]
            ax.scatter(q2[:, 0], q2[:, 2], s=4)
        ax.set_aspect("equal"); ax.set_title(f"cross-section around fwd={-c[1]:.1f} m")
    fig.tight_layout(); fig.savefig(OUT / f"obst_f{i}.png", dpi=65); plt.close(fig)
print("ok")
