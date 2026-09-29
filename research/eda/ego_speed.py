"""Ego speed per bag: 1-D forward-shift search maximizing NN inlier fraction between frames k and k+2."""
import numpy as np
from scipy.spatial import cKDTree

from bagio import BAGS, Bag


def cloud(bag, i):
    a, s, _ = bag.points(i)
    g = a.reshape(-1, 128)[0::2]  # first return only
    v = g[(g["x"] != 0) | (g["y"] != 0)]
    P = np.stack([v["x"], -v["y"], v["z"]], 1).astype(np.float64)
    m = (P[:, 1] > 3) & (P[:, 1] < 45)
    P = P[m]
    # voxel downsample 0.1 m
    k = np.floor(P / 0.1).astype(np.int64)
    _, idx = np.unique(k[:, 0] * 1_000_003 ** 2 + k[:, 1] * 1_000_003 + k[:, 2], return_index=True)
    return P[idx], s


def best_shift(A, B, lo=-0.3, hi=6.0):
    tree = cKDTree(B)
    sub = A[np.random.default_rng(0).choice(len(A), min(len(A), 15000), replace=False)]

    def score(dy):
        d, _ = tree.query(sub - [0, dy, 0], distance_upper_bound=0.08)
        return np.isfinite(d).mean()

    grid = np.arange(lo, hi, 0.05)
    sc = np.array([score(g) for g in grid])
    g0 = grid[sc.argmax()]
    fine = np.arange(g0 - 0.06, g0 + 0.06, 0.005)
    sf = np.array([score(g) for g in fine])
    return fine[sf.argmax()], sf.max(), sc.max() - np.median(sc)


for name in BAGS:
    bag = Bag(name)
    out = []
    for i in range(0, len(bag) - 2, 6):
        A, sa = cloud(bag, i)
        B, sb = cloud(bag, i + 2)
        dy, sc, contrast = best_shift(A, B)
        out.append((i, sa, dy / (sb - sa), sc, contrast))
    out = np.array(out)
    v = out[:, 2]
    print(f"{name:38s} speed m/s: median={np.median(v):5.2f} max={v.max():5.2f} (={v.max() * 3.6:4.1f} km/h)  "
          f"share stopped(<0.2)={np.mean(v < 0.2):.2f}  median peak contrast={np.median(out[:, 4]):.3f}")
    print("   profile (every ~0.6 s):", " ".join(f"{x:.1f}" for x in v))
