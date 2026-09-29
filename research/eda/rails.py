"""Rail visibility: floor-region cross-sections at several distances + low-point BEV (accumulated over a few static frames)."""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from bagio import Bag

OUT = Path(__file__).parent / "plots"
name, i0, nfr = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
bag = Bag(name)
pts = []
for i in range(i0, i0 + nfr):
    a, _, _ = bag.points(i)
    v = a[(a["x"] != 0) | (a["y"] != 0)]
    pts.append(np.stack([v["x"], -v["y"], v["z"], v["intensity"]], 1))
P = np.concatenate(pts)
x, f, z, it = P.T

# floor height estimate near vehicle: low percentile of z in 4..8 m, |x|<1.5
near = (f > 4) & (f < 8) & (np.abs(x) < 1.5)
floor = np.percentile(z[near], 5)
print(f"{name}: floor z (5th pct, 4..8 m ahead) = {floor:.2f}")

dists = [5, 10, 20, 30, 50, 70, 90, 110]
fig, axs = plt.subplots(2, 4, figsize=(24, 8))
for ax, d in zip(axs.ravel(), dists):
    w = 0.5 if d < 30 else 2.0
    s = (np.abs(f - d) < w) & (z < floor + 1.2) & (np.abs(x) < 4)
    ax.scatter(x[s], z[s] - floor, c=it[s], s=4, cmap="viridis", vmin=0, vmax=40)
    ax.set_title(f"{d} m ±{w} (n={s.sum()})"); ax.set_ylim(-0.4, 1.2); ax.set_xlim(-4, 4); ax.grid(alpha=0.3)
fig.suptitle(f"{name} frames {i0}..{i0 + nfr - 1}: floor-region cross sections (z - floor), color=intensity")
fig.tight_layout(); fig.savefig(OUT / f"rails_xsec_{name}_{i0}.png", dpi=60); plt.close(fig)

fig, ax = plt.subplots(1, 1, figsize=(24, 5))
s = (z < floor + 0.35) & (f < 160) & (np.abs(x) < 6)
ax.scatter(f[s], x[s], c=z[s] - floor, s=0.3, cmap="turbo", vmin=-0.1, vmax=0.35)
ax.set_xlim(0, 160); ax.set_ylim(-6, 6); ax.set_title(f"{name}: low points (z < floor+0.35), color = height above floor")
fig.tight_layout(); fig.savefig(OUT / f"rails_bev_{name}_{i0}.png", dpi=60); plt.close(fig)
