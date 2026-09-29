"""Zoom a v2 result: BEV window coloured by height with centreline, plus cross-sections at chosen distances."""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from pipeline_v2 import Params, process
from proto_common import Bag, load_frame

name, fi, x0, x1 = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
cuts = [float(c) for c in sys.argv[5:]]
p = Params()
P, it, _, ring, col = load_frame(Bag(name), fi)
res, _, det = process(P, ring, p, None)
Q = res.Q
xs = (np.arange(len(res.yc)) + 0.5) * p.x_res
fig, axs = plt.subplots(1, 1 + len(cuts), figsize=(10 + 4 * len(cuts), 6), gridspec_kw={"width_ratios": [3] + [1] * len(cuts)})
ax = axs[0]
m = (Q[:, 0] > x0) & (Q[:, 0] < x1) & (Q[:, 2] > -0.5) & (Q[:, 2] < 4.5)
yc_m = np.interp(Q[:, 0], xs, res.yc)
m &= np.abs(Q[:, 1] - yc_m) < 6
sc = ax.scatter(Q[m, 0], Q[m, 1], c=Q[m, 2], s=1.5, cmap="turbo", vmin=-0.3, vmax=4)
ax.scatter(Q[res.inside & m, 0], Q[res.inside & m, 1], s=12, facecolors="none", edgecolors="k", lw=0.6)
xx = np.linspace(x0, x1, 300)
for off in (0, p.zone_hw_up, -p.zone_hw_up):
    ax.plot(xx, np.interp(xx, xs, res.yc) + off, "k-" if off == 0 else "k--", lw=0.7)
if len(det):
    dd = det[(det[:, 0] > x0) & (det[:, 0] < x1)]
    ax.scatter(dd[:, 0], dd[:, 1], c="lime", marker="x", s=25)
plt.colorbar(sc, ax=ax)
ax.set_title(f"{name} f{fi} kappas={[round(k * 1e3, 2) for k in res.kappas]}e-3 near={tuple(round(v, 4) for v in res.near_axis[:3])}")
for a, c in zip(axs[1:], cuts):
    b = np.abs(Q[:, 0] - c) < 0.75
    ycc = np.interp(c, xs, res.yc)
    a.scatter(Q[b, 1] - ycc, Q[b, 2], s=2, c="0.4")
    a.scatter(Q[b & res.inside, 1] - ycc, Q[b & res.inside, 2], s=8, c="r")
    a.axvline(-p.zone_hw_up, ls="--"); a.axvline(p.zone_hw_up, ls="--")
    a.set_aspect("equal"); a.set_xlim(-5, 5); a.set_ylim(-0.6, 5); a.set_title(f"x={c} m (lat rel. corridor)")
fig.tight_layout()
fig.savefig(Path(__file__).parent / "plots" / f"zoom_{name}_{fi}.png", dpi=60)
