"""Whole-range BEV (0..200 m, height-coloured, aspect stretched) with competing axis hypotheses."""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import pipeline_v2 as pv
from proto_common import Bag, load_frame, ransac_ground, to_track, track_transform

name, fi = sys.argv[1], int(sys.argv[2])
p = pv.Params()
P, it, _, ring, _ = load_frame(Bag(name), fi)
n, d = ransac_ground(P)
Q = to_track(P, *track_transform(n, d))
near, det = pv.near_axis(Q, ring, p)
y0s, th0s, ks, cost = pv.lateral_search(Q, p, det, near[0], near[1], near[4])
xs, yc = pv.centreline(p, y0s, th0s, ks)

fig, axs = plt.subplots(2, 1, figsize=(24, 10))
for ax, (zlo, zhi, title) in zip(axs, [(0.9, 3.0, "band 0.9-3.0 m (walls, objects)"), (-0.5, 0.9, "band -0.5-0.9 m (bed, rails, low)")]):
    m = (Q[:, 0] > 0) & (Q[:, 0] < 200) & (np.abs(Q[:, 1]) < 9) & (Q[:, 2] > zlo) & (Q[:, 2] < zhi)
    ax.scatter(Q[m, 0], Q[m, 1], c=Q[m, 2], s=1.2, cmap="turbo", vmin=zlo, vmax=zhi)
    ax.plot(xs, yc, "k-", lw=1, label="chosen")
    ax.plot(xs, yc + 1.35, "k--", lw=0.6)
    ax.plot(xs, yc - 1.35, "k--", lw=0.6)
    ax.plot(xs, near[0] + near[1] * xs, "m-", lw=1, label="straight from rails")
    ax.plot(xs, near[0] + near[1] * xs + 1.35, "m:", lw=0.6)
    ax.plot(xs, near[0] + near[1] * xs - 1.35, "m:", lw=0.6)
    if len(det):
        ax.scatter(det[:, 0], det[:, 1], c="lime", s=12, marker="x")
    ax.set_xlim(0, 200); ax.set_ylim(-9, 9); ax.grid(alpha=0.3); ax.legend(loc="upper left")
    ax.set_title(f"{name} f{fi}: {title}")
fig.tight_layout()
fig.savefig(Path(__file__).parent / "plots" / f"wide_{name}_{fi}.png", dpi=55)
