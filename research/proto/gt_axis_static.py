"""Reference track axis for the static doubleT_obstacle bag: accumulate many frames so rails stay dense far ahead."""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import pipeline_v2 as pv
from proto_common import Bag, load_frame, ransac_ground, to_track, track_transform

bag = Bag("doubleT_obstacle")
p = pv.Params()
P0, _, _, ring0, _ = load_frame(bag, 100)
n, d = ransac_ground(P0)
Rm, t = track_transform(n, d)
Qs, rings = [], []
for fi in range(60, 130, 2):  # people are far from the track bed ahead in this window except A near 55 m
    P, _, _, ring, _ = load_frame(bag, fi)
    Qs.append(to_track(P, Rm, t))
    rings.append(ring)
Q = np.concatenate(Qs)
ring = np.concatenate(rings)
p.near_x = (3.0, 90.0)
p.min_pts = 3
near, det = pv.near_axis(Q, ring, p, prior=(0.0, -0.0124, 0.0))
print("accumulated axis fit:", near)
d = det[np.argsort(det[:, 0])]
for x0 in range(0, 90, 10):
    s = (d[:, 0] >= x0) & (d[:, 0] < x0 + 10)
    if s.any():
        print(f"  x {x0:2d}-{x0 + 10:2d}: n={s.sum():2d} axis y median={np.median(d[s, 1]):+.3f}")
np.save(Path(r"D:\Python Projects\ЛЦТ 2026\data") / "obstacle_axis_detections.npy", d)
# quadratic fit over all detections
A = np.stack([np.ones(len(d)), d[:, 0], 0.5 * d[:, 0] ** 2], 1)
coef, *_ = np.linalg.lstsq(A, d[:, 1], rcond=None)
res = d[:, 1] - A @ coef
print("lstsq quadratic: y0=%.3f theta=%.3f deg kappa=%.2e  rms=%.3f" % (coef[0], np.degrees(coef[1]), coef[2], np.sqrt(np.mean(res ** 2))))
for x in (56.0, 76.7):
    print(f"  axis at {x} m: {coef[0] + coef[1] * x + 0.5 * coef[2] * x * x:+.2f}")
fig, ax = plt.subplots(figsize=(18, 5))
m = (Q[:, 0] > 0) & (Q[:, 0] < 100) & (np.abs(Q[:, 1]) < 5) & (Q[:, 2] > -0.5) & (Q[:, 2] < 0.5)
ax.scatter(Q[m, 0], Q[m, 1], c=Q[m, 2], s=0.2, cmap="turbo", vmin=-0.4, vmax=0.4)
ax.scatter(d[:, 0], d[:, 1], c="k", s=10)
xx = np.linspace(0, 100, 200)
ax.plot(xx, coef[0] + coef[1] * xx + 0.5 * coef[2] * xx ** 2, "m-")
ax.set_ylim(-5, 5); ax.grid(alpha=0.3)
fig.savefig(Path(__file__).parent / "plots" / "gt_axis_obstacle.png", dpi=60)
