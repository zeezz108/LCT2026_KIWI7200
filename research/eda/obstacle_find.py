"""doubleT_obstacle: vehicle is static -> median background range image, foreground = points clearly in front of background."""
import matplotlib

matplotlib.use("Agg")
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage

from bagio import Bag

OUT = Path(__file__).parent / "plots"
bag = Bag("doubleT_obstacle")
n = len(bag)

R = np.zeros((n, 7200, 128), np.float32)
XYZ_last = None
stamps = np.zeros(n)
frames_xyz = []
for i in range(n):
    a, s, _ = bag.points(i)
    g = a.reshape(7200, 128)
    R[i] = np.sqrt(g["x"] ** 2 + g["y"] ** 2 + g["z"] ** 2)
    stamps[i] = s
    frames_xyz.append(np.stack([g["x"], g["y"], g["z"]], -1).astype(np.float32))
print("loaded", R.shape)

Rm = np.where(R > 0, R, np.nan)
bg = np.nanmedian(Rm, axis=0)  # (7200,128)
np.save(Path(r"D:\Python Projects\ЛЦТ 2026\data") / "obstacle_bg_median.npy", bg)

tracks = []
fg_counts = []
for i in range(n):
    r = R[i]
    thr = np.maximum(0.3, 0.03 * bg)  # at least 30 cm or 3% of background range closer
    fg = (r > 0) & np.isfinite(bg) & (r < bg - thr)
    # also points where background is empty (no return) but now there is a return
    fg_new = (r > 0) & ~np.isfinite(bg)
    fg_all = fg | fg_new
    lab, nl = ndimage.label(ndimage.binary_dilation(fg_all, iterations=1))
    xyz = frames_xyz[i]
    comps = []
    for k in range(1, nl + 1):
        m = (lab == k) & fg_all
        cnt = m.sum()
        if cnt < 15:
            continue
        p = xyz[m]
        c = np.median(p, 0)
        ext = p.max(0) - p.min(0)
        comps.append((cnt, c, ext))
    comps.sort(key=lambda t: -t[0])
    fg_counts.append(fg_all.sum())
    tracks.append(comps)
    if i % 10 == 0 or i in (137, 138, 139, 163, 164, 165):
        desc = "; ".join(
            f"n={c[0]} ctr=({c[1][0]:.2f},{-c[1][1]:.2f}fwd,{c[1][2]:.2f}) size=({c[2][0]:.2f},{c[2][1]:.2f},{c[2][2]:.2f}) d={np.hypot(c[1][0], c[1][1]):.1f}m"
            for c in comps[:4]
        )
        print(f"f{i:3d} t={stamps[i] - stamps[0]:6.2f} fg={fg_all.sum():6d} comps={len(comps)} | {desc}")

# plot foreground count over time
fig, ax = plt.subplots(2, 1, figsize=(16, 8))
ax[0].plot(stamps - stamps[0], fg_counts)
ax[0].set_title("foreground pixel count vs time")
# biggest component forward distance over time
t_, d_, x_ = [], [], []
for i, comps in enumerate(tracks):
    for c in comps[:3]:
        t_.append(stamps[i] - stamps[0]); d_.append(-c[1][1]); x_.append(c[1][0])
ax[1].scatter(t_, d_, c=x_, cmap="coolwarm", s=8)
ax[1].set_ylabel("forward dist of top-3 components (m)"); ax[1].set_xlabel("t (s)")
fig.tight_layout(); fig.savefig(OUT / "obstacle_timeline.png", dpi=70)
