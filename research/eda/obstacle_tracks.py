import numpy as np
from pathlib import Path

rows = np.load(Path(r"D:\Python Projects\ЛЦТ 2026\data") / "obstacle_clusters.npy")
# columns: i, stamp, npts, x, y, z, dx, dy, dz, dist
t0 = rows[0, 1]
fwd = -rows[:, 4]
print("=== far object: 45..70 m forward ===")
print("frame   t    npts    x      fwd    zmed   dx    dy    dz")
last = -1
for r_, f in zip(rows, fwd):
    if 45 < f < 70 and r_[2] >= 10:
        i = int(r_[0])
        if i != last and (i % 5 == 0 or r_[2] < 30):
            print(f"{i:4d} {r_[1] - t0:5.1f} {int(r_[2]):5d} {r_[3]:6.2f} {f:6.2f} {r_[5]:6.2f} {r_[6]:5.2f} {r_[7]:5.2f} {r_[8]:5.2f}")
            last = i
print("=== near moving object: largest cluster with npts>150 in 0..45 m, |x-2|<1.5 ===")
last = -1
for r_, f in zip(rows, fwd):
    if 0 < f < 45 and r_[2] >= 150 and abs(r_[3] - 2) < 1.5:
        i = int(r_[0])
        if i != last:
            print(f"{i:4d} {r_[1] - t0:5.1f} {int(r_[2]):5d} {r_[3]:6.2f} {f:6.2f} {r_[5]:6.2f} {r_[6]:5.2f} {r_[7]:5.2f} {r_[8]:5.2f}")
            last = i
