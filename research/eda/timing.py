"""Rough per-frame cost of a numpy pipeline skeleton on this PC (decode -> range image -> corridor mask -> CC labeling)."""
import time

import numpy as np
from scipy import ndimage

from bagio import PT, Bag

for name in ("doubleT_obstacle", "roundT_doubleT"):
    bag = Bag(name)
    raw = [bytes(bag.msg(i).data) for i in range(30)]
    ts = {"decode": [], "range_img": [], "transform+mask": [], "cc_label": []}
    for buf in raw:
        t0 = time.perf_counter()
        a = np.frombuffer(buf, dtype=PT)
        g = a.reshape(-1, 128)
        x, y, z = g["x"], g["y"], g["z"]
        t1 = time.perf_counter()
        r = np.sqrt(x * x + y * y + z * z)
        valid = r > 0
        t2 = time.perf_counter()
        # rotate into track frame (pitch/roll) + straight corridor test as a proxy for per-point work
        c, s = np.cos(0.01), np.sin(0.01)
        f = -y
        zz = z * c + f * s + 1.34
        inside = valid & (np.abs(x) < 1.7) & (zz > 0.25) & (zz < 3.8) & (f > 2)
        t3 = time.perf_counter()
        lab, n = ndimage.label(inside)
        t4 = time.perf_counter()
        for k, v in zip(ts, (t1 - t0, t2 - t1, t3 - t2, t4 - t3)):
            ts[k].append(v * 1000)
    tot = sum(np.median(v) for v in ts.values())
    print(name, "points/frame:", len(a), " | ", "  ".join(f"{k}={np.median(v):.1f}ms" for k, v in ts.items()), f" | total≈{tot:.1f} ms")
