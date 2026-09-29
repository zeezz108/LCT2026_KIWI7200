"""Is the vehicle moving? Frame-to-frame range-image differences for every bag (organized grid makes this trivial)."""
import numpy as np

from bagio import BAGS, Bag


def rimg(bag, i):
    a, stamp, _ = bag.points(i)
    g = a.reshape(-1, 128)
    r = np.sqrt(g["x"].astype(np.float64) ** 2 + g["y"] ** 2 + g["z"] ** 2)
    # keep only first return of each dual pair to halve size
    return r[0::2], stamp


for name in BAGS:
    bag = Bag(name)
    n = len(bag)
    step = max(1, n // 12)
    r0, s0 = rimg(bag, 0)
    prev, sp = r0, s0
    print("=" * 90)
    print(name)
    for i in range(step, n, step):
        r, s = rimg(bag, i)
        both = (r > 0) & (prev > 0)
        d = np.abs(r - prev)[both]
        # central fine rings looking ahead: median range of forward-looking pixels (tunnel end distance proxy)
        print(f"  f{i - step:4d}->f{i:4d} dt={s - sp:5.2f}s  median|dr|={np.median(d):.3f}m  share|dr|>0.2m={(d > 0.2).mean():.3f}  "
              f"share|dr|>1m={(d > 1).mean():.3f}")
        prev, sp = r, s
