"""Per-frame statistics for a few frames of every bag."""
import numpy as np

from bagio import BAGS, Bag

np.set_printoptions(linewidth=200, suppress=True)

for name in BAGS:
    bag = Bag(name)
    print("=" * 110)
    print(name, "frames:", len(bag))
    idxs = [0, len(bag) // 2, len(bag) - 1]
    hdr = []
    for i in idxs:
        a, stamp, m = bag.points(i)
        xyz = np.stack([a["x"], a["y"], a["z"]], 1)
        finite = np.isfinite(xyz).all(1)
        zero = finite & (np.abs(xyz).sum(1) == 0)
        v = a[finite & ~zero]
        r = np.sqrt(v["x"] ** 2 + v["y"] ** 2 + v["z"] ** 2)
        print(f"-- frame {i}: total={len(a)} finite={finite.sum()} zeros={zero.sum()} valid={len(v)}")
        for k in ("x", "y", "z", "intensity"):
            q = np.percentile(v[k], [0, 1, 50, 99, 100])
            print(f"   {k:>9s} p0/p1/p50/p99/p100 = {q}")
        print(f"   range     p0/p1/p50/p99/p100 = {np.percentile(r, [0, 1, 50, 99, 100])}")
        print(f"   range>50m: {(r > 50).sum()}  >100m: {(r > 100).sum()}  >200m: {(r > 200).sum()}  >300m: {(r > 300).sum()}")
        print(f"   ring min/max = {a['ring'].min()}/{a['ring'].max()}  unique={len(np.unique(a['ring']))}")
        t = a["timestamp"]
        print(f"   pt timestamp min={t.min():.6f} max={t.max():.6f} span={t.max() - t.min():.4f}s  header={stamp:.6f} "
              f"min-header={t.min() - stamp:+.4f}s")
        # organized pattern? look at ring sequence of first 300 points
        if i == 0:
            print("   first 40 rings :", a["ring"][:40])
            print("   finite mask of first 40:", finite[:40].astype(int))
            nan_rows = (~finite).nonzero()[0]
            print("   first nan idx:", nan_rows[:10], "last nan idx:", nan_rows[-5:] if len(nan_rows) else None)
    # header stamps across whole bag (deserializing only every message would be slow for data; do it anyway but cheap enough)
print("done")
