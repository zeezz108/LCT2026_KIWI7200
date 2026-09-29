"""Check organized layout (cols x 128), beam elevation table, azimuth coverage, header-stamp timeline."""
import numpy as np

from bagio import BAGS, Bag

np.set_printoptions(linewidth=220, suppress=True, precision=3)

elev_tables = {}
for name in BAGS:
    bag = Bag(name)
    a, stamp, m = bag.points(0)
    ncol = len(a) // 128
    g = a.reshape(ncol, 128)
    ring_ok = (g["ring"] == np.arange(128)[None, :]).all()
    x, y, z = g["x"].astype(np.float64), g["y"].astype(np.float64), g["z"].astype(np.float64)
    r = np.sqrt(x * x + y * y + z * z)
    valid = r > 0
    elev = np.degrees(np.arcsin(np.where(valid, z / np.where(valid, r, 1), 0)))
    az = np.degrees(np.arctan2(x, -y))  # 0 deg = forward (-Y), positive toward +X
    # per-ring median elevation
    ring_el = np.array([np.median(elev[:, k][valid[:, k]]) if valid[:, k].any() else np.nan for k in range(128)])
    ring_el_std = np.array([np.std(elev[:, k][valid[:, k]]) if valid[:, k].sum() > 10 else np.nan for k in range(128)])
    ring_valid = valid.mean(0)
    elev_tables[name] = ring_el
    # per-column azimuth (median over valid rings)
    col_az = np.array([np.median(az[c][valid[c]]) if valid[c].any() else np.nan for c in range(ncol)])
    col_t = g["timestamp"][:, 0] - stamp
    print("=" * 100)
    print(f"{name}: ncol={ncol} ring_layout_ok={ring_ok}")
    print(f"  col azimuth first cols: {col_az[:5]}  last cols: {col_az[-5:]}")
    ok = np.isfinite(col_az)
    daz = np.diff(col_az[ok])
    print(f"  azimuth span: {np.nanmin(col_az):.2f}..{np.nanmax(col_az):.2f} deg, median step={np.median(daz):.4f} deg")
    print(f"  col time offsets first/last: {col_t[0]:.5f} / {col_t[-1]:.5f} s; per-point time within column const: "
          f"{np.allclose(g['timestamp'] - g['timestamp'][:, :1], 0, atol=1e-4)}")
    print(f"  ring elevation (deg) min={np.nanmin(ring_el):.2f} max={np.nanmax(ring_el):.2f}; median intra-ring std={np.nanmedian(ring_el_std):.3f}")
    print(f"  ring 0..9 elev: {ring_el[:10]}")
    print(f"  ring 118..127 elev: {ring_el[-10:]}")
    print(f"  valid fraction per ring (every 8th): {ring_valid[::8]}")

    # header stamps for all frames (sample every frame; data loading is the cost)
    hs = []
    for i in range(len(bag)):
        mm = bag.msg(i)
        hs.append(mm.header.stamp.sec + mm.header.stamp.nanosec * 1e-9)
    hs = np.array(hs)
    dh = np.diff(hs) * 1000
    rv = np.diff(np.array([bag.recv_ns(i) for i in range(len(bag))])) / 1e6
    print(f"  header dt ms: mean={dh.mean():.2f} min={dh.min():.2f} max={dh.max():.2f}; gaps>150ms at {np.nonzero(dh > 150)[0]} -> {dh[dh > 150]}")
    print(f"  recv dt ms  : gaps>150ms at {np.nonzero(rv > 150)[0]} -> {rv[rv > 150]}")
    print(f"  header span {hs[-1] - hs[0]:.2f}s vs frames*0.1={len(hs) * 0.1:.1f}s")

names = list(elev_tables)
ref = elev_tables[names[1]]
for n in names:
    d = elev_tables[n] - ref
    print(f"elev table diff vs {names[1]}: {n}: max abs {np.nanmax(np.abs(d)):.3f} deg")
print("full elevation table (deg), ring order:")
print(ref)
