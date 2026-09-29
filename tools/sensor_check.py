"""Verify the sensor model the detector relies on against the Pandar128E3X datasheet.

The organizers published the lidar documentation (hesaitech.com/downloads -> Pandar128). This script measures the
angular layout, the return mode and the reach directly from the recordings and compares them with the datasheet, so
that every sensor constant in the code (azimuth/elevation resolution, range limits, the reflectivity model of the
synthetic injector) is backed by a number and not by an assumption.

usage:
  python tools/sensor_check.py [--bag NAME] [--frame 60] [--all]
"""
import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "research" / "eda"))
from bagio import Bag, BAGS  # noqa: E402

# Hesai Pandar128E3X, from the manufacturer's specification
SPEC = dict(channels=128, vfov_deg=40.0, fine_elevation_deg=0.125, azimuth_deg=0.1, coarse_azimuth_deg=0.2,
            points_single=3_456_000, points_dual=6_912_000, range_10pct_m=200.0, min_range_m=0.3, rate_hz=10.0)


def layout(bag, frame):
    """Angular layout of one frame: per-ring elevation and azimuth step, split into the two sampling bands."""
    a, _, msg = bag.points(frame)
    v = a[(a["x"] != 0) | (a["y"] != 0)]
    xy = np.hypot(v["x"].astype(float), v["y"].astype(float))
    el = np.degrees(np.arctan2(v["z"].astype(float), xy))
    az = np.degrees(np.arctan2(v["y"].astype(float), v["x"].astype(float)))
    rng = np.hypot(xy, v["z"].astype(float))
    rings = v["ring"].astype(int)
    rows = []
    for k in range(SPEC["channels"]):
        m = rings == k
        if m.sum() < 50:
            continue
        s = np.sort(az[m])
        d = np.diff(s)
        d = d[(d > 1e-4) & (d < 1.0)]
        rows.append(dict(ring=k, el=float(np.median(el[m])), az_step=float(np.median(d)) if d.size else np.nan,
                         points=int(m.sum())))
    fine = [r for r in rows if r["az_step"] < 0.15]
    coarse = [r for r in rows if r["az_step"] >= 0.15]
    el_sorted = np.sort(np.array([r["el"] for r in rows]))
    el_fine = np.sort(np.array([r["el"] for r in fine]))
    el_coarse = np.sort(np.array([r["el"] for r in coarse]))
    # returns per beam: 2.0 in dual-return mode. Measured on one ring - different rings share azimuths.
    ref_ring = max(fine or rows, key=lambda r: r["points"])["ring"]
    az_ring = az[rings == ref_ring]
    dup = az_ring.size / max(np.unique(np.round(az_ring, 3)).size, 1)
    return dict(
        width=int(msg.width), valid=int(v.size), rings=len(rows),
        el_min=float(el_sorted.min()), el_max=float(el_sorted.max()), fov=float(np.ptp(el_sorted)),
        n_fine=len(fine), n_coarse=len(coarse),
        fine_el_min=float(el_fine.min()) if el_fine.size else np.nan,
        fine_el_max=float(el_fine.max()) if el_fine.size else np.nan,
        fine_el_step=float(np.median(np.diff(el_fine))) if el_fine.size > 1 else np.nan,
        coarse_el_step=float(np.median(np.diff(el_coarse)[np.diff(el_coarse) < 2.0])) if el_coarse.size > 1 else np.nan,
        fine_az_step=float(np.median([r["az_step"] for r in fine])) if fine else np.nan,
        coarse_az_step=float(np.median([r["az_step"] for r in coarse])) if coarse else np.nan,
        returns_per_beam=float(dup), range_max=float(np.percentile(rng, 99.99)), range_hi=float(rng.max()),
        range_min=float(rng.min()),
    )


def check(name, measured, spec, tol, unit="", fmt="{:.3f}"):
    ok = abs(measured - spec) <= tol
    print(f"  {'OK ' if ok else 'DIFF'} {name:38s} spec {fmt.format(spec):>10s}{unit}   measured "
          f"{fmt.format(measured):>10s}{unit}")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", default="doubleT_obstacle")
    ap.add_argument("--frame", type=int, default=60)
    ap.add_argument("--all", action="store_true", help="repeat the reach measurement over every recording")
    args = ap.parse_args()

    b = Bag(args.bag)
    m = layout(b, args.frame)
    print(f"{args.bag}, frame {args.frame}: {m['width']} points in the message, {m['valid']} with a return, "
          f"{m['rings']} rings")
    ok = [
        check("channels", m["rings"], SPEC["channels"], 0, fmt="{:.0f}"),
        check("vertical field of view [deg]", m["fov"], SPEC["vfov_deg"], 1.0),
        check("vertical resolution, fine band [deg]", m["fine_el_step"], SPEC["fine_elevation_deg"], 0.01),
        check("azimuth resolution, fine band [deg]", m["fine_az_step"], SPEC["azimuth_deg"], 0.005),
        check("azimuth resolution, coarse band [deg]", m["coarse_az_step"], SPEC["coarse_azimuth_deg"], 0.01),
        check("returns per beam (dual return)", m["returns_per_beam"], 2.0, 0.02, fmt="{:.2f}"),
    ]
    # the datasheet point rate implies exactly this split: half the channels sampled twice as often
    rate = (m["n_fine"] * round(360.0 / m["fine_az_step"]) + m["n_coarse"] * round(360.0 / m["coarse_az_step"])) \
        * SPEC["rate_hz"]
    ok.append(check("point rate, single return [1/s]", rate, SPEC["points_single"], 1, fmt="{:.0f}"))
    print(f"\n  fine band:   {m['n_fine']} channels, elevation {m['fine_el_min']:+.2f} .. {m['fine_el_max']:+.2f} deg, "
          f"grid {m['fine_az_step']:.3f} x {m['fine_el_step']:.3f} deg")
    print(f"  coarse band: {m['n_coarse']} channels (above and below), "
          f"grid {m['coarse_az_step']:.3f} x {m['coarse_el_step']:.3f} deg")
    for h in (1.33, 1.73):
        print(f"  lidar at {h:.2f} m: the track bed leaves the fine band closer than "
              f"{h / np.tan(np.radians(-m['fine_el_min'])):.1f} m")
    print(f"\n  reach: p99.99 {m['range_max']:.1f} m, max {m['range_hi']:.1f} m, min {m['range_min']:.2f} m "
          f"(datasheet: {SPEC['range_10pct_m']:.0f} m at 10% reflectivity, blind zone {SPEC['min_range_m']} m)")
    if args.all:
        print("\n  reach per recording (99.99th percentile of the point range, median over frames):")
        for name in BAGS:
            bb = Bag(name)
            tail = []
            for f in range(0, len(bb), max(1, len(bb) // 6)):
                a, _, _ = bb.points(f)
                v = a[(a["x"] != 0) | (a["y"] != 0)]
                r = np.sqrt(v["x"].astype(float) ** 2 + v["y"].astype(float) ** 2 + v["z"].astype(float) ** 2)
                tail.append(np.percentile(r, 99.99))
            print(f"    {name:40s} {np.median(tail):6.1f} m (max {max(tail):6.1f} m)")
    print(f"\n{sum(ok)}/{len(ok)} checks match the datasheet")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
