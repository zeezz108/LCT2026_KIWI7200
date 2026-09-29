"""How far the estimated track-bed profile is from the floor the lidar actually sees.

The vertical profile decides where the bottom of the clearance envelope is, so an error of 0.3 m either hides
objects lying on the track or lifts the floor itself into the envelope. Ground truth here is the floor in the same
frame: in every 10 m bin along the corridor the height histogram of returns near the axis has a clear peak at the
track bed (the drainage trough and the ballast sit below it and are narrow). Comparing that peak with the estimated
profile needs no synthetic data and no ego-motion.

usage: python tools/profile_error.py [bag ...] [--results DIR] [--bins 10] [--plot]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "research" / "eda"))
sys.path.insert(0, str(ROOT / "tools"))
from bagio import Bag, BAGS  # noqa: E402

# cloud -> mount frame (X ahead, Y left, Z up) as a right-multiplied matrix: Q = P @ MOUNT
MOUNT = {"-y": np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
         "x": np.eye(3)}

DISTANCES = [(20, 40), (40, 60), (60, 80), (80, 110), (110, 150)]
AXIS_X = np.arange(0, 201, 10.0)


def track_rotation(pitch_deg, roll_deg, forward_axis="-y"):
    """Same transform the detector uses: mount frame from the cloud, then align to the estimated bed plane."""
    n = np.array([np.tan(np.radians(pitch_deg)), np.tan(np.radians(roll_deg)), 1.0])
    n /= np.linalg.norm(n)
    x = np.array([1.0, 0.0, 0.0]) - n[0] * n
    x /= np.linalg.norm(x)
    y = np.cross(n, x)
    mount_to_track = np.stack([x, y, n])           # rows
    return MOUNT[forward_axis] @ mount_to_track.T  # cloud -> track, for row vectors: Q = P @ R


def floor_peak(z, bin_size=0.1, min_points=15):
    """Height of the dominant horizontal surface: the peak of the height histogram, refined by its neighbours."""
    if z.size < min_points:
        return None
    hist, edges = np.histogram(z, bins=np.arange(z.min() - bin_size, z.max() + 2 * bin_size, bin_size))
    k = int(np.argmax(hist))
    if hist[k] < min_points // 2:
        return None
    lo, hi = edges[k] - bin_size, edges[k] + 2 * bin_size
    sel = z[(z >= lo) & (z < hi)]
    return float(np.median(sel))


def errors_for(bag_name, results, bins, max_frames=0):
    rows = [json.loads(line) for line in (results / f"{bag_name}.jsonl").read_text(encoding="utf-8").splitlines()]
    bag = Bag(bag_name)
    out = {d: [] for d in DISTANCES}
    step = max(1, len(rows) // max_frames) if max_frames else 1
    for r in rows[::step]:
        if not r.get("valid") or r.get("axis_bed") is None:
            continue
        a, _, _ = bag.points(r["frame"])
        v = a[(a["x"] != 0) | (a["y"] != 0)]
        P = np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64)
        Q = P @ track_rotation(r["pitch"], r["roll"])
        Q[:, 2] += r["height"]
        lat = np.array(r["axis_lat"], float)
        bed = np.array(r["axis_bed"], float)
        dy = Q[:, 1] - np.interp(Q[:, 0], AXIS_X, lat)
        near = np.abs(dy) < 1.0
        for d in DISTANCES:
            m = near & (Q[:, 0] >= d[0]) & (Q[:, 0] < d[1])
            # the floor only: everything above knee height in the corridor is not the bed
            z = Q[m, 2]
            z = z[z < np.interp(0.5 * (d[0] + d[1]), AXIS_X, bed) + 1.0]
            peak = floor_peak(z, bin_size=bins / 100.0)
            if peak is None:
                continue
            est = float(np.interp(0.5 * (d[0] + d[1]), AXIS_X, bed))
            out[d].append(est - peak)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="*", default=None)
    ap.add_argument("--results", default="data/results/bench/final_day2")
    ap.add_argument("--bins", type=float, default=10.0, help="histogram bin for the floor peak [cm]")
    ap.add_argument("--frames", type=int, default=60, help="frames sampled per recording (0 = all)")
    args = ap.parse_args()
    results = Path(args.results)
    if not results.is_absolute():
        results = ROOT / results
    bags = args.bags or [b for b in BAGS if (results / f"{b}.jsonl").exists()]

    print(f"profile error (estimate - observed floor), results from {results.name}")
    print(f"{'recording':40s}" + "".join(f"{lo}-{hi} m".rjust(14) for lo, hi in DISTANCES))
    all_err = {d: [] for d in DISTANCES}
    for bag in bags:
        err = errors_for(bag, results, args.bins, args.frames)
        line = f"{bag:40s}"
        for d in DISTANCES:
            e = np.array(err[d])
            all_err[d].extend(err[d])
            line += f"{np.median(e):+6.2f}/{np.percentile(np.abs(e), 90):5.2f}" if e.size else f"{'-':>14}"
        print(line)
    line = f"{'ALL (median / p90 of |error|)':40s}"
    for d in DISTANCES:
        e = np.array(all_err[d])
        line += f"{np.median(e):+6.2f}/{np.percentile(np.abs(e), 90):5.2f}" if e.size else f"{'-':>14}"
    print(line)
    deep = {d: float(np.mean(np.array(all_err[d]) < -0.3)) if all_err[d] else 0.0 for d in DISTANCES}
    print("share of bins where the profile sits more than 0.3 m below the floor: " +
          ", ".join(f"{lo}-{hi} m {100 * v:.0f}%" for (lo, hi), v in deep.items()))


if __name__ == "__main__":
    main()
