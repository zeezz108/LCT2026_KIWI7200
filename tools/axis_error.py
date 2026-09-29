"""How far the estimated track axis is from the path the train actually drove.

The ground truth comes from tools/ego_motion.py: the future positions of the track-axis point under the lidar,
expressed in the current frame. Comparing it with the corridor of the detector gives the axis error against distance
on real recordings - no synthetic data involved.

usage: python tools/axis_error.py [bag ...] [--max-spread 0.9] [--plot]
"""
import argparse
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
EGO = ROOT / "data" / "ego"
RES = ROOT / "data" / "results"
DISTANCES = [10, 20, 30, 50, 75, 100, 125, 150]


def errors_for(bag, res=None):
    global RES
    if res is not None:
        RES = res
    ego = json.loads((EGO / f"{bag}.json").read_text())
    axis_x = ego["axis_x"]
    gt = ego["axis"]
    rows = {r["frame"]: r for r in map(json.loads, (RES / f"{bag}.jsonl").read_text().splitlines()) if r}
    out = {d: [] for d in DISTANCES}
    speed = {}
    for f in ego["frames"]:
        i = f["frame"]
        r = rows.get(i)
        g = gt.get(str(i))
        if not r or not g or not r.get("axis_lat"):
            continue
        if f["speed"] < 1.0:          # a standing train gives no ground truth ahead
            continue
        speed[i] = f["speed"]
        est = np.array([np.nan if v is None else v for v in r["axis_lat"]], float)
        truth = np.array([np.nan if v is None else v for v in g["lat"]], float)
        for d in DISTANCES:
            if d > g["reach"] - 2:
                continue
            e = np.interp(d, axis_x, est)
            t = np.interp(d, axis_x, truth)
            if np.isfinite(e) and np.isfinite(t):
                out[d].append(abs(e - t))
    return out, speed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="*", default=[])
    ap.add_argument("--results", default="", help="directory with offline results (default data/results)")
    args = ap.parse_args()
    global RES
    if args.results:
        RES = Path(args.results)
    bags = args.bags or sorted(p.stem for p in EGO.glob("*.json"))
    total = {d: [] for d in DISTANCES}
    print("%-38s %6s  %s" % ("bag", "v m/s", "  ".join("%5d m" % d for d in DISTANCES)))
    for bag in bags:
        if not (RES / f"{bag}.jsonl").exists():
            continue
        err, speed = errors_for(bag)
        if not any(err.values()):
            print("%-38s %6s  (train standing)" % (bag, "-"))
            continue
        v = np.median(list(speed.values())) if speed else float("nan")
        cells = []
        for d in DISTANCES:
            e = err[d]
            total[d] += e
            cells.append("%5s" % (("%.2f" % np.median(e)) if e else "-"))
        print("%-38s %6.1f  %s   (median |error|, m)" % (bag, v, "  ".join(cells)))
    print("%-38s %6s  %s" % ("ALL median", "", "  ".join("%5s" % (("%.2f" % np.median(total[d])) if total[d] else "-")
                                                         for d in DISTANCES)))
    print("%-38s %6s  %s" % ("ALL p90", "", "  ".join("%5s" % (("%.2f" % np.percentile(total[d], 90)) if total[d] else "-")
                                                      for d in DISTANCES)))
    print("%-38s %6s  %s" % ("samples", "", "  ".join("%5d" % len(total[d]) for d in DISTANCES)))


if __name__ == "__main__":
    main()
