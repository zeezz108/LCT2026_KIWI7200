"""Summary of one offline run: what the detector reported over a long recording, grouped into episodes.

Made for the customer's second dataset — a single continuous tunnel run of thousands of frames, where a per-frame
table is useless and what matters is "how many separate times did it say DANGER, where, and for how long".

usage: python tools/summarize_run.py data/results/new_data.jsonl [--gap 5] [--level 2]
"""
import argparse
import json
from pathlib import Path

import numpy as np


def episodes(rows, level, max_gap):
    """Consecutive frames (allowing `max_gap` frames of silence) where the verdict reached `level`."""
    out = []
    current = None
    for r in rows:
        hit = r["level"] >= level
        if hit:
            if current is None or r["frame"] - current["last_frame"] > max_gap:
                if current is not None:
                    out.append(current)
                current = {"first_frame": r["frame"], "last_frame": r["frame"], "frames": 0,
                           "distances": [], "laterals": [], "heights": [], "points": []}
            current["last_frame"] = r["frame"]
            current["frames"] += 1
            for o in r["obstacles"]:
                if o["level"] >= level:
                    current["distances"].append(o["distance"])
                    current["laterals"].append(o["lateral"])
                    current["heights"].append(o.get("hmax", float("nan")))
                    current["points"].append(o["n"])
    if current is not None:
        out.append(current)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--gap", type=int, default=5, help="frames of silence that still count as one episode")
    ap.add_argument("--level", type=int, default=2, help="2 = DANGER, 1 = WARNING and above")
    args = ap.parse_args()
    rows = [json.loads(line) for line in Path(args.path).read_text(encoding="utf-8").splitlines() if line.strip()]
    if not rows:
        raise SystemExit("empty run")

    lv = np.array([r["level"] for r in rows])
    ms = np.array([r["ms"]["total"] for r in rows])
    stamps = np.array([r["stamp"] for r in rows])
    locked = np.mean([bool(r["axis_locked"]) for r in rows])
    free = np.array([r["free"] for r in rows])
    print(f"{Path(args.path).name}: {len(rows)} frames, {stamps[-1] - stamps[0]:.0f} s "
          f"({(stamps[-1] - stamps[0]) / 60:.1f} min)")
    print(f"  levels: CLEAR {int((lv == 0).sum())} ({100 * (lv == 0).mean():.2f}%), "
          f"WARNING {int((lv == 1).sum())} ({100 * (lv == 1).mean():.2f}%), "
          f"DANGER {int((lv == 2).sum())} ({100 * (lv == 2).mean():.2f}%)")
    print(f"  frame time: p50 {np.percentile(ms, 50):.1f} ms, p95 {np.percentile(ms, 95):.1f} ms, "
          f"max {ms.max():.1f} ms")
    print(f"  axis locked on rails: {100 * locked:.1f}% of frames; corridor seen ahead: "
          f"median {np.median(free):.0f} m, p10 {np.percentile(free, 10):.0f} m")

    eps = episodes(rows, args.level, args.gap)
    name = "DANGER" if args.level == 2 else "WARNING+"
    print(f"\n  {len(eps)} {name} episodes:")
    print(f"    {'frames':>9} {'time s':>8} {'distance m':>22} {'lateral m':>12} {'height m':>9} {'points':>8}")
    for e in eps:
        d = np.array(e["distances"])
        lat = np.array(e["laterals"])
        h = np.array(e["heights"])
        n = np.array(e["points"])
        span = (e["last_frame"] - e["first_frame"] + 1) * 0.1
        print(f"    {e['first_frame']:4d}-{e['last_frame']:<4d} {span:8.1f} "
              f"{d.min():7.1f} .. {d.max():<7.1f} ({len(d):3d}) {np.median(lat):+11.2f} "
              f"{np.nanmedian(h):9.2f} {int(np.median(n)):8d}")


if __name__ == "__main__":
    main()
