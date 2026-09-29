"""Summarise an offline_runner JSONL result: stage timings, verdicts, confirmed obstacles over time.

usage: python tools/analyze_run.py result.jsonl [--frames]
"""
import json
import sys
from collections import Counter

import numpy as np


def load(path):
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def main():
    path = sys.argv[1]
    show_frames = "--frames" in sys.argv
    rows = load(path)
    n = len(rows)
    print(f"{path}: {n} frames")
    stages = ["total", "transform", "calib", "axis", "corridor", "detect", "track"]
    ms = {s: np.array([r["ms"][s] for r in rows if r["ms"][s] is not None]) for s in stages}
    print("timing ms  " + "  ".join(f"{s}={ms[s].mean():.1f}" for s in stages) + f"  | total p95={np.percentile(ms['total'], 95):.1f}")
    levels = Counter(r["level"] for r in rows)
    print(f"levels: clear={levels.get(0, 0)} warning={levels.get(1, 0)} danger={levels.get(2, 0)}  "
          f"axis locked={np.mean([r['axis_locked'] for r in rows]) * 100:.0f}%  "
          f"height median={np.median([r['height'] for r in rows if r['height'] is not None]):.2f}")
    free = np.array([r["free"] for r in rows if r["free"] is not None])
    if len(free):
        print(f"free distance: median={np.median(free):.0f} m  p10={np.percentile(free, 10):.0f}  p90={np.percentile(free, 90):.0f}")
    clusters = np.array([len(r["clusters"]) for r in rows])
    print(f"candidate clusters per frame: mean={clusters.mean():.2f}  frames with any={np.mean(clusters > 0) * 100:.0f}%")

    # confirmed obstacle tracks
    tracks = {}
    for r in rows:
        for o in r["obstacles"]:
            t = tracks.setdefault(o["id"], {"frames": [], "dist": [], "lat": [], "level": [], "n": []})
            t["frames"].append(r["frame"])
            t["dist"].append(o["distance"])
            t["lat"].append(o["lateral"])
            t["level"].append(o["level"])
            t["n"].append(o["n"])
    print(f"confirmed tracks: {len(tracks)}")
    for tid, t in sorted(tracks.items(), key=lambda kv: kv[1]["frames"][0]):
        lv = Counter(t["level"])
        print(f"  id={tid:4d} frames {t['frames'][0]:4d}-{t['frames'][-1]:4d} ({len(t['frames']):3d})  "
              f"dist {min(t['dist']):6.1f}-{max(t['dist']):6.1f}  lat {np.median(t['lat']):+.2f}  "
              f"pts med {np.median(t['n']):5.0f}  danger={lv.get(2, 0)} warning={lv.get(1, 0)}")
    if show_frames:
        for r in rows:
            obs = "; ".join(f"{'D' if o['level'] == 2 else 'W'}#{o['id']} {o['distance']:.1f}m lat{o['lateral']:+.2f} n{o['n']}"
                            for o in r["obstacles"])
            cl = " ".join(f"{c['x']:.0f}/{c['lat']:+.1f}/{c['n']}" for c in r["clusters"][:6])
            print(f"f{r['frame']:4d} L{r['level']} free={r['free'] or 0:5.0f} k={r['kappas'][:4]} | {obs} || cand: {cl}")


if __name__ == "__main__":
    main()
