"""False-alarm summary of offline results on the recorded bags.

usage: python tools/fp_summary.py [result_dir]   (default data/results)
doubleT_obstacle contains real people and is reported separately from the empty bags.
"""
import collections
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BAGS = ["doubleT_obstacle", "doubleT_platform", "roundT_doubleT", "roundT_pressureGate_roundT",
        "roundT_squareT_pressureGate_squareT", "squareT_platform_squareT_switch"]


def main():
    res = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "data" / "results"
    tot = tot_d = tot_w = 0
    for b in BAGS:
        path = res / f"{b}.jsonl"
        if not path.exists():
            print(f"{b}: missing")
            continue
        rows = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
        lv = collections.Counter(r["level"] for r in rows)
        dd = collections.Counter(int(o["distance"] // 25 * 25) for r in rows for o in r["obstacles"] if o["level"] == 2)
        if b != "doubleT_obstacle":
            tot += len(rows)
            tot_d += lv[2]
            tot_w += lv[1]
        print(f"{b:38s} frames={len(rows):4d} clear={lv[0]:4d} warn={lv[1]:4d} danger={lv[2]:4d} | danger by 25 m: {sorted(dd.items())}")
    if tot:
        print(f"empty bags: danger frames {100 * tot_d / tot:.1f}%  warning frames {100 * tot_w / tot:.1f}%  ({tot} frames)")


if __name__ == "__main__":
    main()
