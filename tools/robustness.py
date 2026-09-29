"""Does the detector survive a recording that was not made the way ours were?

The control recording will come from another train run: possibly another mounting height, another roll, a driver
that publishes no intensity or no ring field, a cheaper sensor, lost packets. Each case here transforms real
recordings (tools/transform_bag.py) so that the scene and the right answer stay the same, and checks two numbers:
the empty recording must stay empty, and the two people in `doubleT_obstacle` must still be found at ~56 m.

usage:
  python tools/robustness.py [--only NAME] [--frames 120] [--keep] [--tag DIR]
"""
import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from evaluate_synthetic import BAGS, offline, run  # noqa: E402

DATA = ROOT / "data"
OUT = DATA / "transformed"
RES = DATA / "results" / "robustness"
EMPTY_BAG = "roundT_pressureGate_roundT"   # 268 frames, no obstacles, round tunnel with a pressure gate
OBSTACLE_BAG = "doubleT_obstacle"          # two people at ~56 m, train standing, 360 deg cloud

CASES = [
    ("baseline", []),
    ("mount 0.4 m higher", ["--height", "0.4"]),
    ("mount 0.3 m lower", ["--height", "-0.3"]),
    ("roll +3 deg", ["--roll", "3"]),
    ("pitch +1.5 deg", ["--pitch", "1.5"]),
    ("64 channels", ["--rings", "64"]),
    ("no intensity", ["--no-intensity"]),
    ("no ring field", ["--no-ring"]),
    ("unorganized cloud", ["--unorganized"]),
    ("20% frames lost", ["--drop-frames", "0.2"]),
    ("range noise 5 cm", ["--range-noise", "0.05"]),
    ("every 2nd point", ["--decimate", "2"]),
]


def transform(bag, name, opts, frames):
    out = OUT / f"{bag}__{name}"
    if out.exists():
        shutil.rmtree(out)
    cmd = [sys.executable, str(ROOT / "tools" / "transform_bag.py"), "--bag", str(BAGS / bag), "--out", str(out),
           "--frames", f"0:{frames}"] + opts
    r = run(cmd)
    if r.returncode != 0:
        raise RuntimeError(r.stdout[-1500:] + r.stderr[-1500:])
    return out


def metrics(path, expect_distance=None):
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    danger = [r for r in rows if r["level"] == 2]
    hits = [o["distance"] for r in danger for o in r["obstacles"] if o["level"] == 2]
    out = {"frames": len(rows), "danger": len(danger), "warning": sum(1 for r in rows if r["level"] == 1)}
    if hits:
        out["distance"] = sorted(hits)[len(hits) // 2]
    if expect_distance is not None and hits:
        out["on_target"] = sum(1 for r in danger
                               if any(abs(o["distance"] - expect_distance) < 3.0 for o in r["obstacles"]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="", help="substring filter on the case name")
    ap.add_argument("--frames", type=int, default=120, help="frames of each recording to use")
    ap.add_argument("--obstacle-frames", type=int, default=60)
    ap.add_argument("--keep", action="store_true", help="keep the transformed bags")
    ap.add_argument("--tag", default="")
    ap.add_argument("--param", action="append", default=[])
    args = ap.parse_args()
    res = RES / args.tag if args.tag else RES
    res.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)

    print(f"{'case':22s}{'empty: DANGER/frames':>24s}{'warn':>6s}{'obstacle: DANGER':>18s}{'at':>7s}{'on target':>11s}")
    table = []
    for name, opts in CASES:
        if args.only and args.only not in name:
            continue
        key = name.replace(" ", "_").replace("%", "pct")
        row = {"case": name}
        for bag, frames, expect in ((EMPTY_BAG, args.frames, None), (OBSTACLE_BAG, args.obstacle_frames, 55.8)):
            path = transform(bag, key, opts, frames)
            out = res / f"{bag}__{key}.jsonl"
            try:
                offline(path, out, args.param)
                row[bag] = metrics(out, expect)
            except RuntimeError as exc:
                row[bag] = {"error": str(exc)[-300:]}
            if not args.keep:
                shutil.rmtree(path, ignore_errors=True)
        e, o = row[EMPTY_BAG], row[OBSTACLE_BAG]
        print(f"{name:22s}{(str(e.get('danger', '?')) + '/' + str(e.get('frames', '?'))):>24s}"
              f"{e.get('warning', '?'):>6}{o.get('danger', '?'):>18}"
              f"{o.get('distance', float('nan')):>7.1f}{o.get('on_target', 0):>11}", flush=True)
        table.append(row)
    (res / "summary.json").write_text(json.dumps(table, indent=1))
    print(f"\nsummary -> {res / 'summary.json'}")


if __name__ == "__main__":
    main()
