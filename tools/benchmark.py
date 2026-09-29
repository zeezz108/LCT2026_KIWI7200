"""One command to measure a configuration: false alarms on the recordings, timing, and (optionally) synthetic recall.

usage:
  python tools/benchmark.py [--tag NAME] [--param corridor.w_wall:=0.5 ...] [--synthetic none|quick]
                            [--profile] [--baseline TAG] [--build]

Writes data/results/bench/<tag>/{bags}.jsonl and metrics.json, prints a table and, with --baseline, the deltas.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from evaluate_synthetic import BAGS, offline  # noqa: E402

BENCH = ROOT / "data" / "results" / "bench"
RECORDINGS = ["doubleT_obstacle", "doubleT_platform", "roundT_doubleT", "roundT_pressureGate_roundT",
              "roundT_squareT_pressureGate_squareT", "squareT_platform_squareT_switch"]
# the customer's second dataset: one continuous 20-minute run (11 271 frames) through tunnels we had not seen.
# Five times more empty frames than everything above, so it is the main false-alarm test - but it takes 6 minutes,
# so it is opt-in with --new-data.
NEW_DATA = "new_data"
WITH_OBSTACLES = "doubleT_obstacle"


def build():
    r = subprocess.run(["wsl.exe", "-d", "Ubuntu-22.04", "--", "bash", "-lc",
                        "cd '/mnt/d/Python Projects/ЛЦТ 2026' && ./scripts/dev_build.sh"],
                       capture_output=True, text=True, encoding="utf-8", errors="replace")
    tail = (r.stdout or "").strip().splitlines()[-1:] or [""]
    print("build:", tail[0])
    if r.returncode != 0:
        raise SystemExit((r.stdout or "")[-3000:] + (r.stderr or "")[-2000:])


def bag_metrics(rows):
    lv = np.array([r["level"] for r in rows])
    ms = np.array([r["ms"]["total"] for r in rows])
    danger = [(o["distance"], o["lateral"]) for r in rows for o in r["obstacles"] if o["level"] == 2]
    return {
        "frames": len(rows),
        "clear": int((lv == 0).sum()), "warning": int((lv == 1).sum()), "danger": int((lv == 2).sum()),
        "ms_mean": float(ms.mean()), "ms_p50": float(np.percentile(ms, 50)), "ms_p95": float(np.percentile(ms, 95)),
        "danger_distances": [round(d, 1) for d, _ in danger[:200]],
        "danger_median_distance": float(np.median([d for d, _ in danger])) if danger else None,
        "axis_locked": float(np.mean([bool(r["axis_locked"]) for r in rows])),
    }


def run_recordings(tag, params, with_new_data=False):
    out_dir = BENCH / tag
    out_dir.mkdir(parents=True, exist_ok=True)
    metrics = {}
    for bag in RECORDINGS + ([NEW_DATA] if with_new_data else []):
        path = out_dir / f"{bag}.jsonl"
        if path.exists():
            path.unlink()
        offline(BAGS / bag, path, params)
        rows = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
        metrics[bag] = bag_metrics(rows)
        m = metrics[bag]
        print(f"  {bag:38s} frames={m['frames']:4d} warn={m['warning']:4d} danger={m['danger']:4d} "
              f"ms p50={m['ms_p50']:4.1f} p95={m['ms_p95']:4.1f}", flush=True)
    empty = [m for b, m in metrics.items() if b != WITH_OBSTACLES]
    frames = sum(m["frames"] for m in empty)
    agg = {
        "empty_frames": frames,
        "danger_rate": 100.0 * sum(m["danger"] for m in empty) / max(frames, 1),
        "warning_rate": 100.0 * sum(m["warning"] for m in empty) / max(frames, 1),
        "ms_p50": float(np.median([m["ms_p50"] for m in metrics.values()])),
        "ms_p95": float(np.max([m["ms_p95"] for m in metrics.values()])),
        "obstacle_danger_frames": metrics[WITH_OBSTACLES]["danger"],
        "obstacle_danger_distance": metrics[WITH_OBSTACLES]["danger_median_distance"],
    }
    return {"bags": metrics, "aggregate": agg}


def run_profile_error(tag, frames=40):
    """Accuracy of the bed profile against the floor actually seen, over the same runs (tools/profile_error.py)."""
    from profile_error import DISTANCES, errors_for  # noqa: PLC0415

    all_err = {d: [] for d in DISTANCES}
    for bag in RECORDINGS:
        for d, e in errors_for(bag, BENCH / tag, 10.0, frames).items():
            all_err[d].extend(e)
    out = {}
    for d, e in all_err.items():
        if e:
            arr = np.array(e)
            out[f"{d[0]}-{d[1]}"] = {"median": float(np.median(arr)),
                                     "p90": float(np.percentile(np.abs(arr), 90)),
                                     "too_low": float(np.mean(arr < -0.3))}
    return out


def run_synthetic(tag, params, suite):
    cmd = [sys.executable, str(ROOT / "tools" / "evaluate_synthetic.py"), "--suite", suite, "--tag", f"bench_{tag}"]
    for p in params:
        cmd += ["--param", p]
    print(f"  synthetic suite '{suite}' ...", flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    summary_path = ROOT / "data" / "results" / "synthetic" / f"bench_{tag}" / "summary.json"
    if not summary_path.exists():
        print((r.stdout or "")[-1500:])
        return None
    s = json.loads(summary_path.read_text())
    static = [x for x in s if x["speed"] == 0]
    approach = [x for x in s if x["speed"] > 0]
    return {
        "scenarios": len(s),
        "recall_mean": float(np.mean([x["recall_danger"] for x in static])) if static else None,
        "recall_by_distance": {str(d): round(float(np.mean([x["recall_danger"] for x in static if x["distance"] == d])), 3)
                               for d in sorted({x["distance"] for x in static})},
        "first_danger": {x["name"]: x["first_danger_distance"] for x in approach},
        "false_danger": int(sum(x["false_danger_obstacles"] for x in s)),
        "worst": sorted([(round(x["recall_danger"], 2), x["name"]) for x in static])[:6],
    }


def show(metrics, baseline=None):
    a = metrics["aggregate"]
    print("\n--- recordings")
    print("%-40s %6s %6s %7s %7s" % ("bag", "warn", "danger", "p50 ms", "p95 ms"))
    for bag, m in metrics["bags"].items():
        print("%-40s %6d %6d %7.1f %7.1f" % (bag, m["warning"], m["danger"], m["ms_p50"], m["ms_p95"]))
    print("\nempty recordings: DANGER %.2f%%  WARNING %.2f%%  (%d frames)"
          % (a["danger_rate"], a["warning_rate"], a["empty_frames"]))
    print("doubleT_obstacle: DANGER in %d frames at %.1f m" % (a["obstacle_danger_frames"],
                                                               a["obstacle_danger_distance"] or float("nan")))
    print("timing: p50 %.1f ms, worst p95 %.1f ms" % (a["ms_p50"], a["ms_p95"]))
    prof = metrics.get("profile_error")
    if prof:
        print("profile error vs the visible floor (median / p90 |err| / share more than 0.3 m too low):")
        print("  " + "   ".join(f"{k} m: {v['median']:+.2f}/{v['p90']:.2f}/{100 * v['too_low']:.0f}%"
                                for k, v in prof.items()))
    syn = metrics.get("synthetic")
    if syn:
        print("\nsynthetic: %d scenarios, mean DANGER recall %.1f%%, false objects %d"
              % (syn["scenarios"], 100 * (syn["recall_mean"] or 0), syn["false_danger"]))
        print("  recall by distance: " + "  ".join(f"{d} m: {100*v:.0f}%" for d, v in syn["recall_by_distance"].items()))
        if syn["first_danger"]:
            print("  first DANGER on approach: " + ", ".join(f"{k}: {v:.0f} m" for k, v in syn["first_danger"].items() if v))
        print("  weakest: " + ", ".join(f"{n} {100*r:.0f}%" for r, n in syn["worst"]))
    if baseline:
        b = baseline["aggregate"]
        print("\n--- vs baseline")
        for key, fmt, better in (("danger_rate", "%+.2f%%", "lower"), ("warning_rate", "%+.2f%%", "lower"),
                                 ("obstacle_danger_frames", "%+d", "higher"), ("ms_p50", "%+.1f ms", "lower")):
            if a.get(key) is None or b.get(key) is None:
                continue
            print("  %-22s %s (%s is better)" % (key, fmt % (a[key] - b[key]), better))
        bs, cs = baseline.get("synthetic"), metrics.get("synthetic")
        if bs and cs and bs.get("recall_mean") is not None and cs.get("recall_mean") is not None:
            print("  %-22s %+.1f%%" % ("synthetic recall", 100 * (cs["recall_mean"] - bs["recall_mean"])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="current")
    ap.add_argument("--param", action="append", default=[])
    ap.add_argument("--synthetic", default="none", choices=["none", "quick", "failure", "range"])
    ap.add_argument("--profile", action="store_true", help="also measure the bed profile against the visible floor")
    ap.add_argument("--new-data", action="store_true",
                    help="also run the customer's 20-minute recording (11 271 frames, ~6 min)")
    ap.add_argument("--baseline", default="")
    ap.add_argument("--build", action="store_true")
    args = ap.parse_args()
    if args.build:
        build()
    print(f"benchmark '{args.tag}'" + (f" with {args.param}" if args.param else ""))
    metrics = run_recordings(args.tag, args.param, args.new_data)
    if args.profile:
        metrics["profile_error"] = run_profile_error(args.tag)
    if args.synthetic != "none":
        metrics["synthetic"] = run_synthetic(args.tag, args.param, args.synthetic)
    metrics["params"] = args.param
    out = BENCH / args.tag / "metrics.json"
    out.write_text(json.dumps(metrics, indent=1))
    baseline = None
    if args.baseline:
        path = BENCH / args.baseline / "metrics.json"
        baseline = json.loads(path.read_text()) if path.exists() else None
        if baseline is None:
            print(f"(baseline '{args.baseline}' not found)")
    show(metrics, baseline)
    print(f"\nmetrics -> {out}")


if __name__ == "__main__":
    main()
