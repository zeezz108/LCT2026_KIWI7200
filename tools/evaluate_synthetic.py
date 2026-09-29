"""Synthetic evaluation: inject obstacles into real recordings, run the C++ offline runner, score against truth.

usage:
  python tools/evaluate_synthetic.py [--suite range|failure|near|low|case|newgen|quick|all] [--only NAME] [--tag DIR]
                                     [--param name:=value] [--keep-bags] [--jobs N]

Bases are segments of the recordings with a known character (straight, curve, station, switch, section change).
Objects are placed relative to the track axis of a reference run of the detector on the same segment without objects.
Results: data/results/synthetic[/tag]/summary.json plus a printed table; bags in data/synthetic/.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
BAGS = DATA / "for_hackathon"
SYN = DATA / "synthetic"
RES = DATA / "results" / "synthetic"
WSL_ROOT = "/mnt/d/Python Projects/ЛЦТ 2026"
PY = sys.executable

# segments of the recordings, chosen from the offline results (axis locked, known geometry)
BASES = {
    "round": ("roundT_squareT_pressureGate_squareT", 0, "прямой круглый тоннель"),
    "square": ("squareT_platform_squareT_switch", 0, "прямой прямоугольный тоннель"),
    "curve": ("roundT_squareT_pressureGate_squareT", 380, "кривая R ~ 370 м"),
    "curve_open": ("roundT_doubleT", 105, "кривая с выходом в двухпутный тоннель"),
    "station": ("doubleT_platform", 30, "станция, платформы с обеих сторон"),
    "switch": ("squareT_platform_squareT_switch", 730, "стрелка"),
    "section": ("roundT_squareT_pressureGate_squareT", 95, "смена сечения: круглый -> прямоугольный"),
    "gate": ("roundT_pressureGate_roundT", 35, "гермозатвор впереди"),
    # segments of the customer's second dataset (tools/cut_parts.py): tunnels we had never seen while tuning
    "new_double": ("new_double", 10, "новый датасет: двухпутный, кривая R ~ 1000 м"),
    "new_narrow": ("new_narrow", 10, "новый датасет: узкий однопутный, прямой"),
    "new_curve": ("new_curve", 10, "новый датасет: кривая R ~ 300 м"),
    "new_round": ("new_round", 10, "новый датасет: прямой перегон, видно 76 м"),
}
FRAMES = 30          # frames per static scenario
ARGS = argparse.Namespace(reflect_range=0.0)
APPROACH_FRAMES = 125


def scenario(base, obj, distance, lateral=0.0, speed=0.0, frames=FRAMES, suffix=""):
    name = f"{base}_{obj}_{distance:g}"
    if lateral:
        name += f"_lat{lateral:g}"
    if speed:
        name += f"_approach"
    return dict(name=name + suffix, base=base, object=obj, distance=distance, lateral=lateral, speed=speed,
                frames=frames)


def scenarios(suite):
    out = []
    if suite in ("range", "all"):
        for base in ("round", "square"):
            for d in (25, 50, 75, 100, 125, 150, 175, 200):
                out.append(scenario(base, "person", d))
            for d in (25, 50, 75, 100, 125):
                out.append(scenario(base, "box_small", d))
            for d in (50, 100, 150):
                out.append(scenario(base, "box_medium", d))
            for d in (150, 175, 200):
                out.append(scenario(base, "box_large", d))
    if suite in ("failure", "all"):
        for base in (b for b in BASES if not b.startswith("new_")):  # the failure map stays comparable run to run
            for d in (50, 100, 150):
                out.append(scenario(base, "person", d))
        for base in ("curve", "station", "switch", "section"):
            out.append(scenario(base, "box_medium", 100))
            out.append(scenario(base, "lying_person", 50))
        for base in ("round", "curve", "station"):
            for lat in (0.6, 1.0):
                out.append(scenario(base, "person", 100, lateral=lat))
        for base in ("round", "curve"):
            out.append(scenario(base, "person", 200, speed=15.0, frames=APPROACH_FRAMES))
    if suite in ("near", "all"):
        # emergency range: the object is already close. Below -6.2 deg elevation the Pandar128 channel spacing
        # is 0.485 deg instead of 0.125 deg, so short range is NOT simply "easier" than 50 m.
        for base in ("round", "square", "curve", "station"):
            for d in (8, 12, 16, 20):
                out.append(scenario(base, "person", d))
            for d in (8, 12, 16, 20, 25):
                out.append(scenario(base, "lying_person", d))
        for base in ("round", "curve"):
            out.append(scenario(base, "box_small", 10))
    if suite in ("low", "all"):
        # how far the low band (objects lying on the track) can be pushed: it is limited by the accuracy of the
        # bed profile, not by the number of returns
        for base in ("round", "square", "curve", "station"):
            for d in (50, 75, 100, 125):
                out.append(scenario(base, "lying_person", d))
    if suite in ("case", "all"):
        # what the organizers named explicitly: the smallest obstacle (300 x 300 x 100 mm) and torn cables
        # hanging into the clearance envelope
        for base in ("round", "square", "station", "curve"):
            for d in (10, 20, 25, 30, 40, 60):
                out.append(scenario(base, "min_object", d))
        for base in ("round", "square", "station"):
            for d in (30, 60, 100):
                out.append(scenario(base, "cable_across", d))
            out.append(scenario(base, "cable_loop", 40))
        # the customer asks to assume 85 km/h: 2.3 m between frames instead of the 1.5 m of the recordings
        for base in ("round", "square"):
            out.append(scenario(base, "person", 200, speed=23.6, frames=80, suffix="_85kmh"))  # 200 m at 23.6 m/s
    if suite in ("newgen", "all"):
        # generalization: the same objects on tunnels from the second dataset, which nothing was tuned on
        for base in ("new_double", "new_narrow", "new_curve", "new_round"):
            for d in (50, 100, 150):
                out.append(scenario(base, "person", d))
            out.append(scenario(base, "lying_person", 50))
            out.append(scenario(base, "box_medium", 100))
            # the customer's clearance is 2.1 m wide, so an object 1.0 m off the axis is still an obstacle:
            # where the walls stop confirming the axis the corridor narrows, and this is where it costs recall
            for d in (50, 75, 100):
                out.append(scenario(base, "person", d, lateral=1.0))
            out.append(scenario(base, "person", 100, lateral=0.6))
    if suite in ("warn", "all"):
        # the warning band: an object beside the clearance, where the level says "look" rather than "brake"
        for base in ("round", "square", "station"):
            for d in (60, 80, 100):
                out.append(scenario(base, "person", d, lateral=1.6))
    if suite == "quick":
        for base in ("round", "square", "curve", "station"):
            for d in (50, 100, 150):
                out.append(scenario(base, "person", d))
        out.append(scenario("round", "person", 200, speed=15.0, frames=APPROACH_FRAMES))
    seen, uniq = set(), []
    for s in out:
        if s["name"] not in seen:
            seen.add(s["name"])
            uniq.append(s)
    return uniq


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", **kw)


def offline(bag_dir, result_path, overrides=(), extra=()):
    drop_caches = ""
    if bag_dir.name == "new_data":  # the customer's second dataset: one 20-minute run, unpacked inside WSL
        wsl_bag = '"$HOME/data/new_data"'
        # 84 GB read through an 8 GB VM: hand the page cache back before and after, or the host runs out of memory
        drop_caches = "sync && echo 3 | sudo tee /proc/sys/vm/drop_caches > /dev/null && "
    elif bag_dir.parent == BAGS:  # the recordings have a faster copy inside WSL
        wsl_bag = f'"$HOME/data/for_hackathon/{bag_dir.name}"'
    else:
        wsl_bag = "'" + WSL_ROOT + "/" + bag_dir.relative_to(ROOT).as_posix() + "'"
    wsl_out = WSL_ROOT + "/" + result_path.relative_to(ROOT).as_posix()
    args = " ".join(f"-p {o}" for o in list(overrides) + list(extra))
    # TOD_INSTALL lets an evaluation run against a snapshot of the build while the main workspace is rebuilt
    install = os.environ.get("TOD_INSTALL") or "~/lct_ws/install"
    script = (
        f"{drop_caches}"
        "source /opt/ros/humble/setup.bash && "
        f"source {install}/setup.bash && "
        "ros2 run tunnel_obstacle_detector offline_runner --ros-args --params-file "
        f"{install}/tunnel_obstacle_detector/share/tunnel_obstacle_detector/config/detector.yaml "
        f"-p bag:={wsl_bag} -p output:='{wsl_out}' {args}"
        + (" ; sync && echo 3 | sudo tee /proc/sys/vm/drop_caches > /dev/null" if drop_caches else "")
    )
    r = run(["wsl.exe", "-d", "Ubuntu-22.04", "--", "bash", "-lc", script])
    if not result_path.exists():
        raise RuntimeError(r.stdout[-2000:] + r.stderr[-2000:])


def reference_axis(base, n_frames):
    """Track axis and bed profile per frame of the unmodified segment: the detector's corridor with the default
    configuration, rolling median over +-5 frames. Objects are placed relative to it, not to the lidar's line."""
    bag_name, f0, _ = BASES[base]
    path = SYN / f"axis_{base}.json"
    if path.exists():
        ref = json.loads(path.read_text())
        if ref.get("frames_needed", 0) >= f0 + n_frames:
            return path
    result = RES / f"base_{base}.jsonl"
    result.parent.mkdir(parents=True, exist_ok=True)
    offline(BAGS / bag_name, result, extra=[f"max_frames:={f0 + n_frames + 5}"])
    rows = [json.loads(l) for l in result.read_text().splitlines() if l.strip()]
    lat = np.array([[np.nan if v is None else v for v in r["axis_lat"]] for r in rows], float)
    bed = np.array([[np.nan if v is None else v for v in r["axis_bed"]] for r in rows], float)
    frames = {}
    for i, r in enumerate(rows):
        lo, hi = max(0, i - 5), min(len(rows), i + 6)
        frames[str(r["frame"])] = {"lat": np.nanmedian(lat[lo:hi], 0).round(3).tolist(),
                                   "bed": np.nanmedian(bed[lo:hi], 0).round(3).tolist()}
    ref = {"x": list(range(0, 201, 10)), "frames_needed": f0 + n_frames, "frames": frames}
    path.write_text(json.dumps(ref))
    return path


def driven_axis(base, n_frames, distance_needed):
    """Axis reference built from the path the train actually drove (tools/ego_motion.py), not from the estimate under
    test. Available where the recording continues far enough ahead of the segment."""
    bag_name, f0, _ = BASES[base]
    ego_path = DATA / "ego" / f"{bag_name}.json"
    if not ego_path.exists():
        return None, "no ego-motion file (run tools/ego_motion.py)"
    ego = json.loads(ego_path.read_text())
    detector_ref = json.loads(Path(reference_axis(base, n_frames)).read_text())
    frames = {}
    short = 0
    for idx in range(f0, f0 + n_frames):
        g = ego["axis"].get(str(idx))
        d = detector_ref["frames"].get(str(idx))
        if not g or not d:
            continue
        if g["reach"] < distance_needed + 3.0:
            short += 1
            continue
        frames[str(idx)] = {"lat": g["lat"], "bed": d["bed"]}
    if len(frames) < 0.5 * n_frames:
        return None, f"driven path reaches {distance_needed:.0f} m in only {len(frames)}/{n_frames} frames"
    path = SYN / f"axis_driven_{base}_{distance_needed:g}.json"
    path.write_text(json.dumps({"x": ego["axis_x"], "frames": frames}))
    return path, f"driven path, {len(frames)}/{n_frames} frames"


def inject(sc, axis_json=None):
    bag_name, f0, _ = BASES[sc["base"]]
    out = SYN / sc["name"]
    if out.exists():
        shutil.rmtree(out)
    cmd = [PY, str(ROOT / "tools" / "inject_obstacles.py"), "--bag", str(BAGS / bag_name), "--out", str(out),
           "--object", sc["object"], "--distance", str(sc["distance"]), "--lateral", str(sc["lateral"]),
           "--speed", str(sc["speed"]), "--frames", f"{f0}:{f0 + sc['frames']}"]
    if ARGS.reflect_range:
        cmd += ["--reflect-range", str(ARGS.reflect_range)]
    if axis_json:
        cmd += ["--axis-json", str(axis_json)]
    r = run(cmd)
    if r.returncode != 0:
        raise RuntimeError(r.stderr[-2000:])
    return out


REASONS = {1: "structure", 2: "touch", 3: "far_touch", 4: "low", 5: "sparse", 6: "floating", 7: "flat",
           8: "very_low"}


def score(sc, gt, rows):
    tol = lambda d: max(2.0, 0.04 * d)  # noqa: E731
    by_frame = {r["frame"]: r for r in rows}
    frames = gt["frames"]
    warm = 8  # frames the tracker needs before a far object can be confirmed
    danger_hits = any_hits = n_eval = false_danger = 0
    first_danger = first_any = None
    stages = {"visible": 0, "candidate": 0, "candidate_danger": 0, "rejected": {}, "missing": 0, "reported": 0}
    for f in frames:
        r = by_frame.get(f["frame"])
        if r is None:
            continue
        d = f["distance"]
        match = [o for o in r["obstacles"] if abs(o["distance"] - d) <= tol(d)]
        danger = [o for o in match if o["level"] == 2]
        false_danger += sum(1 for o in r["obstacles"] if o["level"] == 2 and abs(o["distance"] - d) > tol(d))
        if danger and first_danger is None:
            first_danger = d
        if match and first_any is None:
            first_any = d
        if f["frame"] >= warm or sc["speed"] > 0:
            n_eval += 1
            danger_hits += bool(danger)
            any_hits += bool(match)
        if f["injected_points"] > 0:
            stages["visible"] += 1
        cand = [c for c in r["clusters"] if abs(c["x"] - d) <= tol(d)]
        rej = [c for c in r.get("rejected", []) if isinstance(c, dict) and abs(c["x"] - d) <= tol(d)]
        if cand:
            stages["candidate"] += 1
            stages["candidate_danger"] += any(c["level"] == 2 for c in cand)
        elif rej:
            for c in rej:
                k = REASONS.get(c["reason"], str(c["reason"]))
                stages["rejected"][k] = stages["rejected"].get(k, 0) + 1
        elif f["injected_points"] > 0:
            stages["missing"] += 1
        stages["reported"] += any(abs(o["distance"] - d) <= tol(d) for o in r["obstacles"])
    pts = [f["injected_points"] for f in frames]
    return dict(name=sc["name"], base=sc["base"], object=sc["object"], distance=sc["distance"], lateral=sc["lateral"],
                speed=sc["speed"], injected_points=float(np.median(pts)), recall_danger=danger_hits / max(n_eval, 1),
                recall_any=any_hits / max(n_eval, 1), first_danger_distance=first_danger, first_any_distance=first_any,
                false_danger_obstacles=false_danger, frames=len(frames), stages=stages)


def print_table(summary):
    print("\n%-34s %5s %6s %7s %8s %s" % ("scenario", "pts", "danger", "first", "false", "lost at"))
    for s in sorted(summary, key=lambda s: (s["base"], s["object"], s["distance"], s["lateral"])):
        st = s["stages"]
        lost = []
        if st["missing"]:
            lost.append("no cluster %d" % st["missing"])
        for k, v in sorted(st["rejected"].items(), key=lambda kv: -kv[1]):
            lost.append("%s %d" % (k, v))
        if st["candidate"] and st["candidate"] > st["reported"]:
            lost.append("unconfirmed %d" % (st["candidate"] - st["reported"]))
        print("%-34s %5.0f %5.0f%% %7s %8d %s" % (
            s["name"], s["injected_points"], 100 * s["recall_danger"],
            ("%.0f" % s["first_danger_distance"]) if s["first_danger_distance"] else "-",
            s["false_danger_obstacles"], ", ".join(lost[:3])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default="failure", choices=["range", "failure", "near", "low", "case", "newgen", "warn", "quick", "all"])
    ap.add_argument("--only", default="", help="substring filter on the scenario name")
    ap.add_argument("--reflect-range", type=float, default=0.0,
                    help="return model of the injector: p = 0.9 (1 - (r/R)^3); 230 (default, calibrated on the real "
                         "person at 55 m) ... 400 (datasheet: 200 m at 10%% reflectivity)")
    ap.add_argument("--keep-bags", action="store_true")
    ap.add_argument("--param", action="append", default=[], help="ROS parameter override name:=value (repeatable)")
    ap.add_argument("--tag", default="", help="result sub-directory for this variant")
    ap.add_argument("--axis-source", default="detector", choices=["detector", "driven", "lidar-line"],
                    help="where the track axis for placing objects comes from")
    args = ap.parse_args()
    global RES, ARGS
    ARGS = args
    SYN.mkdir(parents=True, exist_ok=True)
    RES.mkdir(parents=True, exist_ok=True)
    todo = [s for s in scenarios(args.suite) if not args.only or args.only in s["name"]]
    axis_refs = {}
    skipped = []
    if args.axis_source != "lidar-line":
        need = {}
        for sc in todo:
            need[sc["base"]] = max(need.get(sc["base"], 0), sc["frames"])
        for base, n in need.items():
            print(f"reference axis for base {base} ({BASES[base][2]})", flush=True)
            axis_refs[base] = reference_axis(base, n)
    if args.axis_source == "driven":
        for sc in list(todo):
            far = sc["distance"] if sc["speed"] == 0 else sc["distance"]
            path, note = driven_axis(sc["base"], sc["frames"], far)
            if path is None:
                skipped.append((sc["name"], note))
                todo.remove(sc)
            else:
                sc["axis_file"] = path
        print(f"driven-path axis available for {len(todo)} scenarios; skipped {len(skipped)}", flush=True)
        for name, note in skipped[:8]:
            print(f"    skip {name}: {note}")
    if args.tag:
        RES = RES / args.tag
        RES.mkdir(parents=True, exist_ok=True)
    summary = []
    for i, sc in enumerate(todo, 1):
        bag = inject(sc, sc.get("axis_file") or axis_refs.get(sc["base"]))
        result = RES / f"{sc['name']}.jsonl"
        if result.exists():
            result.unlink()
        offline(bag, result, args.param)
        gt = json.loads((bag / "ground_truth.json").read_text())
        rows = [json.loads(l) for l in result.read_text().splitlines() if l.strip()]
        s = score(sc, gt, rows)
        summary.append(s)
        st = s["stages"]
        print(f"[{i}/{len(todo)}] {s['name']:32s} pts={s['injected_points']:5.0f} danger={s['recall_danger']*100:5.1f}% "
              f"any={s['recall_any']*100:5.1f}% first_danger={s['first_danger_distance']} "
              f"false={s['false_danger_obstacles']} | cand={st['candidate']} rejected={st['rejected']} "
              f"missing={st['missing']} reported={st['reported']}", flush=True)
        if not args.keep_bags:
            shutil.rmtree(bag)
        (RES / "summary.json").write_text(json.dumps(summary, indent=1))
        # ... and under the suite name too: several suites share one tag, and the plain file is overwritten
        (RES / f"summary_{ARGS.suite}.json").write_text(json.dumps(summary, indent=1))
    print_table(summary)
    print(f"\nsummary -> {RES / 'summary.json'}")


if __name__ == "__main__":
    main()
