"""How much does a far object gain if returns of several frames are put into one track frame?

At 175 m a person gives ~6 returns per frame and at 200 m ~2, while the smallest cluster we accept is 4 points.
The train moves 1.5-2.3 m between frames and the obstacle does not, so returns of neighbouring frames can be
brought into the current one along the path the train drove (tools/ego_motion.py). This measures what that buys:
how many returns land on the object, and how far they smear, against the same question asked of everything else
inside the clearance envelope (the structures that give us the remaining far false alarms).

usage:
  python tools/accumulation_probe.py --bag data/synthetic/round_person_200_approach --ego roundT_squareT_pressureGate_squareT
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "research" / "eda"))
sys.path.insert(0, str(ROOT / "tools"))
from bagio import Bag  # noqa: E402
import inject_obstacles as inj  # noqa: E402


def track_frames(bag, f0, f1):
    """Points of every frame in its own track frame (X forward, Y left, Z up from the bed)."""
    mount = inj.mount_rotation("-y")
    rng = np.random.default_rng(3)
    plane = None
    out = []
    for i in range(f0, f1):
        arr, stamp, _ = bag.points(i)
        v = arr[(arr["x"] != 0) | (arr["y"] != 0)]
        p = np.stack([v["x"], v["y"], v["z"]], 1).astype(float) @ mount.T
        plane = inj.bed_plane(p, rng, plane)
        n, h = plane
        track = p @ inj.align_to_plane(n).T
        track[:, 2] += h
        out.append((i, track))
    return out


def to_frame(points, src, dst):
    """Points of frame `src` expressed in the track frame of frame `dst`, using the driven path."""
    c, s = np.cos(src["THETA"]), np.sin(src["THETA"])
    gx = src["X"] + c * points[:, 0] - s * points[:, 1]
    gy = src["Y"] + s * points[:, 0] + c * points[:, 1]
    c, s = np.cos(-dst["THETA"]), np.sin(-dst["THETA"])
    dx, dy = gx - dst["X"], gy - dst["Y"]
    out = points.copy()
    out[:, 0] = c * dx - s * dy
    out[:, 1] = s * dx + c * dy
    return out


def box_count(points, x, half_along, lat, half_lat, z_lo, z_hi):
    m = ((np.abs(points[:, 0] - x) < half_along) & (np.abs(points[:, 1] - lat) < half_lat) &
         (points[:, 2] > z_lo) & (points[:, 2] < z_hi))
    return points[m]


def clutter(bag_name, ego_name, window, distances, frames_at, half_lat=1.05):
    """The same accumulation on an empty recording: what else lands inside the clearance envelope, and whether it
    stays compact. A real object keeps a 0.1-0.2 m spread; clutter that smears is harmless after clustering."""
    ego = {f["frame"]: f for f in json.loads((Path("data/ego") / f"{ego_name}.json").read_text())["frames"]}
    bag = Bag(ego_name)
    print()
    print(f"{ego_name}: what accumulates inside the envelope on an empty recording "
          f"({window} frames, {len(frames_at)} sample frames)")
    print(f"{'distance':>9} {'per frame':>10} {'accumulated':>12} {'largest 0.7 m group':>20}")
    for d in distances:
        per, acc, biggest = [], [], []
        for k in frames_at:
            got = dict(track_frames(bag, k - window + 1, k + 1))
            if k not in got or k not in ego:
                continue
            own = box_count(got[k], d, 1.5, 0.0, half_lat, 0.3, 3.0)
            pile = [own]
            for j in range(k - window + 1, k):
                if j in got and j in ego:
                    pile.append(box_count(to_frame(got[j], ego[j], ego[k]), d, 1.5, 0.0, half_lat, 0.3, 3.0))
            a_pts = np.concatenate(pile)
            per.append(len(own))
            acc.append(len(a_pts))
            biggest.append(largest_group(a_pts, 0.7))
        if per:
            print(f"{d:9.0f} {np.mean(per):10.1f} {np.mean(acc):12.1f} {np.max(biggest):20d}")


def largest_group(points, eps):
    """Size of the largest euclidean group at radius `eps` (plain O(n^2), the counts here are small)."""
    n = len(points)
    if n == 0:
        return 0
    seen = np.zeros(n, bool)
    best = 0
    for i in range(n):
        if seen[i]:
            continue
        queue, seen[i], size = [i], True, 0
        while queue:
            a = queue.pop()
            size += 1
            d = np.linalg.norm(points - points[a], axis=1)
            for j in np.nonzero((d < eps) & ~seen)[0]:
                seen[j] = True
                queue.append(int(j))
        best = max(best, size)
    return best


def axis_relative(points, axis_lat, axis_x):
    """(along the track, offset from the axis, height) - the coordinates in which the track itself does not move."""
    out = points.copy()
    out[:, 1] = points[:, 1] - np.interp(points[:, 0], axis_x, axis_lat)
    return out


def shift_only(points, ds, axis_lat, axis_x):
    """The cheap transform: in axis-relative coordinates a frame differs from the next one by the driven distance
    alone - no heading, no lateral. Worth checking on a curve, because it removes ego-motion from the problem."""
    out = axis_relative(points, axis_lat, axis_x)
    out[:, 0] -= ds
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="bag directory (a synthetic scenario with ground_truth.json)")
    ap.add_argument("--ego", required=True, help="name of the ego-motion file in data/ego")
    ap.add_argument("--window", type=int, default=5, help="frames accumulated, including the current one")
    ap.add_argument("--distances", default="150,175,200")
    ap.add_argument("--clutter-frames", default="", help="comma separated frames of the empty recording to sample")
    ap.add_argument("--axis-base", default="", help="base name of data/synthetic/axis_<base>.json for the cheap transform")
    args = ap.parse_args()

    bag_dir = Path(args.bag)
    gt = json.loads((bag_dir / "ground_truth.json").read_text())
    ego = {f["frame"]: f for f in json.loads((Path("data/ego") / f"{args.ego}.json").read_text())["frames"]}
    want = [float(v) for v in args.distances.split(",")]

    # frames where the object is closest to each requested distance
    picks = {}
    for f in gt["frames"]:
        for d in want:
            best = picks.get(d)
            if best is None or abs(f["distance"] - d) < abs(best["distance"] - d):
                picks[d] = f

    axis_path = ROOT / "data" / "synthetic" / f"axis_{args.axis_base}.json" if args.axis_base else None
    axis_ref = json.loads(axis_path.read_text()) if axis_path and axis_path.exists() else None
    bag = Bag(bag_dir)
    lo = min(p["frame"] for p in picks.values()) - args.window
    hi = max(p["frame"] for p in picks.values()) + 1
    frames = dict((i, t) for i, t in track_frames(bag, max(0, lo), min(hi, len(bag))))

    print(f"{'distance':>9} {'frame':>6} {'per frame':>10} {'accumulated':>12} {'spread along':>13} "
          f"{'spread lat':>11} {'envelope/frame':>15} {'envelope acc':>13}")
    for d in want:
        f = picks[d]
        k, src = f["frame"], f["source_frame"]
        if k not in frames or src not in ego:
            continue
        dist, lat = f["distance"], f.get("y", 0.0)
        own = box_count(frames[k], dist, 1.5, lat, 0.6, 0.3, 2.2)
        # everything inside the clearance envelope at the same distance, object aside: what also accumulates
        env = box_count(frames[k], dist, 1.5, 0.0, 1.05, 0.3, 3.0)
        acc, acc_env = [own], [env]
        for j in range(k - args.window + 1, k):
            if j not in frames or (src - (k - j)) not in ego:
                continue
            moved = to_frame(frames[j], ego[src - (k - j)], ego[src])
            acc.append(box_count(moved, dist, 1.5, lat, 0.6, 0.3, 2.2))
            acc_env.append(box_count(moved, dist, 1.5, 0.0, 1.05, 0.3, 3.0))
        a = np.concatenate(acc)
        ae = np.concatenate(acc_env)
        if axis_ref is not None:
            ax = axis_ref["x"]
            cheap = [axis_relative(frames[k], axis_ref["frames"][str(src)]["lat"], ax)]
            ds = 0.0
            for j in range(k - 1, k - args.window, -1):
                if j not in frames or (src - (k - j)) not in ego:
                    continue
                ds += ego[src - (k - j) + 1]["dx_smooth"]
                ref = axis_ref["frames"].get(str(src - (k - j)))
                if ref is None:
                    continue
                cheap.append(shift_only(frames[j], ds, ref["lat"], ax))
            lat_rel = lat - float(np.interp(dist, ax, axis_ref["frames"][str(src)]["lat"]))
            cp = np.concatenate([box_count(c, dist, 1.5, lat_rel, 0.6, 0.3, 2.2) for c in cheap])
            print(f"{'':9s} {'':6s} {'':10s} {len(cp):12d} {float(cp[:, 0].std()) if len(cp) > 1 else 0:13.2f} "
                  f"{float(cp[:, 1].std()) if len(cp) > 1 else 0:11.2f}   (shift only, no heading)")
        spread_x = float(a[:, 0].std()) if len(a) > 1 else 0.0
        spread_y = float(a[:, 1].std()) if len(a) > 1 else 0.0
        print(f"{dist:9.0f} {k:6d} {len(own):10d} {len(a):12d} {spread_x:13.2f} {spread_y:11.2f} "
              f"{len(env):15d} {len(ae):13d}")


    if args.clutter_frames:
        clutter(args.bag, args.ego, args.window, want,
                [int(v) for v in args.clutter_frames.split(",")])


if __name__ == "__main__":
    main()
