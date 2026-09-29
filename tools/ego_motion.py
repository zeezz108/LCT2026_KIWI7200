"""Ego-motion of the train from the lidar alone, and the resulting ground-truth track axis.

The train follows the track, so the path it drives IS the track axis: transforming the future positions of the
track-axis point under the lidar into the current frame gives the true axis ahead, as far as the train has travelled.
Ego-motion itself comes from 2D scan matching of tunnel structures (walls, columns, portals) between consecutive
frames, with a constant-velocity prior for the direction along the tunnel, which geometry constrains only weakly.

usage:
  python tools/ego_motion.py --bag roundT_doubleT [--frames 0:260] [--out data/ego/roundT_doubleT.json]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "research" / "eda"))
sys.path.insert(0, str(ROOT / "tools"))
from bagio import Bag  # noqa: E402
import inject_obstacles as inj  # noqa: E402

AXIS_X = list(range(0, 201, 10))


def structure_points(track_xyz, z_lo=0.6, z_hi=3.2, x_lo=3.0, x_hi=60.0, y_half=8.0, cell=0.15):
    """2D samples of tunnel structures, thinned to one point per cell so that near-field density does not dominate."""
    p = track_xyz
    m = (p[:, 2] > z_lo) & (p[:, 2] < z_hi) & (p[:, 0] > x_lo) & (p[:, 0] < x_hi) & (np.abs(p[:, 1]) < y_half)
    q = p[m][:, :2]
    if len(q) == 0:
        return q
    keys = np.round(q / cell).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return q[np.sort(first)]


def longitudinal_profile(track_xyz, x_lo=5.0, x_hi=90.0, cell=0.25, y_half=3.0, z_lo=1.0, z_hi=3.2):
    """Strength of transverse structures (portals, columns, brackets, wall boxes) against distance ahead.

    Raw return counts fall off with range, and that pattern is fixed to the sensor, not to the tunnel; dividing by the
    local trend leaves the features that actually travel with the tunnel."""
    p = track_xyz
    m = ((p[:, 2] > z_lo) & (p[:, 2] < z_hi) & (np.abs(p[:, 1]) < y_half) & (p[:, 0] > x_lo) & (p[:, 0] < x_hi))
    edges = np.arange(x_lo, x_hi + cell, cell)
    if m.sum() < 50:
        return np.zeros(len(edges) - 1), edges
    hist, _ = np.histogram(p[m, 0], bins=edges)
    trend = np.convolve(hist, np.ones(41) / 41, mode="same") + 1.0
    return hist / trend, edges


def profile_shift(prev_profile, profile, cell=0.25, prior=None, window=1.0, lo=0.0, hi=4.0):
    """Forward displacement between two frames: how far the previous profile must move towards the sensor to match.

    With a prior (previous displacement) the search stays local, which resolves the ambiguity of periodic features
    such as tunnel ring joints."""
    n = len(profile)
    if n < 40:
        return None, 0.0
    a = prev_profile - prev_profile.mean()
    b = profile - profile.mean()
    lo_s, hi_s = (max(lo, prior - window), min(hi, prior + window)) if prior is not None else (lo, hi)
    best, best_score = None, -np.inf
    xs = np.arange(n, dtype=float)
    for d in np.arange(lo_s, hi_s + 1e-9, cell / 5.0):
        shifted = np.interp(xs + d / cell, xs, a, left=0.0, right=0.0)
        k = int(np.ceil(d / cell))
        x1 = shifted[: n - k]
        x2 = b[: n - k]
        if len(x1) < 40 or x1.std() < 1e-6 or x2.std() < 1e-6:
            continue
        score = float(np.corrcoef(x1, x2)[0, 1])
        if score > best_score:
            best_score, best = score, float(d)
    return best, best_score


def icp2d(prev_pts, cur_pts, init, iterations=25, max_dist=1.0, trim=0.75):
    """Trimmed rigid 2D fit of cur_pts onto prev_pts. init and result are (dx, dy, dyaw): the sensor motion between
    the frames, i.e. the transform that maps current points into the previous frame."""
    tx, ty, th = init
    if len(prev_pts) < 100 or len(cur_pts) < 100:
        return tx, ty, th, 0.0, 0
    tree = cKDTree(prev_pts)
    used = 0
    rms = float("inf")
    for _ in range(iterations):
        c, s = np.cos(th), np.sin(th)
        rot = np.array([[c, -s], [s, c]])
        moved = cur_pts @ rot.T + np.array([tx, ty])
        dist, idx = tree.query(moved, distance_upper_bound=max_dist)
        ok = np.isfinite(dist)
        if ok.sum() < 50:
            break
        limit = np.quantile(dist[ok], trim)
        ok &= dist <= max(limit, 0.05)
        used = int(ok.sum())
        a = cur_pts[ok]
        b = prev_pts[idx[ok]]
        ca, cb = a.mean(0), b.mean(0)
        h = (a - ca).T @ (b - cb)
        u, _, vt = np.linalg.svd(h)
        r = vt.T @ u.T
        if np.linalg.det(r) < 0:
            vt[1] *= -1
            r = vt.T @ u.T
        t = cb - r @ ca
        new_th = float(np.arctan2(r[1, 0], r[0, 0]))
        new_tx, new_ty = float(t[0]), float(t[1])
        step = abs(new_tx - tx) + abs(new_ty - ty) + 10.0 * abs(new_th - th)
        tx, ty, th = new_tx, new_ty, new_th
        rms = float(np.sqrt(np.mean(np.minimum(dist[ok], max_dist) ** 2)))
        if step < 1e-4:
            break
    return tx, ty, th, rms, used


def smooth_speed(dx, dt, max_accel=3.0):
    """Median filter plus an acceleration clamp: train speed changes slowly, single-frame matching does not."""
    dt = np.maximum(np.array(dt, dtype=float), 1e-3)
    v = np.array(dx, dtype=float) / dt
    k = 5
    pad = np.pad(v, (k // 2, k // 2), mode="edge")
    med = np.array([np.median(pad[i:i + k]) for i in range(len(v))])
    out = med.copy()
    out[0] = float(np.median(med[: min(len(med), 7)]))  # the first frames must not anchor the clamp at zero
    for i in range(1, len(out)):
        limit = max_accel * dt[i]
        out[i] = np.clip(out[i], out[i - 1] - limit, out[i - 1] + limit)
    return out


def run(bag_name, frame_slice, verbose=True):
    bag = Bag(bag_name)
    f0, f1 = frame_slice
    f1 = min(f1, len(bag))
    mount = inj.mount_rotation("-y")
    rng = np.random.default_rng(3)
    plane = None
    prev = None
    frames = []
    prior = (None, 0.0, 0.0)
    bootstrap = []
    for i in range(f0, f1):
        arr, stamp, _ = bag.points(i)
        v = arr[(arr["x"] != 0) | (arr["y"] != 0)]
        p = np.stack([v["x"], v["y"], v["z"]], 1).astype(float) @ mount.T
        plane = inj.bed_plane(p, rng, plane)
        n, h = plane
        track = p @ inj.align_to_plane(n).T
        track[:, 2] += h
        pts = structure_points(track)
        prof, _ = longitudinal_profile(track)
        entry = {"frame": i, "stamp": float(stamp), "points": len(pts)}
        if prev is not None:
            dt = float(stamp - prev["stamp"])
            # rotation from the walls, forward displacement from the transverse structures along the tunnel
            _, dy, dyaw, rms, used = icp2d(prev["pts"], pts, (prior[0] or 0.0, 0.0, prior[2]))
            # the first frames search the whole range; afterwards a local search around the previous displacement
            # resolves the ambiguity of periodic features (tunnel ring joints, sleepers)
            dx, corr = profile_shift(prev["prof"], prof, prior=prior[0])
            if len(bootstrap) < 10:
                wide, wide_corr = profile_shift(prev["prof"], prof, prior=None)
                if wide is not None and wide_corr > (corr or -1.0):
                    dx, corr = wide, wide_corr
                if dx is not None and corr > 0.4:
                    bootstrap.append(dx)
                    if len(bootstrap) >= 3:
                        dx = float(np.median(bootstrap[-5:]))
            if dx is None or corr < 0.25:
                dx, corr = (prior[0] if prior[0] is not None else 0.0, -1.0)
            entry.update(dx=float(dx), dy=dy, dyaw=dyaw, rms=rms, used=used, dt=dt, corr=float(corr))
            prior = (float(dx), dy, dyaw)
        else:
            entry.update(dx=0.0, dy=0.0, dyaw=0.0, rms=0.0, used=0, dt=0.1, corr=0.0)
        frames.append(entry)
        prev = {"pts": pts, "prof": prof, "stamp": stamp}
        if verbose and (i - f0) % 25 == 0:
            print("  frame %d: dx=%.2f (corr %.2f) dyaw=%+.3f deg rms=%.3f pts=%d"
                  % (i, entry["dx"], entry["corr"], np.degrees(entry["dyaw"]), entry["rms"], len(pts)), flush=True)

    dt = np.array([f["dt"] for f in frames])
    speed = smooth_speed([f["dx"] for f in frames], dt)
    x = y = th = 0.0
    for f, v in zip(frames, speed):
        f["speed"] = float(v)
        step = float(v * f["dt"])
        f["dx_smooth"] = step
        x += step * np.cos(th) - f["dy"] * np.sin(th)
        y += step * np.sin(th) + f["dy"] * np.cos(th)
        th += f["dyaw"]
        f["X"], f["Y"], f["THETA"] = float(x), float(y), float(th)
    return frames


def ground_truth_axis(frames, lateral_offset):
    """Track axis ahead of every frame: future positions of the track-axis point under the lidar, in that frame."""
    xs = np.array([f["X"] for f in frames])
    ys = np.array([f["Y"] for f in frames])
    ths = np.array([f["THETA"] for f in frames])
    off = np.array([lateral_offset.get(f["frame"], 0.0) for f in frames])
    ax = xs - off * np.sin(ths)
    ay = ys + off * np.cos(ths)
    out = {}
    for i, f in enumerate(frames):
        c, s = np.cos(-ths[i]), np.sin(-ths[i])
        dx = ax[i:] - xs[i]
        dy = ay[i:] - ys[i]
        lx = c * dx - s * dy
        ly = s * dx + c * dy
        if len(lx) < 3 or lx[-1] < 5.0:
            out[f["frame"]] = {"lat": [None] * len(AXIS_X), "reach": 0.0}
            continue
        lat = [float(np.interp(v, lx, ly)) if v <= lx[-1] else None for v in AXIS_X]
        out[f["frame"]] = {"lat": lat, "reach": float(lx[-1])}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--frames", default="0:100000")
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    f0, f1 = (int(v) for v in args.frames.split(":"))
    print("ego motion: %s frames %d:%d" % (args.bag, f0, f1), flush=True)
    frames = run(args.bag, (f0, f1))

    lateral = {}
    res = ROOT / "data" / "results" / ("%s.jsonl" % Path(args.bag).name)
    if res.exists():
        for line in res.read_text().splitlines():
            if not line.strip():
                continue
            r = json.loads(line)
            if r.get("axis_lat") and r["axis_lat"][0] is not None:
                lateral[r["frame"]] = r["axis_lat"][0]
    axis = ground_truth_axis(frames, lateral)
    out = Path(args.out) if args.out else ROOT / "data" / "ego" / ("%s.json" % Path(args.bag).name)
    out.parent.mkdir(parents=True, exist_ok=True)
    speeds = np.array([f["speed"] for f in frames])
    out.write_text(json.dumps({"bag": Path(args.bag).name, "axis_x": AXIS_X, "frames": frames,
                               "axis": {str(k): v for k, v in axis.items()}}))
    print("speed m/s: median %.2f min %.2f max %.2f; path %.1f m over %d frames -> %s"
          % (np.median(speeds), speeds.min(), speeds.max(), sum(f["dx_smooth"] for f in frames), len(frames), out))


if __name__ == "__main__":
    main()
