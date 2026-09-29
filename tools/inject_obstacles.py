"""Inject a synthetic obstacle into a ROS 2 bag by ray casting the lidar beams against it.

The object stands on the track bed (RANSAC plane under the lidar) at `distance` metres ahead and `lateral` metres
to the left of the lidar's forward line. With --axis-json the lateral offset and the bed height are taken relative to
a reference track axis per frame instead (the lidar is not exactly on the axis and the track heading differs from the
lidar's by ~0.5 deg, i.e. 1.3 m at 150 m). Every beam of the organized cloud (valid or not) whose ray hits the object
closer than the recorded return is replaced by the hit, dropped with probability 1 - p(range).
The person model (cylinder 0.4 x 1.75 m, p = 0.9 (1 - (r/230)^3)) gives ~54 returns at 55 m, as the real person in
doubleT_obstacle (~50).

usage:
  python tools/inject_obstacles.py --bag <bag dir> --out <new bag dir> --object person --distance 100
         [--lateral 0.0] [--speed 0.0] [--frames 0:150] [--forward-axis -y]
Writes <out>/ (sqlite3 bag readable by ROS 2 Humble) and <out>/ground_truth.json.
"""
import argparse
import json
from dataclasses import replace
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

PT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("ring", "<u2"), ("timestamp", "<f8")])
RINGS = 128

OBJECTS = {
    # kind, size (along, across, height) or (radius, height), intensity
    "person": ("cylinder", (0.20, 1.75), 12.0),
    "box_small": ("box", (0.5, 0.5, 0.5), 30.0),
    "box_medium": ("box", (0.6, 0.6, 1.0), 30.0),
    "box_large": ("box", (1.0, 1.0, 1.0), 30.0),
    "lying_person": ("box", (1.7, 0.5, 0.35), 12.0),
    # the smallest obstacle the organizers named: 300 x 300 x 100 mm
    "min_object": ("box", (0.3, 0.3, 0.1), 20.0),
    # a torn cable hanging into the clearance: they called detecting those "very important"
    "cable_across": ("box", (0.04, 1.6, 0.04), 8.0),
    "cable_loop": ("box", (0.05, 0.05, 1.2), 8.0),
}
# objects that do not stand on the bed: base height above it [m]
HANGING = {"cable_across": 2.0, "cable_loop": 1.6}


def mount_rotation(forward_axis):
    fwd = {"x": [1, 0, 0], "-x": [-1, 0, 0], "y": [0, 1, 0], "-y": [0, -1, 0]}[forward_axis]
    fwd = np.array(fwd, float)
    up = np.array([0.0, 0.0, 1.0])
    left = np.cross(up, fwd)
    return np.stack([fwd, left, up])  # rows: cloud -> mount


def bed_plane(M, rng, previous=None):
    """RANSAC plane 3..20 m ahead below the sensor (mount frame). Returns (normal, height).

    The track bed is the *lowest* large plane: walkways and cable trays in round tunnels form planes too, and an object
    put on one of them floats 0.7 m above the track. Planes with many returns clearly below them are rejected, and a
    plane that jumps away from the previous frame's one is replaced by it."""
    m = (M[:, 0] > 3) & (M[:, 0] < 20) & (np.abs(M[:, 1]) < 2.5) & (M[:, 2] < -0.5)
    Q = M[m]
    if len(Q) > 5000:
        Q = Q[rng.choice(len(Q), 5000, replace=False)]
    best, best_n = None, -1
    for _ in range(400):
        s = Q[rng.choice(len(Q), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        L = np.linalg.norm(n)
        if L < 1e-9:
            continue
        n /= L
        if n[2] < 0:
            n = -n
        if n[2] < 0.96:
            continue
        dist = Q @ n - n @ s[0]
        if np.mean(dist < -0.25) > 0.10:
            continue  # not the lowest surface (a drainage channel between the rails is narrow)
        inl = np.abs(dist) < 0.04
        if inl.sum() > best_n:
            best_n, best = inl.sum(), inl
    if best is None:
        return previous
    R = Q[best]
    c = R.mean(0)
    _, _, vt = np.linalg.svd(R - c, full_matrices=False)
    n = vt[2] if vt[2][2] > 0 else -vt[2]
    plane = (n, float(-n @ c))
    if previous is not None:
        jump = abs(plane[1] - previous[1]) > 0.1 or np.degrees(np.arccos(min(1.0, abs(plane[0] @ previous[0])))) > 0.5
        if jump:
            return previous
    return plane


def align_to_plane(n):
    z = n / np.linalg.norm(n)
    x = np.array([1.0, 0.0, 0.0]) - z[0] * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z])  # rows: mount -> track


def beam_directions(g):
    """Unit ray directions (cloud frame) for every beam of the organized grid (cols x 128), incl. empty returns."""
    xyz = np.stack([g["x"], g["y"], g["z"]], -1).astype(np.float64)
    r = np.linalg.norm(xyz, axis=-1)
    valid = r > 0.1
    el = np.full(r.shape, np.nan)
    az = np.full(r.shape, np.nan)
    el[valid] = np.arcsin(xyz[valid, 2] / r[valid])
    az[valid] = np.arctan2(xyz[valid, 1], xyz[valid, 0])
    ring_el = np.nanmedian(el, axis=0)  # elevation is fixed per ring
    # azimuth is linear in the column pair index within a ring; every ring has its own offset (Hesai channel offsets)
    ncol = g.shape[0]
    pair = (np.arange(ncol) // 2).astype(np.float64)
    az_f = az.copy()
    slopes = []
    fits = []
    for k in range(g.shape[1]):
        v = valid[:, k]
        if v.sum() < 20:
            fits.append(None)
            continue
        a_k = np.unwrap(az[v, k])
        p_k = pair[v]
        slope, icpt = np.polyfit(p_k, a_k, 1)
        slopes.append(slope)
        fits.append((slope, icpt))
    slope_all = np.median(slopes)
    for k in range(g.shape[1]):
        v = valid[:, k]
        if fits[k] is None:
            continue
        icpt = np.median(np.unwrap(az[v, k]) - slope_all * pair[v])
        az_f[~v, k] = icpt + slope_all * pair[~v]
    el_f = np.where(valid, el, ring_el[None, :])
    az_f = np.where(np.isnan(az_f), 0.0, az_f)
    return np.stack([np.cos(el_f) * np.cos(az_f), np.cos(el_f) * np.sin(az_f), np.sin(el_f)], -1), r, valid


def intersect(kind, size, centre, o, d):
    """Ray-object intersection in the track frame. o: (3,), d: (N,3). Returns t (N,), inf where missed."""
    t = np.full(len(d), np.inf)
    if kind == "cylinder":
        rad, h = size
        ox, oy = o[0] - centre[0], o[1] - centre[1]
        a = d[:, 0] ** 2 + d[:, 1] ** 2
        b = 2 * (ox * d[:, 0] + oy * d[:, 1])
        c = ox * ox + oy * oy - rad * rad
        disc = b * b - 4 * a * c
        hit = (disc >= 0) & (a > 1e-12)
        sq = np.sqrt(np.where(hit, disc, 0.0))
        t1 = (-b - sq) / np.where(hit, 2 * a, 1.0)
        z = o[2] + t1 * d[:, 2]
        ok = hit & (t1 > 0) & (z >= centre[2]) & (z <= centre[2] + h)
        t[ok] = t1[ok]
        return t
    L, W, H = size
    lo = np.array([centre[0] - L / 2, centre[1] - W / 2, centre[2]])
    hi = np.array([centre[0] + L / 2, centre[1] + W / 2, centre[2] + H])
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (lo[None, :] - o[None, :]) / d
        t2 = (hi[None, :] - o[None, :]) / d
    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
    ok = (tmax >= tmin) & (tmin > 0)
    t[ok] = tmin[ok]
    return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--object", default="person", choices=sorted(OBJECTS))
    ap.add_argument("--distance", type=float, default=100.0, help="along the track at the first injected frame [m]")
    ap.add_argument("--lateral", type=float, default=0.0, help="left of the lidar forward line [m]")
    ap.add_argument("--speed", type=float, default=0.0, help="approach speed [m/s]")
    ap.add_argument("--frames", default="0:150", help="start:end frame indices to write")
    ap.add_argument("--forward-axis", default="-y")
    ap.add_argument("--reflect-range", type=float, default=230.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--axis-json", default="",
                    help='reference axis {"x": [...], "frames": {"<source frame>": {"lat": [...], "bed": [...]}}}')
    args = ap.parse_args()

    kind, size, intensity = OBJECTS[args.object]
    f0, f1 = (int(v) for v in args.frames.split(":"))
    rng = np.random.default_rng(args.seed)
    ts = get_typestore(Stores.ROS2_HUMBLE)
    R_mount = mount_rotation(args.forward_axis)
    out = Path(args.out)
    axis = json.loads(Path(args.axis_json).read_text()) if args.axis_json else None
    gt = {"object": args.object, "kind": kind, "size": size, "lateral": args.lateral, "speed": args.speed,
          "source_bag": str(Path(args.bag).name), "axis_reference": bool(axis), "frames": []}

    with Reader(args.bag) as reader:
        conn = next(c for c in reader.connections if c.msgtype == "sensor_msgs/msg/PointCloud2")
        if out.exists():
            raise SystemExit(f"{out} exists")
        with Writer(out, version=5) as writer:
            wconn = writer.add_connection(conn.topic, conn.msgtype, typestore=ts, serialization_format="cdr",
                                          offered_qos_profiles=conn.ext.offered_qos_profiles)
            t_first = None
            plane = None
            for idx, (c, bag_ns, raw) in enumerate(reader.messages(connections=[conn])):
                if idx < f0:
                    continue
                if idx >= f1:
                    break
                msg = ts.deserialize_cdr(raw, conn.msgtype)
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                t_first = stamp if t_first is None else t_first
                a = np.frombuffer(bytes(msg.data), dtype=PT).copy()
                g = a.reshape(-1, RINGS)
                dirs, rng_rec, valid = beam_directions(g)

                pts = np.stack([g["x"][valid], g["y"][valid], g["z"][valid]], -1).astype(np.float64)
                plane = bed_plane(pts @ R_mount.T, rng, plane)
                n, h = plane
                R_track = align_to_plane(n) @ R_mount  # cloud -> track rotation
                distance = args.distance - args.speed * (stamp - t_first)
                centre = np.array([distance, args.lateral, 0.0])
                if axis is not None:
                    ref = axis["frames"][str(idx)]
                    centre[1] += float(np.interp(distance, axis["x"], ref["lat"]))
                    centre[2] = float(np.interp(distance, axis["x"], ref["bed"]))
                if kind == "box":
                    centre[0] += size[0] / 2  # distance is the near face
                centre[2] += HANGING.get(args.object, 0.0)  # cables hang above the bed

                o = np.array([0.0, 0.0, h])
                D = dirs.reshape(-1, 3) @ R_track.T
                t = intersect(kind, size, centre, o, D).reshape(g.shape)
                # dual return: columns 2k and 2k+1 share the beam, so they share one return decision
                keep_prob = 0.9 * np.clip(1.0 - (t / args.reflect_range) ** 3, 0.0, 1.0)
                ncol = g.shape[0]
                pair_draw = np.repeat(rng.random((ncol + 1) // 2), 2)[:ncol][:, None]
                hit = np.isfinite(t) & ((~valid) | (t < rng_rec - 0.05)) & (pair_draw < keep_prob)
                tt = np.where(hit, t + rng.normal(0.0, 0.02, g.shape), 0.0)
                for k, comp in enumerate(("x", "y", "z")):
                    g[comp][hit] = (tt * dirs[..., k])[hit].astype(np.float32)
                g["intensity"][hit] = intensity
                new_msg = replace(msg, data=np.frombuffer(a.tobytes(), dtype=np.uint8))
                writer.write(wconn, bag_ns, ts.serialize_cdr(new_msg, conn.msgtype))
                gt["frames"].append({"frame": idx - f0, "source_frame": idx, "stamp": stamp, "bag_ns": int(bag_ns),
                                     "distance": float(distance), "y": float(centre[1]), "z": float(centre[2]),
                                     "injected_points": int(hit.sum())})
    (out / "ground_truth.json").write_text(json.dumps(gt, indent=1))
    pts = [f["injected_points"] for f in gt["frames"]]
    print(f"{out}: {len(pts)} frames, injected points per frame median={np.median(pts):.0f} "
          f"(first {pts[0]}, last {pts[-1]}), distance {gt['frames'][0]['distance']:.1f} -> {gt['frames'][-1]['distance']:.1f} m")


if __name__ == "__main__":
    main()
