"""Transform a recording to test how far the detector generalizes beyond the six bags of the case.

The control recording will differ from what we have: another mounting, another sensor, another driver. Every
transform here keeps the scene and changes only how it was measured, so the expected answer stays the same:
the empty recordings must stay empty and the person in `doubleT_obstacle` must still be found.

  --height +0.4        lidar mounted higher (metres, positive = higher)
  --roll 3 --pitch 1   mounting angles [deg]
  --rings 64           every second channel: a 64-beam sensor
  --no-intensity       intensity field zeroed (a driver that does not publish it)
  --no-ring            ring field zeroed (the organized layout must be rediscovered)
  --unorganized        point order shuffled (no column/ring structure left at all)
  --drop-frames 0.2    drop this share of frames (packet loss)
  --range-noise 0.05   extra range noise [m]
  --decimate 2         keep every n-th point (a weaker sensor / cheaper driver)

usage:
  python tools/transform_bag.py --bag data/for_hackathon/<name> --out data/transformed/<name>_<what> [options]
"""
import argparse
from dataclasses import replace
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

PT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("ring", "<u2"),
               ("timestamp", "<f8")])


def rotation(roll_deg, pitch_deg, forward_axis="-y"):
    """Rotation applied to the points, expressed in the cloud frame (forward = -Y, left = +X, up = +Z)."""
    r, p = np.radians(roll_deg), np.radians(pitch_deg)
    # roll about the forward axis, pitch about the lateral axis
    fwd = {"x": [1, 0, 0], "-x": [-1, 0, 0], "y": [0, 1, 0], "-y": [0, -1, 0]}[forward_axis]
    fwd = np.array(fwd, float)
    up = np.array([0.0, 0.0, 1.0])
    left = np.cross(up, fwd)

    def axis_rotation(axis, angle):
        axis = axis / np.linalg.norm(axis)
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

    return axis_rotation(fwd, r) @ axis_rotation(left, p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--height", type=float, default=0.0)
    ap.add_argument("--roll", type=float, default=0.0)
    ap.add_argument("--pitch", type=float, default=0.0)
    ap.add_argument("--rings", type=int, default=0, help="keep this many channels (128 -> 64: every second)")
    ap.add_argument("--no-intensity", action="store_true")
    ap.add_argument("--no-ring", action="store_true")
    ap.add_argument("--unorganized", action="store_true")
    ap.add_argument("--drop-frames", type=float, default=0.0)
    ap.add_argument("--range-noise", type=float, default=0.0)
    ap.add_argument("--decimate", type=int, default=1)
    ap.add_argument("--frames", default="", help="start:end of source frames")
    ap.add_argument("--forward-axis", default="-y")
    ap.add_argument("--seed", type=int, default=11)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    ts = get_typestore(Stores.ROS2_HUMBLE)
    R = rotation(args.roll, args.pitch, args.forward_axis)
    f0, f1 = (int(v) for v in args.frames.split(":")) if args.frames else (0, 10**9)
    out = Path(args.out)
    if out.exists():
        raise SystemExit(f"{out} exists")

    kept = dropped = 0
    with Reader(args.bag) as reader:
        conn = next(c for c in reader.connections if c.msgtype == "sensor_msgs/msg/PointCloud2")
        with Writer(out, version=5) as writer:
            wconn = writer.add_connection(conn.topic, conn.msgtype, typestore=ts, serialization_format="cdr",
                                          offered_qos_profiles=conn.ext.offered_qos_profiles)
            for idx, (_, bag_ns, raw) in enumerate(reader.messages(connections=[conn])):
                if idx < f0:
                    continue
                if idx >= f1:
                    break
                if args.drop_frames > 0.0 and rng.random() < args.drop_frames:
                    dropped += 1
                    continue
                msg = ts.deserialize_cdr(raw, conn.msgtype)
                a = np.frombuffer(bytes(msg.data), dtype=PT).copy()
                valid = (a["x"] != 0) | (a["y"] != 0) | (a["z"] != 0)

                if args.rings:
                    step = max(1, 128 // args.rings)
                    keep = (a["ring"] % step) == 0
                    a = a[keep]
                    valid = valid[keep]
                if args.decimate > 1:
                    a = a[::args.decimate]
                    valid = valid[::args.decimate]

                P = np.stack([a["x"], a["y"], a["z"]], -1).astype(np.float64)
                if args.range_noise > 0.0:
                    r = np.linalg.norm(P, axis=1, keepdims=True)
                    scale = np.ones_like(r)
                    np.divide(1.0, r, out=scale, where=r > 1e-3)
                    P = P + P * scale * rng.normal(0.0, args.range_noise, size=(len(P), 1))
                if args.roll or args.pitch:
                    P = P @ R.T
                if args.height:
                    P[:, 2] -= args.height  # the sensor moves up, the world moves down in its frame
                P[~valid] = 0.0
                for k, comp in enumerate(("x", "y", "z")):
                    a[comp] = P[:, k].astype(np.float32)
                if args.no_intensity:
                    a["intensity"] = 0.0
                if args.no_ring:
                    a["ring"] = 0
                if args.unorganized:
                    a = a[rng.permutation(len(a))]

                new_msg = replace(msg, data=np.frombuffer(a.tobytes(), dtype=np.uint8),
                                  width=len(a), height=1, row_step=len(a) * msg.point_step)
                writer.write(wconn, bag_ns, ts.serialize_cdr(new_msg, conn.msgtype))
                kept += 1
    print(f"{out}: {kept} frames written, {dropped} dropped, "
          f"{'shuffled' if args.unorganized else 'organized'}, "
          f"height {args.height:+.2f} m, roll {args.roll:+.1f}, pitch {args.pitch:+.1f} deg")


if __name__ == "__main__":
    main()
