"""Cut a short standalone bag out of a long recording, so it can be used as a base for synthetic obstacles.

The customer's second dataset is one 20-minute run split into 221 files (84 GB). Injecting an obstacle into it
directly would mean re-reading all of it per scenario, so the evaluation uses short segments cut out here.

usage:
  python tools/extract_segment.py --bag <bag dir> --out data/for_hackathon/new_curve --start 6600 --frames 60
"""
import argparse
from pathlib import Path

from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import Stores, get_typestore


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="source bag directory (may be split into many files)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--frames", type=int, default=60)
    args = ap.parse_args()

    ts = get_typestore(Stores.ROS2_HUMBLE)
    out = Path(args.out)
    if out.exists():
        raise SystemExit(f"{out} exists")
    written = 0
    with Reader(args.bag) as reader:
        conn = next(c for c in reader.connections if c.msgtype == "sensor_msgs/msg/PointCloud2")
        with Writer(out, version=5) as writer:
            wconn = writer.add_connection(conn.topic, conn.msgtype, typestore=ts, serialization_format="cdr",
                                          offered_qos_profiles=conn.ext.offered_qos_profiles)
            for idx, (_, stamp, raw) in enumerate(reader.messages(connections=[conn])):
                if idx < args.start:
                    continue
                if written >= args.frames:
                    break
                writer.write(wconn, stamp, raw)
                written += 1
    print(f"{out}: {written} frames from {args.start} of {args.bag}")


if __name__ == "__main__":
    main()
