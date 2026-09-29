"""Low-level inspection of every bag: sqlite schema, topics, timing, PointCloud2 layout of the first message."""
import sqlite3
from pathlib import Path

import numpy as np
from rosbags.typesys import Stores, get_typestore

ROOT = Path(r"D:\Python Projects\ЛЦТ 2026\data\for_hackathon")
ts = get_typestore(Stores.ROS2_HUMBLE)
DT = {1: "INT8", 2: "UINT8", 3: "INT16", 4: "UINT16", 5: "INT32", 6: "UINT32", 7: "FLOAT32", 8: "FLOAT64"}

for bag in sorted(ROOT.iterdir()):
    db = next(bag.glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    print("=" * 100)
    print(bag.name)
    tables = [r[0] for r in con.execute("select name from sqlite_master where type='table'")]
    print("tables:", tables)
    for row in con.execute("select id, name, type, serialization_format from topics"):
        print("topic:", row)
    for tid, n, tmin, tmax in con.execute(
        "select topic_id, count(*), min(timestamp), max(timestamp) from messages group by topic_id"
    ):
        print(f"topic_id={tid} n={n} span={(tmax - tmin) / 1e9:.2f}s")
    dup = con.execute("select count(*) - count(distinct timestamp) from messages").fetchone()[0]
    print("duplicate receive timestamps:", dup)

    stamps = np.array([r[0] for r in con.execute("select timestamp from messages order by timestamp")], dtype=np.int64)
    d = np.diff(stamps) / 1e6
    print(f"recv dt ms: mean={d.mean():.1f} std={d.std():.1f} min={d.min():.1f} max={d.max():.1f}  >150ms: {(d > 150).sum()}")

    ttype = dict(con.execute("select id, type from topics").fetchall())
    tid, t, data = con.execute("select topic_id, timestamp, data from messages order by timestamp limit 1").fetchone()
    msg = ts.deserialize_cdr(data, ttype[tid])
    h = msg.header
    print(f"frame_id='{h.frame_id}' header_stamp={h.stamp.sec}.{h.stamp.nanosec:09d} recv={t / 1e9:.9f} "
          f"recv-header={(t - (h.stamp.sec * 10**9 + h.stamp.nanosec)) / 1e6:.1f} ms")
    print(f"height={msg.height} width={msg.width} points={msg.height * msg.width} point_step={msg.point_step} "
          f"row_step={msg.row_step} is_dense={msg.is_dense} bigendian={msg.is_bigendian} data_bytes={len(msg.data)}")
    for f in msg.fields:
        print(f"   field {f.name:>12s} off={f.offset:3d} {DT[f.datatype]:>7s} count={f.count}")

    # header stamps of all messages (cheap: deserialize only header by full deserialize of a few)
    con.close()
