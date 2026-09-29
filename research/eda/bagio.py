"""Minimal fast reader for the hackathon bags (sqlite3 storage, single PointCloud2 topic)."""
import sqlite3
from pathlib import Path

import numpy as np
from rosbags.typesys import Stores, get_typestore

ROOT = Path(r"D:\Python Projects\ЛЦТ 2026\data\for_hackathon")
BAGS = sorted(p.name for p in ROOT.iterdir() if p.is_dir())
PT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("ring", "<u2"), ("timestamp", "<f8")])
_ts = get_typestore(Stores.ROS2_HUMBLE)


class Bag:
    def __init__(self, name):
        self.name = str(name)
        path = Path(name) if Path(name).is_dir() else ROOT / name
        # a recording may be split into several .db3 parts; frames are ordered by time across all of them
        self.cons = [sqlite3.connect(f"file:{db}?mode=ro", uri=True) for db in sorted(path.glob("*.db3"))]
        if not self.cons:
            raise FileNotFoundError(f"no .db3 in {path}")
        self.msgtype = self.cons[0].execute("select type from topics").fetchone()[0]
        rows = []
        for k, con in enumerate(self.cons):
            rows += [(t, k, i) for i, t in con.execute("select id, timestamp from messages")]
        self.rows = sorted(rows)

    def __len__(self):
        return len(self.rows)

    def recv_ns(self, i):
        return self.rows[i][0]

    def msg(self, i):
        stamp, k, row_id = self.rows[i]
        data = self.cons[k].execute("select data from messages where id=?", (row_id,)).fetchone()[0]
        return _ts.deserialize_cdr(data, self.msgtype)

    def points(self, i):
        m = self.msg(i)
        assert m.point_step == 26 and not m.is_bigendian
        arr = np.frombuffer(bytes(m.data), dtype=PT)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        return arr, stamp, m
