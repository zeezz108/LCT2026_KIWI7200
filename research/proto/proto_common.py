"""Shared helpers for algorithm prototypes: frame loading in a forward/left/up frame and track-bed calibration."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "eda"))
from bagio import BAGS, Bag  # noqa: E402,F401

_rng = np.random.default_rng(0)


def load_frame(bag, i, first_return_only=True):
    """Returns (P[N,3] in lidar frame rotated to X=forward, Y=left, Z=up; intensity[N]; stamp; ring[N]; col[N])."""
    a, stamp, _ = bag.points(i)
    g = a.reshape(-1, 128)
    ncol = g.shape[0]
    cols = np.repeat(np.arange(ncol)[:, None], 128, axis=1)
    if first_return_only:
        g, cols = g[0::2], cols[0::2]
    valid = (g["x"] != 0) | (g["y"] != 0) | (g["z"] != 0)
    v = g[valid]
    P = np.stack([-v["y"], v["x"], v["z"]], 1).astype(np.float64)
    return P, v["intensity"].astype(np.float32), stamp, v["ring"].astype(np.int32), cols[valid]


def ransac_ground(P, x_rng=(3.0, 20.0), y_half=2.5, iters=300, thr=0.04):
    """Dominant near-horizontal plane below the sensor in front of the vehicle. Returns (normal, d) with normal.z>0."""
    m = (P[:, 0] > x_rng[0]) & (P[:, 0] < x_rng[1]) & (np.abs(P[:, 1]) < y_half) & (P[:, 2] < -0.5)
    Q = P[m]
    if len(Q) > 20000:
        Q = Q[_rng.choice(len(Q), 20000, replace=False)]
    best, best_n = None, -1
    for _ in range(iters):
        s = Q[_rng.choice(len(Q), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        L = np.linalg.norm(n)
        if L < 1e-9:
            continue
        n /= L
        if abs(n[2]) < 0.95:
            continue
        inl = np.abs(Q @ n - n @ s[0]) < thr
        if inl.sum() > best_n:
            best_n, best = inl.sum(), inl
    R = Q[best]
    c = R.mean(0)
    _, _, vt = np.linalg.svd(R - c, full_matrices=False)
    n = vt[2] if vt[2][2] > 0 else -vt[2]
    return n, -n @ c


def track_transform(n, d):
    """Rotation+translation mapping lidar points to a frame whose XY plane is the track bed (z=0 on the slab)."""
    z = n
    x = np.array([1.0, 0.0, 0.0]) - n[0] * n  # keep forward direction, orthogonalize
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    Rm = np.stack([x, y, z])  # rows
    t = np.array([0.0, 0.0, d])  # signed distance of origin above plane
    return Rm, t


def to_track(P, Rm, t):
    return P @ Rm.T + t
