"""Estimate lidar pose w.r.t. track-bed plane per bag (RANSAC plane on near floor points): height, pitch, roll."""
import numpy as np

from bagio import BAGS, Bag

rng = np.random.default_rng(0)


def ransac_plane(P, iters=400, thr=0.04):
    best, best_n = None, -1
    for _ in range(iters):
        s = P[rng.choice(len(P), 3, replace=False)]
        nrm = np.cross(s[1] - s[0], s[2] - s[0])
        L = np.linalg.norm(nrm)
        if L < 1e-6:
            continue
        nrm /= L
        if abs(nrm[2]) < 0.9:  # near-horizontal planes only
            continue
        d = -nrm @ s[0]
        inl = np.abs(P @ nrm + d) < thr
        c = inl.sum()
        if c > best_n:
            best_n, best = c, inl
    Q = P[best]
    c = Q.mean(0)
    _, _, vt = np.linalg.svd(Q - c, full_matrices=False)
    nrm = vt[2] if vt[2][2] > 0 else -vt[2]
    return nrm, -nrm @ c, best.mean()


for name in BAGS:
    bag = Bag(name)
    res = []
    for i in np.linspace(0, len(bag) - 1, 8).astype(int):
        a, _, _ = bag.points(i)
        v = a[(a["x"] != 0) | (a["y"] != 0)]
        P = np.stack([v["x"], -v["y"], v["z"]], 1).astype(np.float64)  # x right?, forward, up
        s = (P[:, 1] > 3) & (P[:, 1] < 15) & (np.abs(P[:, 0]) < 2.0) & (P[:, 2] < -0.8)
        Q = P[s]
        if len(Q) > 20000:
            Q = Q[rng.choice(len(Q), 20000, replace=False)]
        nrm, d, frac = ransac_plane(Q)
        h = d  # distance from origin to plane (nrm up): plane nrm.p + d = 0, origin above plane -> d>0
        pitch = np.degrees(np.arctan2(nrm[1], nrm[2]))  # plane normal tilted toward forward -> ground rises ahead
        roll = np.degrees(np.arctan2(nrm[0], nrm[2]))
        res.append((i, h, pitch, roll, frac))
    res = np.array(res)
    print(f"{name:38s} height={np.median(res[:, 1]):.2f}m (min {res[:, 1].min():.2f} max {res[:, 1].max():.2f})  "
          f"pitch={np.median(res[:, 2]):+.2f}° [{res[:, 2].min():+.2f}..{res[:, 2].max():+.2f}]  "
          f"roll={np.median(res[:, 3]):+.2f}° [{res[:, 3].min():+.2f}..{res[:, 3].max():+.2f}]  inliers={np.median(res[:, 4]):.2f}")
