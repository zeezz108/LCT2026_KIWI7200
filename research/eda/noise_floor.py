"""Noise floor of naive change detection on an EMPTY static scene (squareT_platform_squareT_switch, stationary frames)."""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from bagio import Bag

bag = Bag("squareT_platform_squareT_switch")
idx = list(range(380, 640, 2))
R, XYZ = [], []
for i in idx:
    a, _, _ = bag.points(i)
    g = a.reshape(2400, 128)
    R.append(np.sqrt(g["x"] ** 2 + g["y"] ** 2 + g["z"] ** 2))
    XYZ.append(np.stack([g["x"], -g["y"], g["z"]], -1))
R = np.array(R)
bg = np.nanmedian(np.where(R > 0, R, np.nan), axis=0)
bg_valid_share = np.mean(R > 0, axis=0)

stats = []
for k in range(len(idx)):
    r = R[k]
    thr = np.maximum(0.3, 0.03 * np.nan_to_num(bg))
    fg = (r > 0) & ((np.isfinite(bg) & (r < bg - thr)) | ~np.isfinite(bg))
    p = XYZ[k][fg].astype(np.float64)
    ncl, big = 0, 0
    if len(p):
        tree = cKDTree(p)
        pr = tree.query_pairs(0.35, output_type="ndarray")
        m = coo_matrix((np.ones(len(pr)), (pr[:, 0], pr[:, 1])), shape=(len(p), len(p)))
        nc, lab = connected_components(m, directed=False)
        sizes = np.bincount(lab)
        ncl, big = (sizes >= 8).sum(), sizes.max()
    # where are fg points: near/far and inside a straight corridor |x|<1.7, z in [-1.1, 2.4] (lidar frame, floor ~ -1.34)
    fwd = p[:, 1] if len(p) else np.array([])
    corr = (np.abs(p[:, 0]) < 1.7) & (p[:, 2] > -1.1) & (p[:, 2] < 2.4) if len(p) else np.array([], bool)
    stats.append((fg.sum(), ncl, big, corr.sum(), (fwd > 50).sum()))
stats = np.array(stats)
print("frames:", len(idx))
print(f"fg pixels/frame: median={np.median(stats[:, 0]):.0f} max={stats[:, 0].max()}")
print(f"clusters>=8 pts/frame: median={np.median(stats[:, 1]):.0f} max={stats[:, 1].max()} ; largest cluster median={np.median(stats[:, 2]):.0f} max={stats[:, 2].max()}")
print(f"fg points inside straight corridor: median={np.median(stats[:, 3]):.0f} max={stats[:, 3].max()} ; fg points beyond 50 m: median={np.median(stats[:, 4]):.0f}")
flick = (bg_valid_share > 0.05) & (bg_valid_share < 0.95)
print(f"flickering beams (valid in 5..95% of frames): {flick.sum()} of {flick.size} ({flick.mean() * 100:.1f}%)")
