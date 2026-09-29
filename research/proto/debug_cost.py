"""Compare cost components of the chosen centreline vs. alternative constant-curvature hypotheses."""
import sys

import numpy as np

import pipeline_v2 as pv
from proto_common import Bag, load_frame, ransac_ground, to_track, track_transform

name, fi = sys.argv[1], int(sys.argv[2])
p = pv.Params()
P, it, _, ring, _ = load_frame(Bag(name), fi)
n, d = ransac_ground(P)
Q = to_track(P, *track_transform(n, d))
near, det = pv.near_axis(Q, ring, p)
occ_up, cs_up = pv._grid_long(Q, p.band_up, p)
occ_lo, cs_lo = pv._grid_long(Q, p.band_lo, p)
raw_up, _ = (lambda: (None, None))()
print("near:", near, "long cells up/lo:", occ_up.sum(), occ_lo.sum())
y0s, th0s, ks, cost = pv.lateral_search(Q, p, det, near[0], near[1], near[4])
print("chosen:", round(y0s, 3), round(np.degrees(th0s), 2), [round(k * 1e3, 2) for k in ks], "cost", round(cost, 1))


def breakdown(y0, th0, kappas):
    rows = []
    ye, the, kprev = y0, th0, 0.0
    tot = 0
    for si in range(len(p.seg_edges) - 1):
        x0, x1 = p.seg_edges[si], p.seg_edges[si + 1]
        i0, i1 = int(x0 / p.x_res), int(x1 / p.x_res)
        dx = (np.arange(i0, i1) + 0.5) * p.x_res - x0
        L = x1 - x0
        k = kappas[si]
        yc = (ye + the * dx + 0.5 * k * dx ** 2)[None, :]
        up = pv._count(cs_up, yc, -p.search_hw_up, p.search_hw_up, i0, i1, p)[0]
        lo = pv._count(cs_lo, yc, -p.search_hw_lo, p.search_hw_lo, i0, i1, p)[0]
        mg = p.w_margin * (pv._count(cs_up, yc, p.search_hw_up, p.search_hw_up + p.margin, i0, i1, p)[0]
                           + pv._count(cs_up, yc, -p.search_hw_up - p.margin, -p.search_hw_up, i0, i1, p)[0])
        sm = p.w_smooth * ((k - kprev) * p.smooth_base ** 2 / 2) ** 2
        dsel = det[(det[:, 0] >= x0) & (det[:, 0] < x1)]
        ch = 0.0
        if len(dsel):
            ddx = dsel[:, 0] - x0
            ch = p.w_channel * np.minimum(np.abs(dsel[:, 1] - (ye + the * ddx + 0.5 * k * ddx ** 2)), 0.3).sum()
        rows.append((x0, x1, up, lo, round(mg, 1), round(sm, 1), round(ch, 1)))
        tot += up + lo + mg + sm + ch
        ye, the, kprev = ye + the * L + 0.5 * k * L ** 2, the + k * L, k
    return tot, rows


for label, (y0, th0, kap) in {
    "chosen": (y0s, th0s, ks),
    "straight from rails": (near[0], near[1], [0.0] * 7),
    "rails quadratic": (near[0], near[1], [near[2]] * 7),
}.items():
    tot, rows = breakdown(y0, th0, kap)
    print(f"--- {label}: total={tot:.1f}")
    for r in rows:
        print("   seg %5.0f-%5.0f  up=%4d lo=%4d margin=%6.1f smooth=%7.1f channel=%6.1f" % r)
