"""Prototype: curved corridor by clearance-maximizing curvature search on a BEV occupancy grid.

Invariant used: the train's clearance envelope passes through the tunnel without touching the walls.
Among candidate centerlines (initial heading + piecewise-constant curvature per range segment) pick the one
whose envelope contains the fewest occupied BEV cells (long structures only), with smoothness priors.
"""
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage

from proto_common import Bag, load_frame, ransac_ground, to_track, track_transform

OUT = Path(__file__).parent / "plots"
OUT.mkdir(exist_ok=True)

X_MAX, X_RES = 200.0, 0.5
Y_HALF, Y_RES = 20.0, 0.1
NX, NY = int(X_MAX / X_RES), int(2 * Y_HALF / Y_RES)
XS = (np.arange(NX) + 0.5) * X_RES
SEG_EDGES = [0, 30, 60, 100, 150, 200]
HALF_W = 1.45         # clearance half-width for the curvature search [m]
Z_BAND = (0.9, 3.0)   # height band above the track bed used for the search (above contact rail) [m]
# danger-zone cross-section (stepped): (z_lo, z_hi, half_width) above the track bed
PROFILE = [(0.30, 0.90, 1.15), (0.90, 3.40, 1.50)]
KAPPA = np.concatenate([np.linspace(-1 / 80, 1 / 80, 101)])  # R >= 80 m
THETA0 = np.radians(np.linspace(-2.0, 2.0, 41))
BEAM = 6
LAMBDA_SMOOTH = 3.0e5  # penalty weight on curvature jumps between segments (units: cells per (1/m)^2 * m^2)


def occupancy(Q):
    m = (Q[:, 0] > 1.0) & (Q[:, 0] < X_MAX) & (np.abs(Q[:, 1]) < Y_HALF) & (Q[:, 2] > Z_BAND[0]) & (Q[:, 2] < Z_BAND[1])
    ix = (Q[m, 0] / X_RES).astype(int)
    iy = ((Q[m, 1] + Y_HALF) / Y_RES).astype(int)
    grid = np.zeros((NX, NY), np.int32)
    np.add.at(grid, (ix, iy), 1)
    occ = grid > 0
    # keep only long structures: components (after slight dilation) spanning > 4 m along X
    dil = ndimage.binary_dilation(occ, structure=np.ones((5, 3), bool))
    lab, n = ndimage.label(dil)
    if n:
        sl = ndimage.find_objects(lab)
        ext = np.array([(s[0].stop - s[0].start) * X_RES for s in sl])
        keep = np.zeros(n + 1, bool)
        keep[1:] = ext > 4.0
        long_occ = occ & keep[lab]
    else:
        long_occ = occ
    return occ, long_occ


def prefix(occ):
    return np.concatenate([np.zeros((NX, 1), np.int32), np.cumsum(occ, axis=1, dtype=np.int32)], axis=1)


def cells_in(cs, yc, w, x0, x1):
    lo = np.clip(np.round((yc - w + Y_HALF) / Y_RES).astype(int), 0, NY)
    hi = np.clip(np.round((yc + w + Y_HALF) / Y_RES).astype(int), 0, NY)
    ii = np.arange(x0, x1)
    return (cs[ii, hi] - cs[ii, lo]).sum(axis=-1)


def search_centerline(cs, y0=0.0, prev=None):
    """Beam search over (theta0, kappa per segment). Returns (theta0, kappas, yc[NX], total_cost)."""
    beams = [(0.0, THETA0[k], [], y0, THETA0[k]) for k in range(len(THETA0))]  # (cost, th0, kappas, y_end, th_end)
    for si in range(len(SEG_EDGES) - 1):
        i0, i1 = int(SEG_EDGES[si] / X_RES), int(SEG_EDGES[si + 1] / X_RES)
        cand = []
        dx = (np.arange(i0, i1) - i0 + 0.5) * X_RES
        for cost, th0, ks, ye, the in beams:
            # vectorized over kappa candidates: yc[k, j]
            yc = ye + the * dx[None, :] + 0.5 * KAPPA[:, None] * dx[None, :] ** 2
            c = cells_in(cs, yc, HALF_W, i0, i1).astype(float)
            kprev = ks[-1] if ks else 0.0
            L = (i1 - i0) * X_RES
            c += LAMBDA_SMOOTH * ((KAPPA - kprev) * L) ** 2 / L  # jump penalty ~ lateral deviation it causes
            if prev is not None:
                c += 0.3 * LAMBDA_SMOOTH * ((KAPPA - prev[1][si]) * L) ** 2 / L
            if si == 0:
                c += 2.0e3 * (th0 - (prev[0] if prev is not None else 0.0)) ** 2 * 1e2
            Lseg = L
            for k in np.argsort(c)[:BEAM]:
                y_end = ye + the * Lseg + 0.5 * KAPPA[k] * Lseg ** 2
                th_end = the + KAPPA[k] * Lseg
                cand.append((cost + c[k], th0, ks + [KAPPA[k]], y_end, th_end))
        cand.sort(key=lambda t: t[0])
        beams = cand[:BEAM * 3]
    best = beams[0]
    th0, ks = best[1], best[2]
    kap = np.zeros(NX)
    for si in range(len(SEG_EDGES) - 1):
        kap[int(SEG_EDGES[si] / X_RES):int(SEG_EDGES[si + 1] / X_RES)] = ks[si]
    theta = th0 + np.cumsum(kap) * X_RES
    yc = y0 + np.cumsum(theta) * X_RES
    return th0, ks, yc, best[0]


def run(bag_name, frames, tag):
    bag = Bag(bag_name)
    prev = None
    fig, axs2 = plt.subplots(len(frames), 2, figsize=(26, 3.3 * len(frames)), gridspec_kw={"width_ratios": [3, 2]})
    axs2 = np.atleast_2d(axs2)
    for (ax, axs_side), fi in zip(axs2, frames):
        P, it, stamp, _, _ = load_frame(bag, fi)
        t0 = time.perf_counter()
        n, d = ransac_ground(P)
        Rm, t = track_transform(n, d)
        Q = to_track(P, Rm, t)
        t1 = time.perf_counter()
        occ, long_occ = occupancy(Q)
        cs = prefix(long_occ.astype(np.int32))
        th0, ks, yc, cost = search_centerline(cs, prev=None)
        t2 = time.perf_counter()
        prev = (th0, ks)
        # points inside the stepped danger-zone profile around the found centerline
        yci = np.interp(Q[:, 0], XS, yc)
        lat = np.abs(Q[:, 1] - yci)
        inside = np.zeros(len(Q), bool)
        for zlo, zhi, hw in PROFILE:
            inside |= (Q[:, 2] > zlo) & (Q[:, 2] <= zhi) & (lat < hw)
        inside &= (Q[:, 0] > 2) & (Q[:, 0] < X_MAX)
        s = (Q[:, 0] > 0) & (Q[:, 0] < 160) & (np.abs(Q[:, 1]) < 14) & (Q[:, 2] > 0.3) & (Q[:, 2] < 3.3)
        ax.scatter(Q[s, 0], Q[s, 1], s=0.15, c="0.6")
        ax.scatter(Q[inside, 0], Q[inside, 1], s=4, c=Q[inside, 2], cmap="autumn", vmin=0.3, vmax=3.4)
        band = (lat < 2.0) & (Q[:, 0] > 0) & (Q[:, 0] < 160)
        axs_side.scatter(Q[band, 0], Q[band, 2], s=0.15, c="0.6")
        axs_side.scatter(Q[inside, 0], Q[inside, 2], s=4, c="red")
        axs_side.axhline(0.3, color="b", lw=0.6); axs_side.axhline(3.4, color="b", lw=0.6)
        axs_side.set_xlim(0, 160); axs_side.set_ylim(-3, 6); axs_side.set_title("side view, |lat|<2 m", fontsize=9)
        ax.plot(XS, yc, "b-", lw=1)
        ax.plot(XS, yc + 1.5, "b--", lw=0.8)
        ax.plot(XS, yc - 1.5, "b--", lw=0.8)
        ax.set_xlim(0, 160); ax.set_ylim(-14, 14)
        ax.set_title(f"{bag_name} f{fi}: h={d:.2f} th0={np.degrees(th0):+.2f}° kappa(1/m)={[f'{k * 1e3:+.1f}e-3' for k in ks]} "
                     f"cost={cost:.0f} inside_pts={inside.sum()}  calib {1e3 * (t1 - t0):.0f}ms search {1e3 * (t2 - t1):.0f}ms", fontsize=9)
        print(f"{bag_name} f{fi}: h={d:.2f} th0={np.degrees(th0):+.2f} R per seg={[int(1 / k) if abs(k) > 1e-6 else 'inf' for k in ks]} "
              f"inside={inside.sum()} search={1e3 * (t2 - t1):.0f}ms")
    fig.tight_layout()
    fig.savefig(OUT / f"corridor_{tag}.png", dpi=55)
    plt.close(fig)


if __name__ == "__main__":
    sets = {
        "curveA": ("roundT_squareT_pressureGate_squareT", [300, 326, 380, 435, 490, 544]),
        "scurve": ("roundT_pressureGate_roundT", [0, 53, 106, 160, 213, 267]),
        "platform": ("doubleT_platform", [0, 68, 137, 206, 275, 344]),
        "transition": ("roundT_doubleT", [0, 50, 100, 150, 200, 251]),
        "switch": ("squareT_platform_squareT_switch", [0, 175, 350, 525, 700, 876]),
        "obstacle": ("doubleT_obstacle", [0, 60, 120, 160, 200]),
    }
    which = sys.argv[1:] or list(sets)
    for tag in which:
        run(*sets[tag], tag)
