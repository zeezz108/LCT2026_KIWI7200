"""Prototype v2 of the full per-frame pipeline (reference for the C++ port).

track frame: X forward along the train, Y left, Z up, Z=0 on the track-bed slab (RANSAC plane).
1. calibration      : RANSAC bed plane -> track frame
2. near-field axis  : drainage channel + rails template in each lidar ring (3..32 m) -> robust quadratic y(X)
3. far-field axis   : beam search of piecewise curvature on BEV occupancy of long structures, with clearance margin
4. vertical profile : beam search of piecewise grade on side-view occupancy inside the lateral corridor
5. danger zone      : stepped cross-section, shrinking with distance; candidates -> euclidean clusters
"""
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from proto_common import ransac_ground, to_track, track_transform

# ----------------------------------------------------------------------------- parameters


@dataclass
class Params:
    x_max: float = 200.0
    x_res: float = 0.5
    y_half: float = 20.0
    y_res: float = 0.1
    # near-field axis
    near_x: tuple = (3.0, 32.0)
    gauge_half: float = 0.795    # half distance between rail head centres (1520 mm gauge + head width)
    # far-field search
    seg_edges: tuple = (0.0, 10.0, 20.0, 35.0, 60.0, 100.0, 150.0, 200.0)
    w_channel: float = 40.0      # cost per metre of centreline deviation from a channel detection
    kappa_max: float = 1 / 250.0
    kappa_step: float = 1 / 10000.0
    beam: int = 8                # candidates kept per parent beam
    beam_diverse: int = 24       # beams kept per segment (one per heading/offset cell)
    w_smooth: float = 15.0       # cost per (metre of lateral deviation induced by a curvature jump over smooth_base)^2
    smooth_base: float = 40.0
    w_margin: float = 0.25       # weight of structure cells in the clearance margin band
    search_hw_up: float = 1.45   # search half-width, upper band
    search_hw_lo: float = 1.20   # search half-width, lower band
    margin: float = 0.55
    band_up: tuple = (0.9, 3.0)
    tall_margin: float = 0.3     # compact component spanning band_up within this margin is a structure (column)
    band_lo: tuple = (0.35, 0.9)
    # vertical
    grade_max: float = 0.04
    grade_step: float = 0.0025
    w_vsmooth: float = 40.0      # cost per (metre of vertical deviation induced by a grade jump over a segment)^2
    # danger zone
    zone_hw_up: float = 1.35
    zone_hw_lo: float = 1.10
    zone_top: float = 2.9
    zone_bottom: float = 0.30
    lat_sigma0: float = 0.05
    lat_sigma_per_m: float = 0.002
    bottom_per_m: float = 0.002
    # clustering
    eps_near: float = 0.35
    eps_far: float = 0.7
    min_pts: int = 3


@dataclass
class FrameResult:
    Q: np.ndarray
    calib: tuple
    near_axis: tuple               # (y0, th0, k0, n_det, ok)
    kappas: list
    grades: list
    yc: np.ndarray                 # centreline lateral offset per X cell
    zc: np.ndarray                 # bed height per X cell
    inside: np.ndarray             # bool per point
    clusters: list = field(default_factory=list)


# ----------------------------------------------------------------------------- 2. near-field axis


def near_axis(Q, ring, p: Params, prior=(0.0, 0.0, 0.0)):
    """Detect channel+rails lateral centre in each ring; robust quadratic fit y = y0 + th*X + k*X^2/2."""
    y0p, thp, kp = prior
    m = (Q[:, 0] > p.near_x[0]) & (Q[:, 0] < p.near_x[1]) & (Q[:, 2] > -0.7) & (Q[:, 2] < 0.6)
    yprior = y0p + thp * Q[:, 0] + 0.5 * kp * Q[:, 0] ** 2
    m &= np.abs(Q[:, 1] - yprior) < 1.8
    det = []
    for r in np.unique(ring[m]):
        s = m & (ring == r)
        if s.sum() < 25:
            continue
        X, Y, Z = Q[s, 0], Q[s, 1], Q[s, 2]
        xm = np.median(X)
        yp = y0p + thp * xm + 0.5 * kp * xm ** 2
        # lateral min-Z profile, 4 cm bins
        edges = np.arange(yp - 1.8, yp + 1.8 + 1e-6, 0.04)
        nb = len(edges) - 1
        idx = np.digitize(Y, edges) - 1
        ok = (idx >= 0) & (idx < nb)
        prof = np.full(nb, np.inf)
        pmax = np.full(nb, -np.inf)
        np.minimum.at(prof, idx[ok], Z[ok])
        np.maximum.at(pmax, idx[ok], Z[ok])
        prof[np.isinf(prof)] = np.nan
        pmax[np.isinf(pmax)] = np.nan
        centers = 0.5 * (edges[:-1] + edges[1:])
        best = None
        for ci in range(len(centers)):
            c = centers[ci]
            if abs(c - yp) > 1.0:
                continue
            # rails: two narrow bumps at +-gauge_half standing above the bed between them; channel is an optional bonus
            rl = pmax[(centers > c - p.gauge_half - 0.06) & (centers < c - p.gauge_half + 0.06)]
            rr = pmax[(centers > c + p.gauge_half - 0.06) & (centers < c + p.gauge_half + 0.06)]
            mid = prof[(centers > c - 0.55) & (centers < c + 0.55)]
            if np.isnan(rl).all() or np.isnan(rr).all() or np.isnan(mid).all():
                continue
            rlh, rrh = np.nanmax(rl), np.nanmax(rr)
            bed = np.nanpercentile(mid, 60)
            if rlh < 0.08 or rrh < 0.08 or min(rlh, rrh) - bed < 0.08:
                continue
            # outside neighbours of each rail must be lower than the rail head (rejects flat raised surfaces)
            ol = pmax[(centers > c - p.gauge_half - 0.30) & (centers < c - p.gauge_half - 0.12)]
            orr = pmax[(centers > c + p.gauge_half + 0.12) & (centers < c + p.gauge_half + 0.30)]
            if (not np.isnan(ol).all() and np.nanmax(ol) > rlh - 0.05) or (not np.isnan(orr).all() and np.nanmax(orr) > rrh - 0.05):
                continue
            ch = prof[(centers > c - 0.12) & (centers < c + 0.12)]
            chd = np.nanmedian(ch) if not np.isnan(ch).all() else 0.0
            sc = 0.5 * (rlh + rrh) - bed - 0.5 * abs(rlh - rrh) + (0.5 * max(0.0, -chd - 0.1))
            if best is None or sc > best[1]:
                best = (c, sc)
        if best is not None:
            det.append((xm, best[0], best[1]))
    det = np.array(det).reshape(-1, 3)
    if len(det) < 5:
        return (y0p, thp, kp, len(det), False), det
    # RANSAC quadratic, used only to initialise (y0, th0) of the joint search: a line fit would bias the heading
    # by kappa * mean(x) in curves. Curvature itself is left to the search.
    rng = np.random.default_rng(1)
    A = np.stack([np.ones(len(det)), det[:, 0], 0.5 * det[:, 0] ** 2], 1)
    best_inl, best_n = None, 0
    for _ in range(150):
        sel = rng.choice(len(det), 3, replace=False)
        if np.ptp(det[sel, 0]) < 4.0:
            continue
        try:
            coef = np.linalg.solve(A[sel], det[sel, 1])
        except np.linalg.LinAlgError:
            continue
        if abs(coef[2]) > 1.5 * p.kappa_max:
            continue
        inl = np.abs(A @ coef - det[:, 1]) < 0.10
        if inl.sum() > best_n:
            best_n, best_inl = inl.sum(), inl
    if best_n < 5:
        return (y0p, thp, kp, len(det), False), det
    # ridge on curvature keeps short noisy baselines from producing wild values
    Ai, bi = A[best_inl], det[best_inl, 1]
    reg = np.diag([0.0, 0.0, 400.0 ** 2 * 1e-2])
    coef = np.linalg.solve(Ai.T @ Ai + reg, Ai.T @ bi)
    return (coef[0], coef[1], coef[2], int(best_n), True), det[best_inl]


# ----------------------------------------------------------------------------- 3. far-field lateral search


def _grid_long(Q, band, p: Params, min_len=4.0, keep_tall=False):
    """Occupancy of structures: components long along X, or (keep_tall) compact but spanning the whole height band
    (columns, portal frames). People and objects are compact and shorter, so they never steer the corridor."""
    NX, NY = int(p.x_max / p.x_res), int(2 * p.y_half / p.y_res)
    m = (Q[:, 0] > 0.5) & (Q[:, 0] < p.x_max) & (np.abs(Q[:, 1]) < p.y_half) & (Q[:, 2] > band[0]) & (Q[:, 2] < band[1])
    occ = np.zeros((NX, NY), bool)
    ix = (Q[m, 0] / p.x_res).astype(int)
    iy = ((Q[m, 1] + p.y_half) / p.y_res).astype(int)
    occ[ix, iy] = True
    if keep_tall:
        zmax = np.full((NX, NY), -np.inf)
        zmin = np.full((NX, NY), np.inf)
        np.maximum.at(zmax, (ix, iy), Q[m, 2])
        np.minimum.at(zmin, (ix, iy), Q[m, 2])
    dil = ndimage.binary_dilation(occ, structure=np.ones((5, 3), bool))
    lab, n = ndimage.label(dil)
    if n:
        sl = ndimage.find_objects(lab)
        keep = np.zeros(n + 1, bool)
        keep[1:] = np.array([(s[0].stop - s[0].start) * p.x_res for s in sl]) > min_len
        if keep_tall:
            cell_lab = np.where(occ, lab, 0)
            top = ndimage.maximum(np.where(occ, zmax, -np.inf), labels=cell_lab, index=np.arange(1, n + 1))
            bot = ndimage.minimum(np.where(occ, zmin, np.inf), labels=cell_lab, index=np.arange(1, n + 1))
            keep[1:] |= (np.asarray(top) > band[1] - p.tall_margin) & (np.asarray(bot) < band[0] + p.tall_margin)
        occ = occ & keep[lab]
    cs = np.concatenate([np.zeros((NX, 1), np.int32), np.cumsum(occ, axis=1, dtype=np.int32)], axis=1)
    return occ, cs


def _count(cs, yc, lo_off, hi_off, i0, i1, p: Params):
    NY = cs.shape[1] - 1
    lo = np.clip(np.round((yc + lo_off + p.y_half) / p.y_res).astype(int), 0, NY)
    hi = np.clip(np.round((yc + hi_off + p.y_half) / p.y_res).astype(int), 0, NY)
    ii = np.arange(i0, i1)
    return (cs[ii, hi] - cs[ii, lo]).sum(axis=-1)


def lateral_search(Q, p: Params, det, y0_init, th0_init, have_line, prev_kappas=None):
    """Joint beam search from X=0: (y0, th0) around the channel line fit, piecewise curvature per segment.
    Near segments are driven by channel detections, far segments by wall/structure intrusions."""
    _, cs_up = _grid_long(Q, p.band_up, p, keep_tall=True)
    _, cs_lo = _grid_long(Q, p.band_lo, p)
    kgrid = np.arange(-p.kappa_max, p.kappa_max + 1e-12, p.kappa_step)
    if have_line:
        y_grid = y0_init + np.arange(-0.10, 0.101, 0.05)
        t_grid = th0_init + np.radians(np.arange(-0.6, 0.601, 0.15))
    else:
        y_grid = y0_init + np.arange(-0.3, 0.301, 0.1)
        t_grid = th0_init + np.radians(np.arange(-1.5, 1.501, 0.25))
    beams = [(0.0, [], y, t, None, y, t) for y in y_grid for t in t_grid]
    for si in range(len(p.seg_edges) - 1):
        x0, x1 = p.seg_edges[si], p.seg_edges[si + 1]
        i0, i1 = int(x0 / p.x_res), int(x1 / p.x_res)
        dx = (np.arange(i0, i1) + 0.5) * p.x_res - x0
        L = x1 - x0
        dsel = det[(det[:, 0] >= x0) & (det[:, 0] < x1)] if len(det) else np.zeros((0, 3))
        cand = []
        for cost, ks, ye, the, kprev, y0b, t0b in beams:
            yc = ye + the * dx[None, :] + 0.5 * kgrid[:, None] * dx[None, :] ** 2
            c = 1.0 * _count(cs_up, yc, -p.search_hw_up, p.search_hw_up, i0, i1, p)
            c = c + 1.0 * _count(cs_lo, yc, -p.search_hw_lo, p.search_hw_lo, i0, i1, p)
            c = c + p.w_margin * (_count(cs_up, yc, p.search_hw_up, p.search_hw_up + p.margin, i0, i1, p)
                                  + _count(cs_up, yc, -p.search_hw_up - p.margin, -p.search_hw_up, i0, i1, p))
            # curvature changes are judged by the lateral deviation they cause over a fixed base (their effect persists);
            # the first segment has no predecessor, so its curvature is free
            if kprev is not None:
                c = c + p.w_smooth * ((kgrid - kprev) * p.smooth_base ** 2 / 2) ** 2
            if len(dsel):
                ddx = dsel[:, 0] - x0
                r = dsel[None, :, 1] - (ye + the * ddx[None, :] + 0.5 * kgrid[:, None] * ddx[None, :] ** 2)
                c = c + p.w_channel * np.minimum(np.abs(r), 0.3).sum(axis=1)
            if prev_kappas is not None:
                c = c + 0.5 * p.w_smooth * ((kgrid - prev_kappas[si]) * p.smooth_base ** 2 / 2) ** 2
            for k in np.argsort(c)[:p.beam]:
                cand.append((cost + c[k], ks + [kgrid[k]], ye + the * L + 0.5 * kgrid[k] * L ** 2, the + kgrid[k] * L,
                             kgrid[k], y0b, t0b))
        # diverse beam: best candidate per (end heading, end offset) cell, so distinct geometric hypotheses survive
        # until far segments reveal the walls
        cand.sort(key=lambda t: t[0])
        seen, beams = set(), []
        for cnd in cand:
            key = (round(np.degrees(cnd[3]) / 0.25), round(cnd[2] / 0.25))
            if key in seen:
                continue
            seen.add(key)
            beams.append(cnd)
            if len(beams) >= p.beam_diverse:
                break
    b = beams[0]
    return b[5], b[6], b[1], b[0]


def centreline(p: Params, y0, th0, kappas):
    NX = int(p.x_max / p.x_res)
    xs = (np.arange(NX) + 0.5) * p.x_res
    edges = np.asarray(p.seg_edges)
    si = np.clip(np.searchsorted(edges, xs, side="right") - 1, 0, len(kappas) - 1)
    kap = np.asarray(kappas)[si]
    th = th0 + np.cumsum(kap) * p.x_res
    yc = y0 + np.cumsum(th) * p.x_res
    return xs, yc


# ----------------------------------------------------------------------------- 4. vertical profile


def vertical_search(Q, p: Params, xs, yc):
    NX, NZ = len(xs), 120
    z_lo = -4.0
    lat = Q[:, 1] - np.interp(Q[:, 0], xs, yc)
    m = (Q[:, 0] > 0.5) & (Q[:, 0] < p.x_max) & (np.abs(lat) < p.zone_hw_up) & (Q[:, 2] > z_lo) & (Q[:, 2] < z_lo + NZ * 0.1)
    occ = np.zeros((NX, NZ), bool)
    occ[(Q[m, 0] / p.x_res).astype(int), ((Q[m, 2] - z_lo) / 0.1).astype(int)] = True
    dil = ndimage.binary_dilation(occ, structure=np.ones((5, 3), bool))
    lab, n = ndimage.label(dil)
    if n:
        sl = ndimage.find_objects(lab)
        keep = np.zeros(n + 1, bool)
        keep[1:] = np.array([(s[0].stop - s[0].start) * p.x_res for s in sl]) > 4.0
        occ = occ & keep[lab]
    cs = np.concatenate([np.zeros((NX, 1), np.int32), np.cumsum(occ, axis=1, dtype=np.int32)], axis=1)

    def cnt(zb, a, b, i0, i1):
        lo = np.clip(np.round((zb + a - z_lo) / 0.1).astype(int), 0, NZ)
        hi = np.clip(np.round((zb + b - z_lo) / 0.1).astype(int), 0, NZ)
        ii = np.arange(i0, i1)
        return (cs[ii, hi] - cs[ii, lo]).sum(axis=-1)

    ggrid_all = np.arange(-p.grade_max, p.grade_max + 1e-12, p.grade_step)
    beams = [(0.0, [], 0.0, 0.0)]
    for si in range(len(p.seg_edges) - 1):
        x0, x1 = p.seg_edges[si], p.seg_edges[si + 1]
        i0, i1 = int(x0 / p.x_res), int(x1 / p.x_res)
        dx = (np.arange(i0, i1) + 0.5) * p.x_res - x0
        L = x1 - x0
        ggrid = ggrid_all if x0 >= 20.0 else np.array([0.0])  # near field is fixed by the bed-plane calibration
        cand = []
        for cost, gs, ze, gprev in beams:
            zb = ze + ggrid[:, None] * dx[None, :]
            c = cnt(zb, p.zone_bottom + 0.15, p.zone_top, i0, i1) + 0.3 * cnt(zb, p.zone_top, p.zone_top + 0.5, i0, i1)
            c = c + p.w_vsmooth * ((ggrid - gprev) * L) ** 2
            for k in np.argsort(c)[:p.beam]:
                cand.append((cost + c[k], gs + [ggrid[k]], ze + ggrid[k] * L, ggrid[k]))
        cand.sort(key=lambda t: t[0])
        beams = cand[:p.beam]
    gs = beams[0][1]
    zc = np.zeros(NX)
    z = 0.0
    for j in range(NX):
        x = xs[j]
        if x >= p.seg_edges[0]:
            si = min(np.searchsorted(p.seg_edges, x, side="right") - 1, len(gs) - 1)
            z += gs[si] * p.x_res
        zc[j] = z
    return gs, zc


# ----------------------------------------------------------------------------- 5. zone + clusters


def zone_mask(Q, p: Params, xs, yc, zc):
    X = Q[:, 0]
    lat = Q[:, 1] - np.interp(X, xs, yc)
    h = Q[:, 2] - np.interp(X, xs, zc)
    sig = p.lat_sigma0 + p.lat_sigma_per_m * X
    bottom = p.zone_bottom + p.bottom_per_m * X
    inside = (X > 1.5) & (X < p.x_max)
    inside &= ((h > bottom) & (h <= 0.9) & (np.abs(lat) < p.zone_hw_lo - sig)) | ((h > 0.9) & (h < p.zone_top) & (np.abs(lat) < p.zone_hw_up - sig))
    return inside, lat, h


def clusters(Q, idx, p: Params):
    if len(idx) == 0:
        return []
    P = Q[idx]
    out = []
    for far in (False, True):
        sel = (P[:, 0] > 60) if far else (P[:, 0] <= 60)
        if sel.sum() == 0:
            continue
        S = P[sel]
        tree = cKDTree(S)
        pr = tree.query_pairs(p.eps_far if far else p.eps_near, output_type="ndarray")
        mat = coo_matrix((np.ones(len(pr)), (pr[:, 0], pr[:, 1])), shape=(len(S), len(S)))
        nc, lab = connected_components(mat, directed=False)
        for k in range(nc):
            q = S[lab == k]
            if len(q) >= p.min_pts:
                out.append(dict(n=len(q), x=float(np.median(q[:, 0])), y=float(np.median(q[:, 1])), z=float(np.median(q[:, 2])),
                                dx=float(np.ptp(q[:, 0])), dy=float(np.ptp(q[:, 1])), dz=float(np.ptp(q[:, 2])), zmax=float(q[:, 2].max())))
    return sorted(out, key=lambda c: c["x"])


# ----------------------------------------------------------------------------- frame driver


def process(P, ring, p: Params, state=None):
    n, d = ransac_ground(P)
    Rm, t = track_transform(n, d)
    Q = to_track(P, Rm, t)
    prior = state["near"][:3] if state and state.get("near") else (0.0, 0.0, 0.0)
    near, det = near_axis(Q, ring, p, prior=prior if (state and state.get("near_ok")) else (prior[0], 0.0, 0.0))
    if not near[4] and state and state.get("near"):
        near = (*state["near"][:3], near[3], False)
    y0, th0, k0 = near[:3]
    y0, th0, kappas, _ = lateral_search(Q, p, det, y0, th0, near[4], prev_kappas=state.get("kappas") if state else None)
    near = (y0, th0, kappas[0], near[3], near[4])
    xs, yc = centreline(p, y0, th0, kappas)
    grades, zc = vertical_search(Q, p, xs, yc)
    inside, lat, h = zone_mask(Q, p, xs, yc, zc)
    cl = clusters(Q, np.nonzero(inside)[0], p)
    res = FrameResult(Q=Q, calib=(d, n), near_axis=near, kappas=kappas, grades=grades, yc=yc, zc=zc, inside=inside, clusters=cl)
    new_state = dict(near=near[:3], near_ok=near[4], kappas=kappas)
    return res, new_state, det
