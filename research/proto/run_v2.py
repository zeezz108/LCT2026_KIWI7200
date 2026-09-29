"""Run prototype v2 over sampled frames of every bag: FP-proxy stats on empty bags, detections on the obstacle bag, plots."""
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from pipeline_v2 import Params, process
from proto_common import BAGS, Bag, load_frame

OUT = Path(__file__).parent / "plots"
OUT.mkdir(exist_ok=True)
p = Params()


def plot_frame(ax_bev, ax_side, name, fi, res, det):
    Q = res.Q
    xs = (np.arange(len(res.yc)) + 0.5) * p.x_res
    s = (Q[:, 0] > 0) & (Q[:, 0] < 160) & (np.abs(Q[:, 1]) < 14) & (Q[:, 2] > 0.3) & (Q[:, 2] < 3.5)
    ax_bev.scatter(Q[s, 0], Q[s, 1], s=0.12, c="0.65")
    ins = res.inside
    ax_bev.scatter(Q[ins, 0], Q[ins, 1], s=5, c="red")
    for off, ls in ((0, "-"), (p.zone_hw_up, "--"), (-p.zone_hw_up, "--")):
        ax_bev.plot(xs, res.yc + off, "b" + ls, lw=0.7)
    if len(det):
        ax_bev.scatter(det[:, 0], det[:, 1], s=10, c="lime", marker="x")
    ax_bev.set_xlim(0, 160); ax_bev.set_ylim(-14, 14)
    y0, th0, k0, nd, ok = res.near_axis
    ax_bev.set_title(f"{name} f{fi}: near y0={y0:+.2f} th={np.degrees(th0):+.2f}° k={k0 * 1e3:+.2f}e-3 n={nd} ok={ok} | "
                     f"far k={[f'{k * 1e3:+.1f}' for k in res.kappas]}e-3 | grades={[f'{g * 100:+.1f}' for g in res.grades]}% | "
                     f"inside={ins.sum()} clusters={len(res.clusters)}", fontsize=8)
    lat = Q[:, 1] - np.interp(Q[:, 0], xs, res.yc)
    b = (np.abs(lat) < 2.0) & (Q[:, 0] > 0) & (Q[:, 0] < 160)
    ax_side.scatter(Q[b, 0], Q[b, 2], s=0.12, c="0.65")
    ax_side.scatter(Q[ins, 0], Q[ins, 2], s=5, c="red")
    ax_side.plot(xs, res.zc, "b-", lw=0.7)
    ax_side.plot(xs, res.zc + p.zone_top, "b--", lw=0.7)
    ax_side.set_xlim(0, 160); ax_side.set_ylim(-4, 6)


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "stats"
    if mode == "stats":
        for name in BAGS:
            bag = Bag(name)
            step = 3 if name != "doubleT_obstacle" else 5
            state = None
            rows = []
            t_all = []
            for fi in range(0, len(bag), step):
                P, it, stamp, ring, col = load_frame(bag, fi)
                t0 = time.perf_counter()
                res, state, det = process(P, ring, p, state)
                t_all.append(time.perf_counter() - t0)
                big = [c for c in res.clusters if c["n"] >= 5]
                rows.append((fi, res.inside.sum(), len(res.clusters), len(big), res.near_axis[4],
                             min([c["x"] for c in big], default=np.nan)))
                if name == "doubleT_obstacle" and fi % 25 == 0:
                    print(f"   f{fi}: " + "; ".join(f"n={c['n']} x={c['x']:.1f} y={c['y']:+.2f} z={c['z']:.2f} dz={c['dz']:.2f}" for c in big[:5]))
            r = np.array(rows, dtype=float)
            print(f"{name:38s} frames={len(r)} inside_pts median={np.median(r[:, 1]):.0f} p90={np.percentile(r[:, 1], 90):.0f} | "
                  f"frames with cluster>=5pts: {np.mean(r[:, 3] > 0) * 100:.0f}% | near-axis ok: {np.mean(r[:, 4]) * 100:.0f}% | "
                  f"{np.mean(t_all) * 1e3:.0f} ms/frame")
    else:
        name = mode
        frames = [int(f) for f in sys.argv[2:]]
        fig, axs = plt.subplots(len(frames), 2, figsize=(26, 3.4 * len(frames)), gridspec_kw={"width_ratios": [3, 2]})
        axs = np.atleast_2d(axs)
        bag = Bag(name)
        state = None
        for (a1, a2), fi in zip(axs, frames):
            P, it, stamp, ring, col = load_frame(bag, fi)
            res, state, det = process(P, ring, p, None)
            plot_frame(a1, a2, name, fi, res, det)
            print(name, fi, "clusters:", [(c["n"], round(c["x"], 1), round(c["y"], 2), round(c["z"], 2)) for c in res.clusters if c["n"] >= 3][:8])
        fig.tight_layout()
        fig.savefig(OUT / f"v2_{name}.png", dpi=50)
