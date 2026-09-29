"""Overlay a C++ debug dump (offline_runner debug_frames) on the raw bag frame: BEV + side view with corridor,
zones, rail detections and clusters.

usage: python tools/plot_debug_frame.py <bag_name> <debug_json> [x_max]
"""
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "research" / "eda"))
from bagio import Bag  # noqa: E402

HW_UP, HW_LO, WARN = 1.35, 1.10, 1.0


def main():
    bag_name, dbg_path = sys.argv[1], Path(sys.argv[2])
    x_max = float(sys.argv[3]) if len(sys.argv) > 3 else 160.0
    d = json.loads(dbg_path.read_text())
    a, _, _ = Bag(bag_name).points(d["frame"])
    v = a[(a["x"] != 0) | (a["y"] != 0)]
    P = np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64)
    R = np.array(d["track_to_cloud"]).reshape(3, 3)  # track -> cloud rotation; cloud -> track is R^T
    Q = P @ R  # (R^T p) for row vectors
    Q[:, 2] += d["height"]

    xs = (np.arange(len(d["lateral"])) + 0.5) * d["x_res"]
    lat = np.array(d["lateral"])
    bed = np.array(d["bed"])
    spread = np.array(d["spread"])
    fig, axs = plt.subplots(2, 1, figsize=(24, 11))
    ax = axs[0]
    m = (Q[:, 0] > 0) & (Q[:, 0] < x_max) & (np.abs(Q[:, 1]) < 14) & (Q[:, 2] > 0.3) & (Q[:, 2] < 3.5)
    ax.scatter(Q[m, 0], Q[m, 1], s=0.3, c=Q[m, 2], cmap="Greys", vmin=-1, vmax=4)
    ax.plot(xs, lat, "b-", lw=1)
    for off, st in ((HW_UP, "r--"), (-HW_UP, "r--"), (HW_UP + WARN, "y--"), (-HW_UP - WARN, "y--")):
        ax.plot(xs, lat + off, st, lw=0.7)
    ax.fill_between(xs, lat - spread, lat + spread, color="c", alpha=0.25)
    if d["axis_detections"]:
        ad = np.array(d["axis_detections"])
        ax.scatter(ad[:, 0], ad[:, 1], c="lime", marker="x", s=15)
    for cl in d["clusters"]:
        pts = np.array(cl["points"])
        col = "0.4" if cl["rejected"] else ("red" if cl["level"] == 2 else "orange")
        ax.scatter(pts[:, 0], pts[:, 1], s=14, c=col, edgecolors="k", linewidths=0.2)
    ax.axvline(d["trusted"], color="m", lw=1)
    ax.set_xlim(0, x_max); ax.set_ylim(-14, 14); ax.grid(alpha=0.3)
    ax.set_title(f"{bag_name} frame {d['frame']}: BEV z 0.3..3.5 (grey), axis (blue), envelope (red), warning (yellow), "
                 f"spread (cyan), trusted={d['trusted']} (magenta), clusters red=danger orange=warning grey=rejected")
    ax = axs[1]
    yc = np.interp(Q[:, 0], xs, lat)
    b = (np.abs(Q[:, 1] - yc) < 2.5) & (Q[:, 0] > 0) & (Q[:, 0] < x_max)
    ax.scatter(Q[b, 0], Q[b, 2], s=0.3, c="0.5")
    ax.plot(xs, bed, "b-", lw=1)
    ax.plot(xs, bed + 0.3 + 0.002 * xs, "r--", lw=0.7)
    ax.plot(xs, bed + 2.9, "r--", lw=0.7)
    for cl in d["clusters"]:
        pts = np.array(cl["points"])
        col = "0.4" if cl["rejected"] else ("red" if cl["level"] == 2 else "orange")
        ax.scatter(pts[:, 0], pts[:, 2], s=14, c=col, edgecolors="k", linewidths=0.2)
    ax.set_xlim(0, x_max); ax.set_ylim(-4, 6); ax.grid(alpha=0.3)
    ax.set_title("side view |lat| < 2.5 m, bed profile (blue), envelope bottom/top (red)")
    fig.tight_layout()
    out = dbg_path.with_suffix(".png")
    fig.savefig(out, dpi=55)
    print(out)

    if "upper_cells" in d:
        plot_structure_grids(d, Q, xs, lat, x_max, dbg_path)


def plot_structure_grids(d, Q, xs, lat, x_max, dbg_path):
    """Structure cells of the corridor search (walls/columns that the axis must avoid) over the raw points."""
    fig, axs = plt.subplots(2, 1, figsize=(24, 11))
    halves = [(0.0, x_max / 2), (x_max / 2, x_max)]
    up = np.array(d["upper_cells"]).reshape(-1, 2)
    lo = np.array(d["lower_cells"]).reshape(-1, 2)
    for ax, (x0, x1) in zip(axs, halves):
        m = (Q[:, 0] > x0) & (Q[:, 0] < x1) & (Q[:, 2] > 0.3) & (Q[:, 2] < 3.5)
        ax.scatter(Q[m, 0], Q[m, 1], s=0.5, c="0.75")
        for cells, col, lab in ((up, "tab:red", "upper band structures"), (lo, "tab:blue", "lower band structures")):
            k = (cells[:, 0] > x0) & (cells[:, 0] < x1)
            ax.scatter(cells[k, 0], cells[k, 1], s=6, c=col, marker="s", label=lab)
        sel = (xs > x0) & (xs < x1)
        ax.plot(xs[sel], lat[sel], "k-", lw=1)
        for off in (1.45, -1.45):
            ax.plot(xs[sel], lat[sel] + off, "m--", lw=0.8)
        yc = lat[sel]
        ax.set_xlim(x0, x1)
        ax.set_ylim(yc.min() - 7, yc.max() + 7)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right")
    axs[0].set_title(f"frame {d['frame']}: search grids (structures), axis (black), search half-width 1.45 (magenta)")
    fig.tight_layout()
    out = dbg_path.with_name(dbg_path.stem + "_grid.png")
    fig.savefig(out, dpi=55)
    print(out)


if __name__ == "__main__":
    main()
