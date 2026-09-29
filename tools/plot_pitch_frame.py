"""Clean bird's-eye figure of one processed frame for slides and docs: tunnel points, rail detections, track axis,
clearance envelope (shrinking with distance), warning band and detected objects.

usage: python tools/plot_pitch_frame.py <bag_name> <debug_json> <out_png> [x_max] [y_half]
The debug JSON comes from scripts/debug_frames.sh (offline_runner debug_frames).
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

HALF_WIDTH, WARN, SIGMA0, SIGMA_PER_M, WARN_MAX = 1.35, 1.0, 0.05, 0.004, 60.0


def main():
    bag_name, dbg_path, out = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3])
    x_max = float(sys.argv[4]) if len(sys.argv) > 4 else 160.0
    y_half = float(sys.argv[5]) if len(sys.argv) > 5 else 8.0
    d = json.loads(dbg_path.read_text())
    a, _, _ = Bag(bag_name).points(d["frame"])
    v = a[(a["x"] != 0) | (a["y"] != 0)]
    P = np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64)
    R = np.array(d["track_to_cloud"]).reshape(3, 3)
    Q = P @ R
    Q[:, 2] += d["height"]

    xs = (np.arange(len(d["lateral"])) + 0.5) * d["x_res"]
    lat = np.array(d["lateral"])
    spread = np.array(d["spread"])
    shrink = np.maximum(SIGMA0 + SIGMA_PER_M * xs, spread)
    sel = xs <= min(x_max, d["trusted"])

    # everything relative to the found track axis: the corridor becomes straight, walls keep their offsets
    rel = lambda x, y: y - np.interp(x, xs, lat)  # noqa: E731
    fig, ax = plt.subplots(figsize=(16, 4.4))
    m = (Q[:, 0] > 0) & (Q[:, 0] < x_max) & (Q[:, 2] > 0.3) & (Q[:, 2] < 3.2)
    ax.scatter(Q[m, 0], rel(Q[m, 0], Q[m, 1]), s=0.5, c="#9aa3ad", linewidths=0, label="точки лидара 0.3–3.2 м")
    warn = sel & (xs < WARN_MAX)
    ax.fill_between(xs[warn], HALF_WIDTH - shrink[warn], HALF_WIDTH + WARN - shrink[warn],
                    color="#F49D37", alpha=0.22, linewidth=0, label="полоса ВНИМАНИЯ (до 60 м)")
    ax.fill_between(xs[warn], -HALF_WIDTH - WARN + shrink[warn], -HALF_WIDTH + shrink[warn],
                    color="#F49D37", alpha=0.22, linewidth=0)
    ax.fill_between(xs[sel], -HALF_WIDTH + shrink[sel], HALF_WIDTH - shrink[sel],
                    color="#D7263D", alpha=0.16, linewidth=0, label="габарит поезда, сужение на неуверенность оси")
    ax.plot(xs[sel], np.zeros(sel.sum()), color="#1B2A41", lw=1.4, label="ось пути")
    if d["axis_detections"]:
        ad = np.array(d["axis_detections"])
        ax.scatter(ad[:, 0], rel(ad[:, 0], ad[:, 1]), c="#2E8B57", marker="x", s=18, linewidths=1.2,
                   label="центр пути по рельсам")
    shown = set()
    for cl in d["clusters"]:
        if cl["rejected"]:
            continue
        pts = np.array(cl["points"])
        danger = cl["level"] == 2
        label = "препятствие: ОПАСНОСТЬ" if danger else "объект: ВНИМАНИЕ"
        py = rel(pts[:, 0], pts[:, 1])
        ax.scatter(pts[:, 0], py, s=28, c="#D7263D" if danger else "#F49D37", edgecolors="k", linewidths=0.4,
                   label=None if label in shown else label, zorder=5)
        shown.add(label)
        ax.annotate(f"{cl['x']:.0f} м", (pts[:, 0].min(), py.max()), textcoords="offset points", xytext=(0, 8),
                    ha="center", fontsize=12, color="#D7263D" if danger else "#b86b00", weight="bold")
    ax.set_xlim(0, x_max)
    ax.set_ylim(-y_half, y_half)
    ax.set_xlabel("дистанция вдоль пути, м")
    ax.set_ylabel("от оси пути, м")
    ax.grid(alpha=0.25)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16), fontsize=10, ncol=3, frameon=False, markerscale=2)
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(out)


if __name__ == "__main__":
    main()
