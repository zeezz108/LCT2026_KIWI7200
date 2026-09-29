"""Charts for docs/experiments.md from the offline results.

usage: python tools/plot_experiments.py [--synthetic data/results/synthetic/<tag>/summary.json]
                                       [--results data/results/bench/<tag>]
Writes docs/img/exp_recall.png, exp_approach.png, exp_obstacle_timeline.png, exp_timing.png.
"""
import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
IMG = ROOT / "docs" / "img"
RESULTS_DIR = ROOT / "data" / "results"
SYNTHETIC_DIR = RESULTS_DIR / "synthetic"
BAGS = ["doubleT_obstacle", "doubleT_platform", "roundT_doubleT", "roundT_pressureGate_roundT",
        "roundT_squareT_pressureGate_squareT", "squareT_platform_squareT_switch"]


def recall_chart(summary):
    fig, ax = plt.subplots(figsize=(9, 4.8))
    styles = {"person": ("человек 0.4×1.75 м", "tab:red"), "box_medium": ("ящик 0.6×0.6×1.0 м", "tab:blue"),
              "box_small": ("ящик 0.5×0.5×0.5 м", "tab:green")}
    for obj, (label, color) in styles.items():
        for base, ls, marker in (("round", "-", "o"), ("square", "--", "s")):
            rows = sorted((s for s in summary if s["object"] == obj and s["speed"] == 0 and s["lateral"] == 0
                           and s["name"].startswith(base)), key=lambda s: s["distance"])
            if not rows:
                continue
            d = [s["distance"] for s in rows]
            ax.plot(d, [100 * s["recall_danger"] for s in rows], ls, marker=marker, color=color,
                    label=f"{label}, {'круглый' if base == 'round' else 'прямоугольный'} тоннель")
    ax.set_xlabel("дистанция, м")
    ax.set_ylabel("кадров с ОПАСНОСТЬЮ, %")
    ax.set_ylim(-3, 103)
    ax.set_xlim(20, 205)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="lower left")
    ax.set_title("Полнота по дальности (синтетические препятствия в реальных записях)")
    fig.tight_layout()
    fig.savefig(IMG / "exp_recall.png", dpi=110)
    plt.close(fig)


def approach_chart(summary):
    rows = [s for s in summary if s["speed"] > 0]
    if not rows:
        return
    fig, ax = plt.subplots(figsize=(9, 1.0 + 0.5 * len(rows)))
    for i, s in enumerate(rows):
        res = SYNTHETIC_DIR / f"{s['name']}.jsonl"
        gt = ROOT / "data" / "synthetic" / s["name"] / "ground_truth.json"
        if not res.exists() or not gt.exists():
            continue
        truth = {f["frame"]: f["distance"] for f in json.loads(gt.read_text())["frames"]}
        frames = [json.loads(l) for l in res.read_text().splitlines() if l.strip()]
        d_true, lvl = [], []
        for r in frames:
            if r["frame"] not in truth:
                continue
            d = truth[r["frame"]]
            tol = max(2.0, 0.04 * d)
            match = [o["level"] for o in r["obstacles"] if abs(o["distance"] - d) <= tol]
            d_true.append(d)
            lvl.append(max(match) if match else 0)
        d_true, lvl = np.array(d_true), np.array(lvl)
        y = np.full(len(d_true), i)
        for level, color, label in ((0, "0.85", "не обнаружен"), (1, "orange", "ВНИМАНИЕ"), (2, "red", "ОПАСНОСТЬ")):
            m = lvl == level
            ax.scatter(d_true[m], y[m], c=color, s=40, marker="|", label=label if i == 0 else None)
    objects = {"person": "человек", "box_medium": "ящик 0.6×0.6×1.0 м", "box_small": "ящик 0.5 м",
               "box_large": "ящик 1.0 м", "lying_person": "лежащий человек", "min_object": "предмет 0.3×0.3×0.1 м"}
    bases = {"round": "круглый тоннель", "square": "прямоугольный", "curve": "кривая R ≈ 370 м",
             "station": "станция", "switch": "стрелка", "section": "смена сечения"}
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels(["%s, %s" % (objects.get(s["object"], s["object"]), bases.get(s["base"], s["base"]))
                        for s in rows])
    ax.set_ylim(-0.6, len(rows) - 0.4)
    ax.invert_xaxis()
    ax.set_xlabel("истинная дистанция до объекта, м (объект приближается со скоростью 15 м/с)")
    ax.grid(alpha=0.3, axis="x")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.45), fontsize=8, ncol=3, frameon=False)
    ax.set_title("Сближение с препятствием с 200 м")
    fig.savefig(IMG / "exp_approach.png", dpi=110, bbox_inches="tight")
    plt.close(fig)


def obstacle_timeline_chart():
    """Real people in doubleT_obstacle: lateral offset of confirmed objects over time, coloured by reported level."""
    path = RESULTS_DIR / "doubleT_obstacle.jsonl"
    if not path.exists():
        return
    rows = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
    t0 = rows[0]["stamp"]
    fig, axs = plt.subplots(2, 1, figsize=(10, 5.6), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    ax = axs[0]
    ax.axhspan(-1.35, 1.35, color="#D7263D", alpha=0.10, label="габарит поезда ±1.35 м")
    ax.axhspan(1.35, 2.35, color="#F49D37", alpha=0.12, label="полоса ВНИМАНИЯ")
    ax.axhspan(-2.35, -1.35, color="#F49D37", alpha=0.12)
    colors = {1: "#F49D37", 2: "#D7263D"}
    for level, label in ((2, "ОПАСНОСТЬ"), (1, "ВНИМАНИЕ")):
        pts = [(r["stamp"] - t0, o["lateral"], o["distance"]) for r in rows for o in r["obstacles"] if o["level"] == level]
        if pts:
            p = np.array(pts)
            ax.scatter(p[:, 0], p[:, 1], s=14, c=colors[level], label=f"объект: {label}")
    ax.set_ylabel("смещение от оси пути, м")
    ax.set_ylim(-3, 3)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="lower right", ncol=2)
    ax.set_title("doubleT_obstacle: человек A (~56 м) пересекает путь, человек B идёт в междупутье")
    ax = axs[1]
    for level in (1, 2):
        pts = [(r["stamp"] - t0, o["distance"]) for r in rows for o in r["obstacles"] if o["level"] == level]
        if pts:
            p = np.array(pts)
            ax.scatter(p[:, 0], p[:, 1], s=10, c=colors[level])
    ax.set_ylabel("дистанция, м")
    ax.set_xlabel("время, с")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(IMG / "exp_obstacle_timeline.png", dpi=110)
    plt.close(fig)


def visibility_chart():
    """How far the track is actually observed. The range criterion is about metres, but a tunnel curves away long
    before the sensor runs out: this is the distribution of the observed corridor length over every recorded frame."""
    names = {"doubleT_obstacle": "двухпутный, поезд стоит", "doubleT_platform": "станция, двухпутный",
             "roundT_doubleT": "круглый -> двухпутный", "roundT_pressureGate_roundT": "круглый, гермозатвор",
             "roundT_squareT_pressureGate_squareT": "смена сечения", "squareT_platform_squareT_switch": "станция, стрелка",
             "new_data": "проезд 20 минут (второй датасет)"}
    fig, ax = plt.subplots(figsize=(9, 4.2))
    everything = []
    for b in BAGS + ["new_data"]:
        path = RESULTS_DIR / f"{b}.jsonl"
        if not path.exists():
            continue
        free = np.array([json.loads(l)["free"] for l in path.read_text().splitlines() if l.strip()])
        everything.append(free)
        x = np.sort(free)
        y = 100.0 * (1.0 - np.arange(len(x)) / max(len(x) - 1, 1))
        ax.plot(x, y, lw=2.0 if b == "new_data" else 1.2, label=names.get(b, b),
                color="crimson" if b == "new_data" else None)
    if not everything:
        plt.close(fig)
        return
    for d, text in ((100, "100 м: хорошо"), (200, "200 м: очень хорошо")):
        ax.axvline(d, color="0.5", ls="--", lw=1)
        ax.text(d + 2, 92, text, fontsize=8, color="0.35")
    ax.set_xlim(0, 210)
    ax.set_ylim(0, 100)
    ax.set_xlabel("дальность, м")
    ax.set_ylabel("доля кадров, где путь просматривается дальше, %")
    ax.set_title("Насколько далеко вообще виден путь (все %d кадров)" % sum(len(e) for e in everything))
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="upper right", frameon=False)
    fig.savefig(IMG / "exp_visibility.png", dpi=110, bbox_inches="tight")
    plt.close(fig)


def timing_chart():
    fig, ax = plt.subplots(figsize=(9, 4.0))
    stages = ["transform", "calib", "axis", "corridor", "detect", "track"]
    names = {"transform": "преобразование", "calib": "калибровка", "axis": "рельсы", "corridor": "коридор",
             "detect": "зоны+кластеры", "track": "трекинг"}
    labels, bottoms = [], None
    means = []
    for b in BAGS:
        path = RESULTS_DIR / f"{b}.jsonl"
        if not path.exists():
            continue
        rows = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
        means.append([np.mean([r["ms"][s] for r in rows]) for s in stages])
        labels.append(b)
    means = np.array(means)
    bottoms = np.zeros(len(labels))
    for k, s in enumerate(stages):
        ax.barh(labels, means[:, k], left=bottoms, label=names[s])
        bottoms += means[:, k]
    ax.set_xlabel("среднее время на кадр, мс (офлайн, Ryzen 5 5600, 4 потока)")
    ax.legend(fontsize=8, ncol=6, loc="upper center", bbox_to_anchor=(0.4, -0.22), frameon=False)
    ax.grid(alpha=0.3, axis="x")
    fig.savefig(IMG / "exp_timing.png", dpi=110, bbox_inches="tight")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--synthetic", default=str(ROOT / "data" / "results" / "synthetic" / "summary.json"))
    ap.add_argument("--results", default=str(ROOT / "data" / "results"),
                    help="directory with the per-bag JSONL of the run to plot")
    args = ap.parse_args()
    global RESULTS_DIR, SYNTHETIC_DIR
    RESULTS_DIR = Path(args.results)
    SYNTHETIC_DIR = Path(args.synthetic).parent
    IMG.mkdir(parents=True, exist_ok=True)
    summary_path = Path(args.synthetic)
    if summary_path.exists():
        summary = json.loads(summary_path.read_text())
        recall_chart(summary)
        approach_chart(summary)
    obstacle_timeline_chart()
    timing_chart()
    visibility_chart()
    print("charts written to", IMG)


if __name__ == "__main__":
    main()
