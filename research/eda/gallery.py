"""Per-bag gallery: BEV (0..160 m) at several moments + far-visibility time series."""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from bagio import BAGS, Bag

OUT = Path(__file__).parent / "plots"
FINE = slice(26, 90)  # 0.125 deg rings

for name in BAGS:
    bag = Bag(name)
    n = len(bag)
    sel = np.linspace(0, n - 1, 6).astype(int)
    fig, axs = plt.subplots(len(sel), 1, figsize=(22, 3.2 * len(sel)))
    far = []
    for k, i in enumerate(range(0, n, 5)):
        a, s, _ = bag.points(i)
        g = a.reshape(-1, 128)
        r = np.sqrt(g["x"] ** 2 + g["y"] ** 2 + g["z"] ** 2)
        rf = r[:, FINE]
        far.append((i, np.percentile(rf[rf > 0], 99.9), (rf > 50).sum(), (rf > 100).sum(), (rf > 150).sum()))
    for ax, i in zip(axs, sel):
        a, s, _ = bag.points(i)
        v = a[(a["x"] != 0) | (a["y"] != 0)]
        f = -v["y"]
        m = (f > 0) & (f < 160) & (np.abs(v["x"]) < 12)
        ax.scatter(f[m], v["x"][m], c=v["z"][m], s=0.15, cmap="turbo", vmin=-2.5, vmax=3.0)
        ax.set_xlim(0, 160); ax.set_ylim(-10, 10)
        ax.set_title(f"{name} frame {i}", fontsize=9)
    fig.tight_layout(); fig.savefig(OUT / f"gallery_{name}.png", dpi=55); plt.close(fig)
    far = np.array(far)
    print(f"{name}: p99.9 range fine rings  min={far[:, 1].min():.0f} med={np.median(far[:, 1]):.0f} max={far[:, 1].max():.0f} | "
          f"pts>100m med={np.median(far[:, 3]):.0f} | pts>150m med={np.median(far[:, 4]):.0f} min={far[:, 4].min():.0f}")
