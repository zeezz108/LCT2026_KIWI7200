"""BEV of the rail height band (just above the track bed) for several scenes: are the rails usable as lateral anchor?"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from proto_common import Bag, load_frame, ransac_ground, to_track, track_transform

cases = [("roundT_pressureGate_roundT", 267), ("roundT_pressureGate_roundT", 106), ("doubleT_platform", 68),
         ("doubleT_platform", 300), ("squareT_platform_squareT_switch", 525), ("doubleT_obstacle", 100),
         ("roundT_doubleT", 200), ("squareT_platform_squareT_switch", 800)]
fig, axs = plt.subplots(len(cases), 2, figsize=(26, 3.2 * len(cases)), gridspec_kw={"width_ratios": [3, 1]})
for (name, fi), (ax, ax2) in zip(cases, axs):
    bag = Bag(name)
    P, it, _, ring, col = load_frame(bag, fi)
    n, d = ransac_ground(P)
    Q = to_track(P, *track_transform(n, d))
    m = (Q[:, 0] > 2) & (Q[:, 0] < 45) & (np.abs(Q[:, 1]) < 4) & (Q[:, 2] > -0.4) & (Q[:, 2] < 0.8)
    sc = ax.scatter(Q[m, 0], Q[m, 1], c=Q[m, 2], s=1.5, cmap="turbo", vmin=-0.3, vmax=0.7)
    ax.set_xlim(2, 45); ax.set_ylim(-4, 4); ax.set_title(f"{name} f{fi}: near-floor band (z -0.4..0.8), colour=z", fontsize=9)
    ax.grid(alpha=0.3)
    # lateral height profile accumulated over 6..14 m
    s = (Q[:, 0] > 6) & (Q[:, 0] < 14) & (np.abs(Q[:, 1]) < 4) & (Q[:, 2] < 1.0)
    ax2.scatter(Q[s, 1], Q[s, 2], s=1, c="k")
    ax2.set_xlim(-4, 4); ax2.set_ylim(-0.5, 1.0); ax2.grid(alpha=0.3); ax2.set_title("cross-section 6..14 m", fontsize=9)
fig.colorbar(sc, ax=axs[:, 0], shrink=0.3)
fig.savefig(Path(__file__).parent / "plots" / "rails_band.png", dpi=55)
