"""Track-bed cross-sections (track frame) for chosen frames, several distance slices."""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from proto_common import Bag, load_frame, ransac_ground, to_track, track_transform

name = sys.argv[1]
frames = [int(f) for f in sys.argv[2:]]
slices = [(5, 7), (9, 11), (14, 16), (20, 23)]
fig, axs = plt.subplots(len(frames), len(slices), figsize=(5 * len(slices), 2.6 * len(frames)))
axs = np.atleast_2d(axs)
bag = Bag(name)
for r, fi in enumerate(frames):
    P, it, _, ring, _ = load_frame(bag, fi)
    n, d = ransac_ground(P)
    Q = to_track(P, *track_transform(n, d))
    for c, (a, b) in enumerate(slices):
        s = (Q[:, 0] > a) & (Q[:, 0] < b) & (np.abs(Q[:, 1]) < 3) & (Q[:, 2] < 1.0) & (Q[:, 2] > -0.8)
        axs[r, c].scatter(Q[s, 1], Q[s, 2], s=1.5, c="k")
        axs[r, c].set_ylim(-0.8, 1.0); axs[r, c].set_xlim(-3, 3); axs[r, c].grid(alpha=0.3)
        axs[r, c].set_title(f"{name[:22]} f{fi} x={a}-{b} h={d:.2f}", fontsize=8)
fig.tight_layout()
fig.savefig(Path(__file__).parent / "plots" / f"xsec_{name}.png", dpi=55)
