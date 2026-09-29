"""Zoom on corridor intrusions: BEV around a distance window + cross-section, points coloured by height."""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import corridor_proto as cp
from proto_common import Bag, load_frame, ransac_ground, to_track, track_transform

bag_name, fi, x0, x1 = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
bag = Bag(bag_name)
P, it, _, ring, col = load_frame(bag, fi)
n, d = ransac_ground(P)
Rm, t = track_transform(n, d)
Q = to_track(P, Rm, t)
occ, long_occ = cp.occupancy(Q)
th0, ks, yc, cost = cp.search_centerline(cp.prefix(long_occ.astype(np.int32)))
yci = np.interp(Q[:, 0], cp.XS, yc)
lat = Q[:, 1] - yci

fig, axs = plt.subplots(1, 3, figsize=(26, 7), gridspec_kw={"width_ratios": [3, 1.2, 1.2]})
m = (Q[:, 0] > x0) & (Q[:, 0] < x1) & (np.abs(Q[:, 1]) < 10) & (Q[:, 2] > -0.5) & (Q[:, 2] < 4.5)
sc = axs[0].scatter(Q[m, 0], Q[m, 1], c=Q[m, 2], s=2, cmap="turbo", vmin=-0.3, vmax=4)
xs = np.linspace(x0, x1, 200)
yy = np.interp(xs, cp.XS, yc)
for off, ls in ((0, "-"), (1.5, "--"), (-1.5, "--"), (1.15, ":"), (-1.15, ":")):
    axs[0].plot(xs, yy + off, "k" + ls, lw=0.8)
axs[0].set_aspect("equal"); plt.colorbar(sc, ax=axs[0], label="z above bed")
axs[0].set_title(f"{bag_name} f{fi} BEV {x0}-{x1} m; kappas={[f'{k * 1e3:+.2f}e-3' for k in ks]}")

inside = (Q[:, 0] > x0) & (Q[:, 0] < x1) & (((Q[:, 2] > 0.3) & (Q[:, 2] <= 0.9) & (np.abs(lat) < 1.15)) | ((Q[:, 2] > 0.9) & (Q[:, 2] < 3.4) & (np.abs(lat) < 1.5)))
if inside.any():
    xc = np.median(Q[inside, 0])
    band = np.abs(Q[:, 0] - xc) < 1.0
    axs[1].scatter(lat[band], Q[band, 2], s=3, c="0.5")
    axs[1].scatter(lat[band & inside], Q[band & inside, 2], s=6, c="r")
    axs[1].set_aspect("equal"); axs[1].set_title(f"cross-section at x={xc:.1f}±1 m (lat rel. corridor)")
    axs[1].axvline(1.5, ls="--"); axs[1].axvline(-1.5, ls="--")
    axs[2].scatter(Q[band, 1], Q[band, 2], s=3, c=it[band], cmap="viridis", vmin=0, vmax=60)
    axs[2].set_aspect("equal"); axs[2].set_title("same slice, raw Y, colour=intensity")
    print("inside pts:", inside.sum(), "median x:", xc, "lat range:", lat[inside].min(), lat[inside].max(),
          "z range:", Q[inside, 2].min(), Q[inside, 2].max(), "intensity median:", np.median(it[inside]))
fig.tight_layout()
fig.savefig(Path(__file__).parent / "plots" / f"intrusion_{bag_name}_f{fi}_{int(x0)}.png", dpi=60)
