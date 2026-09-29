"""Overview plots of one frame per bag: range image, BEV (near/far), side view. Plus dual-return check."""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from bagio import BAGS, Bag

OUT = Path(__file__).parent / "plots"
OUT.mkdir(exist_ok=True)
frame_sel = {n: 0 for n in BAGS}
if len(sys.argv) > 2:
    frame_sel = {sys.argv[1]: int(sys.argv[2])}

for name, fi in frame_sel.items():
    bag = Bag(name)
    a, stamp, m = bag.points(fi)
    ncol = len(a) // 128
    g = a.reshape(ncol, 128)
    x, y, z, it = (g[k].astype(np.float64) for k in ("x", "y", "z", "intensity"))
    r = np.sqrt(x * x + y * y + z * z)
    valid = r > 0

    # dual return check: columns 2k vs 2k+1
    az = np.degrees(np.arctan2(x, -y))
    pair_both = valid[0::2] & valid[1::2]
    daz = np.abs(az[0::2] - az[1::2])[pair_both]
    dr = np.abs(r[0::2] - r[1::2])[pair_both]
    print(f"{name} f{fi}: pairs both valid={pair_both.sum()}  |daz| median={np.median(daz):.4f} p99={np.percentile(daz, 99):.4f}  "
          f"identical range share={(dr < 1e-3).mean():.3f}  |dr|>0.5m share={(dr > 0.5).mean():.4f}")
    # also consecutive azimuth step between pairs
    col_az = np.nanmedian(np.where(valid, az, np.nan), axis=1)
    print(f"   col az [2000:2010]: {np.round(col_az[2000:2010], 3) if ncol > 2010 else ''}")

    fig = plt.figure(figsize=(22, 16))
    ax = fig.add_subplot(4, 1, 1)
    rim = np.where(valid, np.log10(np.maximum(r, 0.3)), np.nan).T
    ax.imshow(rim, aspect="auto", cmap="turbo", interpolation="nearest", vmin=0, vmax=np.log10(210))
    ax.set_title(f"{name} frame {fi}: range image log10(range) rows=ring cols={ncol}")
    ax = fig.add_subplot(4, 1, 2)
    iim = np.where(valid, it, np.nan).T
    ax.imshow(iim, aspect="auto", cmap="gray", interpolation="nearest", vmin=0, vmax=60)
    ax.set_title("intensity image")

    X, Y, Z, R = x[valid], -y[valid], z[valid], r[valid]
    ax = fig.add_subplot(4, 2, 5)
    s = (Y > 0) & (Y < 60)
    sc = ax.scatter(Y[s], X[s], c=Z[s], s=0.2, cmap="turbo", vmin=-2.5, vmax=3)
    ax.set_xlabel("forward = -y (m)"); ax.set_ylabel("x (m)"); ax.set_title("BEV 0..60 m, color=z")
    ax.set_ylim(-10, 10)
    ax = fig.add_subplot(4, 2, 6)
    s = Y > 0
    ax.scatter(Y[s], X[s], c=Z[s], s=0.3, cmap="turbo", vmin=-2.5, vmax=3)
    ax.set_xlabel("forward (m)"); ax.set_title("BEV full, color=z"); ax.set_ylim(-25, 25)
    ax = fig.add_subplot(4, 2, 7)
    s = (Y > 0) & (np.abs(X) < 10)
    ax.scatter(Y[s], Z[s], c=X[s], s=0.3, cmap="coolwarm", vmin=-4, vmax=4)
    ax.set_xlabel("forward (m)"); ax.set_ylabel("z (m)"); ax.set_title("side view, color=x")
    ax = fig.add_subplot(4, 2, 8)
    s = (Y > 8) & (Y < 12)
    ax.scatter(X[s], Z[s], c=Y[s], s=1, cmap="viridis")
    ax.set_aspect("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("z (m)"); ax.set_title("cross-section 8..12 m ahead")
    fig.tight_layout()
    fig.savefig(OUT / f"overview_{name}_f{fi}.png", dpi=70)
    plt.close(fig)
print("saved to", OUT)
