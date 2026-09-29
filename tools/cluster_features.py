"""Compare candidate-cluster features of real objects (synthetic runs) and false candidates (empty bags).

Requires offline results with feature fields (above, hmin, hmax, n, len, wid) produced with the height filter off:
  data/results/<bag>.jsonl for the empty bags, data/results/synthetic/*.jsonl + summary.json.
usage: python tools/cluster_features.py
"""
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "data" / "results"
EMPTY = ["doubleT_platform", "roundT_doubleT", "roundT_pressureGate_roundT", "roundT_squareT_pressureGate_squareT",
         "squareT_platform_squareT_switch"]
FEATS = ["above", "hmin", "hmax", "n", "len", "wid", "lat"]
BINS = [(0, 40), (40, 70), (70, 100), (100, 130), (130, 160), (160, 200)]


def load(path):
    return [json.loads(l) for l in path.read_text().splitlines() if l.strip()]


def negatives():
    out = []
    for b in EMPTY:
        for r in load(RES / f"{b}.jsonl"):
            for c in r["clusters"]:
                if c["level"] == 2:
                    out.append(c)
    return out


def positives():
    summary = json.loads((RES / "synthetic" / "summary.json").read_text())
    out = []
    for s in summary:
        rows = load(RES / "synthetic" / f"{s['name']}.jsonl")
        t0 = rows[0]["stamp"]
        for r in rows:
            d = s["distance"] - s["speed"] * (r["stamp"] - t0)
            tol = max(2.0, 0.04 * d)
            for c in r["clusters"]:
                if abs(c["x"] - d) <= tol and abs(c["cy"] - s["lateral"]) < 1.2 and c["level"] == 2:
                    c = dict(c, object=s["object"])
                    out.append(c)
    return out


def main():
    neg, pos = negatives(), positives()
    print(f"danger-level candidates: false={len(neg)} (empty bags), true={len(pos)} (synthetic)")
    for lo, hi in BINS:
        N = [c for c in neg if lo <= c["x"] < hi]
        P = [c for c in pos if lo <= c["x"] < hi]
        if not P and not N:
            continue
        print(f"--- {lo:3d}-{hi:3d} m: false={len(N)} true={len(P)}")
        for f in FEATS:
            nv = np.array([c[f] for c in N if c.get(f) is not None], float)
            pv = np.array([c[f] for c in P if c.get(f) is not None], float)
            q = lambda v: np.percentile(v, [10, 50, 90]).round(2) if len(v) else "-"  # noqa: E731
            print(f"   {f:6s} false p10/50/90={q(nv)}  true p10/50/90={q(pv)}")
        # height threshold keeping >= 90% of true objects: how many false ones remain
        if P and N:
            pa = np.array([c["above"] for c in P])
            na = np.array([c["above"] for c in N])
            thr = np.percentile(pa, 10)
            print(f"   above >= {thr:.2f} keeps 90% true, {np.mean(na >= thr) * 100:.0f}% false")
            ph = np.array([c["hmax"] for c in P])
            nh = np.array([c["hmax"] for c in N])
            thr2 = np.percentile(ph, 10)
            print(f"   hmax  >= {thr2:.2f} keeps 90% true, {np.mean(nh >= thr2) * 100:.0f}% false")


if __name__ == "__main__":
    main()
