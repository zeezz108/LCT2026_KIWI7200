"""Make a short standalone bag from a few files of a split recording, without reading the whole thing.

The customer's 20-minute run is one bag split into 221 files of ~51 frames each (84 GB). To use a piece of it as a
base for synthetic obstacles we only need the file (or two) that hold the frames of interest: they are copied out
and given their own metadata, which is thousands of times cheaper than streaming the whole recording.

usage:
  python tools/cut_parts.py --bag <bag dir> --out data/for_hackathon/new_curve --start 6600 [--frames 60]
  python tools/cut_parts.py --repair data/for_hackathon/new_curve      # rewrite metadata.yaml of an existing cut
"""
import argparse
import re
import shutil
import sqlite3
from pathlib import Path


def stats(db):
    """Frames and time span of one part, read from the sqlite file itself."""
    with sqlite3.connect(f"file:{db}?mode=ro", uri=True) as con:
        n, lo, hi = con.execute("select count(*), min(timestamp), max(timestamp) from messages").fetchone()
    return n, lo, hi


def write_metadata(out, meta, picked):
    """rosbag2 v5 metadata for the copied parts. The `files:` block is not optional: the C++ reader parses it and
    aborts on an empty one, and the times must describe the cut, not the recording it came from."""
    info = [(name, *stats(out / name)) for name in picked]
    count = sum(i[1] for i in info)
    start = min(i[2] for i in info)
    end = max(i[3] for i in info)
    body = meta.split("  relative_file_paths:")[0]
    body = re.sub(r"^  message_count: \d+", f"  message_count: {count}", body, count=1, flags=re.M)
    body = re.sub(r"^      message_count: \d+", f"      message_count: {count}", body, count=1, flags=re.M)
    body = re.sub(r"^    nanoseconds: \d+", f"    nanoseconds: {end - start}", body, count=1, flags=re.M)
    body = re.sub(r"^    nanoseconds_since_epoch: \d+", f"    nanoseconds_since_epoch: {start}", body, count=1,
                  flags=re.M)
    lines = ["  relative_file_paths:"]
    lines += [f"    - {name}" for name in picked]
    lines.append("  files:")
    for name, n_msg, lo, hi in info:
        lines += [f"    - path: {name}",
                  "      starting_time:",
                  f"        nanoseconds_since_epoch: {lo}",
                  "      duration:",
                  f"        nanoseconds: {hi - lo}",
                  f"      message_count: {n_msg}"]
    (out / "metadata.yaml").write_text(body + "\n".join(lines) + "\n", encoding="utf-8")
    return count


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag")
    ap.add_argument("--out")
    ap.add_argument("--start", type=int, help="frame index in the whole recording")
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--repair", help="existing cut: rewrite its metadata.yaml from the .db3 files it holds")
    args = ap.parse_args()

    if args.repair:
        out = Path(args.repair)
        meta = (out / "metadata.yaml").read_text(encoding="utf-8")
        picked = re.findall(r"- (\S+\.db3)", meta)
        print(f"{out}: metadata rewritten, {write_metadata(out, meta, picked)} frames in {len(picked)} file(s)")
        return

    src = Path(args.bag)
    meta = (src / "metadata.yaml").read_text(encoding="utf-8")
    files = re.findall(r"- (\S+\.db3)", meta)
    total = int(re.search(r"^  message_count: (\d+)", meta, re.M).group(1))
    per_file = total // len(files)  # the recorder splits by size, so the parts are equal
    first = args.start // per_file
    last = min(len(files) - 1, (args.start + args.frames) // per_file)
    picked = files[first:last + 1]

    out = Path(args.out)
    if out.exists():
        raise SystemExit(f"{out} exists")
    out.mkdir(parents=True)
    for name in picked:
        shutil.copy2(src / name, out / name)
    kept = write_metadata(out, meta, picked)
    print(f"{out}: {len(picked)} file(s) {picked[0]}..{picked[-1]}, {kept} frames "
          f"(wanted {args.frames} from {args.start}; the piece starts at frame {first * per_file})")


if __name__ == "__main__":
    main()
