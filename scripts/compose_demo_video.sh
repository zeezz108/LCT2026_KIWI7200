#!/usr/bin/env bash
# Glue recorded demo clips (scripts/record_demo_headless.sh) into one video with a caption bar on each clip.
# Usage (WSL / Linux): bash scripts/compose_demo_video.sh <out.mp4> <clip.mp4> <start_s> <duration_s> "<caption>" [...]
set -eo pipefail
OUT="$1"; shift
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
WORK=$(mktemp -d)
trap 'rm -rf "${WORK}"' EXIT
LIST="${WORK}/list.txt"
i=0
while [[ $# -ge 4 ]]; do
  CLIP="$1"; START="$2"; DURATION="$3"; CAPTION="$4"; shift 4
  printf '%s' "${CAPTION}" > "${WORK}/caption_${i}.txt"
  ffmpeg -y -loglevel error -ss "${START}" -t "${DURATION}" -i "${CLIP}" \
    -vf "drawbox=x=0:y=0:w=iw:h=96:color=black@0.72:t=fill,drawtext=fontfile=${FONT}:textfile=${WORK}/caption_${i}.txt:fontcolor=white:fontsize=34:x=36:y=30" \
    -r 15 -c:v libx264 -preset slow -crf 22 -pix_fmt yuv420p "${WORK}/part_${i}.mp4"
  echo "file '${WORK}/part_${i}.mp4'" >> "${LIST}"
  i=$((i + 1))
done
ffmpeg -y -loglevel error -f concat -safe 0 -i "${LIST}" -c copy "${OUT}"
echo "saved ${OUT} ($(ffprobe -v error -show_entries format=duration -of csv=p=0 "${OUT}") s)"
