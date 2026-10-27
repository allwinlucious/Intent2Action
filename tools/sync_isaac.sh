#!/bin/bash
# Copies the files Isaac reads to Windows (phase_2.md decision 16):
#   tools/sync_isaac.sh                 sim/, config/, assets/ -> C:\isaac-sim-work\repo\,
#                                       container/dds/fastdds.xml -> C:\isaac-sim-work\fastdds.xml
#   tools/sync_isaac.sh --pull-catalog  C:\isaac-sim-work\repo\assets\objects\*\model.usd -> assets/objects/
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK=/mnt/c/isaac-sim-work
REPO="$WORK/repo"

if [[ "${1:-}" == "--pull-catalog" ]]; then
  for usd in "$REPO"/assets/objects/*/model.usd; do
    name="$(basename "$(dirname "$usd")")"
    mkdir -p "$ROOT/assets/objects/$name"
    cp --no-preserve=mode "$usd" "$ROOT/assets/objects/$name/model.usd"
  done
  exit 0
fi

mkdir -p "$REPO"
cp -r "$ROOT/sim" "$ROOT/config" "$ROOT/assets" "$REPO/"
cp "$ROOT/container/dds/fastdds.xml" "$WORK/fastdds.xml"
