"""Tiles successful takes into one 1920x1080 grid video (real speed, 30 fps): the video's "it does all of these" shot.

  .venv/bin/python tools/grid_video.py --cols 4 --rows 4 --out C:/isaac-sim-work/videos/grid_4x4.mp4

Takes come from C:\\isaac-sim-work\\videos\\grid\\ (tools/record_successes.py), only those marked OK in index.json,
mixed across task families. A shorter take holds its last frame until the longest one ends; --max-s trims all.
"""

import argparse
import itertools
import json
from pathlib import Path
import subprocess

import imageio.v2 as imageio
import imageio_ffmpeg

GRID = Path("/mnt/c/isaac-sim-work/videos/grid")
W, H = 1920, 1080


def pick(names: list[str], count: int) -> list[str]:
  """Round-robin over families, so the grid shows every kind of task."""
  by_family: dict[str, list[str]] = {}
  for name in sorted(names):
    by_family.setdefault(name.rsplit("_", 1)[0], []).append(name)
  mixed = [n for group in itertools.zip_longest(*by_family.values()) for n in group if n]
  return mixed[:count]


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--cols", type=int, default=4)
  parser.add_argument("--rows", type=int, default=4)
  parser.add_argument("--max-s", type=float, default=0.0)
  parser.add_argument("--out", default=str(GRID.parent / "grid.mp4"))
  args = parser.parse_args()
  index = json.loads((GRID / "index.json").read_text())
  names = pick([n for n, v in index.items() if v["ok"] and (GRID / f"{n}.mp4").exists()], args.cols * args.rows)
  if len(names) < args.cols * args.rows:
    raise SystemExit(f"only {len(names)} OK takes for a {args.cols}x{args.rows} grid")
  tw, th = W // args.cols, H // args.rows
  inputs = [arg for n in names for arg in ("-i", str(GRID / f"{n}.mp4"))]
  # 2 px gaps on a black background; each tile holds its last frame (tpad) until the longest input ends.
  tiles = [f"[{i}:v]scale={tw - 2}:{th - 2},tpad=stop_mode=clone:stop_duration=600,setsar=1[t{i}]"
           for i in range(len(names))]
  layout = "|".join(f"{(i % args.cols) * tw}_{(i // args.cols) * th}" for i in range(len(names)))
  graph = ";".join(tiles) + ";" + "".join(f"[t{i}]" for i in range(len(names))) + \
      f"xstack=inputs={len(names)}:layout={layout}:fill=black,pad={W}:{H}[v]"
  longest = max(imageio.get_reader(GRID / f"{n}.mp4").get_meta_data()["duration"] for n in names)
  duration = min(longest, args.max_s) if args.max_s else longest
  subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", *inputs, "-filter_complex", graph,
                  "-map", "[v]", "-t", f"{duration:.2f}", "-r", "30", "-c:v", "libx264", "-crf", "18",
                  "-pix_fmt", "yuv420p", args.out], check=True)
  print(f"{args.out}: {args.cols}x{args.rows}, {duration:.1f} s: {', '.join(names)}")


if __name__ == "__main__":
  main()
