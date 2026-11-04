"""Makes the README's GIFs from the recorded takes (C:\\isaac-sim-work\\videos\\, tools/record_take.py and friends).

  .venv/bin/python tools/readme_gifs.py        -> docs/media/*.gif

Each GIF is a real take, sped up as its name in the README says, 10 fps, palette-optimised. Side-by-side GIFs hold a
shorter take's last frame until the longer one ends.
"""

from pathlib import Path
import subprocess

import imageio_ffmpeg

VIDEOS = Path("/mnt/c/isaac-sim-work/videos")
OUT = Path(__file__).resolve().parents[1] / "docs/media"
FPS = 10

# name: (takes, speed-up, width of each tile)
GIFS = {
    "gift_pack": (["gift_pack_glimmer"], 2.0, 480),
    "tower_untrained_vs_finetuned": (["tower_unload_untrained", "tower_unload_finetuned"], 3.0, 360),
    "experience_wonder_vs_ours": (["experience_wonder", "experience_ours"], 3.5, 360),
    "push_finale": (["finale_push_g1e"], 10.0, 480),
    "success_grid": (["grid_success_4x4"], 6.0, 560),
}


def make(name: str, takes: list[str], speed: float, width: int) -> Path:
  ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
  inputs = [arg for take in takes for arg in ("-i", str(VIDEOS / f"{take}.mp4"))]
  tiles = [f"[{i}:v]setpts=PTS/{speed},fps={FPS},scale={width}:-2:flags=lanczos,"
           f"tpad=stop_mode=clone:stop_duration=30[t{i}]" for i in range(len(takes))]
  joined = "".join(f"[t{i}]" for i in range(len(takes)))
  stack = f"{joined}hstack=inputs={len(takes)}[v]" if len(takes) > 1 else "[t0]copy[v]"
  # The longest take decides the length (the others hold their last frame).
  import imageio.v2 as imageio
  seconds = max(imageio.get_reader(VIDEOS / f"{take}.mp4").get_meta_data()["duration"] for take in takes) / speed
  graph = ";".join(tiles) + ";" + stack + ";[v]split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];" \
      "[b][p]paletteuse=dither=bayer:bayer_scale=4"
  out = OUT / f"{name}.gif"
  subprocess.run([ffmpeg, "-y", "-loglevel", "error", *inputs, "-filter_complex", graph, "-t", f"{seconds:.2f}",
                  "-loop", "0", str(out)], check=True)
  return out


def main() -> None:
  OUT.mkdir(parents=True, exist_ok=True)
  for name, (takes, speed, width) in GIFS.items():
    out = make(name, takes, speed, width)
    print(f"{out}: {out.stat().st_size / 1e6:.1f} MB", flush=True)


if __name__ == "__main__":
  main()
