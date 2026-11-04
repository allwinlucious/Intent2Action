"""Records a take of every held-out task a version passed in the Isaac evaluation, for the video's grid of successes.

  .venv/bin/python tools/record_successes.py G1d        (the stack must be up: tests/e2e/demo_stack.py)

Each take runs the exact program that was evaluated (logs/eval/<VERSION>_isaac_episodes.jsonl) through
tools/record_take.py, as C:\\isaac-sim-work\\videos\\grid\\<family>_<seed>.mp4. Physics can differ between runs, so a
take that fails now is reported and kept out of the grid (listed in grid/index.json with its error).
"""

import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
GRID = Path("/mnt/c/isaac-sim-work/videos/grid")


FAIL_GRID = Path("/mnt/c/isaac-sim-work/videos/grid_fail")


def record_failure(name: str, take: Path, result: str) -> None:
  """Copies a failed take into grid_fail/ (its index marks it usable for the failure grid)."""
  import shutil
  FAIL_GRID.mkdir(parents=True, exist_ok=True)
  shutil.copy(take, FAIL_GRID / take.name)
  path = FAIL_GRID / "index.json"
  index = json.loads(path.read_text()) if path.exists() else {}
  index[name] = {"ok": True, "result": result, "scripted": False}
  path.write_text(json.dumps(index, indent=1))


def main(version: str) -> None:
  GRID.mkdir(parents=True, exist_ok=True)
  index_path = GRID / "index.json"
  index = json.loads(index_path.read_text()) if index_path.exists() else {}
  episodes = [json.loads(line) for line in (ROOT / f"logs/eval/{version}_isaac_episodes.jsonl").open()]
  for episode in (e for e in episodes if e["ok"]):
    name = f"{episode['family']}_{episode['seed']}"
    if index.get(name, {}).get("ok"):
      continue  # already recorded
    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as program:
      program.write(episode["program"])
    out = subprocess.run([sys.executable, "tools/record_take.py", f"{episode['family']}:{episode['seed']}",
                          "--program", program.name, "--name", f"grid_{name}"],
                         cwd=ROOT, capture_output=True, text=True, timeout=1200)
    lines = out.stdout.strip().splitlines()
    ok = bool(lines) and lines[0] == "OK"
    take = Path(f"/mnt/c/isaac-sim-work/videos/grid_{name}.mp4")
    if take.exists():
      take.replace(GRID / f"{name}.mp4")
      if not ok:  # a physics failure this time: a copy for the failure grid
        record_failure(name, GRID / f"{name}.mp4", f"{version} program, physics failure: {lines[0] if lines else ''}")
    index[name] = {"ok": ok, "result": lines[0] if lines else out.stderr[-300:], "intent": episode["intent"]}
    index_path.write_text(json.dumps(index, indent=1))
    print(f"{name}: {'OK' if ok else index[name]['result'][:160]}", flush=True)
  print(f"{sum(v['ok'] for v in index.values())} of {len(index)} takes OK")


if __name__ == "__main__":
  main(sys.argv[1] if len(sys.argv) > 1 else "G1d")
