"""The published experiment data: every number in docs/designs/learning_results.md can be rebuilt from it.

  python tools/results.py restore      results/**/*.jsonl.gz -> logs/ (existing files are kept unless --force)
  python tools/results.py pack         logs/ -> results/ (maintainers, after new runs)
  python tools/results.py isaac        Isaac physics successes per version (logs/eval/*_isaac_episodes.jsonl)

logs/ is where every script reads and writes (git-ignored, it also holds bulky logs); results/ is the curated,
compressed copy in the repository: expert runs, evaluation programs and episodes of every version, the requests,
GRPO rewards, physics labels, and the training sets of the published adapters (G1b, G1d).
"""

import argparse
import gzip
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
LOGS, RESULTS = ROOT / "logs", ROOT / "results"
PATTERNS = (
    "expert/heldout_all.jsonl", "expert/train_all_sorting.jsonl", "expert/recovery_best.jsonl",
    "eval/requests.jsonl", "eval/*_replies.jsonl", "eval/*_programs*.jsonl", "eval/*_episodes*.jsonl", "eval/*.rewards.jsonl",
    "eval/PHYS_labels.jsonl",
    "sft/G1b/*.json*", "sft/G1d/*.json*",
)


def pack() -> None:
  count = 0
  for pattern in PATTERNS:
    for source in sorted(LOGS.glob(pattern)):
      if ".shard" in source.name:
        continue
      target = RESULTS / source.relative_to(LOGS).with_name(source.name + ".gz")
      target.parent.mkdir(parents=True, exist_ok=True)
      with source.open("rb") as raw, gzip.GzipFile(target, "wb", mtime=0) as packed:  # mtime 0: stable bytes
        shutil.copyfileobj(raw, packed)
      count += 1
  print(f"packed {count} files into {RESULTS.relative_to(ROOT)}/")


def restore(force: bool) -> None:
  written = kept = 0
  for packed in sorted(RESULTS.rglob("*.gz")):
    target = LOGS / packed.relative_to(RESULTS).with_suffix("")
    if target.exists() and not force:
      kept += 1
      continue
    target.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(packed, "rb") as source, target.open("wb") as out:
      shutil.copyfileobj(source, out)
    written += 1
  print(f"restored {written} files into logs/ ({kept} already there; --force overwrites)")


def isaac() -> None:
  import json
  for path in sorted((LOGS / "eval").glob("*_isaac_episodes.jsonl")):
    episodes = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    print(f"{path.name.removesuffix('_isaac_episodes.jsonl'):>8}  Isaac physics: "
          f"{sum(bool(e['ok']) for e in episodes)}/{len(episodes)}")


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("command", choices=("pack", "restore", "isaac"))
  parser.add_argument("--force", action="store_true")
  args = parser.parse_args()
  {"pack": pack, "restore": lambda: restore(args.force), "isaac": isaac}[args.command]()
