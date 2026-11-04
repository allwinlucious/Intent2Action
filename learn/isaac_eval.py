"""The sim tier: a version's held-out programs on Isaac physics, and how often physics agrees with the plan tier.

  .venv/bin/python learn/isaac_eval.py run --version G1b --per-family 10      # starts Isaac (headless) + the stack
  .venv/bin/python learn/isaac_eval.py agreement --version G1b

`run` takes the first `per-family` held-out programs of each family from logs/eval/<V>_programs.jsonl and writes
logs/eval/<V>_isaac_episodes.jsonl (resumes). `agreement` compares each with the plan-tier episode of the same
program: both ok, plan ok / physics failed (what planning misses), plan failed / physics ok.
"""

import argparse
import collections
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen"), str(ROOT / "tests" / "e2e")]

from learn.episodes import read_jsonl  # noqa: E402

LOGS = ROOT / "logs" / "eval"


def run(version: str, per_family: int) -> None:
  import harness
  import phase1_check as p1
  import phase3_cuj_check as cuj
  import phase3_hwm_check as hwm
  from learn import glimmer_sample
  from learn.episodes import connect

  programs = read_jsonl(LOGS / f"{version}_programs.jsonl")
  chosen, count = [], collections.Counter()
  for value in sorted(programs, key=lambda v: (v["family"], v["seed"])):
    if count[value["family"]] < per_family:
      chosen.append(value)
      count[value["family"]] += 1
  harness.ISAAC_COMMAND[-1] += " --physics-only"
  harness.sync_isaac()
  hwm.CONTAINER_LOG = LOGS / f"{version}_isaac_container.log"
  stack = hwm._Run()
  try:
    stack.start_isaac(LOGS / f"{version}_isaac.log")
    stack.start_container()
    p1._wait_for_ports()
    hwm._require_state("ENABLED", hwm.ENABLED_TIMEOUT_S)
    cuj._wait_ready()
    sim_stub, robot_stub = connect()
    glimmer_sample.run(chosen, LOGS / f"{version}_isaac_episodes.jsonl", sim_stub, robot_stub)
  finally:
    stack.stop_container()
    stack.stop_isaac()


def label(per_family: int = 30) -> None:
  """Physics-verified labels for G4: on Isaac, each task's plan-tier label (logs/eval/PHYS_programs.jsonl), then its
  alternatives (other plan-tier successes, most reliable first) until one succeeds in physics. Writes
  logs/eval/PHYS_labels.jsonl: {family, seed, program (the first physics success, or null), attempts}. Resumes."""
  import harness
  import phase1_check as p1
  import phase3_cuj_check as cuj
  import phase3_hwm_check as hwm
  from learn import families
  from learn.episodes import connect, run_episode

  out_path = LOGS / "PHYS_labels.jsonl"
  done = {(v["family"], v["seed"]) for v in read_jsonl(out_path)}
  todo = [v for v in read_jsonl(LOGS / "PHYS_programs.jsonl") if (v["family"], v["seed"]) not in done]
  print(f"resume: {len(done)} labelled, {len(todo)} to go", flush=True)
  harness.ISAAC_COMMAND[-1] += " --physics-only"
  harness.sync_isaac()
  hwm.CONTAINER_LOG = LOGS / "PHYS_isaac_container.log"
  stack = hwm._Run()
  try:
    stack.start_isaac(LOGS / "PHYS_isaac.log")
    stack.start_container()
    p1._wait_for_ports()
    hwm._require_state("ENABLED", hwm.ENABLED_TIMEOUT_S)
    cuj._wait_ready()
    sim_stub, robot_stub = connect()
    with out_path.open("a") as out:
      for value in todo:
        task = families.generate(value["family"], value["seed"])
        attempts, verified = [], None
        for program in [value["program"], *value.get("alternatives", [])]:
          episode = run_episode(task, program, sim_stub, robot_stub)
          attempts.append({"ok": episode.ok, "error": episode.error})
          if episode.ok:
            verified = program
            break
        out.write(json.dumps({"family": value["family"], "seed": value["seed"], "program": verified,
                              "label_ok": attempts[0]["ok"], "attempts": attempts}) + "\n")
        out.flush()
        print(f"{'OK  ' if verified else 'FAIL'} {value['family']} {value['seed']} attempts {len(attempts)}", flush=True)
  finally:
    stack.stop_container()
    stack.stop_isaac()


def video(tasks: list[str], versions: list[str]) -> None:
  """One rendered Isaac session (studio look): every version's held-out program on every task, one clip each
  (logs/eval/videos/<family>_<seed>_<version>.mp4) plus its last frame."""
  import shutil
  import harness
  import imageio.v2 as imageio
  import demo_gui
  import phase1_check as p1
  import phase3_cuj_check as cuj
  import phase3_hwm_check as hwm
  from learn import families
  from learn.episodes import connect, record, run_episode

  programs = {(v["version"] if "version" in v else version, v["family"], v["seed"]): v["program"]
              for version in versions for v in read_jsonl(LOGS / f"{version}_programs.jsonl")}
  out_dir = LOGS / "videos"
  out_dir.mkdir(parents=True, exist_ok=True)
  harness.ISAAC_COMMAND[-1] += " --gui --pretty"
  harness.READY_TIMEOUT_S = 900
  harness.sync_isaac()
  hwm.CONTAINER_LOG = out_dir / "container.log"
  stack = hwm._Run()
  clips = []
  try:
    stack.start_isaac(out_dir / "isaac.log")
    stack.start_container()
    p1._wait_for_ports()
    hwm._require_state("ENABLED", hwm.ENABLED_TIMEOUT_S)
    cuj._wait_ready()
    sim_stub, robot_stub = connect()
    for spec in tasks:
      family, seed = spec.split(":")
      task = families.generate(family, int(seed))
      for version in versions:
        first = len(list(demo_gui.FRAMES.glob("frame_*.png")))
        episode = run_episode(task, programs[(version, family, int(seed))], sim_stub, robot_stub)
        last = len(list(demo_gui.FRAMES.glob("frame_*.png")))
        clips.append((f"{family}_{seed}_{version}", first, last, episode))
        print(f"{'OK  ' if episode.ok else 'FAIL'} {family} {seed} {version} {episode.error or ''}"[:200], flush=True)
        (out_dir / f"{family}_{seed}_{version}.json").write_text(json.dumps(record(episode), indent=1))
  finally:
    stack.stop_container()
    stack.stop_isaac()
  frames = sorted(demo_gui.FRAMES.glob("frame_*.png"))
  for name, first, last, _ in clips:
    part = frames[first:last]
    if not part:
      continue
    with imageio.get_writer(out_dir / f"{name}.mp4", fps=15, codec="libx264", quality=8, macro_block_size=8) as writer:
      for frame in part:
        try:
          writer.append_data(imageio.imread(frame)[..., :3])
        except (OSError, ValueError):
          continue
    shutil.copy(part[-1], out_dir / f"{name}_end.png")
    print(f"wrote {out_dir / name}.mp4 ({len(part)} frames)")


def agreement(version: str) -> None:
  single = LOGS / f"{version}_episodes.jsonl"
  # Versions graded only with repeated runs: their first run, the same single-run basis as the physics side.
  plan_rows = read_jsonl(single) if single.exists() else [
      v for v in read_jsonl(LOGS / f"{version}_episodes_r3.jsonl") if v.get("sample") == 0]
  plan = {(v["family"], v["seed"]): v["ok"] for v in plan_rows}
  physics = {(v["family"], v["seed"]): v for v in read_jsonl(LOGS / f"{version}_isaac_episodes.jsonl")}
  table = collections.Counter()
  for key, value in physics.items():
    table[(plan[key], value["ok"])] += 1
  n = sum(table.values())
  print(f"{version}: {n} programs on both tiers")
  print(f"  both ok {table[(True, True)]}, both failed {table[(False, False)]}, "
        f"plan ok / physics failed {table[(True, False)]}, plan failed / physics ok {table[(False, True)]}")
  print(f"  agreement {100 * (table[(True, True)] + table[(False, False)]) / n:.0f} %; "
        f"success plan {100 * sum(plan[k] for k in physics) / n:.0f} %, physics "
        f"{100 * sum(v['ok'] for v in physics.values()) / n:.0f} %")
  for key, value in sorted(physics.items()):
    if plan[key] and not value["ok"]:
      print(f"  physics-only failure {key}: {(value.get('error') or '')[:160]}")


def reuse(version: str, source: str) -> None:
  """Seeds <version>'s Isaac episodes with <source>'s where both versions wrote the identical program for the task
  (marked reused_from): `run` then resumes and runs only the tasks whose programs differ."""
  programs = {(v["family"], v["seed"]): v["program"] for v in read_jsonl(LOGS / f"{version}_programs.jsonl")}
  out = LOGS / f"{version}_isaac_episodes.jsonl"
  have = {(v["family"], v["seed"], v["sample"]) for v in read_jsonl(out)}
  reused = [{**v, "reused_from": source} for v in read_jsonl(LOGS / f"{source}_isaac_episodes.jsonl")
            if programs.get((v["family"], v["seed"])) == v["program"] and (v["family"], v["seed"], v["sample"]) not in have]
  with out.open("a") as handle:
    handle.writelines(json.dumps(v) + "\n" for v in reused)
  print(f"{version}: {len(reused)} Isaac episodes reused from {source} (identical programs)", flush=True)


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  sub = parser.add_subparsers(dest="command", required=True)
  p = sub.add_parser("run")
  p.add_argument("--version", required=True)
  p.add_argument("--per-family", type=int, default=10)
  p = sub.add_parser("agreement")
  p.add_argument("--version", required=True)
  sub.add_parser("label")
  p = sub.add_parser("reuse")
  p.add_argument("--version", required=True)
  p.add_argument("--from", dest="source", required=True)
  p = sub.add_parser("video")
  p.add_argument("--tasks", nargs="+", required=True, help="family:seed ...")
  p.add_argument("--versions", nargs="+", required=True)
  args = parser.parse_args()
  if args.command == "run":
    run(args.version, args.per_family)
  elif args.command == "agreement":
    agreement(args.version)
  elif args.command == "label":
    label()
  elif args.command == "reuse":
    reuse(args.version, args.source)
  else:
    video(args.tasks, args.versions)
