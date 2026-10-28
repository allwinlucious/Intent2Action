"""Runs the experiment queue around the owner's hours: nothing runs from 17:00 to 00:00 or while
logs/learn/PAUSE exists. A running step is stopped (SIGTERM, then cleanup of the container and Isaac) when
the window starts, and resumed afterwards; every step resumes from its own files.

  setsid nohup .venv-ml/bin/python learn/supervisor.py > logs/learn/supervisor.out 2>&1 &

State: logs/learn/done/<step> (finished), logs/learn/failed/<step>, logs/learn/steps/<step>.log (output),
logs/learn/supervisor.log (what it did, when).
"""

import datetime
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT / "logs/learn"
PY = str(ROOT / ".venv-ml/bin/python")
BLOCKED_HOURS = range(17, 24)  # the owner uses the computer 17:00-00:00
PAUSE = LOG / "PAUSE"
STOP_GRACE_S = 150
RETRIES = 2
S1, BENCH, S2 = "logs/learn/stage1_train.jsonl", "logs/learn/stage1_bench.jsonl", "logs/learn/expert_train.jsonl"
EPOCHS = os.environ.get("SFT_EPOCHS", "3")


def _py(*args: str) -> list[str]:
  return [PY, *args]


def _policy_steps(name: str, policy: str) -> list[tuple[str, list[str]]]:
  return [(f"bench_{name}", _py("learn/bench.py", "--policy", policy, "--data", BENCH, "--out", f"bench_{name}")),
          (f"eval_{name}", _py("learn/run.py", "--policy", policy, "--split", "eval", "--count", "30", "--out", f"eval_{name}"))]


STEPS = [
    ("collect_stage1", _py("learn/stage1.py", "--intents", "2500", "--seed", "1", "--out", "stage1_train")),
    ("expert_eval", _py("learn/run.py", "--policy", "expert", "--split", "eval", "--count", "30", "--out", "expert_eval")),
    ("expert_train", _py("learn/run.py", "--policy", "expert", "--split", "train", "--count", "200", "--out", "expert_train")),
    *_policy_steps("qwen", "qwen"),
    ("train_b_sft", _py("learn/train_trl.py", "sft", "--stage1", S1, "--epochs", EPOCHS, "--out", "runs/b_sft")),
    *_policy_steps("b_sft", "lora:runs/b_sft"),
    ("train_b_dpo", _py("learn/train_trl.py", "dpo", "--stage1", S1, "--init", "runs/b_sft", "--out", "runs/b_dpo")),
    *_policy_steps("b_dpo", "lora:runs/b_dpo"),
    ("train_b_sft2", _py("learn/train_trl.py", "sft", "--stage1", S1, "--stage2", S2, "--init", "runs/b_sft",
                         "--epochs", "1", "--out", "runs/b_sft2")),
    *_policy_steps("b_sft2", "lora:runs/b_sft2"),
    ("feat_s1", _py("learn/head.py", "featurize", "--stage1", S1, "--out", "runs/feat_s1.pt")),
    ("feat_s2", _py("learn/head.py", "featurize", "--stage2", S2, "--out", "runs/feat_s2.pt")),
    ("train_c1", _py("learn/head.py", "train", "--features", "runs/feat_s1.pt", "--out", "runs/c1")),
    ("train_c2", _py("learn/head.py", "train", "--features", "runs/feat_s2.pt", "--replay-from", "runs/feat_s1.pt",
                     "--init", "runs/c1", "--out", "runs/c2")),
    *_policy_steps("c1", "head:runs/c1"),
    *_policy_steps("c2", "head:runs/c2"),
    ("train_b_grpo", _py("learn/train_trl.py", "grpo", "--stage1", S1, "--init", "runs/b_sft", "--max-steps", "150",
                         "--out", "runs/b_grpo")),
    *_policy_steps("b_grpo", "lora:runs/b_grpo"),
]


def log(message: str) -> None:
  line = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {message}"
  print(line, flush=True)
  with open(LOG / "supervisor.log", "a") as f:
    f.write(line + "\n")


def blocked() -> bool:
  return datetime.datetime.now().hour in BLOCKED_HOURS or PAUSE.exists()


def _gpu() -> str:
  return subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader"],
                        capture_output=True, text=True).stdout.strip()


def cleanup() -> None:
  """Removes the container and stops any Isaac this pipeline started (checked by image name, not just PID)."""
  subprocess.run(["sg", "docker", "-c", "docker rm -f intrinsic"], capture_output=True)
  for path in LOG.glob("*_isaac.log"):
    for pid in re.findall(r"READY pid=(\d+)", path.read_text(errors="replace")):
      task = subprocess.run(["tasklist.exe", "/FI", f"PID eq {pid}", "/NH"], capture_output=True, text=True).stdout
      if "python" in task.lower():
        subprocess.run(["taskkill.exe", "/PID", pid, "/T", "/F"], capture_output=True)
        log(f"stopped Isaac pid {pid}")


def stop(process: subprocess.Popen) -> None:
  os.killpg(process.pid, signal.SIGTERM)
  try:
    process.wait(STOP_GRACE_S)
  except subprocess.TimeoutExpired:
    os.killpg(process.pid, signal.SIGKILL)
    process.wait()
  cleanup()
  log(f"stopped; GPU now {_gpu()}")


def main() -> None:
  for d in ("done", "failed", "steps"):
    (LOG / d).mkdir(parents=True, exist_ok=True)
  log(f"start: {len(STEPS)} steps, blocked {BLOCKED_HOURS.start}:00-{BLOCKED_HOURS.stop}:00 or while {PAUSE} exists")
  for name, command in STEPS:
    if (LOG / "done" / name).exists() or (LOG / "failed" / name).exists():
      continue
    attempts = 0
    while True:
      while blocked():
        time.sleep(30)
      log(f"run {name} (attempt {attempts + 1}): {' '.join(command[1:])}")
      with open(LOG / "steps" / f"{name}.log", "a") as out:
        process = subprocess.Popen(command, cwd=ROOT, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
        while process.poll() is None:
          if blocked():
            log(f"window/pause: stopping {name}")
            stop(process)
            break
          time.sleep(10)
      if process.returncode == 0:
        (LOG / "done" / name).touch()
        log(f"done {name}")
        break
      if process.returncode in (143, -15, -9):  # stopped by us: resume later, not a failure
        continue
      attempts += 1
      cleanup()
      log(f"{name} exited {process.returncode}")
      if attempts > RETRIES:
        (LOG / "failed" / name).touch()
        log(f"FAILED {name}; continuing with the next step")
        break
  log("queue finished")
  subprocess.run(_py("learn/report.py", "--runs", "expert_eval", *[f"eval_{n}" for n in
                     ("qwen", "b_sft", "b_dpo", "b_sft2", "b_grpo", "c1", "c2")],
                     "--benches", *[f"bench_{n}" for n in ("qwen", "b_sft", "b_dpo", "b_sft2", "b_grpo", "c1", "c2")]),
                 cwd=ROOT, stdout=open(LOG / "report.md", "w"))
  log("report written to logs/learn/report.md")


if __name__ == "__main__":
  sys.exit(main())
