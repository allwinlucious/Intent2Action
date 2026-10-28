"""Runs one batch of training jobs in a Kaggle notebook (2 x T4): one lane of commands per GPU, in parallel.

The bundle (a private Kaggle dataset built by kaggle_lane.py) holds jobs.json, the training code and the
data. Outputs (adapters, checkpoints, feature shards, heads, logs) go to /kaggle/working/out, which
kaggle_lane.py downloads. A lane still running after DEADLINE_H is stopped, so the session ends normally and
its checkpoints are kept; the next batch run resumes from them.
"""

import glob
import json
import os
import shutil
import subprocess
import sys
import threading
import time

DEADLINE_H = 11.0  # Kaggle ends sessions at 12 h
PIP = ["trl==1.16.0", "peft==0.21.2", "transformers==5.21.0", "bitsandbytes", "accelerate", "datasets"]
MODEL = "Qwen/Qwen3-8B"

start = time.monotonic()
bundle = os.path.dirname(glob.glob("/kaggle/input/**/jobs.json", recursive=True)[0])
work, out = "/kaggle/working/w", "/kaggle/working/out"
shutil.copytree(bundle, work)
os.makedirs(f"{out}/logs", exist_ok=True)
subprocess.run([sys.executable, "-m", "pip", "install", "-q", *PIP], check=True)
env = {**os.environ, "LEARN_DTYPE": "float16", "LEARN_LIGER": "0", "HF_HOME": "/tmp/hf", "PYTHONUNBUFFERED": "1"}
subprocess.run([sys.executable, "-c", f"from huggingface_hub import snapshot_download; snapshot_download('{MODEL}')"],
               env=env, check=True)  # once, before both lanes load it
jobs = json.load(open(f"{work}/jobs.json"))
status = {}


def lane(gpu: str, commands: list) -> None:
  for command, skip_if in commands:
    name = " ".join(command)
    if skip_if and os.path.exists(f"{work}/{skip_if}"):
      status[name] = "skipped (already done)"
      continue
    remaining = DEADLINE_H * 3600 - (time.monotonic() - start)
    if remaining <= 60:
      status[name] = "not started (deadline)"
      continue
    with open(f"{out}/logs/gpu{gpu}.log", "a") as log:
      log.write(f"\n=== {name}\n")
      log.flush()
      process = subprocess.Popen([sys.executable, *command], cwd=work, stdout=log, stderr=subprocess.STDOUT,
                                 env={**env, "CUDA_VISIBLE_DEVICES": gpu})
      try:
        status[name] = f"exit {process.wait(timeout=remaining)}"
      except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(60)
        status[name] = "stopped at the deadline"
    if not status[name].startswith("exit 0"):
      break  # later commands of this lane depend on this one


threads = [threading.Thread(target=lane, args=(gpu, commands)) for gpu, commands in jobs.items()]
for t in threads:
  t.start()
for t in threads:
  t.join()
if os.path.isdir(f"{work}/runs"):
  shutil.copytree(f"{work}/runs", f"{out}/runs", dirs_exist_ok=True)
json.dump({"status": status, "hours": (time.monotonic() - start) / 3600}, open(f"{out}/status.json", "w"), indent=1)
print(json.dumps(status, indent=1))
