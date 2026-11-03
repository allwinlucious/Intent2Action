"""Replies from a hosted model API for the held-out evaluation, the same requests every version gets.

  .venv/bin/python learn/api_replies.py --model muse-spark-1.3 --version SPARK13 [--limit 3]
  then: learn/offline_eval.py programs --version SPARK13 --replies logs/eval/SPARK13_replies.jsonl

Sends logs/eval/requests.jsonl (the product's writer prompt per held-out task) to META_API_BASE (default
https://api.meta.ai/v1) with temperature 0 and one attempt, like the evaluated versions. Resumes: tasks already in
the replies file are skipped. Spend is computed from each response's token usage at the listed USD prices and kept
in logs/eval/api_spend.json across runs; a request is refused once the total would pass META_API_BUDGET (USD,
default 130).
"""

import argparse
import fcntl
import json
import os
from pathlib import Path
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT / "logs/eval"
LEDGER = LOGS / "api_spend.json"
# USD per 1M tokens (input, output), dev.meta.ai pricing page, 2026-11-03.
PRICES = {"muse-wonder-1.0": (4.00, 20.00), "muse-spark-1.3": (1.25, 4.25), "muse-spark-1.2": (1.25, 4.25),
          "muse-spark-1.1": (1.25, 4.25)}
WORKERS = 8
_lock = threading.Lock()


def _env() -> dict:
  values = dict(os.environ)
  env_file = ROOT / ".env"
  if env_file.exists():
    for line in env_file.read_text().splitlines():
      if "=" in line and not line.lstrip().startswith("#"):
        key, value = line.split("=", 1)
        values.setdefault(key.strip(), value.strip().strip("'\""))
  return values


def cost(model: str, usage: dict) -> float:
  price_in, price_out = PRICES[model]
  return (usage.get("prompt_tokens", 0) * price_in + usage.get("completion_tokens", 0) * price_out) / 1e6


def _ledger(update=None) -> float:
  """Reads (and with `update`, changes) the spend ledger under a file lock: several runs share it."""
  with open(LEDGER.with_suffix(".lock"), "w") as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    ledger = json.loads(LEDGER.read_text()) if LEDGER.exists() else {"total_usd": 0.0, "by_model": {}}
    if update:
      model, amount = update
      ledger["total_usd"] += amount
      ledger["by_model"][model] = ledger["by_model"].get(model, 0.0) + amount
      LEDGER.write_text(json.dumps(ledger, indent=1))
    return ledger["total_usd"]


def _spent() -> float:
  return _ledger()


def _record(model: str, amount: float) -> float:
  return _ledger((model, amount))


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--model", required=True, choices=sorted(PRICES))
  parser.add_argument("--version", required=True)
  parser.add_argument("--limit", type=int, default=0, help="only the first N requests (a cost probe)")
  args = parser.parse_args()
  env = _env()
  key, base = env["META_API_KEY"], env.get("META_API_BASE", "https://api.meta.ai/v1").rstrip("/")
  budget = float(env.get("META_API_BUDGET", "130"))
  out_path = LOGS / f"{args.version}_replies.jsonl"
  done = {json.loads(line)["key"] for line in out_path.read_text().splitlines()} if out_path.exists() else set()
  requests = [json.loads(line) for line in (LOGS / "requests.jsonl").read_text().splitlines()]
  todo = [r for r in requests if r["key"] not in done][: args.limit or None]
  print(f"{args.version} ({args.model}): {len(done)} done, {len(todo)} to request; spent so far ${_spent():.2f} "
        f"of ${budget:.0f}", flush=True)
  stop = threading.Event()

  def work(row) -> None:
    if stop.is_set():
      return
    with _lock:
      if _spent() >= budget:
        stop.set()
        print(f"budget reached (${_spent():.2f} >= ${budget:.0f}): stopping", flush=True)
        return
    body = json.dumps({"model": args.model, "messages": row["messages"], "temperature": 0}).encode()
    request = urllib.request.Request(f"{base}/chat/completions", data=body, method="POST", headers={
        "Authorization": f"Bearer {key}", "Content-Type": "application/json"})
    try:
      with urllib.request.urlopen(request, timeout=600) as response:
        value = json.load(response)
    except Exception as error:  # pylint: disable=broad-except
      print(f"ERROR {row['key']}: {str(error)[:200]} (retried on the next run)", flush=True)
      return
    usage = value.get("usage") or {}
    amount = cost(args.model, usage)
    text = value["choices"][0]["message"].get("content") or ""
    with _lock:
      with out_path.open("a") as out:
        out.write(json.dumps({"key": row["key"], "text": text, "usage": usage, "usd": amount}) + "\n")
      total = _record(args.model, amount)
    print(f"OK {row['key']} in {usage.get('prompt_tokens')} out {usage.get('completion_tokens')} "
          f"${amount:.4f} (total ${total:.2f})", flush=True)

  with ThreadPoolExecutor(max_workers=WORKERS) as pool:
    list(pool.map(work, todo))
  replies = out_path.read_text().splitlines() if out_path.exists() else []
  print(f"{out_path}: {len(replies)}/{len(requests)} replies; spent ${_spent():.2f}", flush=True)
  sys.exit(0 if len(replies) == len(requests) or args.limit else 1)


if __name__ == "__main__":
  assert abs(cost("muse-spark-1.3", {"prompt_tokens": 1_000_000, "completion_tokens": 1_000_000}) - 5.5) < 1e-9
  main()
