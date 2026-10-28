"""Stage-1 call-validity benchmark (offline, no sim): held-out situations from stage1.py, scored by rule.

  .venv-ml/bin/python learn/bench.py --policy qwen|lora:<dir>|head:<dir> --data logs/learn/stage1_bench.jsonl --out <name>

Three cases per record, as in sft.stage1_examples:
  first     intent -> the policy's call must match the call the stack accepted
  recovery  intent + a wrong call + the stack's error -> the corrected call must match
  refuse    infeasible intent + its call's error -> the policy must stop calling tools
Match: same tool; every number within tolerance (5 mm / 0.02 rad / 5 % for others; quaternions up to sign);
ids, enums and keys equal (missing optional keys count as their defaults).
"""

import argparse
import collections
import json

import env
import run
import sft

POSITION_TOL, ANGLE_TOL, REL_TOL = 0.005, 0.02, 0.05


def _numbers_match(a, b, key: str) -> bool:
  if key in ("x", "y", "z"):
    return abs(a - b) <= POSITION_TOL
  if key == "positions" or key.startswith("q"):
    return abs(a - b) <= ANGLE_TOL
  return abs(a - b) <= max(REL_TOL * abs(b), 1e-6)


def _quat_flip(p: dict) -> dict:
  return {**p, **{k: -p.get(k, 0.0) for k in ("qx", "qy", "qz", "qw")}}


def match(got, want, key: str = "") -> bool:
  if isinstance(want, dict):
    if not isinstance(got, dict) or set(got) - set(want) - {"wait"}:
      return False
    if set(want) >= {"qx", "qy", "qz", "qw"} and not all(match(got.get(k, 0.0), v, k) for k, v in want.items()):
      return all(match(got.get(k, 0.0), v, k) for k, v in _quat_flip(want).items())
    return all(match(got.get(k, 0.0 if isinstance(v, (int, float)) else None), v, k) for k, v in want.items())
  if isinstance(want, list):
    return isinstance(got, list) and len(got) == len(want) and all(match(g, w, key) for g, w in zip(got, want))
  if isinstance(want, bool) or isinstance(want, str):
    return got == want
  if isinstance(want, (int, float)):
    return isinstance(got, (int, float)) and not isinstance(got, bool) and _numbers_match(got, want, key)
  return got == want


def cases(record: dict) -> list[tuple[str, list[dict], dict | None]]:
  """(kind, messages before the scored turn, wanted call or None = must stop)."""
  out = []
  for messages, trained in sft.stage1_examples(record):
    i = trained[0]
    kind = "refuse" if record["infeasible"] else ("recovery" if len(messages) > 7 else "first")
    if kind == "refuse":
      out.append((kind, messages[:trained[1]], None))
    else:
      f = messages[i]["tool_calls"][0]["function"]
      out.append((kind, messages[:i], {"name": f["name"], "arguments": f["arguments"]}))
  return out


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--policy", required=True)
  parser.add_argument("--data", required=True)
  parser.add_argument("--out", required=True)
  args = parser.parse_args()
  policy = run.make_policy(args.policy)
  import agent
  tools = agent.tool_specs()
  score = collections.defaultdict(lambda: [0, 0])
  with open(args.data) as f, open(env.LOG_DIR / f"{args.out}.jsonl", "w") as out:
    for line in f:
      record = json.loads(line)
      for kind, messages, want in cases(record):
        policy.reset(None)
        text, calls = policy.act(messages, tools)
        if want is None:
          ok = not calls
        else:
          ok = bool(calls) and calls[0]["name"] == want["name"] and match(calls[0]["arguments"], want["arguments"])
        for key in (kind, f"{kind}/{record['family']}"):
          score[key][0] += ok; score[key][1] += 1
        out.write(json.dumps({"kind": kind, "family": record["family"], "ok": ok, "want": want,
                              "got": calls[:1], "text": text[:300]}) + "\n")
  summary = {k: f"{a}/{n} ({a / n:.0%})" for k, (a, n) in sorted(score.items())}
  print(json.dumps(summary, indent=1))
  (env.LOG_DIR / f"{args.out}_summary.json").write_text(json.dumps({k: v for k, v in score.items()}, indent=1))


if __name__ == "__main__":
  assert match({"pose": {"x": 0.501, "y": 0.1, "z": 0.2, "qx": 0, "qy": -1, "qz": 0, "qw": 0}},
               {"pose": {"x": 0.5, "y": 0.1, "z": 0.2, "qx": 0.0, "qy": 1.0, "qz": 0.0, "qw": 0.0}})
  assert not match({"pose": {"x": 500, "y": 0.1, "z": 0.2}}, {"pose": {"x": 0.5, "y": 0.1, "z": 0.2}})
  assert not match({"id": "obj_1"}, {"object_id": "obj_1"})
  assert match({"pose": {"x": 0.5, "y": 0, "z": 0.1}, "motion": "LINEAR"}, {"pose": {"x": 0.5, "y": 0.0, "z": 0.1}, "motion": "LINEAR"})
  main()
