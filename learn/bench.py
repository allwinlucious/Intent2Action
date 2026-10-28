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
import sys

import env
import run
import sft

POSITION_TOL, ANGLE_TOL, REL_TOL = 0.005, 0.02, 0.05
DEFAULTS = (0, 0.0, "", False, None, "TCP", "ANY", [], {})  # explicit defaults are as good as omitted keys


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
    if not isinstance(got, dict) or any(k != "wait" and got[k] not in DEFAULTS for k in set(got) - set(want)):
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


def call_ok(got: dict, want: dict) -> bool:
  if got["name"] != want["name"]:
    return False
  if want["name"] == "move_to_contact":  # the intent names no speed: any speed the stack accepts will do
    force, speed = got["arguments"].get("max_force", 0), got["arguments"].get("speed", 0)
    if isinstance(force, (int, float)) and isinstance(speed, (int, float)) and 0.00015 * force <= speed <= 0.002 * force:
      return match({**got["arguments"], "speed": want["arguments"]["speed"]}, want["arguments"])
  return match(got["arguments"], want["arguments"])


def rescore(path) -> dict:
  score = collections.defaultdict(lambda: [0, 0])
  for line in open(path):
    r = json.loads(line)
    ok = (not r["got"]) if r["want"] is None else bool(r["got"]) and call_ok(r["got"][0], r["want"])
    for key in (r["kind"], f"{r['kind']}/{r['family']}"):
      score[key][0] += ok; score[key][1] += 1
  return score


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
  path = env.LOG_DIR / f"{args.out}.jsonl"
  skip = sum(1 for _ in open(path)) if path.exists() else 0  # resume after a stop
  score = collections.defaultdict(lambda: [0, 0])
  n = 0
  with open(args.data) as f, open(path, "a") as out:
    for line in f:
      record = json.loads(line)
      for kind, messages, want in cases(record):
        n += 1
        if n <= skip:
          continue
        policy.reset(None)
        text, calls = policy.act(messages, tools)
        if want is None:
          ok = not calls
        else:
          ok = bool(calls) and call_ok(calls[0], want)
        for key in (kind, f"{kind}/{record['family']}"):
          score[key][0] += ok; score[key][1] += 1
        out.write(json.dumps({"kind": kind, "family": record["family"], "ok": ok, "want": want,
                              "got": calls[:1], "text": text[:300]}) + "\n")
        out.flush()
  score = rescore(env.LOG_DIR / f"{args.out}.jsonl")
  summary = {k: f"{a}/{n} ({a / n:.0%})" for k, (a, n) in sorted(score.items())}
  print(json.dumps(summary, indent=1))
  (env.LOG_DIR / f"{args.out}_summary.json").write_text(json.dumps({k: v for k, v in score.items()}, indent=1))


if __name__ == "__main__":
  assert match({"pose": {"x": 0.501, "y": 0.1, "z": 0.2, "qx": 0, "qy": -1, "qz": 0, "qw": 0}},
               {"pose": {"x": 0.5, "y": 0.1, "z": 0.2, "qx": 0.0, "qy": 1.0, "qz": 0.0, "qw": 0.0}})
  assert not match({"pose": {"x": 500, "y": 0.1, "z": 0.2}}, {"pose": {"x": 0.5, "y": 0.1, "z": 0.2}})
  assert not match({"id": "obj_1"}, {"object_id": "obj_1"})
  assert match({"mass": 0.2, "com_x": 0.0}, {"mass": 0.2})
  assert call_ok({"name": "move_to_contact", "arguments": {"dz": -1.0, "max_force": 5.0, "speed": 0.01}},
                 {"name": "move_to_contact", "arguments": {"dx": 0.0, "dy": 0.0, "dz": -1.0, "max_force": 5.0, "speed": 0.003}})
  if sys.argv[1:2] == ["--rescore"]:
    print(json.dumps({k: f"{a}/{n}" for k, (a, n) in sorted(rescore(sys.argv[2]).items()) if "/" not in k}))
    sys.exit()
  assert match({"pose": {"x": 0.5, "y": 0, "z": 0.1}, "motion": "LINEAR"}, {"pose": {"x": 0.5, "y": 0.0, "z": 0.1}, "motion": "LINEAR"})
  main()
