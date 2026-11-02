import importlib
import json
import sys
import types

import pytest

from learn import families, sft_data
from mcp_server.server import DESCRIPTOR_SET, Tools
from program_runtime import agent
from program_runtime.api import Obj

OBJECTS = [{"id": "obj_12", "category": "cube", "color": "red", "size": "medium", "x": 0.5, "y": 0.1, "z": 0.0,
            "yaw": 0.0, "sx": 0.05, "sy": 0.05, "sz": 0.05, "attached": False}]
CONTEXT = {"program": "1: pick(cube)", "line": 1, "kind": "grasp", "message": "no feasible grasp", "call": "pick(obj_12)",
           "objects": OBJECTS, "state": {}, "history": [], "budget": {"handoffs": 2}}
DECISION = {"rung": "patch", "reason": "grasp failure", "patch": "verify(lambda: True)\n"}


def program(seed, name="p"):
  return f"{name} = {seed}\nverify(lambda: True)\n"


def test_program_row_is_writer_prompt_and_fenced_program():
  intent = families.generate("cluster", 0).intent
  objects = [Obj(**obj) for obj in OBJECTS]
  row = sft_data.program_row(intent, objects, program(0))
  assert row["messages"][:-1] == agent.program_messages(intent, objects)
  assert row["messages"][-1] == {"role": "assistant", "content": "```python\n" + program(0) + "```"}
  assert agent._program_text(row["messages"][-1]["content"]) == program(0)


def test_recovery_row_prompt_equals_what_the_agent_sends(monkeypatch):
  captured = []

  def chat(self, messages, tools=None, tool_choice=None, model=None):
    del model
    captured.append((list(messages), tools))
    return {"content": json.dumps(DECISION)}

  monkeypatch.setattr(agent.ChatClient, "chat", chat)
  tools = Tools(DESCRIPTOR_SET)
  decision = agent.RecoveryAgent(agent.ChatClient("http://model", "key", "glimmer"), object(), tools).recover(
      {**CONTEXT, "_backend": object()})
  row = sft_data.recovery_row({"context": CONTEXT, "decision": DECISION}, agent.tool_specs(tools))
  assert decision == DECISION
  assert (row["messages"][:-1], row["tools"]) == captured[0]
  assert row["messages"][-1] == {"role": "assistant", "content": json.dumps(DECISION)}


def solve_record(seed, expert, objects=True):
  value = {"family": "cluster", "seed": seed, "split": "train", "intent": families.generate("cluster", seed).intent,
           "expert_program": expert, "episodes": []}
  return value | ({"objects": OBJECTS} if objects else {})


def write(path, values):
  path.write_text("".join(json.dumps(value) + "\n" for value in values))
  return path


@pytest.fixture
def sources(tmp_path):
  solve = write(tmp_path / "train.jsonl", [
      solve_record(0, program(0)), solve_record(1, program(1)), solve_record(2, None),
      solve_record(3, program(3), objects=False),  # written before L4: absent
      solve_record(100000, program(9)) | {"split": "heldout"},
  ])
  samples = write(tmp_path / "train_episodes.jsonl", [
      {"family": "cluster", "seed": 0, "split": "train", "sample": i, "ok": True, "program": program(0, name)}
      for i, name in ((3, "c"), (0, "a"), (1, "a"), (2, "b"))
  ] + [
      {"family": "cluster", "seed": 2, "split": "train", "sample": 1, "ok": False, "program": program(2, "x")},
      {"family": "cluster", "seed": 2, "split": "train", "sample": 3, "ok": True, "program": program(2, "d")},
      {"family": "cluster", "seed": 3, "split": "train", "sample": 0, "ok": True, "program": program(3, "e")},
  ])
  recovery = write(tmp_path / "recovery_train.jsonl", [
      {"family": "cluster", "seed": seed, "split": "train", "context": CONTEXT, "decision": DECISION, "confirmed": True}
      for seed in (0, 1, 2)
  ])
  return solve, samples, recovery, tmp_path


def rows(path):
  return [json.loads(line) for line in path.read_text().splitlines()]


def assistant_texts(out):
  return sorted(row["messages"][-1]["content"] for name in ("train", "validation") for row in rows(out / f"{name}.jsonl"))


def test_build_versions_have_the_right_rows(sources):
  solve, samples, recovery, tmp = sources
  g1 = sft_data.build("G1", solve, samples, recovery, tmp / "G1")
  assert g1["rows"] == {"program": 2, "self": 0, "recovery": 0}
  assert assistant_texts(tmp / "G1") == sorted("```python\n" + program(seed) + "```" for seed in (0, 1))

  g2 = sft_data.build("G2", solve, samples, recovery, tmp / "G2")
  assert g2["rows"] == {"program": 2, "self": 3, "recovery": 0}  # seed 0: a, b (the lowest distinct); seed 2: d
  fenced = ["```python\n" + text + "```" for text in (program(0), program(1), program(0, "a"), program(0, "b"),
                                                       program(2, "d"))]
  assert assistant_texts(tmp / "G2") == sorted(fenced)

  g3 = sft_data.build("G3", solve, samples, recovery, tmp / "G3")
  assert g3["rows"] == {"program": 2, "self": 3, "recovery": 2}  # 3 confirmed, capped: 2 <= 30 % of 7
  assert g3["families"] == {"cluster": g3["rows"]} and g3["fallback"] is None
  recovery_rows = [row for name in ("train", "validation") for row in rows(tmp / "G3" / f"{name}.jsonl")
                   if "tools" in row]
  assert len(recovery_rows) == 2 and recovery_rows[0]["tools"] == agent.tool_specs(Tools(DESCRIPTOR_SET))
  assert all(set(row) == {"messages"} for name in ("train", "validation") for row in rows(tmp / "G2" / f"{name}.jsonl"))
  assert json.loads((tmp / "G3" / "manifest.json").read_text()) == g3


def test_build_without_samples(sources):
  solve, _, recovery, tmp = sources
  missing = tmp / "missing.jsonl"
  with pytest.raises(SystemExit):
    sft_data.build("G2", solve, missing, recovery, tmp / "G2")
  g3 = sft_data.build("G3", solve, missing, recovery, tmp / "G3")
  assert g3["fallback"] and g3["rows"] == {"program": 2, "self": 0, "recovery": 0}  # cap: 3 * 2 // 7 = 0
  assert g3["sources"] == [str(solve), str(recovery)]


@pytest.mark.parametrize("n,expected", [(0, 0), (1, 0), (2, 1), (5, 1), (10, 2), (79, 15), (100, 16), (320, 16),
                                        (321, 17), (1000, 50)])
def test_validation_size(n, expected):
  assert sft_data.validation_size(n) == expected


def test_split_is_deterministic_and_complete():
  train, validation = sft_data.split(range(100))
  assert (train, validation) == sft_data.split(range(100))
  assert len(validation) == 16 and sorted(train + validation) == list(range(100))
  assert validation != list(range(16))


class FakeTokenizer:
  """One token per character; the template is the messages' role:content lines."""

  def apply_chat_template(self, messages, tools=None, add_generation_prompt=False, tokenize=True,
                          reasoning_strength=None):
    assert not tokenize and reasoning_strength == "low"
    text = (f"tools:{len(tools)}\n" if tools else "") + "".join(f"{m['role']}:{m['content']}\n" for m in messages)
    return text + ("assistant:" if add_generation_prompt else "")

  def __call__(self, text):
    return {"input_ids": list(text)}


@pytest.fixture
def render_rows(monkeypatch):
  monkeypatch.setitem(sys.modules, "transformers", types.ModuleType("transformers"))
  monkeypatch.syspath_prepend(str(sft_data.ROOT / "learn"))
  return importlib.import_module("render_rows")


def test_render_splits_prompt_and_reply(render_rows):
  messages = [{"role": "system", "content": "s"}, {"role": "user", "content": "u"},
              {"role": "assistant", "content": "a"}]
  assert render_rows.render(messages, [{"type": "function"}], FakeTokenizer()) == {"segments": [
      {"label": False, "text": "tools:1\nsystem:s\nuser:u\nassistant:"}, {"label": True, "text": "a\n"}]}


def test_render_file_drops_rows_over_the_token_limit(render_rows, tmp_path):
  short = {"messages": [{"role": "user", "content": "u"}, {"role": "assistant", "content": "a"}]}
  long = {"messages": [{"role": "user", "content": "u" * 4096}, {"role": "assistant", "content": "a"}]}
  source = write(tmp_path / "train.jsonl", [short, long, short])
  target = tmp_path / "rendered.jsonl"
  assert render_rows.render_file(source, target, FakeTokenizer()) == {"rows": 2, "dropped_too_long": 1}
  assert len(rows(target)) == 2 and rows(target)[0]["segments"][1] == {"label": True, "text": "a\n"}


def test_label_is_the_most_reliable_successful_strategy():
  from learn import sft_data

  def record(seed, outcomes):
    return {"family": "stacking", "seed": seed, "episodes": [
        {"strategy": name, "ok": ok, "program": f"{name} program"} for name, ok in outcomes]}

  solve = [record(1, [("listed", True), ("top_first", True)]),
           record(2, [("listed", False), ("top_first", True)]),
           record(3, [("listed", False), ("top_first", True)])]
  rates = sft_data.strategy_rates(solve)
  assert rates[("stacking", "listed")] == 1 / 3 and rates[("stacking", "top_first")] == 1.0
  assert sft_data.best_program(solve[0], rates) == "top_first program"
  assert sft_data.best_program(record(4, [("listed", False)]), rates) is None  # no expert_program either
