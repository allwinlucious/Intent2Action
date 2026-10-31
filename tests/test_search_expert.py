import argparse
import json

import pytest

from learn import families, search_expert, strategies
from learn.episodes import Episode


OBJECTS = [{"id": "obj_12", "category": "cube", "color": "red", "size": "medium", "x": 0.5, "y": 0.1, "z": 0.0,
            "yaw": 0.0, "sx": 0.05, "sy": 0.05, "sz": 0.05, "attached": False}]


def episode(task, program, strategy, ok):
  return Episode(task.family, task.seed, task.split, strategy, program, ok, ok, ok, "", {}, None, None, [], 2, 0.5,
                 [{"kind": "done"}], OBJECTS)


@pytest.fixture
def fake_runs(monkeypatch):
  """Strategy -> ok for the fake run_episode; the strategies it ran, in order."""
  outcomes, ran = {}, []

  def run_episode(task, program, sim_stub, robot_stub, agent=None, strategy=None):
    del sim_stub, robot_stub, agent
    ran.append(strategy)
    return episode(task, program, strategy, outcomes.get(strategy, False))

  monkeypatch.setattr(search_expert, "run_episode", run_episode)
  return outcomes, ran


def test_solve_runs_listed_first_and_stops_at_first_success(fake_runs):
  outcomes, ran = fake_runs
  outcomes.update(most_free_first=True, outer_first=True)
  task = families.generate("cluster", 0)
  value = search_expert.task_record(task, search_expert.solve(task, None, None))
  assert ran == ["listed", "most_free_first"]
  assert value["expert_strategy"] == "most_free_first"
  assert value["expert_program"] == strategies.program(task.intent, "most_free_first")
  assert [item["strategy"] for item in value["episodes"]] == ran
  assert all("events" not in item for item in value["episodes"])
  assert value["objects"] == OBJECTS and value["episodes"][0]["objects"] == OBJECTS
  json.dumps(value)


def test_solve_resumes_and_reruns_records_without_objects(fake_runs, tmp_path, capsys):
  _, ran = fake_runs
  out = tmp_path / "train.jsonl"
  old = search_expert.task_record(families.generate("cluster", 1), [episode(families.generate("cluster", 1), "",
                                                                             "listed", True)])
  kept = dict(old, seed=0)
  del old["objects"]
  out.write_text(json.dumps(kept) + "\n" + json.dumps(old) + "\n")
  args = argparse.Namespace(split="train", n=3, out=str(out))
  search_expert._run_solve(args, ["cluster"], None, None)
  values = [json.loads(line) for line in out.read_text().splitlines()]
  assert [value["seed"] for value in values] == [0, 1, 1, 2]
  assert ran.count("listed") == 2  # seeds 1 (no objects) and 2
  assert "cluster: tasks=3 " in capsys.readouterr().out


def test_solve_skips_top_first_without_stacks_and_records_no_expert(fake_runs):
  _, ran = fake_runs
  task = families.generate("cluster", 0)
  assert not search_expert.stacked(task)
  value = search_expert.task_record(task, search_expert.solve(task, None, None))
  assert ran == ["listed", "most_free_first", "outer_first", "nearest_target_first"]
  assert value["expert_strategy"] is None and value["expert_program"] is None


def test_solve_runs_top_first_with_stacks(fake_runs):
  _, ran = fake_runs
  task = families.generate("stacking", 0)
  assert search_expert.stacked(task)
  search_expert.solve(task, None, None)
  assert ran == list(strategies.STRATEGIES)


def test_recovery_writes_only_confirmed_examples(monkeypatch, tmp_path, capsys):
  task = families.generate("cluster", 0)
  # seed -> (handoff call or None, episode ok)
  plans = {0: ("pick(obj_12)", True), 1: ("pick(obj_12)", False), 2: (None, False), 3: ("place(obj_12, obj_13)", True)}

  def run_episode(task, program, sim_stub, robot_stub, agent=None, strategy=None):
    del sim_stub, robot_stub
    call, ok = plans[task.seed]
    if call is not None:
      decision = agent.recover({"kind": "grasp", "call": call, "objects": [], "_backend": object()})
      assert decision["patch"] == expert
    return episode(task, program, strategy, ok)

  monkeypatch.setattr(search_expert, "run_episode", run_episode)
  expert = strategies.program(task.intent, "outer_first")
  source = tmp_path / "train.jsonl"
  lines = []
  for seed in (0, 1, 2, 3, 4):
    value = search_expert.task_record(
      families.Task(task.family, seed, task.split, task.intent, task.placements, 0),
      [episode(task, strategies.program(task.intent, "listed"), "listed", seed == 4),
       episode(task, expert, "outer_first", True)])
    lines.append(json.dumps(value))
  source.write_text("\n".join(lines) + "\n")
  out = tmp_path / "recovery_train.jsonl"
  args = argparse.Namespace(split="train", n=20, input=str(source), out=str(out))
  search_expert._run_recovery(args, ["cluster"], None, None)
  assert "cluster: tasks=4 confirmed=2 not_confirmed=1 no_handoff=1" in capsys.readouterr().out
  examples = [json.loads(line) for line in out.read_text().splitlines()]
  assert [example["seed"] for example in examples] == [0, 3]  # 3: a failed place, the patch places the held cube
  example = examples[0]
  assert "_backend" not in example["context"] and example["context"]["call"] == "pick(obj_12)"
  assert example["confirmed"] is True and "events" not in example["episode"]
  assert example["decision"] == {"rung": "patch", "reason": "grasp failure at pick(obj_12): switch to the outer_first "
                                                            "order", "patch": expert}
  search_expert._run_recovery(args, ["cluster"], None, None)  # resume: the confirmed seeds are not run again
  assert "cluster: tasks=2 confirmed=0 not_confirmed=1 no_handoff=1" in capsys.readouterr().out
  assert len(out.read_text().splitlines()) == 2
