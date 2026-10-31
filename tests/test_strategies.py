import ast

import pytest

from learn import families, strategies
from learn.glimmer_program_test import truth
from program_runtime.api import FakeBackend, RobotAPI
from program_runtime.check import check
from program_runtime.rules import check_intent
from program_runtime.runtime import run


def _split(family):
  return "heldout" if family == "mixed" else "train"


def _cubes(task):
  return sum(placement.catalog in families.CUBE_CATALOGS for placement in task.placements)


@pytest.fixture(scope="module")
def tasks():
  return {family: families.tasks(family, _split(family), 20) for family in families.FAMILIES}


def _run(task, program, log=lambda event: None):
  objects = families.fake_objects(task)
  backend = FakeBackend(objects)
  result = run(program, RobotAPI(backend), log=log)
  rules = check_intent(task.intent, truth(objects), truth(list(backend.objects.values())))
  return result, rules


@pytest.mark.parametrize("strategy", strategies.STRATEGIES)
@pytest.mark.parametrize("family", families.FAMILIES)
def test_programs_pass_check_and_achieve_intent(tasks, family, strategy):
  for task in tasks[family]:
    program = strategies.program(task.intent, strategy)
    assert check(program) == [], (task.seed, program)
    assert "obj_" not in program
    assert not [node for node in ast.walk(ast.parse(program))
                if isinstance(node, ast.Constant) and isinstance(node.value, float)]
    result, rules = _run(task, program)
    assert result.ok, (task.seed, result.error)
    assert rules.ok, (task.seed, rules.summary, rules.per_object)


def test_most_free_first_takes_an_outer_cube_of_a_plus():
  pitch = 0.05 + 0.015
  cubes = [families.Placement("red_cube", 0.5 + dx * pitch, 0.1 + dy * pitch, 0.0, 0.0)
           for dx, dy in families.SHAPES["plus"]]
  intent = {"description": "every cube goes into the red box", "others": "stay",
            "rules": [{"select": {"category": "cube"}, "relation": "in", "target": {"category": "box", "color": "red"}}]}
  task = families.Task("cluster", 0, "train", intent,
                       (*cubes, families.Placement("red_box", 0.4, -0.27, 0.0, 0.0)), 0)
  ids = [obj.id for obj in families.fake_objects(task)]
  centre, outer = ids[0], set(ids[1:5])
  events = []
  result, rules = _run(task, strategies.program(intent, "most_free_first"), events.append)
  picks = [event["args"][0] for event in events if event["kind"] == "call" and event["fn"] == "pick"]
  assert result.ok and rules.ok
  assert picks[0] in outer and picks[0] != centre


@pytest.mark.parametrize("family", families.FAMILIES)
def test_largest_tasks_run_within_default_budget(family):
  generated = families.tasks(family, _split(family), 100)
  most = max(_cubes(task) for task in generated)
  largest = [task for task in generated if _cubes(task) == most]
  if family == "cluster":
    assert most == max(high for _, high in families.SHAPE_COUNTS.values())
  for task in largest:
    for strategy in strategies.STRATEGIES:
      result, rules = _run(task, strategies.program(task.intent, strategy))
      assert result.ok and rules.ok, (task.seed, strategy, result.error, rules.summary)
