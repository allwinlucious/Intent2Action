from dataclasses import replace

import pytest

from program_runtime.rules import TrueObj, check_intent, validate_intent


def true_obj(object_id, category="cube", color="red", x=0.4, y=0.1, z=0.0):
  size = "large" if category == "box" else "medium"
  sx, sy, sz = ((0.25, 0.21, 0.08) if category == "box" else (0.05, 0.05, 0.05))
  return TrueObj(object_id, category, color, size, x, y, z, 0.0, sx, sy, sz)


def intent():
  return {
    "description": "cubes in red box",
    "rules": [{
      "select": {"category": "cube"}, "relation": "in",
      "target": {"category": "box", "color": "red"},
    }],
    "others": "stay",
  }


@pytest.mark.parametrize("change", [
  {"extra": True},
  {"description": 2},
  {"others": "move"},
  {"rules": []},
  {"rules": [{}]},
  {"rules": [{"select": {"shape": "cube"}, "relation": "in", "target": {"category": "box"}}]},
  {"rules": [{"select": {"size": "tiny"}, "relation": "in", "target": {"category": "box"}}]},
  {"rules": [{"select": {}, "relation": "beside", "target": {"category": "box"}}]},
  {"rules": [{"select": {}, "relation": "in", "target": {}}]},
])
def test_validate_intent_invalid_cases(change):
  value = intent()
  value.update(change)
  assert validate_intent(value)


@pytest.mark.parametrize("change", [
  {"description": None},
  {"others": None},
  {"rules": [{"select": {}, "relation": "on", "target": {"category": "cube", "color": "red"}}]},
  {"rules": [{"select": {"size": "small"}, "relation": "in", "target": {"size": "large"}}]},
  {"rules": [{"select": {"color": color}, "relation": "in", "target": {"category": "box"}}
             for color in ["red", "blue", "green", "yellow", "white", "black"]]},
])
def test_validate_intent_valid_cases(change):
  value = intent()
  value.update(change)
  for key in [key for key, item in change.items() if item is None]:
    del value[key]
  assert validate_intent(value) == []


@pytest.mark.parametrize("change", [
  {"rules": "cube"},
  {"rules": [{"select": {"color": color}, "relation": "in", "target": {"category": "box"}}
             for color in ["red", "blue", "green", "yellow", "white", "black", "pink"]]},
  {"rules": [{"select": "cube", "relation": "in", "target": {"category": "box"}}]},
  {"rules": [{"select": {"color": ""}, "relation": "in", "target": {"category": "box"}}]},
  {"rules": [{"select": {}, "relation": "in", "target": {"category": "box"}, "extra": 1}]},
])
def test_validate_intent_more_invalid_cases(change):
  value = intent()
  value.update(change)
  assert validate_intent(value)


def test_validate_intent_valid_and_duplicate():
  assert validate_intent(intent()) == []
  value = intent()
  value["rules"] = value["rules"] * 2
  assert any("duplicates" in problem for problem in validate_intent(value))


def scene():
  before = {
    "a": true_obj("a", x=0.4),
    "b": true_obj("b", color="blue", x=0.55),
    "box": true_obj("box", "box", "red", 0.45, -0.22),
    "c": true_obj("c", "cylinder", "green", 0.6, 0.1),
  }
  after = dict(before)
  after["a"] = replace(before["a"], x=0.42, y=-0.22, z=0.01)
  after["b"] = replace(before["b"], x=0.48, y=-0.22, z=0.01)
  return before, after


def test_rule_checker_all_cubes_inside():
  before, after = scene()
  assert check_intent(intent(), before, after).ok


def test_rule_checker_cube_outside():
  before, after = scene()
  after["a"] = before["a"]
  result = check_intent(intent(), before, after)
  assert not result.ok and result.per_object["a"] == "not in target"


def test_rule_checker_unselected_object_moved():
  before, after = scene()
  after["c"] = replace(before["c"], x=before["c"].x + 0.05)
  result = check_intent(intent(), before, after)
  assert result.per_object["c"] == "moved"


def test_rule_checker_missing_target_is_invalid_for_scene():
  before, after = scene()
  del before["box"]
  del after["box"]
  assert check_intent(intent(), before, after).invalid_for_scene
