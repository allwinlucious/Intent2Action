"""Unit tests of learn/push_experience.py's push model (no plan tier needed) and its scenes."""

import json
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "learn"))
import push_experience as pe  # noqa: E402

CONTEXT = {
  "call": "pick(obj_1)",
  "objects": [
    {"id": "obj_1", "category": "crate", "color": "blue", "x": 0.5, "y": 0.2, "z": 0.0, "yaw": 0.0,
     "sx": 0.14, "sy": 0.10, "sz": 0.07},
    {"id": "obj_2", "category": "mat", "color": "blue", "x": 0.5, "y": -0.1, "z": -0.0019, "yaw": 0.0,
     "sx": 0.18, "sy": 0.15, "sz": 0.002},
  ],
  "state": {"tool_pose": {"x": 0.5, "y": 0.3, "z": 0.035, "qx": 0.0, "qy": 1.0, "qz": 0.0, "qw": 0.0}},
}


def _world() -> pe.PushWorld:
  return pe.PushWorld(None, None, CONTEXT, random.Random(0))


def _pose(x, y, z=0.035):
  return {"x": x, "y": y, "z": z, "qx": 0.0, "qy": 1.0, "qz": 0.0, "qw": 0.0}


def test_contact_stops_at_the_face():
  world = _world()
  text = world.move_to_contact({"dx": 0.0, "dy": -1.0, "dz": 0.0, "max_force": 10.0, "speed": 0.01})
  assert json.loads(text)["state"] == "DONE"
  # Face at y = 0.2 + 0.05, fingers 1 cm thick.
  assert world.tcp[1] == pytest.approx(0.26)


def test_compliant_push_slides_the_object_and_settles_short():
  world = _world()
  world.move_to_contact({"dx": 0.0, "dy": -1.0, "dz": 0.0, "max_force": 10.0, "speed": 0.01})
  world.move_compliant({"pose": _pose(0.5, -0.04), "stiffness": 50.0, "max_force": 25.0})
  # 30 cm pushed at 50 N/m: the spring stops about 1.2 / 50 = 2.4 cm short.
  assert world.object_xy[1] == pytest.approx(0.2 - 0.30 + 0.024, abs=0.006)
  assert not world.violations


def test_too_stiff_in_contact_trips_the_force_limit():
  world = _world()
  world.move_to_contact({"dx": 0.0, "dy": -1.0, "dz": 0.0, "max_force": 10.0, "speed": 0.01})
  text = world.move_compliant({"pose": _pose(0.5, -0.04), "stiffness": 300.0, "max_force": 25.0})
  assert "force limit reached" in text and world.violations
  assert world.object_xy == [0.5, 0.2]


def test_free_space_moves_never_trip_and_a_miss_has_no_contact():
  world = _world()
  world.move_compliant({"pose": _pose(0.5, 0.3, 0.15), "stiffness": 300.0, "max_force": 25.0})
  assert not world.violations and world.tcp[2] == pytest.approx(0.15)
  text = world.move_to_contact({"dx": 0.0, "dy": -1.0, "dz": 0.0, "max_force": 10.0, "speed": 0.01})
  assert "no contact" in text  # above the crate


def test_invalid_arguments_get_the_robot_service_errors():
  world = _world()
  assert "stiffness must be in" in world.move_compliant({"pose": _pose(0.5, 0.0), "stiffness": 10.0,
                                                        "max_force": 25.0})
  assert "speed must be between" in world.move_to_contact({"dx": 0.0, "dy": -1.0, "dz": 0.0, "max_force": 10.0,
                                                           "speed": 1.0})


def test_training_and_held_out_scenes_use_different_objects():
  train = {pe.scene(seed)[0].catalog for seed in range(50)} | {pe.scene(seed)[1].catalog for seed in range(50)}
  held_out = {pe.scene(seed)[0].catalog for seed in range(100000, 100050)}
  assert held_out == {"big_carton"} and not train & {"big_carton", "target_zone"}


def test_backing_off_after_a_push_leaves_the_object():
  world = _world()
  world.move_to_contact({"dx": 0.0, "dy": -1.0, "dz": 0.0, "max_force": 10.0, "speed": 0.01})
  world.move_compliant({"pose": _pose(0.5, 0.1), "stiffness": 100.0, "max_force": 25.0})
  pushed = list(world.object_xy)
  world.object_yaw += 0.05  # turned a little: the fingers end a hair inside its outline
  world.move_compliant({"pose": _pose(0.5, world.tcp[1] + 0.03, 0.045), "stiffness": 300.0, "max_force": 25.0})
  assert world.object_xy == pushed and not world.violations
