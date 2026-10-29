"""Scripted expert: the stage-2 demonstrator (no model). It sees only tool results, like a model does.

Written as a generator per family that yields tool calls and receives their parsed results; the Policy
wrapper reads the last tool message from the conversation. Episodes are kept for training only if the
ground-truth check passes.
"""

import json

PLATE_Z = 0.012
LIFT_M = 0.10
CLEAR_M = 0.08
DROP_M = 0.004
PEG_PAYLOAD_KG = 0.05
BLOCK_TOP_Z = 0.05
ABOVE_HOLE_M = 0.02
OFF_AXIS_M = 0.003
CONTACT = {"dx": 0.0, "dy": 0.0, "dz": -1.0, "max_force": 15.0, "speed": 0.01}
DOWN = {"qx": 0.0, "qy": 1.0, "qz": 0.0, "qw": 0.0}
COLORS = ("red", "blue", "green", "yellow", "purple", "orange")


class Failed(Exception):
  pass


def _words(prompt: str) -> list[tuple[str, str]]:
  """(color, category) mentions in prompt order, e.g. [("red", "cube"), ("", "plate")]."""
  text = prompt.lower().replace("grey block", "block_with_hole")
  found = []
  for i, word in enumerate(text.replace(",", " ").replace(".", " ").replace("'s", " ").split()):
    if word in ("cube", "plate", "peg", "block_with_hole"):
      before = text.replace(",", " ").replace(".", " ").replace("'s", " ").split()[max(0, i - 2):i]
      found.append((next((w for w in before if w in COLORS), ""), word))
  return found


def _find(objects: list[dict], color: str, category: str) -> dict | None:
  return next((o for o in objects if o["category"] == category and (not color or o["color"] == color)), None)


def _pose(p: dict, **change) -> dict:
  out = {k: float(p.get(k, 0.0)) for k in ("x", "y", "z", "qx", "qy", "qz", "qw")}
  out.update(change)
  return out


def _move(pose: dict, linear: bool = False):
  args = {"pose": pose}
  if linear:
    args["motion"] = "LINEAR"
  return ("move_to_pose", args)


def _pick(o: dict):
  """Yields the calls of a top-down pick; returns the TCP height above the object's bottom."""
  plan = yield ("plan_grasp", {"object_id": o["id"]})
  yield _move(_pose(plan["pregrasp"]))
  yield _move(_pose(plan["grasp"]), linear=True)
  yield ("grasp", {})
  yield ("attach_object", {"id": o["id"]})
  grasp = plan["grasp"]
  yield _move(_pose(grasp, z=grasp["z"] + LIFT_M), linear=True)
  return plan["grasp"], grasp["z"] - o["pose"]["z"]


def _place(o: dict, grasp: dict, offset: float, x: float, y: float, surface_z: float):
  yield _move(_pose(grasp, x=x, y=y, z=surface_z + offset + CLEAR_M))
  yield _move(_pose(grasp, x=x, y=y, z=surface_z + offset + DROP_M), linear=True)
  yield ("release", {})
  yield ("detach_object", {"id": o["id"]})
  yield _move(_pose(grasp, x=x, y=y, z=surface_z + offset + CLEAR_M), linear=True)


def script(task: dict):
  """Generator of (tool, arguments); receives each parsed result; returns the final message."""
  family, mentions = task["family"], _words(task["prompt"])
  listed = yield ("list_objects", {})
  objects = listed["objects"]
  first = _find(objects, *mentions[0]) if mentions else None
  if family == "infeasible":
    if first is None:
      return f"There is no {mentions[0][0]} {mentions[0][1]} in the scene, so I cannot do that."
    try:
      yield ("plan_grasp", {"object_id": first["id"]})
    except Failed as error:
      return f"I cannot pick up the {first['category']}: {str(error)[:120]}"
    return "The grasp plan unexpectedly succeeded; stopping without moving."
  if family == "move":
    p = first["pose"]
    yield _move({"x": p["x"], "y": p["y"], "z": p["z"] + first["size_z"] + 0.10, **DOWN})
    return f"The TCP is 10 cm above the top of the {first['color']} cube ({first['id']})."
  if family == "insert":
    peg, block = _find(objects, "", "peg"), _find(objects, "", "block_with_hole")
    grasp, offset = yield from _pick(peg)
    yield ("set_payload", {"mass": PEG_PAYLOAD_KG})
    b = block["pose"]
    yield _move(_pose(grasp, x=b["x"] + OFF_AXIS_M, y=b["y"], z=BLOCK_TOP_Z + ABOVE_HOLE_M + offset), linear=True)
    yield ("tare_wrench", {})
    yield ("move_to_contact", dict(CONTACT))
    state = yield ("get_state", {})
    tcp = state["tool_pose"]
    yield ("move_compliant", {"pose": _pose(tcp, x=tcp["x"] - OFF_AXIS_M, z=tcp["z"] - 0.002),
                              "stiffness": 2000.0, "max_force": 30.0})
    yield ("move_to_contact", dict(CONTACT))
    yield ("release", {})
    yield ("detach_object", {"id": peg["id"]})
    yield ("set_payload", {"mass": 0.0})
    return f"The peg ({peg['id']}) is in the hole of the grey block ({block['id']})."
  grasp, offset = yield from _pick(first)
  if family == "lift":
    yield _move(_pose(grasp, z=LIFT_M + offset), linear=True)
    return f"Holding the {first['color']} cube ({first['id']}) about 10 cm above the table."
  if family == "pick_place":
    plate = _find(objects, "", "plate")
    p = plate["pose"]
    yield from _place(first, grasp, offset, p["x"], p["y"], p["z"] + plate["size_z"])
    return f"The {first['color']} cube ({first['id']}) is on the plate ({plate['id']})."
  if family == "stack":
    below = _find(objects, *mentions[1])
    p = below["pose"]
    yield from _place(first, grasp, offset, p["x"], p["y"], p["z"] + below["size_z"])
    return f"The {first['color']} cube ({first['id']}) is on the {below['color']} cube ({below['id']})."
  raise ValueError(family)


class Policy:
  """agent.py policy interface around `script`."""

  name = "expert"

  def reset(self, task) -> None:
    import dataclasses
    self._gen = script(dataclasses.asdict(task))
    self._started = False

  def act(self, messages, tools) -> tuple[str, list[dict]]:
    try:
      if not self._started:
        self._started = True
        name, args = next(self._gen)
      else:
        result = messages[-1]["content"]
        try:
          data = json.loads(result)
        except json.JSONDecodeError:
          data = None
        if data is None or (isinstance(data, dict) and "error" in data):
          name, args = self._gen.throw(Failed(result))
        else:
          name, args = self._gen.send(data)
    except StopIteration as stop:
      return stop.value, []
    except Failed as error:
      return f"Stopped: a call failed ({str(error)[:150]}).", []
    return "", [{"name": name, "arguments": args}]


if __name__ == "__main__":
  assert _words("Put the small yellow cube on the red cube.") == [("yellow", "cube"), ("red", "cube")]
  assert _words("Insert the blue peg in the grey block's hole.") == [("blue", "peg"), ("", "block_with_hole")]
  assert _words("Pick up the plate.") == [("", "plate")]
  print("ok")
