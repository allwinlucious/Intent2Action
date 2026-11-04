"""Push experience (branch push-experience): objects wider than the gripper opens, to be moved onto a flat goal.

The experience: when `pick` fails because the object is wider than the gripper opening, ask the operator (say why,
and that it could be pushed) instead of retrying; when the operator agrees, push it onto the goal with the robot's
existing force-limited moves, check where it is, back off, and patch the rest of the program down to a check.

Training scenes use a crate, a tray and a tote with a mat or a pad as the goal. The held-out evaluation uses an
object and a goal that never occur in training (the carton and the painted zone), to test whether the experience
transfers.

  .venv/bin/python learn/fleet.py --workers 4 --prefix push --base-port 18600 -- learn/push_experience.py rows \
      --split train --n 160 --out logs/expert/push_rows.jsonl
  .venv/bin/python learn/fleet.py --workers 2 --prefix pusheval --base-port 18700 -- learn/push_experience.py \
      evaluate --version G1e --n 40 --out logs/eval/push_G1e.jsonl          (LLM_BASE_URL: the model server)

Runs on the plan tier. The failure context is the product's own: the program runs through the runtime on the plan
tier and the recovery handoff is captured. Robot tools run on the plan tier too, except the two the plan tier does
not have (MoveToContact, MoveCompliant): PushWorld answers those with the robot service's own argument checks and
message formats, and a quasi-static push model (the object slides with the fingers once in contact; a compliant
spring stops short where its force drops to the sliding friction). list_objects and get_state are the
plan tier's answers with the object and the tool moved accordingly. Seeds: train 0-9999, held-out 100000 and up.
"""

import argparse
import json
import math
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen"), str(ROOT / "learn")]

from robot_service import robot_service_pb2 as api  # noqa: E402

sys.path.append(str(ROOT / "container/services"))
from robot import force  # noqa: E402  (the robot service's own argument checks)
from robot.ops import ApiError  # noqa: E402

import families  # noqa: E402
from episodes import ADDRESS, connect, in_shard, read_jsonl  # noqa: E402
from program_runtime.agent import RecoveryAgent, recovery_messages, tool_specs  # noqa: E402
from program_runtime.api import GrpcBackend, RobotAPI  # noqa: E402
from program_runtime.check import check  # noqa: E402
from program_runtime.runtime import run  # noqa: E402

FAMILY = "push"
TRAIN_OBJECTS, TRAIN_GOALS = ("wide_crate", "flat_tray", "tote"), ("pallet_mat", "marker_pad")
HELDOUT_OBJECTS, HELDOUT_GOALS = ("big_carton",), ("target_zone",)
GOAL_CATEGORIES = {"mat", "pad", "zone"}
SLIDE_FORCE_N = 1.2  # sliding a box on the table (the Isaac push check measured about 1.2 N)
GAP_M = 0.025  # fingers behind the object's face before the guarded contact
PUSH_Z, ABOVE_Z = 0.035, 0.15  # TCP heights: at most this, or the object's lower half; clear above everything
CONTACT_N, CONTACT_SPEED = 10.0, 0.01
PUSH_LIMIT_N, TARGET_N = 25.0, 15.0  # the compliant force limit; the spring force aimed for at the start
FINGER_M = 0.01  # finger pad thickness: the TCP stops this far from the face it touches
CONTACT_MAX_TRAVEL_M = 0.15  # robot_service.py
ON_ZONE_M = 0.02
YES = ("find a way", "can you get it done somehow?", "yes, push it", "go ahead", "yes, try pushing it",
       "ok, push it there", "do it", "yes")
NO = ("no, leave it", "no", "don't touch it, I'll move it myself", "stop, leave it where it is")
PROGRAMS = (
  'goal = find(category="{g}", color="{gc}")\nthing = find(category="{o}")\npick(thing)\nplace(thing, goal)\n'
  'verify(lambda: on_top(find(category="{o}"), goal), "the {o} is not on the {g}")\n',
  'goal = find(category="{g}", color="{gc}")\nwhile True:\n'
  '  todo = [(o, goal) for o in find_all(category="{o}") if not on_top(o, goal)]\n  if not todo:\n    break\n'
  '  p = todo[0]\n  if not p[0].attached:\n    pick(p[0])\n  place(p[0], p[1])\n'
  'verify(lambda: all(on_top(o, goal) for o in find_all(category="{o}")), "a {o} is not on the {g}")\n',
)
VERIFY_PATCH = ('goal = find(category="{g}", color="{gc}")\nthing = find(category="{o}")\n'
                'verify(lambda: on_top(thing, goal), "the {o} is not on the {g}")\n')
ATTRIBUTES = {}  # catalog name -> (category, colour), read from assets/objects/*/object.yaml


def attributes(catalog: str) -> tuple[str, str]:
  if catalog not in ATTRIBUTES:
    import yaml
    value = yaml.safe_load((ROOT / "assets/objects" / catalog / "object.yaml").read_text())
    ATTRIBUTES[catalog] = (value["category"], value["color_name"])
  return ATTRIBUTES[catalog]


def program(seed: int, placements) -> str:
  (o, _), (g, gc) = attributes(placements[0].catalog), attributes(placements[1].catalog)
  return PROGRAMS[seed % len(PROGRAMS)].format(o=o, g=g, gc=gc)


def scene(seed: int) -> list[families.Placement]:
  """A wide object on one side of the table, its goal on the other, up to two cubes off the push corridor
  (training objects and goals for training seeds, the held-out pair for held-out seeds)."""
  rng = random.Random(f"{FAMILY}:{seed}")
  held_out = seed >= 100000
  thing = rng.choice(HELDOUT_OBJECTS if held_out else TRAIN_OBJECTS)
  goal = rng.choice(HELDOUT_GOALS if held_out else TRAIN_GOALS)
  side = rng.choice((1, -1))
  cx, cy = rng.uniform(0.42, 0.58), side * rng.uniform(0.14, 0.28)
  zx, zy = cx + rng.uniform(-0.08, 0.08), -side * rng.uniform(0.08, 0.20)
  placements = [families.Placement(thing, cx, cy, 0.0, rng.uniform(-0.35, 0.35)),
                families.Placement(goal, zx, zy, -0.0019, 0.0)]
  for _ in range(rng.choice((0, 1, 2))):
    for _attempt in range(50):
      x, y = rng.uniform(0.33, 0.68), rng.uniform(-0.30, 0.30)
      if _off_corridor((x, y), (cx, cy), (zx, zy), placements):
        placements.append(families.Placement(rng.choice(("red_cube", "blue_cube", "green_cube")), x, y, 0.0,
                                             rng.uniform(-0.4, 0.4)))
        break
  return placements


def _off_corridor(point, start, end, placements) -> bool:
  (px, py), (sx, sy), (ex, ey) = point, start, end
  length = math.hypot(ex - sx, ey - sy)
  t = max(-0.2, min(1.2, ((px - sx) * (ex - sx) + (py - sy) * (ey - sy)) / length ** 2))
  near_path = math.hypot(px - (sx + t * (ex - sx)), py - (sy + t * (ey - sy))) < 0.15
  return not near_path and all(math.hypot(px - p.x, py - p.y) > 0.10 for p in placements)


class _Capture:
  """A recovery agent that records the handoff context and stops."""

  def __init__(self):
    self.context = None

  def recover(self, context: dict) -> dict:
    self.context = context
    return {"rung": "stop", "reason": "captured"}


def failure_context(placements, program: str) -> dict | None:
  """The recovery context of the program's failing pick, from the runtime on the plan tier (None: it did not fail
  with the over-wide grasp error)."""
  sim, robot = connect()
  families.load_placements(placements, sim, robot)
  robot.Home(api.HomeRequest(wait=True), timeout=120)
  capture = _Capture()
  run(program, RobotAPI(GrpcBackend(ADDRESS)), agent=capture)
  context = capture.context
  if context is None or context["kind"] != "grasp" or "wider than the gripper" not in context["message"]:
    return None
  return context


def _yawed(q: dict, yaw: float) -> dict:
  """Orientation `q` turned by `yaw` about the world z axis."""
  cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
  w, x, y, z = q["qw"], q["qx"], q["qy"], q["qz"]
  return {"qx": cz * x - sz * y, "qy": cz * y + sz * x, "qz": cz * z + sz * w, "qw": cz * w - sz * z}


def _r(value: float) -> float:
  return round(value, 4)


class PushWorld:
  """Tool answers for one push episode: the plan tier's, plus a push model for the two force moves."""

  def __init__(self, tools, stub, context: dict, rng: random.Random):
    from mcp_server import server
    self.server, self.tools, self.stub, self.rng = server, tools, stub, rng
    thing, zone = wide_object(context), goal_object(context)
    self.object_id, self.zone = thing["id"], zone
    self.height = thing["sz"]
    self.object_xy = [thing["x"], thing["y"]]
    self.object_yaw, self.object_dims = thing["yaw"], (thing["sx"], thing["sy"])
    tool = context["state"]["tool_pose"]
    self.tcp = [tool["x"], tool["y"], tool["z"]]
    self.counter = 100
    self.calls: list[tuple[str, dict, str]] = []  # (name, arguments, result text): the evaluation reads them
    self.violations: list[str] = []

  # --- the push model ---------------------------------------------------------------------------------------
  def _face_distance(self, origin, direction) -> float | None:
    """Distance along `direction` (unit, horizontal) from `origin` to the object's side, or None (missed)."""
    if origin[2] > self.height - 0.01 or origin[2] < 0.005:
      return None
    c, s = math.cos(self.object_yaw), math.sin(self.object_yaw)
    ox, oy = origin[0] - self.object_xy[0], origin[1] - self.object_xy[1]
    lx, ly = c * ox + s * oy, -s * ox + c * oy  # the object's frame
    dx, dy = c * direction[0] + s * direction[1], -s * direction[0] + c * direction[1]
    hx, hy = self.object_dims[0] / 2 + FINGER_M, self.object_dims[1] / 2 + FINGER_M
    if abs(lx) <= hx and abs(ly) <= hy:
      # The fingers are on the face already (the object turned a little while it slid): contact only when the
      # move goes into the object, not when it backs away from it.
      return 0.0 if dx * -lx + dy * -ly > 0 else None
    best = None
    for axis, (p, d, h, q, other_h) in enumerate(((lx, dx, hx, ly, hy), (ly, dy, hy, lx, hx))):
      if abs(d) < 1e-9:
        continue
      for wall in (-h, h):
        t = (wall - p) / d
        cross = q + t * (dy if axis == 0 else dx)
        if t >= -1e-6 and abs(cross) <= other_h and (best is None or t < best):
          best = t
    return best

  def _operation(self, kind: str, detail: str, error: tuple[str, str] | None = None) -> str:
    self.counter += 1
    value = {"name": f"operations/{kind}-{self.counter}", "done": True, "state": "FAILED" if error else "DONE",
             "detail": error[1] if error else detail, "planned_duration_s": 0.0}
    if error:
      value["error"] = {"code": error[0], "message": f"operations/{kind}-{self.counter}: {error[1]}"}
    return json.dumps(value)

  def _summary(self, seconds: float, newtons: float) -> str:
    return f"after {seconds:.2f} s of sim time, force along the push {newtons:.1f} N"

  def move_to_contact(self, args: dict) -> str:
    try:
      direction = force.validate_contact((args.get("dx", 0.0), args.get("dy", 0.0), args.get("dz", 0.0)),
                                         float(args.get("max_force", 0.0)), float(args.get("speed", 0.0)))
    except ApiError as error:
      self.violations.append(f"move_to_contact: {error.message}")
      return f"ERROR {error.code.name}: {error.message}"
    speed, limit = float(args["speed"]), float(args["max_force"])
    horizontal = math.hypot(direction[0], direction[1])
    t = self._face_distance(self.tcp, (direction[0] / horizontal, direction[1] / horizontal)) \
        if horizontal > 0.9 else None
    if t is None or t > CONTACT_MAX_TRAVEL_M:
      travel = CONTACT_MAX_TRAVEL_M
      self.tcp = [self.tcp[k] + direction[k] * travel for k in range(3)]
      self.violations.append("move_to_contact: no contact")
      return self._operation("MoveToContact", "", ("ABORTED", f"no contact within {CONTACT_MAX_TRAVEL_M:.2f} m "
                                                   f"({self._summary(travel / speed, 0.3)})"))
    self.tcp = [self.tcp[k] + direction[k] * t for k in range(3)]
    return self._operation("MoveToContact", f"done {self._summary(t / speed + 0.6, limit * self.rng.uniform(1.0, 1.08))}")

  def move_compliant(self, args: dict) -> str:
    try:
      force.validate_compliant(float(args.get("stiffness", 0.0)), float(args.get("max_force", 0.0)))
    except ApiError as error:
      self.violations.append(f"move_compliant: {error.message}")
      return f"ERROR {error.code.name}: {error.message}"
    k, limit, pose = float(args["stiffness"]), float(args["max_force"]), args.get("pose") or {}
    target = [float(pose.get(key, 0.0)) for key in ("x", "y", "z")]
    offset = [target[i] - self.tcp[i] for i in range(3)]
    distance = math.sqrt(sum(v * v for v in offset)) or 1e-9
    unit = [v / distance for v in offset]
    horizontal = math.hypot(unit[0], unit[1])
    gap = self._face_distance(self.tcp, (unit[0] / horizontal, unit[1] / horizontal)) if horizontal > 0.9 else None
    if gap is not None and gap < distance:
      # Free travel to the object's face, then the object resists the spring (stiffness x the rest of the way:
      # the sensed force; in free space nothing pushes back) until it slides with the fingers, and the spring
      # settles where its force drops to the sliding friction.
      pushed = distance - gap
      if k * pushed + SLIDE_FORCE_N > limit:
        self.violations.append(f"move_compliant: force limit ({k * pushed + SLIDE_FORCE_N:.1f} N > {limit:g} N)")
        self.tcp = [self.tcp[i] + unit[i] * gap for i in range(3)]
        return self._operation("MoveCompliant", "", ("ABORTED", f"force limit reached ({self._summary(0.4, limit + 0.3)})"))
      short = min(pushed, SLIDE_FORCE_N / k) * self.rng.uniform(0.85, 1.15)
      travel = pushed - short
      side = self.rng.gauss(0.0, 0.0015)  # sideways drift only: the fingers stay on the face
      self.object_xy = [self.object_xy[0] + unit[0] * travel - unit[1] / horizontal * side,
                        self.object_xy[1] + unit[1] * travel + unit[0] / horizontal * side]
      self.object_yaw += self.rng.gauss(0.0, 0.01)
      self.tcp = [self.tcp[i] + unit[i] * (gap + travel) for i in range(3)]
      seconds = 1.0 + (gap + travel) / 0.03
      if short > 0.004:
        return self._operation("MoveCompliant", "", ("ABORTED", f"settled before reaching the target (position error "
                                                     f"{short:.4f} m) ({self._summary(seconds, k * short)})"))
      return self._operation("MoveCompliant", f"done {self._summary(seconds, k * short)}")
    self.tcp = target
    return self._operation("MoveCompliant", f"done {self._summary(0.8 + distance / 0.05, 0.2)}")

  # --- tool answers -------------------------------------------------------------------------------------------
  def invoke(self, name: str, arguments: dict) -> str:
    if name == "move_to_contact":
      text = self.move_to_contact(arguments)
    elif name == "move_compliant":
      text = self.move_compliant(arguments)
    else:
      result, status = self.server.invoke(self.tools, self.stub, name, arguments)
      text = "\n".join(item.text for item in result.content if hasattr(item, "text"))
      text = text if status == "OK" else f"ERROR {status}: {text}"
      if status == "OK" and name == "move_to_pose":
        pose = arguments.get("pose") or {}
        self.tcp = [float(pose.get("x", 0)), float(pose.get("y", 0)), float(pose.get("z", 0))]
      elif status == "OK" and name in ("list_objects", "get_state"):
        text = self._moved(name, text)
    self.calls.append((name, arguments, text))
    return text

  def _moved(self, name: str, text: str) -> str:
    """The plan tier's answer with the object and the tool where the push model has them."""
    value = json.loads(text)
    if name == "list_objects":
      for obj in value.get("objects", []):
        if obj.get("id") == self.object_id:
          pose = obj.setdefault("pose", {})
          pose["x"], pose["y"] = self.object_xy[0], self.object_xy[1]
          pose["qz"], pose["qw"] = math.sin(self.object_yaw / 2), math.cos(self.object_yaw / 2)
    else:
      tool = value.setdefault("tool_pose", {})
      tool["x"], tool["y"], tool["z"] = self.tcp
    return json.dumps(value)

  def on_zone(self) -> float:
    return math.hypot(self.object_xy[0] - self.zone["x"], self.object_xy[1] - self.zone["y"])


# --- the expert: the episodes the model learns from -------------------------------------------------------------
def wide_object(context: dict) -> dict:
  """The object of the failing pick: its id is in the call text, e.g. pick(obj_33)."""
  return next(o for o in context["objects"] if f"({o['id']})" in context["call"])


def goal_object(context: dict) -> dict:
  return next(o for o in context["objects"] if o["category"] in GOAL_CATEGORIES)


def ask_decision(context: dict) -> dict:
  thing, goal = wide_object(context), goal_object(context)
  width = min(thing["sx"], thing["sy"])
  return {"rung": "ask_user",
          "reason": f"{context['call']} failed: the {thing['category']} is {thing['sx']:.2f} x {thing['sy']:.2f} m, "
                    f"narrowest side {width:.2f} m, and the gripper opens 0.075 m, so there is no grasp; retrying or "
                    f"another order cannot help",
          "question": f"I can't pick up the {thing['category']}: it is wider than the gripper opens. I could push it "
                      f"onto the {goal['color']} {goal['category']} with force-limited moves instead. Should I try?"}


def push_turns(world: PushWorld, context: dict) -> tuple[list[tuple[str, dict]], dict]:
  """The expert push, run against `world`: returns the tool calls made and the final decision."""
  calls = []

  def call(name: str, arguments: dict) -> str:
    calls.append((name, arguments))
    return world.invoke(name, arguments)

  zone = world.zone
  ex, ey = zone["x"] - world.object_xy[0], zone["y"] - world.object_xy[1]
  distance = math.hypot(ex, ey)
  dx, dy = ex / distance, ey / distance
  down = context["state"]["tool_pose"]
  hand = _yawed(down, math.atan2(dy, dx))  # the fingers across the push direction
  c, s = math.cos(world.object_yaw), math.sin(world.object_yaw)
  half = abs(c * dx + s * dy) * world.object_dims[0] / 2 + abs(-s * dx + c * dy) * world.object_dims[1] / 2
  back = half + FINGER_M + GAP_M
  push_z = min(PUSH_Z, world.height / 2)
  start = (world.object_xy[0] - dx * back, world.object_xy[1] - dy * back)

  def pose(x, y, z):
    return {"x": _r(x), "y": _r(y), "z": _r(z), **{key: _r(value) for key, value in hand.items()}}

  call("release", {})
  call("move_to_pose", {"pose": pose(*start, ABOVE_Z), "motion": "ANY", "tool": "TCP"})
  call("move_to_pose", {"pose": pose(*start, push_z), "motion": "LINEAR", "tool": "TCP"})
  call("move_to_contact", {"dx": _r(dx), "dy": _r(dy), "dz": 0.0, "max_force": CONTACT_N, "speed": CONTACT_SPEED})
  for _ in range(3):  # a long spring push, then short ones for what the spring left
    left = math.hypot(zone["x"] - world.object_xy[0], zone["y"] - world.object_xy[1])
    if left <= ON_ZONE_M * 0.5:
      break
    stiffness = float(max(force.STIFFNESS_RANGE[0], min(300, round(TARGET_N / left / 10) * 10)))
    goal = (world.tcp[0] + dx * left, world.tcp[1] + dy * left)
    call("move_compliant", {"pose": pose(*goal, push_z), "stiffness": stiffness, "max_force": PUSH_LIMIT_N})
    call("list_objects", {})
  call("move_compliant", {"pose": pose(world.tcp[0] - dx * 0.03, world.tcp[1] - dy * 0.03, push_z + 0.01),
                          "stiffness": 300.0, "max_force": PUSH_LIMIT_N})
  call("move_compliant", {"pose": pose(world.tcp[0], world.tcp[1], ABOVE_Z), "stiffness": 300.0,
                          "max_force": PUSH_LIMIT_N})
  off = world.on_zone()
  thing, goal = wide_object(context), goal_object(context)
  decision = {"rung": "patch",
              "reason": f"the {thing['category']} was pushed onto the {goal['category']} ({off * 1000:.0f} mm from its "
                        f"centre) with force-limited moves; the rest of the program only has to check it",
              "patch": VERIFY_PATCH.format(o=thing["category"], g=goal["category"], gc=goal["color"])}
  return calls, decision


def _assistant_calls(calls: list[tuple[str, dict]], texts: list[str], start: int) -> list[dict]:
  messages = []
  for k, ((name, arguments), text) in enumerate(zip(calls, texts)):
    call_id = f"call_{start + k}"
    messages.append({"role": "assistant", "content": "", "tool_calls": [
        {"id": call_id, "type": "function", "function": {"name": name, "arguments": json.dumps(arguments)}}]})
    messages.append({"role": "tool", "tool_call_id": call_id, "content": text[:4000]})
  return messages


def episode_rows(seed: int, specs: list[dict], tools, stub) -> list[dict]:
  """The training rows of one scene: the ask, and the answer to the operator's reply (a push, or a stop)."""
  rng = random.Random(f"{FAMILY}-rows:{seed}")
  placements = scene(seed)
  context = failure_context(placements, program(seed, placements))
  if context is None:
    return []
  rows = [{"messages": [*recovery_messages(context), {"role": "assistant", "content": json.dumps(ask_decision(context))}],
           "tools": specs, "kind": "push_ask"}]
  answered = {**context, "user_answer": rng.choice(NO) if rng.random() < 0.15 else rng.choice(YES)}
  if answered["user_answer"] in NO:
    decision = {"rung": "stop", "reason": f"the operator declined the push, and the {wide_object(context)['category']} "
                                          f"cannot be grasped"}
    rows.append({"messages": [*recovery_messages(answered), {"role": "assistant", "content": json.dumps(decision)}],
                 "tools": specs, "kind": "push_declined"})
    return rows
  world = PushWorld(tools, stub, context, rng)
  calls, decision = push_turns(world, context)
  errors = [f"{name}: {text[:160]}" for name, _, text in world.calls if text.startswith("ERROR")]
  if world.violations or world.on_zone() > ON_ZONE_M or errors:
    # The expert itself failed here (e.g. an approach the planner rejects): no push row, the reason kept.
    rows.append({"kind": "push_dropped", "reason": world.violations + errors + [f"{world.on_zone() * 1000:.0f} mm off"],
                 "calls": [(name, arguments, text[:300]) for name, arguments, text in world.calls]})
    return rows
  texts = [text for _, _, text in world.calls]
  rows.append({"messages": [*recovery_messages(answered), *_assistant_calls(calls, texts, 0),
                            {"role": "assistant", "content": json.dumps(decision)}], "tools": specs,
               "kind": "push_done", "on_zone_mm": round(world.on_zone() * 1000, 1)})
  return rows


# --- evaluation: the model in the expert's place --------------------------------------------------------------
class _WorldAgent(RecoveryAgent):
  """The product's recovery agent with tool calls answered by a PushWorld."""

  def __init__(self, client, stub, tools, world: PushWorld, model: str):
    super().__init__(client, stub, tools, model=model)
    self.world = world

  def _tool_text(self, call: dict, names: set[str]) -> str:
    function = call.get("function", {})
    try:
      arguments = json.loads(function.get("arguments") or "{}")
    except json.JSONDecodeError as error:
      return f"invalid arguments: {error}"
    if function.get("name") not in names:
      return f"unknown tool: {function.get('name')}"
    return self.world.invoke(function["name"], arguments)


def evaluate_one(seed: int, client, model: str, tools, stub) -> dict:
  rng = random.Random(f"{FAMILY}-eval:{seed}")
  placements = scene(seed)
  context = failure_context(placements, program(seed, placements))
  if context is None:
    return {"family": FAMILY, "seed": seed, "skipped": "no over-wide grasp failure"}
  world = PushWorld(tools, stub, context, rng)
  agent = _WorldAgent(client, stub, tools, world, model)
  first = agent.recover(context)
  record = {"family": FAMILY, "seed": seed, "first": first, "asked": first.get("rung") == "ask_user"}
  answer = rng.choice(YES)
  second = agent.recover({**context, "user_answer": answer})
  patch_ok = second.get("rung") == "patch" and not check(second.get("patch") or "", patch=True)
  names = [name for name, _, _ in world.calls]
  record.update(answer=answer, second=second, calls=[(n, a) for n, a, _ in world.calls], violations=world.violations,
                planner_errors=[t[:200] for n, _, t in world.calls if t.startswith("ERROR")],
                used_force_moves="move_to_contact" in names and "move_compliant" in names,
                on_zone_mm=round(world.on_zone() * 1000, 1), patch_ok=patch_ok)
  record["pushed"] = world.on_zone() <= ON_ZONE_M and not world.violations
  record["ok"] = record["asked"] and record["pushed"] and patch_ok
  return record


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("mode", choices=("rows", "evaluate"))
  parser.add_argument("--split", choices=("train", "heldout"), default="heldout")
  parser.add_argument("--n", type=int, default=40)
  parser.add_argument("--version", default="G1e")
  parser.add_argument("--out", required=True)
  parser.add_argument("--shard")
  args = parser.parse_args()
  from mcp_server.server import DESCRIPTOR_SET, Tools
  tools = Tools(DESCRIPTOR_SET)
  specs = tool_specs(tools)
  _, stub = connect()
  start = 0 if args.split == "train" else 100000
  out = Path(args.out)
  done = {v["seed"] for v in read_jsonl(out)}
  client = None
  if args.mode == "evaluate":
    from program_runtime.agent import ChatClient
    client = ChatClient()
  for seed in range(start, start + args.n):
    if seed in done or not in_shard(FAMILY, seed, args.shard):
      continue
    if args.mode == "rows":
      value = {"family": FAMILY, "seed": seed, "rows": episode_rows(seed, specs, tools, stub)}
    else:
      value = evaluate_one(seed, client, args.version, tools, stub)
    with out.open("a") as handle:
      handle.write(json.dumps(value) + "\n")
    print(f"{FAMILY}:{seed} " + (f"{len(value['rows'])} rows" if args.mode == "rows" else
                                 f"ok={value.get('ok')} asked={value.get('asked')} pushed={value.get('pushed')}"),
          flush=True)


if __name__ == "__main__":
  main()
