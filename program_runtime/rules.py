"""Intent validation and ground-truth rule evaluation."""

from dataclasses import dataclass
import math
from pathlib import Path

from google.protobuf import empty_pb2
import yaml

from .api import IMMOVABLE, size_class


ROOT = Path(__file__).resolve().parents[1]
FILTER_KEYS = {"category", "color", "size"}
SIZE_VALUES = {"small", "medium", "large"}


@dataclass(frozen=True)
class TrueObj:
  id: str
  category: str
  color: str
  size: str
  x: float
  y: float
  z: float
  yaw: float
  sx: float
  sy: float
  sz: float


@dataclass
class RuleResult:
  ok: bool
  per_object: dict[str, str]
  invalid_for_scene: bool
  summary: str


def _validate_filter(value, label: str, *, nonempty: bool) -> list[str]:
  if not isinstance(value, dict):
    return [f"{label} must be an object"]
  problems = []
  if nonempty and not value:
    problems.append(f"{label} must not be empty")
  for key, item in value.items():
    if key not in FILTER_KEYS:
      problems.append(f"{label} has unknown key {key!r}")
      continue
    if not isinstance(item, str) or not item:
      problems.append(f"{label}.{key} must be a non-empty string")
    elif key == "size" and item not in SIZE_VALUES:
      problems.append(f"{label}.size must be small, medium, or large")
  return problems


def validate_intent(value) -> list[str]:
  """Returns all schema problems in an intent dictionary."""
  if not isinstance(value, dict):
    return ["intent must be an object"]
  problems = []
  allowed = {"rules", "description", "others"}
  for key in value:
    if key not in allowed:
      problems.append(f"unknown root key {key!r}")
  if "description" in value and not isinstance(value["description"], str):
    problems.append("description must be a string")
  if "others" in value and value["others"] != "stay":
    problems.append("others must equal 'stay'")
  rules = value.get("rules")
  if not isinstance(rules, list):
    problems.append("rules must be a list")
    return problems
  if not 1 <= len(rules) <= 6:
    problems.append("rules must contain 1 to 6 rules")
  seen = []
  for index, rule in enumerate(rules):
    label = f"rules[{index}]"
    if not isinstance(rule, dict):
      problems.append(f"{label} must be an object")
      continue
    if set(rule) != {"select", "relation", "target"}:
      problems.append(f"{label} keys must be exactly select, relation, target")
    problems.extend(_validate_filter(rule.get("select"), f"{label}.select", nonempty=False))
    problems.extend(_validate_filter(rule.get("target"), f"{label}.target", nonempty=True))
    if rule.get("relation") not in {"in", "on"}:
      problems.append(f"{label}.relation must be 'in' or 'on'")
    if rule in seen:
      problems.append(f"{label} duplicates an earlier rule")
    seen.append(rule)
  return problems


def _matches(obj: TrueObj, filters: dict) -> bool:
  return all(getattr(obj, key) == value for key, value in filters.items())


def _inside(obj: TrueObj, box: TrueObj) -> bool:
  dx, dy = obj.x - box.x, obj.y - box.y
  local_x = math.cos(box.yaw) * dx + math.sin(box.yaw) * dy
  local_y = -math.sin(box.yaw) * dx + math.cos(box.yaw) * dy
  return (
    abs(local_x) <= box.sx / 2 and abs(local_y) <= box.sy / 2
    and box.z <= obj.z <= box.z + box.sz
  )


def _on_top(obj: TrueObj, target: TrueObj) -> bool:
  return (
    math.hypot(obj.x - target.x, obj.y - target.y) <= max(target.sx, target.sy) / 2
    and abs(obj.z - (target.z + target.sz)) <= 0.01
  )


def check_intent(intent: dict, before: dict[str, TrueObj], after: dict[str, TrueObj]) -> RuleResult:
  """Scores a final true scene against an intent and its true initial scene."""
  schema = validate_intent(intent)
  if schema:
    return RuleResult(False, {}, True, "; ".join(schema))
  invalid = False
  assignments: dict[str, list[tuple[dict, str, str]]] = {}
  target_ids: set[str] = set()
  for rule in intent["rules"]:
    targets = [obj for obj in before.values() if _matches(obj, rule["target"])]
    if len(targets) != 1:
      invalid = True
      continue
    target = targets[0]
    target_ids.add(target.id)
    if target.id not in after:
      invalid = True
    for obj in before.values():
      if obj.category in IMMOVABLE or not _matches(obj, rule["select"]):
        continue
      assignments.setdefault(obj.id, []).append((rule["target"], rule["relation"], target.id))

  for assigned in assignments.values():
    if len({target_id for _, _, target_id in assigned}) > 1:
      invalid = True
  for object_id in assignments:
    if object_id not in after:
      invalid = True

  statuses = {}
  for object_id, original in before.items():
    if original.category in IMMOVABLE:
      continue
    current = after.get(object_id)
    if current is None:
      statuses[object_id] = "missing"
      continue
    failed_relation = False
    for _, relation, target_id in assignments.get(object_id, []):
      target = after.get(target_id)
      if target is None:
        failed_relation = True
      elif relation == "in" and not _inside(current, target):
        failed_relation = True
      elif relation == "on" and not _on_top(current, target):
        failed_relation = True
    if failed_relation:
      statuses[object_id] = "not in target"
    elif object_id not in assignments and math.dist(
        (original.x, original.y, original.z), (current.x, current.y, current.z)
    ) > 0.01:
      statuses[object_id] = "moved"
    else:
      statuses[object_id] = "ok"
  ok = not invalid and all(status == "ok" for status in statuses.values())
  if invalid:
    summary = "invalid for scene"
  elif ok:
    summary = "all rules satisfied"
  else:
    failed = sum(status != "ok" for status in statuses.values())
    summary = f"{failed} object(s) failed"
  return RuleResult(ok, statuses, invalid, summary)


def truth_snapshot(sim_stub) -> dict[str, TrueObj]:
  """Returns catalog-enriched ground truth from SimService.ListTrueObjects."""
  response = sim_stub.ListTrueObjects(empty_pb2.Empty(), timeout=90)
  output = {}
  for value in response.objects:
    spec = yaml.safe_load((ROOT / "assets" / "objects" / value.catalog / "object.yaml").read_text())
    sx, sy, sz = (float(number) for number in spec["dims"])
    pose = value.pose
    yaw = math.atan2(
      2 * (pose.qw * pose.qz + pose.qx * pose.qy),
      1 - 2 * (pose.qy * pose.qy + pose.qz * pose.qz),
    )
    output[value.id] = TrueObj(
      id=value.id,
      category=spec["category"],
      color=spec["color_name"],
      size=size_class(sx, sy, sz),
      x=pose.x,
      y=pose.y,
      z=pose.z,
      yaw=yaw,
      sx=sx,
      sy=sy,
      sz=sz,
    )
  return output
