"""Deterministic task families for the learning experiments."""

import argparse
import dataclasses
import json
import math
import random
import statistics
import sys
import time
from pathlib import Path

import yaml
from google.protobuf import empty_pb2
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "gen"))

from program_runtime.api import Obj, size_class  # noqa: E402
from program_runtime.rules import validate_intent  # noqa: E402
from robot_service import robot_service_pb2 as api  # noqa: E402

sys.path.append(str(ROOT / "container/services"))
from robot import place  # noqa: E402


CATALOG_ROOT = ROOT / "assets" / "objects"
TABLE_SCENE = yaml.safe_load((ROOT / "sim" / "scenes" / "default.yaml").read_text())["objects"]["table"]
FAMILIES = ("cluster", "packing", "stacking", "reach", "mixed", "sorting")
CUBE_CATALOGS = ("red_cube", "blue_cube", "green_cube", "small_yellow_cube")
MEDIUM_CUBES = ("red_cube", "blue_cube", "green_cube")
BOX_CATALOGS = ("red_box", "blue_box")
SHAPES = {
  "plus": ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)),
  "L": ((0, 0), (1, 0), (2, 0), (0, 1), (0, 2), (3, 0), (0, 3)),
  "T": ((0, 0), (1, 0), (-1, 0), (0, 1), (0, 2), (2, 0), (-2, 0)),
}
# At most 5 cubes: a box takes 6 at best (place planner), and 1.0 cm gaps left some layouts with no grasp at all
# (expert run of 2026-10-31: 25 % of clusters solved).
SHAPE_COUNTS = {"plus": (4, 5), "L": (4, 5), "T": (4, 5), "line": (3, 5), "block": (4, 5)}
GAP_RANGE_M = (0.015, 0.035)
COLOURS = {
  "red": (204, 26, 26),
  "blue": (26, 51, 204),
  "green": (26, 179, 51),
  "yellow": (230, 204, 26),
  "grey": (128, 128, 128),
}


@dataclasses.dataclass(frozen=True)
class Placement:
  catalog: str
  x: float
  y: float
  z: float
  yaw: float


@dataclasses.dataclass(frozen=True)
class Task:
  family: str
  seed: int
  split: str
  intent: dict
  placements: tuple[Placement, ...]
  retries: int = dataclasses.field(compare=False)


def _catalog(catalog_root: Path, name: str) -> dict:
  return yaml.safe_load((Path(catalog_root) / name / "object.yaml").read_text())


def _specs(catalog_root: Path) -> dict[str, dict]:
  names = (*CUBE_CATALOGS, *BOX_CATALOGS, "obstacle", "table")
  return {name: _catalog(catalog_root, name) for name in names}


def _split(family: str, seed: int) -> str:
  if family not in FAMILIES:
    raise ValueError(f"unknown family {family!r}")
  if 0 <= seed <= 9999:
    if family == "mixed":
      raise ValueError("mixed is held out entirely")
    return "train"
  if 100000 <= seed <= 100999:
    return "heldout"
  raise ValueError(f"seed {seed} is outside the train and heldout ranges")


def _rule(select: dict, relation: str, target: dict) -> dict:
  return {"select": select, "relation": relation, "target": target}


def _intent(description: str, rules: list[dict]) -> dict:
  value = {"description": description, "rules": rules, "others": "stay"}
  problems = validate_intent(value)
  if problems:
    raise RuntimeError(f"generated invalid intent: {problems}")
  return value


def _box(rng: random.Random) -> tuple[str, Placement]:
  catalog = rng.choice(BOX_CATALOGS)
  return catalog, Placement(catalog, rng.uniform(0.35, 0.45), rng.uniform(-0.30, -0.24), 0.0, 0.0)


def _shape_offsets(shape: str, n: int) -> tuple[tuple[int, int], ...]:
  if shape in SHAPES:
    return SHAPES[shape][:n]
  if shape == "line":
    return tuple((i, 0) for i in range(n))
  columns = math.ceil(n / 2)
  return tuple((i, row) for row in range(2) for i in range(columns))[:n]


def _cluster_placements(rng: random.Random, specs: dict[str, dict], *, mixed: bool = False,
                        shape: str | None = None, n: int | None = None,
                        centre: tuple[float, float] | None = None) -> list[Placement]:
  shape = shape or rng.choice(tuple(SHAPE_COUNTS))
  low, high = SHAPE_COUNTS[shape]
  if mixed:
    low, high = max(low, 4), min(high, 6)
  n = n or rng.randint(low, high)
  gap = rng.uniform(*GAP_RANGE_M)
  pitch = float(specs["red_cube"]["dims"][0]) + gap
  theta = rng.uniform(-math.pi / 4, math.pi / 4)
  centre_x, centre_y = centre or (rng.uniform(0.45, 0.55), rng.uniform(0.00, 0.15))
  offsets = _shape_offsets(shape, n)
  mean_x = sum(x for x, _ in offsets) / n
  mean_y = sum(y for _, y in offsets) / n
  choices = ("red_cube", "blue_cube") if mixed else MEDIUM_CUBES
  catalogs = [rng.choice(choices) for _ in offsets]
  if not mixed and rng.random() < 0.25:
    catalogs[rng.randrange(n)] = "small_yellow_cube"
  output = []
  cosine, sine = math.cos(theta), math.sin(theta)
  for catalog, (offset_x, offset_y) in zip(catalogs, offsets):
    local_x, local_y = (offset_x - mean_x) * pitch, (offset_y - mean_y) * pitch
    x = centre_x + cosine * local_x - sine * local_y
    y = centre_y + sine * local_x + cosine * local_y
    output.append(Placement(catalog, x, y, 0.0, theta))
  return output


def _planner_spots(specs: dict[str, dict], box: Placement, cube: str, finger_yaw: float,
                   taken: list[tuple[float, float]]) -> list[tuple[float, float]]:
  """The spots Place itself would fill next, in its order, with the open-finger footprint: pre-filled boxes are
  packed the way the robot packs them, so the remaining spots are really usable (a regular grid leaves gaps the
  fingers cannot open into)."""
  spec = specs[box.catalog]
  box_pose = api.Pose(x=box.x, y=box.y, z=box.z, qz=math.sin(box.yaw / 2), qw=math.cos(box.yaw / 2))
  dims = [float(v) for v in specs[cube]["dims"]]
  footprint = place.held_footprint(dims, box.yaw, box.yaw + finger_yaw, box.yaw)
  poses = [api.Pose(x=x, y=y, z=box.z + float(spec["container"]["floor"]), qz=box_pose.qz, qw=box_pose.qw)
           for x, y in taken]
  spots = []
  while True:
    occupied = place.occupied(box_pose, [float(v) for v in spec["dims"]], [(q, dims) for q in poses])
    free = place.candidates(box_pose, [float(v) for v in spec["dims"]], spec["container"], footprint, occupied)
    if not free:
      return spots
    x, y, z = free[0]
    spots.append((x, y))
    poses.append(api.Pose(x=x, y=y, z=z, qz=box_pose.qz, qw=box_pose.qw))


def _prefill(rng: random.Random, specs: dict[str, dict], box: Placement, k: int) -> tuple[list[tuple[float, float]], int]:
  """k spots packed by the place planner (finger axis along the box's x or y, random) and the number of medium
  cubes that still fit whatever the finger direction of the later places."""
  filled = _planner_spots(specs, box, "blue_cube", rng.choice((0.0, math.pi / 2)), [])[:k]
  room = min(len(_planner_spots(specs, box, "red_cube", yaw, filled)) for yaw in (0.0, math.pi / 2))
  return filled, room


def _inside_cube(catalog: str, xy: tuple[float, float], box: Placement, specs: dict[str, dict]) -> Placement:
  floor = float(specs[box.catalog]["container"]["floor"])
  return Placement(catalog, xy[0], xy[1], box.z + floor, box.yaw)


def _outside_cube(rng: random.Random, catalog: str) -> Placement:
  return Placement(catalog, rng.uniform(0.35, 0.60), rng.uniform(0.00, 0.25), 0.0,
                   rng.uniform(-0.3, 0.3))


def _cluster_attempt(rng: random.Random, specs: dict[str, dict]) -> tuple[list[Placement], dict]:
  placements = _cluster_placements(rng, specs)
  box_catalog, box = _box(rng)
  placements.append(box)
  colour = specs[box_catalog]["color_name"]
  intent = _intent(f"every cube goes into the {colour} box", [
    _rule({"category": "cube"}, "in", {"category": "box", "color": colour}),
  ])
  return placements, intent


def _packing_attempt(rng: random.Random, specs: dict[str, dict]) -> tuple[list[Placement], dict]:
  box_catalog, box = _box(rng)
  k = rng.randint(2, 4)
  filled, room = _prefill(rng, specs, box, k)
  while room < 1:  # the drawn finger direction packed too loosely for k: fewer pre-filled cubes
    k -= 1
    filled, room = _prefill(rng, specs, box, k)
  placements = [box, *(_inside_cube("blue_cube", xy, box, specs) for xy in filled)]
  m = rng.randint(1, room)
  outside_catalogs = [rng.choice(("red_cube", "green_cube")) for _ in range(m)]
  placements.extend(_outside_cube(rng, catalog) for catalog in outside_catalogs)
  box_colour = specs[box_catalog]["color_name"]
  colours = [colour for colour in ("red", "green")
             if any(specs[catalog]["color_name"] == colour for catalog in outside_catalogs)]
  description_colours = " and ".join(colours)
  rules = [_rule({"category": "cube", "color": colour}, "in",
                 {"category": "box", "color": box_colour}) for colour in colours]
  return placements, _intent(f"the {description_colours} cubes go into the {box_colour} box", rules)


def _stack_catalogs(rng: random.Random, n: int) -> list[str]:
  catalogs = rng.sample(list(CUBE_CATALOGS), n)
  if "small_yellow_cube" in catalogs:
    index = catalogs.index("small_yellow_cube")
    catalogs[index], catalogs[-1] = catalogs[-1], catalogs[index]
  return catalogs


def _stacking_attempt(rng: random.Random, specs: dict[str, dict]) -> tuple[list[Placement], dict]:
  n = rng.randint(2, 4)
  catalogs = _stack_catalogs(rng, n)
  if rng.random() < 0.5:
    placements = [_outside_cube(rng, catalog) for catalog in catalogs]
    colours = [specs[catalog]["color_name"] for catalog in catalogs]
    rules = [_rule({"category": "cube", "color": colours[index]}, "on",
                   {"category": "cube", "color": colours[index - 1]}) for index in range(1, n)]
    description = "stack the cubes: " + " on ".join(reversed(colours))
    return placements, _intent(description, rules)
  x, y, yaw = rng.uniform(0.45, 0.55), rng.uniform(0.00, 0.15), rng.uniform(-0.3, 0.3)
  placements = []
  z = 0.0
  for catalog in catalogs:
    placements.append(Placement(catalog, x, y, z, yaw))
    z += float(specs[catalog]["dims"][2])
  box_catalog, box = _box(rng)
  placements.append(box)
  colour = specs[box_catalog]["color_name"]
  return placements, _intent(f"every cube goes into the {colour} box", [
    _rule({"category": "cube"}, "in", {"category": "box", "color": colour}),
  ])


def _reach_attempt(rng: random.Random, specs: dict[str, dict]) -> tuple[list[Placement], dict]:
  placements = []
  for _ in range(rng.randint(3, 5)):
    catalog = rng.choice(MEDIUM_CUBES)
    if rng.random() < 0.5:
      radius = rng.uniform(0.62, 0.72)
      angle = rng.uniform(math.radians(-25), math.radians(25))
      x, y = radius * math.cos(angle), radius * math.sin(angle)
    else:
      x, y = rng.uniform(0.30, 0.45), rng.uniform(0.28, 0.34)
    placements.append(Placement(catalog, x, y, 0.0, rng.uniform(-0.3, 0.3)))
  cubes = tuple(placements)
  for _ in range(rng.randint(0, 2)):
    cube = rng.choice(cubes)
    placements.append(Placement("obstacle", 0.7 * cube.x, 0.7 * cube.y, 0.0, 0.0))
  box_catalog, box = _box(rng)
  placements.append(box)
  colour = specs[box_catalog]["color_name"]
  return placements, _intent(f"every cube goes into the {colour} box", [
    _rule({"category": "cube"}, "in", {"category": "box", "color": colour}),
  ])


def _mixed_attempt(rng: random.Random, specs: dict[str, dict]) -> tuple[list[Placement], dict]:
  placements = _cluster_placements(rng, specs, mixed=True)
  red_box = Placement("red_box", rng.uniform(0.34, 0.38), rng.uniform(-0.30, -0.24), 0.0, 0.0)
  blue_box = Placement("blue_box", rng.uniform(0.66, 0.70), rng.uniform(-0.24, -0.18), 0.0, 0.0)
  placements.extend((red_box, blue_box))
  filled_box = rng.choice((red_box, blue_box))
  for xy in _prefill(rng, specs, filled_box, rng.randint(2, 3))[0]:
    placements.append(_inside_cube("green_cube", xy, filled_box, specs))
  intent = _intent("sort by colour: red cubes into the red box, blue cubes into the blue box", [
    _rule({"category": "cube", "color": "red"}, "in", {"category": "box", "color": "red"}),
    _rule({"category": "cube", "color": "blue"}, "in", {"category": "box", "color": "blue"}),
  ])
  return placements, intent


def _sorting_attempt(rng: random.Random, specs: dict[str, dict]) -> tuple[list[Placement], dict]:
  """Two-box colour sorting with layouts unlike `mixed` (trained; `mixed` stays held out): two short lines of
  red and blue cubes, no pre-filled box, the same boxes and intent as `mixed`."""
  placements = []
  for centre in ((rng.uniform(0.40, 0.46), rng.uniform(-0.02, 0.06)), (rng.uniform(0.52, 0.58), rng.uniform(0.12, 0.20))):
    placements += _cluster_placements(rng, specs, mixed=True, shape="line", n=rng.randint(2, 3), centre=centre)
  placements += [Placement("red_box", rng.uniform(0.34, 0.38), rng.uniform(-0.30, -0.24), 0.0, 0.0),
                 Placement("blue_box", rng.uniform(0.66, 0.70), rng.uniform(-0.24, -0.18), 0.0, 0.0)]
  intent = _intent("sort by colour: red cubes into the red box, blue cubes into the blue box", [
    _rule({"category": "cube", "color": "red"}, "in", {"category": "box", "color": "red"}),
    _rule({"category": "cube", "color": "blue"}, "in", {"category": "box", "color": "blue"}),
  ])
  return placements, intent


ATTEMPTS = {
  "cluster": _cluster_attempt,
  "packing": _packing_attempt,
  "stacking": _stacking_attempt,
  "reach": _reach_attempt,
  "mixed": _mixed_attempt,
  "sorting": _sorting_attempt,
}


def _corners(placement: Placement, specs: dict[str, dict]) -> tuple[tuple[float, float], ...]:
  sx, sy = (float(value) for value in specs[placement.catalog]["dims"][:2])
  cosine, sine = math.cos(placement.yaw), math.sin(placement.yaw)
  output = []
  for local_x, local_y in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)):
    output.append((placement.x + cosine * local_x - sine * local_y,
                   placement.y + sine * local_x + cosine * local_y))
  return tuple(output)


def _axes(polygon: tuple[tuple[float, float], ...]):
  for start, end in zip(polygon, (*polygon[1:], polygon[0])):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    yield -dy / length, dx / length


def _overlap(first: tuple[tuple[float, float], ...], second: tuple[tuple[float, float], ...]) -> bool:
  for axis_x, axis_y in (*_axes(first), *_axes(second)):
    first_projection = [x * axis_x + y * axis_y for x, y in first]
    second_projection = [x * axis_x + y * axis_y for x, y in second]
    if max(first_projection) < min(second_projection) - 1e-12 or max(second_projection) < min(first_projection) - 1e-12:
      return False
  return True


def _point_segment_distance(point, start, end) -> float:
  dx, dy = end[0] - start[0], end[1] - start[1]
  denominator = dx * dx + dy * dy
  fraction = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / denominator
  fraction = max(0.0, min(1.0, fraction))
  closest = start[0] + fraction * dx, start[1] + fraction * dy
  return math.hypot(point[0] - closest[0], point[1] - closest[1])


def _polygon_distance(first: tuple[tuple[float, float], ...], second: tuple[tuple[float, float], ...]) -> float:
  if _overlap(first, second):
    return 0.0
  distances = []
  for polygon, other in ((first, second), (second, first)):
    for point in polygon:
      for start, end in zip(other, (*other[1:], other[0])):
        distances.append(_point_segment_distance(point, start, end))
  return min(distances)


def _inside_box(cube: Placement, box: Placement, specs: dict[str, dict]) -> bool:
  box_spec = specs[box.catalog]
  sx, sy = (float(value) for value in box_spec["dims"][:2])
  wall = float(box_spec["container"]["wall"])
  cosine, sine = math.cos(box.yaw), math.sin(box.yaw)
  for x, y in _corners(cube, specs):
    dx, dy = x - box.x, y - box.y
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    if abs(local_x) > sx / 2 - wall + 1e-12 or abs(local_y) > sy / 2 - wall + 1e-12:
      return False
  floor_z = box.z + float(box_spec["container"]["floor"])
  return math.isclose(cube.z, floor_z, abs_tol=1e-12)


def _table_contains(placement: Placement, specs: dict[str, dict]) -> bool:
  centre_x, centre_y = (float(value) for value in TABLE_SCENE["position"][:2])
  yaw = math.radians(float(TABLE_SCENE.get("yaw", 0.0)))
  sx, sy = (float(value) - 0.04 for value in specs["table"]["dims"][:2])
  cosine, sine = math.cos(yaw), math.sin(yaw)
  for x, y in _corners(placement, specs):
    dx, dy = x - centre_x, y - centre_y
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    if abs(local_x) > sx / 2 + 1e-12 or abs(local_y) > sy / 2 + 1e-12:
      return False
  return True


def _valid(family: str, placements: list[Placement], specs: dict[str, dict]) -> bool:
  for placement in placements:
    if not 0.30 <= math.hypot(placement.x, placement.y) <= 0.75:
      return False
    if math.isclose(placement.z, 0.0, abs_tol=1e-12) and not _table_contains(placement, specs):
      return False
  boxes = [placement for placement in placements if specs[placement.catalog]["category"] == "box"]
  cubes = [placement for placement in placements if specs[placement.catalog]["category"] == "cube"]
  inside = []
  for cube in cubes:
    containing = [box for box in boxes if _inside_box(cube, box, specs)]
    if containing:
      if len(containing) != 1:
        return False
      inside.append(cube)
    elif cube.z > 1e-12:
      below = [other for other in cubes if other is not cube
               and math.isclose(cube.x, other.x, abs_tol=1e-12)
               and math.isclose(cube.y, other.y, abs_tol=1e-12)
               and math.isclose(cube.yaw, other.yaw, abs_tol=1e-12)
               and math.isclose(cube.z, other.z + float(specs[other.catalog]["dims"][2]), abs_tol=1e-12)]
      if len(below) != 1:
        return False
  for index, cube in enumerate(inside):
    for other in inside[index + 1:]:
      if any(_inside_box(cube, box, specs) and _inside_box(other, box, specs) for box in boxes):
        if _overlap(_corners(cube, specs), _corners(other, specs)):
          return False
  resting = [placement for placement in placements if math.isclose(placement.z, 0.0, abs_tol=1e-12)]
  for index, first in enumerate(resting):
    first_category = specs[first.catalog]["category"]
    for second in resting[index + 1:]:
      second_category = specs[second.catalog]["category"]
      distance = _polygon_distance(_corners(first, specs), _corners(second, specs))
      if distance <= 1e-12:
        return False
      categories = {first_category, second_category}
      if categories == {"cube", "box"} and distance < 0.10 - 1e-12:
        return False
      if first_category == second_category == "box" and distance < 0.02 - 1e-12:
        return False
      if family in {"packing", "stacking"} and first_category == second_category == "cube" and distance < 0.06 - 1e-12:
        return False
      if family == "reach" and "box" not in categories and distance < 0.06 - 1e-12:
        return False
      if family == "reach" and "pillar" in categories and "box" in categories and distance < 0.06 - 1e-12:
        return False
  return True


def generate(family: str, seed: int, catalog_root: Path = CATALOG_ROOT) -> Task:
  split = _split(family, seed)
  specs = _specs(Path(catalog_root))
  rng = random.Random(f"{family}:{seed}")
  for retries in range(1000):
    placements, intent = ATTEMPTS[family](rng, specs)
    if _valid(family, placements, specs):
      return Task(family, seed, split, intent, tuple(placements), retries)
  raise RuntimeError(f"{family} seed {seed}: no valid layout in 1000 draws")


def tasks(family: str, split: str, n: int, catalog_root: Path = CATALOG_ROOT) -> list[Task]:
  if family not in FAMILIES:
    raise ValueError(f"unknown family {family!r}")
  if split not in {"train", "heldout"}:
    raise ValueError(f"unknown split {split!r}")
  if family == "mixed" and split == "train":
    raise ValueError("mixed is held out entirely")
  limit = 10000 if split == "train" else 1000
  if n < 0 or n > limit:
    raise ValueError(f"n must be between 0 and {limit} for {split}")
  start = 0 if split == "train" else 100000
  return [generate(family, seed, catalog_root) for seed in range(start, start + n)]


def load_placements(placements, sim_stub, robot_stub, timeout_s: float = 30.0) -> list[str]:
  sim_stub.ResetScene(empty_pb2.Empty(), timeout=30)
  true_objects = sim_stub.ListTrueObjects(empty_pb2.Empty(), timeout=30).objects
  for value in true_objects:
    if value.catalog != "table":
      sim_stub.RemoveObject(api.ObjectRequest(id=value.id), timeout=30)
  ids = []
  for placement in placements:
    pose = api.Pose(x=placement.x, y=placement.y, z=placement.z,
                    qz=math.sin(placement.yaw / 2), qw=math.cos(placement.yaw / 2))
    response = sim_stub.AddObject(api.AddObjectRequest(catalog=placement.catalog, pose=pose), timeout=30)
    ids.append(response.id)
  deadline = time.monotonic() + timeout_s
  while True:
    listed = {value.id for value in robot_stub.ListObjects(empty_pb2.Empty(), timeout=30).objects}
    if set(ids) <= listed:
      return ids
    remaining = deadline - time.monotonic()
    if remaining <= 0:
      raise RuntimeError(f"objects did not appear in ListObjects: {ids}")
    time.sleep(min(0.5, remaining))


def load(task: Task, sim_stub, robot_stub, timeout_s: float = 30.0) -> list[str]:
  return load_placements(task.placements, sim_stub, robot_stub, timeout_s)


def fake_objects(task: Task, catalog_root: Path = CATALOG_ROOT) -> list[Obj]:
  objects = []
  for index, placement in enumerate(task.placements):
    spec = _catalog(Path(catalog_root), placement.catalog)
    sx, sy, sz = (float(value) for value in spec["dims"])
    objects.append(Obj(
      id=f"obj_{10 + index}", category=spec["category"], color=spec["color_name"],
      size=size_class(sx, sy, sz), x=placement.x, y=placement.y, z=placement.z, yaw=placement.yaw,
      sx=sx, sy=sy, sz=sz, attached=False,
    ))
  return objects


def _preview(task: Task, catalog_root: Path = CATALOG_ROOT) -> Path:
  specs = _specs(Path(catalog_root))
  image = Image.new("RGB", (800, 800), "white")
  draw = ImageDraw.Draw(image)
  scale = 800.0

  def pixel(point):
    return point[0] * scale, 400 - point[1] * scale

  reach_box = (-0.75 * scale, 400 - 0.75 * scale, 0.75 * scale, 400 + 0.75 * scale)
  draw.arc(reach_box, 270, 90, fill=(100, 100, 100), width=2)
  draw.ellipse((-5, 395, 5, 405), fill=(0, 0, 0))
  for placement in task.placements:
    spec = specs[placement.catalog]
    polygon = [pixel(point) for point in _corners(placement, specs)]
    category = spec["category"]
    colour = COLOURS.get(spec["color_name"], (128, 128, 128))
    if category == "box":
      draw.line([*polygon, polygon[0]], fill=colour, width=3, joint="curve")
    else:
      draw.polygon(polygon, fill=colour, outline=(30, 30, 30))
  output = ROOT / "logs" / "families" / f"{task.family}_{task.seed}.png"
  output.parent.mkdir(parents=True, exist_ok=True)
  image.save(output, format="PNG")
  return output


def _cube_gaps(task: Task, catalog_root: Path = CATALOG_ROOT) -> list[float]:
  specs = _specs(Path(catalog_root))
  cubes = [placement for placement in task.placements
           if specs[placement.catalog]["category"] == "cube" and math.isclose(placement.z, 0.0, abs_tol=1e-12)]
  output = []
  for index, cube in enumerate(cubes):
    others = [other for other_index, other in enumerate(cubes) if other_index != index]
    if others:
      output.append(min(_polygon_distance(_corners(cube, specs), _corners(other, specs)) for other in others))
  return output


def _stats(n: int) -> None:
  for family in FAMILIES:
    for split in ("train", "heldout"):
      if family == "mixed" and split == "train":
        continue
      generated = tasks(family, split, n)
      gaps = [gap for task in generated for gap in _cube_gaps(task)]
      record = {
        "family": family,
        "split": split,
        "n": n,
        "mean_objects": statistics.mean(len(task.placements) for task in generated),
        "min_gap_m": min(gaps),
        "median_gap_m": statistics.median(gaps),
        "mean_retries": statistics.mean(task.retries for task in generated),
      }
      print(json.dumps(record, separators=(",", ":")))


def main() -> None:
  parser = argparse.ArgumentParser()
  subparsers = parser.add_subparsers(dest="command", required=True)
  preview_parser = subparsers.add_parser("preview")
  preview_parser.add_argument("--family", choices=FAMILIES, required=True)
  preview_parser.add_argument("--seed", type=int, required=True)
  stats_parser = subparsers.add_parser("stats")
  stats_parser.add_argument("--n", type=int, default=200)
  args = parser.parse_args()
  if args.command == "preview":
    task = generate(args.family, args.seed)
    print(json.dumps(dataclasses.asdict(task), indent=2))
    _preview(task)
  else:
    _stats(args.n)


if __name__ == "__main__":
  main()
