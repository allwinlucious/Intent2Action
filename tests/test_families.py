import json
import math
import shutil
from types import SimpleNamespace

from PIL import Image
import pytest
import yaml

from learn import families
from program_runtime.rules import TrueObj, check_intent, validate_intent


PAIRS = [
  (family, split)
  for family in families.FAMILIES
  for split in ("train", "heldout")
  if not (family == "mixed" and split == "train")
]


@pytest.fixture(scope="session")
def specs():
  return families._specs(families.CATALOG_ROOT)


@pytest.fixture(scope="session")
def generated():
  return {(family, split): families.tasks(family, split, 200) for family, split in PAIRS}


def _truth(task):
  output = {}
  for obj in families.fake_objects(task):
    output[obj.id] = TrueObj(
      id=obj.id, category=obj.category, color=obj.color, size=obj.size,
      x=obj.x, y=obj.y, z=obj.z, yaw=obj.yaw, sx=obj.sx, sy=obj.sy, sz=obj.sz,
    )
  return output


def _matches(obj, filters):
  return all(getattr(obj, key) == value for key, value in filters.items())


def _category(placement, specs):
  return specs[placement.catalog]["category"]


def _shape(cubes, specs, *, mixed=False):
  n = len(cubes)
  mean_x = sum(cube.x for cube in cubes) / n
  mean_y = sum(cube.y for cube in cubes) / n
  yaw = cubes[0].yaw
  assert all(math.isclose(cube.yaw, yaw, abs_tol=1e-12) for cube in cubes)
  pitch = min(
    math.hypot(first.x - second.x, first.y - second.y)
    for index, first in enumerate(cubes)
    for second in cubes[index + 1:]
  )
  cosine, sine = math.cos(yaw), math.sin(yaw)
  actual = []
  for cube in cubes:
    dx, dy = cube.x - mean_x, cube.y - mean_y
    actual.append(((cosine * dx + sine * dy) / pitch,
                   (-sine * dx + cosine * dy) / pitch))

  found = []
  for name, (low, high) in families.SHAPE_COUNTS.items():
    if mixed:
      low, high = max(low, 4), min(high, 6)
    if not low <= n <= high:
      continue
    offsets = families._shape_offsets(name, n)
    offset_x = sum(x for x, _ in offsets) / n
    offset_y = sum(y for _, y in offsets) / n
    expected = [(x - offset_x, y - offset_y) for x, y in offsets]
    if all(math.isclose(ax, ex, abs_tol=1e-9) and math.isclose(ay, ey, abs_tol=1e-9)
           for (ax, ay), (ex, ey) in zip(actual, expected)):
      found.append(name)
  assert found
  gap = pitch - float(specs["red_cube"]["dims"][0])
  assert families.GAP_RANGE_M[0] <= gap <= families.GAP_RANGE_M[1]
  assert -math.pi / 4 <= yaw <= math.pi / 4
  assert 0.45 <= mean_x <= 0.55
  assert 0.00 <= mean_y <= 0.15
  return set(found)


def _assert_standard_box(box):
  assert box.catalog in families.BOX_CATALOGS
  assert 0.35 <= box.x <= 0.45
  assert -0.30 <= box.y <= -0.24
  assert box.z == 0.0
  assert box.yaw == 0.0


@pytest.mark.parametrize("family,split", PAIRS)
def test_generation_split_determinism_and_intents(family, split, generated):
  tasks = generated[(family, split)]
  start = 0 if split == "train" else 100000
  assert len(tasks) == 200
  assert [task.seed for task in tasks] == list(range(start, start + 200))
  assert all(task.family == family and task.split == split for task in tasks)
  assert families.generate(family, start) == tasks[0]
  for task in tasks:
    assert task.retries >= 0
    assert validate_intent(task.intent) == []
    truth = _truth(task)
    for rule in task.intent["rules"]:
      assert sum(_matches(obj, rule["target"]) for obj in truth.values()) == 1
    assert not check_intent(task.intent, truth, truth).ok


def test_split_errors_and_disjoint_seeds(generated):
  for family in [f for f in families.FAMILIES if f != "mixed"]:
    assert not ({task.seed for task in generated[(family, "train")]} &
                {task.seed for task in generated[(family, "heldout")]})
  with pytest.raises(ValueError, match="mixed is held out entirely"):
    families.generate("mixed", 0)
  with pytest.raises(ValueError, match="mixed is held out entirely"):
    families.tasks("mixed", "train", 1)
  with pytest.raises(ValueError):
    families.generate("cluster", -1)
  with pytest.raises(ValueError):
    families.generate("cluster", 10000)
  with pytest.raises(ValueError):
    families.generate("cluster", 101000)
  with pytest.raises(ValueError):
    families.tasks("cluster", "train", 10001)
  with pytest.raises(ValueError):
    families.tasks("cluster", "heldout", 1001)
  with pytest.raises(ValueError):
    families.tasks("cluster", "invalid", 1)


def test_validity_rules(generated, specs):
  table = families.TABLE_SCENE
  table_x, table_y = (float(value) for value in table["position"][:2])
  table_yaw = math.radians(float(table.get("yaw", 0.0)))
  table_sx, table_sy = (float(value) - 0.04 for value in specs["table"]["dims"][:2])
  table_cosine, table_sine = math.cos(table_yaw), math.sin(table_yaw)

  for tasks in generated.values():
    for task in tasks:
      boxes = [placement for placement in task.placements if _category(placement, specs) == "box"]
      cubes = [placement for placement in task.placements if _category(placement, specs) == "cube"]
      for placement in task.placements:
        assert 0.30 <= math.hypot(placement.x, placement.y) <= 0.75
      resting = [placement for placement in task.placements if math.isclose(placement.z, 0.0, abs_tol=1e-12)]
      for placement in resting:
        for x, y in families._corners(placement, specs):
          dx, dy = x - table_x, y - table_y
          local_x = table_cosine * dx + table_sine * dy
          local_y = -table_sine * dx + table_cosine * dy
          assert abs(local_x) <= table_sx / 2 + 1e-12
          assert abs(local_y) <= table_sy / 2 + 1e-12

      for index, first in enumerate(resting):
        first_category = _category(first, specs)
        for second in resting[index + 1:]:
          second_category = _category(second, specs)
          distance = families._polygon_distance(
            families._corners(first, specs), families._corners(second, specs)
          )
          assert distance > 1e-12
          categories = {first_category, second_category}
          if categories == {"cube", "box"}:
            assert distance >= 0.10 - 1e-12
          if first_category == second_category == "box":
            assert distance >= 0.02 - 1e-12
          if task.family in {"packing", "stacking"} and first_category == second_category == "cube":
            assert distance >= 0.06 - 1e-12
          if task.family == "reach" and "box" not in categories:
            assert distance >= 0.06 - 1e-12
          if task.family == "reach" and "pillar" in categories and "box" in categories:
            assert distance >= 0.06 - 1e-12

      contained = []
      for cube in cubes:
        containers = [box for box in boxes if families._inside_box(cube, box, specs)]
        if containers:
          assert len(containers) == 1
          contained.append((cube, containers[0]))
        elif cube.z > 1e-12:
          below = [other for other in cubes if other is not cube
                   and math.isclose(cube.x, other.x, abs_tol=1e-12)
                   and math.isclose(cube.y, other.y, abs_tol=1e-12)
                   and math.isclose(cube.yaw, other.yaw, abs_tol=1e-12)
                   and math.isclose(cube.z, other.z + float(specs[other.catalog]["dims"][2]), abs_tol=1e-12)]
          assert len(below) == 1
      for index, (cube, box) in enumerate(contained):
        for other, other_box in contained[index + 1:]:
          if box is other_box:
            assert families._polygon_distance(
              families._corners(cube, specs), families._corners(other, specs)
            ) > 1e-12


def test_cluster_family_ranges(generated, specs):
  for split in ("train", "heldout"):
    shapes = set()
    for task in generated[("cluster", split)]:
      cubes = [placement for placement in task.placements if _category(placement, specs) == "cube"]
      boxes = [placement for placement in task.placements if _category(placement, specs) == "box"]
      assert 3 <= len(cubes) <= 7
      assert len(boxes) == 1
      assert sum(cube.catalog == "small_yellow_cube" for cube in cubes) <= 1
      assert all(cube.catalog in families.CUBE_CATALOGS for cube in cubes)
      shapes.update(_shape(cubes, specs))
      _assert_standard_box(boxes[0])
    assert shapes == set(families.SHAPE_COUNTS)


def test_packing_family_ranges(generated, specs):
  for split in ("train", "heldout"):
    for task in generated[("packing", split)]:
      box = next(placement for placement in task.placements if _category(placement, specs) == "box")
      cubes = [placement for placement in task.placements if _category(placement, specs) == "cube"]
      inside = [cube for cube in cubes if families._inside_box(cube, box, specs)]
      outside = [cube for cube in cubes if cube not in inside]
      assert 1 <= len(inside) <= 4
      spots = [(c.x, c.y) for c in inside]
      # Pre-filled the way Place packs (its own spot order), and every outside cube still fits.
      assert any(spots == families._planner_spots(specs, box, "blue_cube", yaw, [])[:len(inside)]
                 for yaw in (0.0, math.pi / 2))
      room = min(len(families._planner_spots(specs, box, "red_cube", yaw, spots)) for yaw in (0.0, math.pi / 2))
      assert 1 <= len(outside) <= room
      assert all(cube.catalog == "blue_cube" for cube in inside)
      assert all(cube.catalog in {"red_cube", "green_cube"} for cube in outside)
      assert all(0.35 <= cube.x <= 0.60 and 0.00 <= cube.y <= 0.25 and -0.3 <= cube.yaw <= 0.3
                 for cube in outside)
      _assert_standard_box(box)


def test_stacking_family_ranges(generated, specs):
  for split in ("train", "heldout"):
    kinds = set()
    for task in generated[("stacking", split)]:
      cubes = [placement for placement in task.placements if _category(placement, specs) == "cube"]
      boxes = [placement for placement in task.placements if _category(placement, specs) == "box"]
      assert 2 <= len(cubes) <= 4
      assert len({specs[cube.catalog]["color_name"] for cube in cubes}) == len(cubes)
      if any(cube.catalog == "small_yellow_cube" for cube in cubes):
        assert cubes[-1].catalog == "small_yellow_cube"
      if boxes:
        kinds.add("unstack")
        assert len(boxes) == 1
        _assert_standard_box(boxes[0])
        height = 0.0
        for cube in cubes:
          assert math.isclose(cube.z, height, abs_tol=1e-12)
          assert math.isclose(cube.x, cubes[0].x, abs_tol=1e-12)
          assert math.isclose(cube.y, cubes[0].y, abs_tol=1e-12)
          assert math.isclose(cube.yaw, cubes[0].yaw, abs_tol=1e-12)
          height += float(specs[cube.catalog]["dims"][2])
        assert len(task.intent["rules"]) == 1
        assert task.intent["rules"][0]["relation"] == "in"
      else:
        kinds.add("build")
        assert all(cube.z == 0.0 for cube in cubes)
        assert len(task.intent["rules"]) == len(cubes) - 1
        colours = [specs[cube.catalog]["color_name"] for cube in cubes]
        for index, rule in enumerate(task.intent["rules"], 1):
          assert rule == {
            "select": {"category": "cube", "color": colours[index]},
            "relation": "on",
            "target": {"category": "cube", "color": colours[index - 1]},
          }
    assert kinds == {"build", "unstack"}


def test_reach_family_ranges(generated, specs):
  for split in ("train", "heldout"):
    for task in generated[("reach", split)]:
      cubes = [placement for placement in task.placements if _category(placement, specs) == "cube"]
      obstacles = [placement for placement in task.placements if placement.catalog == "obstacle"]
      boxes = [placement for placement in task.placements if _category(placement, specs) == "box"]
      assert 3 <= len(cubes) <= 5
      assert 0 <= len(obstacles) <= 2
      assert len(boxes) == 1
      for cube in cubes:
        radius = math.hypot(cube.x, cube.y)
        far = 0.62 <= radius <= 0.72 and -math.radians(25) <= math.atan2(cube.y, cube.x) <= math.radians(25)
        side = 0.30 <= cube.x <= 0.45 and 0.28 <= cube.y <= 0.34
        assert far or side
        assert -0.3 <= cube.yaw <= 0.3
      for obstacle in obstacles:
        assert obstacle.yaw == 0.0
        assert any(math.isclose(obstacle.x, 0.7 * cube.x, abs_tol=1e-12)
                   and math.isclose(obstacle.y, 0.7 * cube.y, abs_tol=1e-12) for cube in cubes)
      _assert_standard_box(boxes[0])


def test_mixed_family_ranges(generated, specs):
  for task in generated[("mixed", "heldout")]:
    cluster = [placement for placement in task.placements
               if placement.z == 0.0 and placement.catalog in {"red_cube", "blue_cube"}]
    green = [placement for placement in task.placements if placement.catalog == "green_cube"]
    boxes = {placement.catalog: placement for placement in task.placements if _category(placement, specs) == "box"}
    assert 4 <= len(cluster) <= 6
    _shape(cluster, specs, mixed=True)
    assert set(boxes) == {"red_box", "blue_box"}
    assert 0.34 <= boxes["red_box"].x <= 0.38
    assert -0.30 <= boxes["red_box"].y <= -0.24
    assert 0.66 <= boxes["blue_box"].x <= 0.70
    assert -0.24 <= boxes["blue_box"].y <= -0.18
    assert 2 <= len(green) <= 3
    containing = [[box for box in boxes.values() if families._inside_box(cube, box, specs)] for cube in green]
    assert all(len(values) == 1 for values in containing)
    assert len({values[0].catalog for values in containing}) == 1
    assert task.intent["description"] == "sort by colour: red cubes into the red box, blue cubes into the blue box"


def test_catalog_root_override(tmp_path):
  catalog_root = tmp_path / "objects"
  shutil.copytree(families.CATALOG_ROOT, catalog_root)
  path = catalog_root / "red_cube" / "object.yaml"
  spec = yaml.safe_load(path.read_text())
  spec["dims"][0] = 0.06
  path.write_text(yaml.safe_dump(spec, sort_keys=False))
  task = families.generate("cluster", 3, catalog_root)
  cubes = [placement for placement in task.placements if placement.catalog.endswith("cube")]
  pitch = min(math.hypot(first.x - second.x, first.y - second.y)
              for index, first in enumerate(cubes) for second in cubes[index + 1:])
  assert families.GAP_RANGE_M[0] <= pitch - 0.06 <= families.GAP_RANGE_M[1]
  manual = families.Task("cluster", 3, "train", task.intent,
                         (families.Placement("red_cube", 0.5, 0.0, 0.0, 0.0),), 0)
  assert families.fake_objects(manual, catalog_root)[0].sx == 0.06


class FakeSimStub:
  def __init__(self, events):
    self.events = events
    self.added = 0

  def ResetScene(self, request, timeout):
    self.events.append(("ResetScene", timeout))
    return SimpleNamespace()

  def ListTrueObjects(self, request, timeout):
    self.events.append(("ListTrueObjects", timeout))
    return SimpleNamespace(objects=[
      SimpleNamespace(id="table_id", catalog="table"),
      SimpleNamespace(id="old_id", catalog="red_cube"),
    ])

  def RemoveObject(self, request, timeout):
    self.events.append(("RemoveObject", request.id, timeout))
    return SimpleNamespace()

  def AddObject(self, request, timeout):
    object_id = f"new_{self.added}"
    self.added += 1
    self.events.append(("AddObject", request.catalog, request.pose, timeout))
    return SimpleNamespace(id=object_id)


class FakeRobotStub:
  def __init__(self, events, ids, *, appear=True):
    self.events = events
    self.ids = ids
    self.appear = appear
    self.calls = 0

  def ListObjects(self, request, timeout):
    self.calls += 1
    self.events.append(("ListObjects", timeout))
    listed = self.ids if self.appear and self.calls > 1 else []
    return SimpleNamespace(objects=[SimpleNamespace(id=object_id) for object_id in listed])


def test_load_placements_call_order_and_ids(monkeypatch):
  events = []
  sim = FakeSimStub(events)
  robot = FakeRobotStub(events, ["new_0", "new_1"])
  monkeypatch.setattr(families.time, "sleep", lambda seconds: None)
  placements = [
    families.Placement("red_cube", 0.5, 0.1, 0.0, math.pi / 2),
    families.Placement("blue_box", 0.4, -0.25, 0.0, 0.0),
  ]
  assert families.load_placements(placements, sim, robot) == ["new_0", "new_1"]
  assert [event[0] for event in events] == [
    "ResetScene", "ListTrueObjects", "RemoveObject", "AddObject", "AddObject", "ListObjects", "ListObjects",
  ]
  assert events[2] == ("RemoveObject", "old_id", 30)
  assert all(event[-1] == (60 if event[0] == "ResetScene" else 30) for event in events
             if event[0] != "RemoveObject" or len(event) == 3)
  first_pose = events[3][2]
  assert math.isclose(first_pose.qz, math.sin(math.pi / 4))
  assert math.isclose(first_pose.qw, math.cos(math.pi / 4))


def test_load_placements_timeout():
  events = []
  sim = FakeSimStub(events)
  robot = FakeRobotStub(events, ["new_0"], appear=False)
  with pytest.raises(RuntimeError, match=r"objects did not appear in ListObjects: \['new_0'\]"):
    families.load_placements([families.Placement("red_cube", 0.5, 0.1, 0.0, 0.0)],
                             sim, robot, timeout_s=0.0)


def test_preview_png(generated, tmp_path, monkeypatch):
  monkeypatch.setattr(families, "ROOT", tmp_path)
  output = families._preview(generated[("cluster", "train")][3])
  assert output == tmp_path / "logs" / "families" / "cluster_3.png"
  with Image.open(output) as image:
    assert image.format == "PNG"
    assert image.size == (800, 800)


def test_stats_json_lines(generated, monkeypatch, capsys):
  monkeypatch.setattr(
    families, "tasks",
    lambda family, split, n, catalog_root=families.CATALOG_ROOT: generated[(family, split)][:n],
  )
  families._stats(200)
  records = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
  assert len(records) == len(PAIRS)
  assert {(record["family"], record["split"]) for record in records} == set(PAIRS)
  assert all(set(record) == {
    "family", "split", "n", "mean_objects", "min_gap_m", "median_gap_m", "mean_retries",
  } for record in records)
  assert all(record["n"] == 200 for record in records)
