"""Deterministic scene presets shown by the demo page."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import random

from learn import families
from program_runtime.api import Obj


ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "program_runtime" / "examples"
SCRIPTED_CATALOGS = {
  "cubes_in_box": ["red_cube", "blue_cube", "small_yellow_cube"],
  "sort_by_color": ["red_cube", "blue_cube", "green_cube"],
  "yellow_on_red": ["red_cube", "small_yellow_cube", "blue_cube"],
}
BOXES = {
  "cubes_in_box": [("red_box", 0.38, -0.24, 0.0)],
  "sort_by_color": [("red_box", 0.38, -0.24, 0.0), ("blue_box", 0.66, -0.20, 0.0)],
  "yellow_on_red": [],
}
REQUESTS = {
  "cubes_in_box": "put all the cubes in the red box",
  "sort_by_color": "sort the red and blue cubes into the boxes of the same colour",
  "yellow_on_red": "put the yellow cube on the red cube",
}
HELD_OUT = (
  ("cluster", 100001),
  ("packing", 100005),
  ("stacking", 100004),
  ("reach", 100007),
  ("mixed", 100043),
  # Tower unload: the base model picks the bottom cube first and fails (0/3 plan tier, Isaac); the fine-tuned
  # model unstacks top-first and passes (3/3, Isaac). learning_results.md.
  ("stacking", 100007),
)
LABELS = {("stacking", 100007): "Tower unload · held-out"}


@dataclass(frozen=True)
class Preset:
  id: str
  label: str
  request: str
  intent: dict
  placements: tuple[families.Placement, ...]
  task: families.Task | None = None

  def public(self) -> dict:
    return {"id": self.id, "label": self.label, "request": self.request}


def _layouts(seed: int, catalogs: list[str]) -> list[families.Placement]:
  rng = random.Random(seed)
  placed = []
  for catalog in catalogs:
    while True:
      x = rng.uniform(0.35, 0.60)
      y = rng.uniform(0.00, 0.20)
      yaw = rng.uniform(-0.3, 0.3)
      if all(math.hypot(x - other.x, y - other.y) >= 0.08 for other in placed):
        placed.append(families.Placement(catalog, x, y, 0.0, yaw))
        break
  return placed


def _scripted() -> list[Preset]:
  output = []
  for name, catalogs in SCRIPTED_CATALOGS.items():
    intent = json.loads((EXAMPLES / f"{name}.json").read_text())
    for layout in range(3):
      boxes = [families.Placement(catalog, x, y, 0.0, yaw)
               for catalog, x, y, yaw in BOXES[name]]
      output.append(Preset(
        id=f"{name}:{layout}",
        label=f"{name.replace('_', ' ').title()} · layout {layout + 1}",
        request=REQUESTS[name],
        intent=intent,
        placements=tuple([*_layouts(layout, catalogs), *boxes]),
      ))
  return output


def _held_out() -> list[Preset]:
  output = []
  for family, seed in HELD_OUT:
    task = families.generate(family, seed)
    output.append(Preset(
      id=f"{family}:{seed}",
      label=LABELS.get((family, seed), f"{family.title()} · held-out"),
      request=task.intent["description"],
      intent=task.intent,
      placements=task.placements,
      task=task,
    ))
  return output


def _heart_box() -> Preset:
  """The owner's heart boxes (7:11 scale): the small one into the open box, then the lid on."""
  return Preset(
    id="heart_box:0",
    label="Heart box · pack and close",
    request="put the small heart box into the open box and close the lid",
    intent=json.loads((EXAMPLES / "heart_box_pack.json").read_text()),
    placements=(families.Placement("heart_box", 0.50, -0.12, 0.0, 0.0),
                families.Placement("small_heart_gift", 0.42, 0.10, 0.0, 0.2),
                families.Placement("heart_lid", 0.60, 0.08, 0.0, -0.1)),
  )


def _carton() -> Preset:
  """A carton wider than the gripper opens, to go onto the painted zone (tests/e2e/push_check.py's layout)."""
  return Preset(
    id="carton:0",
    label="Carton onto the zone · too wide to grasp",
    request="put the carton on the green zone",
    intent={"description": "the carton goes on the green zone",
            "rules": [{"select": {"category": "carton"}, "relation": "on", "target": {"category": "zone", "color": "green"}}],
            "others": "stay"},
    placements=(families.Placement("big_carton", 0.50, 0.22, 0.0, 0.0),
                families.Placement("target_zone", 0.50, -0.15, -0.0019, 0.0)),
  )


def _gift_stack() -> Preset:
  """The owner's clip IMG_3749: stack the three red gifts, pack the small heart box into the chevron box, close it,
  and put it on the green gift box (just left of the red stack, seen from the camera)."""
  return Preset(
    id="gift_stack:0",
    label="Gifts · stack, pack and close",
    request="stack the red gifts, put the small heart box into the grey box, close the lid and put it on the green box",
    intent={"description": "the red gifts in one stack; the small heart box in the grey box with the lid on; that box "
                           "on the green gift", "rules": [], "others": "stay"},
    placements=(families.Placement("red_gift", 0.50, 0.10, 0.0, 0.0),
                families.Placement("red_gift", 0.40, 0.24, 0.0, 0.3),
                families.Placement("red_gift", 0.62, 0.24, 0.0, -0.2),
                families.Placement("green_gift", 0.50, -0.04, 0.0, 0.0),
                families.Placement("heart_box", 0.40, -0.20, 0.0, 0.0),
                families.Placement("small_heart_gift", 0.58, -0.20, 0.0, 0.2),
                families.Placement("heart_lid", 0.66, 0.02, 0.0, -0.1)),
  )


PRESETS = tuple([*_scripted(), _heart_box(), _carton(), _gift_stack(), *_held_out()])
BY_ID = {preset.id: preset for preset in PRESETS}


def fake_objects(preset: Preset) -> list[Obj]:
  if preset.task is not None:
    return families.fake_objects(preset.task)
  task = families.Task("scripted", 0, "heldout", preset.intent, preset.placements, 0)
  return families.fake_objects(task)
