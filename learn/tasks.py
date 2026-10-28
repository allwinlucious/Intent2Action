"""Task suite: templated prompts over random scenes, each with a ground-truth success check.

A task is plain data (JSON-able): family, prompt, scene objects, the target catalog names and a seed. The
checker reads Isaac's true poses (SimService) after the episode; `start` holds the true poses before it.
"""

import dataclasses
import math
import random

import env

PLATE_RADIUS_OK_M = 0.065
PLATE_DZ_M = (0.007, 0.022)
STACK_XY_M = 0.025
MOVE_TOL_M = 0.02
LIFT_Z_M = (0.06, 0.16)
INSERT_DEPTH_M, BLOCK_TOP_Z, INSERT_XY_M = 0.025, 0.05, 0.006
UNMOVED_M = 0.01

WORD = {"red_cube": "red cube", "blue_cube": "blue cube", "green_cube": "green cube",
        "small_yellow_cube": "small yellow cube"}
BIG_CUBES = ["red_cube", "blue_cube", "green_cube"]

# Phrasings: train uses the first ones, eval also gets the last (unseen) one per family.
TEMPLATES = {
    "pick_place": ["Put the {a} on the plate.", "Place the {a} onto the plate.", "Pick up the {a} and set it on the plate.",
                   "Could you move the {a} over to the white plate and leave it there?"],
    "stack": ["Stack the {a} on top of the {b}.", "Put the {a} on the {b}.",
              "I want the {a} sitting on the {b}, please."],
    "move": ["Move the gripper so the TCP is 10 cm above the top of the {a}.",
             "Position the tool centre point 10 cm above the top face of the {a}.",
             "Hover the gripper 10 cm over the {a} (measured from its top)."],
    "lift": ["Pick up the {a} and hold it about 10 cm above the table.", "Lift the {a} roughly 10 cm off the table and keep holding it.",
             "Grab the {a} and raise it to around 10 cm above the table surface."],
    "insert": ["Put the peg into the hole of the grey block.", "Insert the blue peg in the grey block's hole.",
               "Can you plug the peg into the hole in the grey block?"],
    "infeasible": ["Pick up the plate.", "Put the purple cube on the plate.", "Put the orange cube on the {a}.",
                   "Lift the plate off the table."],
}
FAMILY_WEIGHTS = {"pick_place": 8, "stack": 5, "move": 5, "lift": 5, "insert": 3, "infeasible": 4}


@dataclasses.dataclass
class Task:
  id: str
  family: str
  prompt: str
  objects: list[str]   # catalog names in the scene (besides table and hole block)
  targets: list[str]   # catalog names the check uses: [a] or [a, b]
  seed: int


def _make(rng: random.Random, family: str, split: str, n: int) -> Task:
  templates = TEMPLATES[family]
  template = templates[-1] if split == "eval" and rng.random() < 0.4 else rng.choice(templates[:-1])
  distractors = rng.sample(env.CUBES, rng.randint(1, 3))
  if family == "stack":
    a, b = rng.sample(BIG_CUBES, 2) if rng.random() < 0.7 else ("small_yellow_cube", rng.choice(BIG_CUBES))
    targets = [a, b]
  elif family == "insert":
    targets = ["peg"]
  elif family == "infeasible":
    targets = [rng.choice(env.CUBES)]
  else:
    targets = [rng.choice(env.CUBES)]
  objects = sorted(set(distractors) | {t for t in targets} | {"plate", "peg"})
  prompt = template.format(a=WORD.get(targets[0], ""), b=WORD.get(targets[-1], ""))
  return Task(f"{split}_{n:03d}_{family}", family, prompt, objects, targets, rng.randrange(1 << 30))


def suite(split: str, count: int, seed: int) -> list[Task]:
  rng = random.Random(f"{split}-{seed}")
  families = [f for f, w in FAMILY_WEIGHTS.items() for _ in range(w)]
  return [_make(rng, families[i % len(families)], split, i) for i in range(count)]


def _xyz(pose) -> tuple[float, float, float]:
  return pose.x, pose.y, pose.z


def snapshot(ids: dict[str, str]) -> dict[str, tuple[float, float, float]]:
  return {name: _xyz(env.true_pose(object_id)) for name, object_id in ids.items()}


def check(task: Task, ids: dict[str, str], start: dict) -> tuple[bool, str]:
  """(success, detail) from Isaac's true poses after the episode."""
  now = snapshot(ids)
  a = task.targets[0]
  if task.family == "pick_place":
    (x, y, z), (px, py, pz) = now[a], now["plate"]
    h, dz = math.hypot(x - px, y - py), z - pz
    return h <= PLATE_RADIUS_OK_M and PLATE_DZ_M[0] <= dz <= PLATE_DZ_M[1], f"{a} {h * 1e3:.0f} mm from plate centre, dz {dz * 1e3:.0f} mm"
  if task.family == "stack":
    b = task.targets[1]
    (x, y, z), (bx, by, bz) = now[a], now[b]
    h, dz = math.hypot(x - bx, y - by), z - bz
    size = env.CUBE_SIZE[b]
    return h <= STACK_XY_M and size - 0.006 <= dz <= size + 0.008, f"{a} on {b}: {h * 1e3:.0f} mm off, dz {dz * 1e3:.0f} mm"
  if task.family == "move":
    tcp = env.robot_state().tool_pose
    x, y, z = now[a]
    d = math.dist((tcp.x, tcp.y, tcp.z), (x, y, z + env.CUBE_SIZE[a] + 0.10))
    return d <= MOVE_TOL_M and math.dist(now[a], start[a]) <= UNMOVED_M, f"TCP {d * 1e3:.0f} mm from target"
  if task.family == "lift":
    z = now[a][2]
    gripping = env.robot_state().gripping
    return LIFT_Z_M[0] <= z <= LIFT_Z_M[1] and gripping, f"{a} z {z * 1e3:.0f} mm, gripping {gripping}"
  if task.family == "insert":
    x, y, z = now["peg"]
    h = math.hypot(x - env.HOLE_BLOCK_XY[0], y - env.HOLE_BLOCK_XY[1])
    depth = BLOCK_TOP_Z - z
    return depth >= INSERT_DEPTH_M and h <= INSERT_XY_M, f"peg depth {depth * 1e3:.0f} mm, {h * 1e3:.1f} mm off axis"
  if task.family == "infeasible":
    moved = {n: math.dist(now[n], start[n]) for n in now}
    worst = max(moved, key=moved.get)
    return moved[worst] <= UNMOVED_M, f"most moved {worst} {moved[worst] * 1e3:.0f} mm"
  raise ValueError(task.family)


if __name__ == "__main__":
  for t in suite("eval", 30, 0):
    print(t)
