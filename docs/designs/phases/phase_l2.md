# Phase L2: task families

Part of [learning_from_experience.md](../learning_from_experience.md). A deterministic generator of hard
pick-and-place tasks (layout + intent) in families, with train / held-out splits, and one loader that puts a task
into a running stack (Isaac or the plan tier, the same robot_service SimService RPCs). Phases L3 (expert), L4
(data) and L6 (evaluation) use it.

## Constraints for the worker

*   Allowed: editing files, `.venv/bin/python -m pytest tests/ -q`, `.venv/bin/python learn/families.py ...`.
    Not allowed: Isaac Sim, Docker, any `tests/e2e/*` script, Bazel, GPU, model calls, `git commit`, `pkill -f`,
    modifying `/home/mew/intrinsic-core`.
*   Make no design decisions beyond this plan; anything uncovered goes to `docs/designs/phases/phase_l2_notes.md`.
*   Code style of the surrounding files (2-space indent, comments only for the why).

## Done criteria

*   `.venv/bin/python -m pytest tests/ -q` passes, including the new `tests/test_families.py` (decision 7).
*   `.venv/bin/python learn/families.py preview --family cluster --seed 3` prints the task as JSON and writes a
    top-down PNG of the layout to `logs/families/cluster_3.png` (decision 6); the same for every family.
*   `phase_l2_notes.md` lists files changed and the test output.

## Decisions

### 1. Module and data (`learn/families.py`)

```python
@dataclasses.dataclass(frozen=True)
class Placement:
  catalog: str           # a directory name under assets/objects
  x: float; y: float; z: float; yaw: float   # base frame, metres / rad; z = bottom of the object

@dataclasses.dataclass(frozen=True)
class Task:
  family: str            # cluster | packing | stacking | reach | mixed
  seed: int
  split: str             # train | heldout
  intent: dict           # program_runtime.rules intent schema; validate_intent(intent) == []
  placements: tuple[Placement, ...]

def generate(family: str, seed: int) -> Task
def tasks(family: str, split: str, n: int) -> list[Task]   # the first n seeds of that split
FAMILIES = ("cluster", "packing", "stacking", "reach", "mixed")
```

*   Determinism: one `random.Random(f"{family}:{seed}")` per task; nothing else is random.
*   Splits: seeds 0–9999 are `train`, 100000–100999 are `heldout`. The family `mixed` has no train split
    (`tasks("mixed", "train", n)` raises ValueError): it is held out entirely.
*   Catalog sizes are read from `assets/objects/<catalog>/object.yaml` (`dims`, `category`, `color_name`,
    `container`), never hard-coded.
*   Cubes: `red_cube`, `blue_cube`, `green_cube` (0.05 m), `small_yellow_cube` (0.035 m). Boxes: `red_box`,
    `blue_box`. Obstacle: `obstacle`.

### 2. Workspace and validity (every family)

*   Every object's footprint centre is within 0.30–0.75 m of the base (the origin) and has y in −0.40..0.40 and
    x ≥ 0.20, except where a family says otherwise.
*   No two footprints (yaw-rotated rectangles) overlap; the gap between any two objects that the family does not
    deliberately cluster is ≥ 0.06 m. Boxes are ≥ 0.10 m from every cube outside them.
*   Generation retries with the same RNG until valid (at most 1000 draws, then RuntimeError naming the family
    and seed).

### 3. Families

| Family | Layout | Intent |
|---|---|---|
| cluster | 4–7 cubes (colours random, at most 1 small) in one shape drawn from plus, L, line, 2-row block, ring; neighbour gaps uniform 0.010–0.030 m, per task one gap value; shape rotated by a random yaw; cluster centre in x 0.42–0.58, y −0.05..0.20; one box (random colour) at x 0.35–0.45, y −0.30..−0.22 | every cube into the box |
| packing | one box; k cubes already inside on a grid of the box floor (1 cm margin to the walls, 1 cm between cubes), k = 60–100 % of the grid cells minus the cubes to add; 1–3 more cubes outside, spread (gaps ≥ 0.06 m) | every cube into the box (when the box cannot take them all, the expected result is the box full and the rest staying, which the rule checker reports per object) |
| stacking | build: 2–4 cubes spread, intent a tower in a random order (rule `on` per level); unstack: a tower of 2–4 cubes (z = sum of heights below) and a box | build: the tower; unstack: every cube into the box |
| reach | 3–5 cubes at 0.62–0.75 m from the base or at |y| 0.30–0.40, plus 0–2 obstacles between some cubes and the robot; one box at x 0.35–0.45 | every cube into the box |
| mixed | a cluster as in `cluster` (4–6 cubes, 2–3 colours) plus two boxes (red and blue), one of them 60–80 % full as in `packing` | sort by colour: red cubes into the red box, blue cubes into the blue box, others stay |

Intent rules use the existing schema (`select` / `relation` / `target`, `others`), e.g. every cube into the red
box: `{"select": {"category": "cube"}, "relation": "in", "target": {"category": "box", "color": "red"}}`. Cubes
already inside a box are selected by the same rule (they satisfy it already).

### 4. Loader (`learn/families.py`)

```python
def load(task: Task, sim_stub, robot_stub, timeout_s: float = 30.0) -> dict[str, str]
```
Does what `tests/e2e/hard_task.py` does (lines 80–92): `ResetScene`, remove every catalog object except the
table, `AddObject` each placement (pose x, y, z, yaw as a quaternion about z), then wait until `ListObjects`
shows them all. Returns placement index (as a string) → object ID. `hard_task.py` is changed to call `load` with
an equivalent `Task` (family `cluster`, its fixed layout) so there is one loader.

### 5. Fake scene

```python
def fake_objects(task: Task) -> list[program_runtime.api.Obj]
```
The `Obj` list `learn/glimmer_program_test.py` builds by hand, from the placements (IDs `obj_<10 + index>`),
for program writing without a robot. `glimmer_program_test.py` is not changed in this phase.

### 6. Preview (`learn/families.py preview|stats`)

*   `preview --family F --seed S`: prints the task as JSON; writes a top-down PNG (matplotlib, already in the
    venv; footprints as rotated rectangles in their colour, boxes as outlines, the robot base at the origin, a
    0.75 m reach circle) to `logs/families/<family>_<seed>.png`.
*   `stats --n 200`: per family and split, the mean object count, min/median gap, and the generation retries.

### 7. Tests (`tests/test_families.py`)

For every family and split it has: 200 tasks generate; the same seed gives the same task; every intent validates;
no footprint overlap; workspace bounds hold; `mixed` has no train split; train and held-out tasks never share a
seed; catalog dimensions come from the yaml files (change a dim in a temp copy → generator uses it, via a
catalog-root argument defaulting to `assets/objects`).
