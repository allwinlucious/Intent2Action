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

## Addendum A (review resolutions; supersedes the decisions above where they differ)

### A.1 Scope

Create `learn/families.py`, `tests/test_families.py`. Modify `tests/e2e/hard_task.py` (A.9) and
`docs/designs/learning_from_experience.md` (mixed family: colour sorting only). No new dependency: the preview
uses Pillow (`PIL.ImageDraw`, already in `.venv`) instead of matplotlib.

### A.2 Seeds and splits

`generate(family, seed, catalog_root=CATALOG_ROOT)`: seed 0–9999 → `train`, 100000–100999 → `heldout`, any
other seed → `ValueError`; `mixed` with a train seed → `ValueError("mixed is held out entirely")`. `tasks(family,
split, n, catalog_root=CATALOG_ROOT)` returns seeds `range(start, start + n)` of that split (n beyond the range →
ValueError). `Task` gains `retries: int` (`dataclasses.field(compare=False)`): the number of rejected draws.
Done criteria previews: `--seed 3` for cluster, packing, stacking, reach; `--seed 100003` for mixed.

### A.3 Validity (replaces decision 2)

*   Table-resting objects (z = 0) have pairwise non-overlapping footprints and the family's gap rule.
*   Footprint corners (yaw-rotated) lie inside the table's footprint (catalog `table` dims, pose from
    `sim/scenes/default.yaml`) shrunk by 0.02 m; footprint centres are 0.30–0.75 m from the origin.
*   Cubes inside a box (z = the box floor top) lie with all corners inside the box's inner floor (box dims minus
    the catalog wall thickness on each side) and do not overlap each other.
*   Stacked cubes: same x, y and yaw as the cube below; z = its z + its height.
*   A cube resting on the table is ≥ 0.10 m from every box footprint; boxes are ≥ 0.06 m apart.
*   One draw = one full layout attempt; at most 1000 draws, then `RuntimeError(f"{family} seed {seed}: no valid
    layout in 1000 draws")`.

### A.4 Families (replaces the table in decision 3)

Constants: medium cube 0.05 m, small 0.035 m (from the yaml); `pitch = 0.05 + gap`.
*   **cluster**: shape uniform from plus, L, T, line, block, with member offsets (in pitch units) taken in this
    order and n uniform in the range: plus `(0,0),(1,0),(-1,0),(0,1),(0,-1)` n 4–5; L
    `(0,0),(1,0),(2,0),(0,1),(0,2),(3,0),(0,3)` n 4–7; T `(0,0),(1,0),(-1,0),(0,1),(0,2),(2,0),(-2,0)` n 4–7;
    line `(i,0)` i = 0..n−1 n 3–5; block `(i,r)` r = 0,1, i = 0..⌈n/2⌉−1, first n, n 4–6. Gap uniform 0.010–0.030 m
    (one per task). Offsets centred on their mean, rotated by θ ~ U(−π/4, π/4), placed at centre x ~ U(0.45, 0.55),
    y ~ U(0.00, 0.15); every cube yaw θ. Colours uniform from red, blue, green; with probability 0.25 one random
    member becomes `small_yellow_cube`. One box, colour uniform, at x ~ U(0.35, 0.45), y ~ U(−0.30, −0.24), yaw 0.
    Intent: every cube into that box.
*   **packing**: one box as in cluster. Inner-floor grid: cell pitch 0.065 m, columns and rows = ⌊(inner − 0.01) /
    0.065⌋ (6 cells for the current boxes), cells centred in the floor. k pre-filled cells uniform 2–4 (random cells),
    filled with `blue_cube`; m outside cubes uniform 1..(cells − k), colours uniform from red and green, spread
    on the table (gaps ≥ 0.06 m) at x ~ U(0.35, 0.60), y ~ U(0.00, 0.25), yaw ~ U(−0.3, 0.3). Intent: one rule
    per colour present outside (`{"category": "cube", "color": c}` in the box), `others: stay`. All outside cubes
    fit by construction (no overflow tasks: the rule checker has no partial success).
*   **stacking**: build or unstack, 50/50. Build: 2–4 cubes, colours distinct, drawn without replacement from red,
    blue, green, yellow; yellow (small) may only be the top; spread as packing's outside cubes; intent: rule
    i (`select` colour of level i, `on`, `target` colour of level i−1) for i = 1..n−1, `others: stay`. Unstack: a
    tower of 2–4 such cubes at x ~ U(0.45, 0.55), y ~ U(0.00, 0.15), yaw ~ U(−0.3, 0.3), plus one box as in
    cluster; intent every cube into the box.
*   **reach**: 3–5 cubes (colours uniform red/blue/green); each, 50/50: far (radius ~ U(0.62, 0.72), angle ~
    U(−25°, 25°)) or side (x ~ U(0.30, 0.45), y ~ U(0.28, 0.34)); yaw ~ U(−0.3, 0.3); gaps ≥ 0.06 m. Obstacles
    0–2 (`obstacle`), each at 70 % of the way from the origin to a random cube, yaw 0, subject to the same
    validity. Box as in cluster. Intent every cube into the box.
*   **mixed** (held out): a cluster as in cluster with n 4–6 and colours from red and blue only; `red_box` at x ~
    U(0.35, 0.42), y ~ U(−0.30, −0.24) and `blue_box` at x ~ U(0.60, 0.66), y ~ U(−0.24, −0.18); one of them
    (50/50) pre-filled with 2–3 `green_cube` as in packing. Intent: red cubes in the red box, blue cubes in the
    blue box, `others: stay` (the green cubes stay).

### A.5 Intents

Every `Task.intent` is a complete intent (`{"description": str, "rules": [...], "others": "stay"}`) for which
`validate_intent(intent) == []`, and every rule's `target` matches exactly one object of the task (unique
colours where a target is a cube). Descriptions: cluster/reach/unstack "every cube goes into the <c> box";
packing "the <colours> cubes go into the <c> box"; build "stack the cubes: <top> on ... on <bottom>"; mixed
"sort by colour: red cubes into the red box, blue cubes into the blue box".

### A.6 Loader (replaces decision 4)

`load_placements(placements, sim_stub, robot_stub, timeout_s=30.0) -> list[str]` and `load(task, ...)` (calls it).
Every RPC has a 30 s deadline. After the AddObjects it polls `ListObjects` every 0.5 s until all IDs are listed or
`timeout_s` passes, then raises `RuntimeError("objects did not appear in ListObjects: <ids>")`. No cleanup on
failure (the next load resets). Returns the IDs in placement order.

### A.7 Fake scene

`fake_objects(task, catalog_root=CATALOG_ROOT)`: IDs `obj_<10 + index>`, category/colour/size and dims from the
yaml, z as placed.

### A.8 Preview and stats

`preview` PNG via Pillow, 800 × 800 px, 1 px = 1.25 mm, origin at the left-centre; footprints filled in their
colour, boxes as 3 px outlines, obstacles grey, the 0.75 m reach arc. `stats --n 200`: JSON lines, one per
existing family/split pair: `{"family", "split", "n", "mean_objects", "min_gap_m", "median_gap_m",
"mean_retries"}`; gaps are nearest-neighbour edge distances between table-resting cubes.

### A.9 hard_task

`hard_task.py` keeps its layout constants and calls `families.load_placements([...])` instead of its own loop.

### A.10 Tests (replace decision 7)

All of decision 7 except the overlap test, plus: per-family counts and ranges (shapes, n, gap equal per task,
pre-fill k, stack heights and z, reach radii/sides, obstacle count); validity rules of A.3; split errors of A.2;
intent targets unique and `check_intent(intent, truth, truth)` not ok (the task is not already solved) with
`fake_objects` converted to `TrueObj`; `load_placements` with fake stubs: call order, returned IDs, timeout
raises; `catalog_root` override; preview writes a PNG of 800 × 800; stats lines parse.
