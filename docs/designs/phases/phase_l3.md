# Phase L3: episodes, strategy programs and the search expert

Part of [learning_from_experience.md](../learning_from_experience.md). Needs L1 (plan tier) and L2 (task families).
The expert is a search, not a model: it runs candidate **strategy programs** on a task in the plan tier, the rule
checker grades each run, and the first program that succeeds is the expert's program for that task. Failed runs
of other strategies become recovery examples whose expert decision is a patch that the plan tier confirms works.
Everything here produces JSONL records that phase L4 turns into training rows.

## Constraints for the worker

*   Allowed: editing files; unit tests (`.venv/bin/python -m pytest tests/ -q`); the plan tier
    (`ROBOT_BACKEND=plan container/run.sh` via the harness, see L1) and the commands in the done criteria.
    Not allowed: Isaac Sim, model calls (no LLM endpoint), Modal, GPU, `git commit`, `pkill -f` (stop processes by
    PID), modifying `/home/mew/intrinsic-core`. Leave the container stopped at the end.
*   No design decisions beyond this plan; anything uncovered goes to `docs/designs/phases/phase_l3_notes.md`.
*   Code style of the surrounding files (2-space indent, comments only for the why). Observability: every
    episode prints one status line (`OK|FAIL family seed strategy duration_s calls error`) and is written as one
    JSON line.

## Done criteria

*   `.venv/bin/python -m pytest tests/ -q` passes, including the new `tests/test_strategies.py` and
    `tests/test_search_expert.py`.
*   With the plan tier running: `.venv/bin/python learn/search_expert.py solve --split train --n 20` (all four
    train families, 80 tasks) finishes, writes `logs/expert/train.jsonl` and prints the per-family summary
    (decision 6); ≥ 90 % of tasks solved per family. If a family is below, record the failing tasks' errors in
    the notes (no tuning of the families).
*   `.venv/bin/python learn/search_expert.py recovery --split train --n 20` writes
    `logs/expert/recovery_train.jsonl` and prints how many recovery examples were confirmed.
*   `phase_l3_notes.md`: files changed, test output, both summaries.

## Decisions

### 1. Scene text with geometry (`program_runtime/agent.py`, Modify)

`program_messages` and `RecoveryAgent` describe each object on one line:
`obj_12 cube red medium x=512 y=83 yaw=12 size=50x50x50` (positions and sizes in mm, yaw in degrees, all
rounded to integers; `attached` appended as ` held` when true). The program rules are unchanged (no ids, no float
literals): the geometry is for the model's reasoning; programs read it from object attributes. Add the sentence
"Coordinates are in millimetres in the robot's base frame; programs read them from object attributes (x, y, sx,
...) in metres." to the system prompt. Update tests that assert the old scene text.

### 2. Episode runner (`learn/episodes.py`, Create)

```python
@dataclasses.dataclass
class Episode:
  family: str; seed: int; split: str
  strategy: str | None          # strategy name, or None for a model-written program
  program: str
  ok: bool                      # runtime ok and rules ok
  runtime_ok: bool; rules_ok: bool; rules_summary: str; per_object: dict
  error: str | None             # runtime error text
  failure: dict | None          # first failure event: kind, message, call, line (from the runtime log)
  handoffs: list                # runtime handoff records
  calls: int                    # pick + place calls made
  duration_s: float
  events: list[dict]            # the runtime's JSONL events

def run_episode(task, program, sim_stub, robot_stub, agent=None, strategy=None) -> Episode
```
Loads the task with `families.load`, snapshots truth (`program_runtime.rules.truth_snapshot`), runs the program
with `program_runtime.runtime.run(program, RobotAPI(GrpcBackend()), agent=agent, log=<collects events>)`, checks
the intent with `check_intent`, counts `pick`/`place` result events. Never raises for program or robot failures
(they are the episode's result); raises only for infrastructure errors (loader timeout, RPC UNAVAILABLE).

### 3. Strategy programs (`learn/strategies.py`, Create)

`program(intent: dict, strategy: str) -> str` builds a program from the intent's rules (each rule:
`select` filters → `find_all(**select)`, `target` filters → `find(**target)`, relation `in` → `place`, `on` →
`place`), and ends with a `verify(lambda: ...)` that checks every rule with `inside` / `on_top`. Programs pass
`program_runtime.check.check` and use no ids and no float literals (integers are allowed). Strategies decide the
**order** of the objects to move; they re-read objects each iteration (`find_all` returns fresh snapshots):

| Strategy | Order |
|---|---|
| `listed` | rule by rule, `find_all` order (what the API docs suggest; the zero-shot baseline) |
| `most_free_first` | repeat: among all remaining objects to move, the one whose nearest other cube on the table is farthest (centre distance) |
| `outer_first` | repeat: the remaining object farthest from the centroid of all remaining objects to move |
| `nearest_target_first` | repeat: the remaining object closest to its rule's target |
| `top_first` | repeat: the remaining object with the highest `z` (for towers) |

"Remaining" = selected by a rule and not yet satisfying it (`not inside(o, box)` / `not on_top(o, target)`). For
`on` rules (building a tower), every strategy moves the rules in their listed order (level 1 first): the order is
given by the intent. Example output for `most_free_first` with one `in` rule:

```python
box = find(category="box", color="red")
while True:
  left = [c for c in find_all(category="cube") if not inside(c, box)]
  if not left:
    break
  c = max(left, key=lambda c: min([abs(c.x - o.x) + abs(c.y - o.y) for o in find_all(category="cube")
                                  if o != c and not inside(o, box)] + [c.sx * 100]))
  pick(c)
  place(c, box)
verify(lambda: all(inside(c, box) for c in find_all(category="cube")), "a cube is not in the red box")
```
(Manhattan distance; `c.sx * 100` is the "no neighbour" value.) Multi-rule intents (packing with two colours,
mixed) take the union of the rules' remaining objects and place each into its own rule's target.

### 4. Search expert (`learn/search_expert.py`, Create; not `learn/expert.py`, which belongs to the Qwen study)

*   `solve(task)`: runs the strategies in this order: `listed`, `most_free_first`, `outer_first`,
    `nearest_target_first`, `top_first` (skip `top_first` unless the task has a stacked object), each one
    episode in the plan tier, and **stops at the first success**. Returns all episodes; the expert program is
    the first successful one. `listed` is always run first so every task records whether the docs-level program
    already works.
*   Output record per task (JSON line): `{"family", "seed", "split", "intent", "placements", "expert_strategy" |
    null, "expert_program" | null, "episodes": [Episode as dict, events dropped]}`.

### 5. Recovery examples (`search_expert.py recovery`)

For each task whose `listed` episode failed and that has an expert program:
1.  Run `listed` again with a capturing agent: an agent object whose `recover(context)` stores the context dict
    (the exact dict the runtime gives `RecoveryAgent`) and returns `{"rung": "patch", "reason": <text>, "patch":
    <expert program>}`, where `<text>` is `f"{strategy}: the failed object is blocked; move the objects in an
    order that keeps grasps free"` for grasp / reachability / collision failures and `f"{strategy}: a different
    order"` otherwise.
2.  The episode's `ok` confirms the decision: only confirmed examples are written.
3.  Record: `{"family", "seed", "split", "context": <captured dict without "_backend">, "decision": <dict>,
    "confirmed": true, "episode": <Episode as dict, events dropped>}`.
Strategy programs are idempotent (they act only on objects not yet satisfying their rule), which is why the full
expert program is a valid patch after a partial run.

### 6. CLI and summary

`solve --split S --n N [--families a,b] [--out PATH]` and `recovery --split S --n N [...]` (defaults: all
families valid for the split, `--out logs/expert/<split>.jsonl` / `recovery_<split>.jsonl`). Both expect a
running stack (plan tier or Isaac: the same ports) and fail fast with a clear message if port 18100 is closed.
Summary printed at the end, one line per family: tasks, solved %, `listed` success %, wins per strategy, mean
episode seconds, mean calls.

### 7. Tests

*   `tests/test_strategies.py`: for every family (L2) and 20 seeds, every strategy's program passes `check()`
    and its dry run on `families.fake_objects(task)` (`program_runtime.runtime.dry_run` with `FakeBackend`)
    achieves the intent (`check_intent` on the fake truth); `most_free_first` picks the expected first cube on a
    hand-made plus layout (an outer cube, never the centre); no program contains `obj_` or a float literal.
*   `tests/test_search_expert.py`: with a fake `run_episode` (monkeypatched) — stops at the first success, runs
    `listed` first, skips `top_first` without stacks, records `expert_strategy` null when nothing succeeds;
    recovery writes only confirmed examples and drops `_backend` from the context.
*   Scene-text test for decision 1.

## Addendum A (review resolutions; supersedes the decisions above where they differ)

### A.1 Entry condition and stack lifecycle

L1 is complete when `tests/e2e/plan_tier_check.py` passes; `ResetScene` now returns the plan-tier arm to HOME
with the gripper open (as Isaac's reset does), so every episode starts from the same state. Docker via
`sg docker -c` is allowed. Both CLI commands take `--start-stack {none,plan}` (default `none`): `plan` starts the
container with `phase3_hwm_check._Run().start_container(backend="plan")`, waits with
`plan_tier_check._wait_ready()`, and stops it (`stop_container`) in a `finally`. `none` expects a running stack.

### A.2 Connections and infrastructure errors (replaces parts of decision 2)

`connect() -> (sim_stub, robot_stub)`: `SimServiceStub` and `RobotServiceStub` on one channel to
127.0.0.1:18100 (robot_service serves both). `run_episode` uses `GrpcBackend(stub=robot_stub)`. Infrastructure
errors: before each episode `robot_stub.GetState(Empty(), timeout=10)`; a `grpc.RpcError` there raises
`RuntimeError("stack not reachable: ...")`. `Episode.failure` = `RunResult.failures[0]` if any, else None.

### A.3 Call budget

Every expert program must run within the default `Budget` of the runtime (it is what Glimmer learns from).
Strategy programs therefore compute their work list once per loop iteration and their `key=` lambdas use only
object attributes of objects already in that list (no API call inside a key). Test: every strategy on every
family's 7-cube tasks (the maximum) runs within the default budget on the fake backend.

### A.4 Program shape (replaces the example and the multi-rule sentence of decision 3)

One `while True` loop over a list of `(object, target)` pairs rebuilt each iteration from the rules
(`in` rules: `[(o, t) for o in find_all(**select) if not inside(o, t)]`; `on` rules handled in listed order before
any key choice). Choose a pair with `max(todo, key=lambda p: ...)` on `p[0]`, then `pick(p[0])`,
`place(p[0], p[1])`. Example (`most_free_first`, one rule):

```python
box = find(category="box", color="red")
while True:
  todo = [(c, box) for c in find_all(category="cube") if not inside(c, box)]
  if not todo:
    break
  todo_objects = [p[0] for p in todo]
  p = max(todo, key=lambda p: sum(sorted([(p[0].x - o.x) * (p[0].x - o.x) + (p[0].y - o.y) * (p[0].y - o.y)
                                          for o in todo_objects if o != p[0]] + [p[0].sx * p[0].sx * 100] * 2)[:2]))
  pick(p[0])
  place(p[0], p[1])
verify(lambda: all(inside(c, box) for c in find_all(category="cube")), "a cube is not in the red box")
```

### A.5 Strategy metrics (replaces the table's wording; all distances are squared centre distances in x, y)

*   `most_free_first`: key = sum of the two smallest distances to the other objects in the work list (missing
    neighbours count `sx * sx * 100` each). In a plus, the centre's two nearest are both one pitch, an outer
    cube's are one pitch and √2 pitches, so the centre is never first. Ties: the next key is the distance from
    the work list's centroid (larger first); then `find_all` order.
*   `outer_first`: key = distance from the centroid of the work list's objects.
*   `nearest_target_first`: key = minus the distance from the object to its pair's target.
*   `top_first`: key = `z`. Used only when the task has a stacked cube: a cube whose placement has the same x, y
    (within 0.01 m) as another cube's and a larger z.

### A.6 Tests of intent achievement (replaces decision 7's "dry run" wording)

Run `program_runtime.runtime.run(program, RobotAPI(backend))` with `backend = FakeBackend(fake_objects(task))`,
convert `backend.objects` (and the initial objects) with `learn/glimmer_program_test.truth`, then `check_intent`.
The plus test uses `families` placements of a plus (gap 0.015 m) with the centre listed first.

### A.7 Recovery inputs and limits (replaces decision 5's inputs)

*   `recovery --split S --in PATH` reads `solve`'s output (default `logs/expert/<split>.jsonl`); it re-runs
    nothing but the `listed` episodes of tasks with `expert_program` set and a failed `listed` episode.
*   If the capturing agent is never called (the runtime ended without a handoff: rules failed, budget exceeded,
    dry-run error), the task is skipped and counted as `no_handoff`.
*   If the failure left an object held (`context["held"]` non-empty, or the failure call is a `place`), the task
    is skipped and counted as `held` (the expert program would start with a pick while holding).
*   Reason text: `f"{failure['kind']} failure at {failure['call']}: switch to the {strategy} order"`.
*   Summary adds the counts `confirmed`, `not_confirmed`, `no_handoff`, `held`.

### A.8 Scene text (replaces decision 1)

Only `program_messages` (the writer's prompt) changes: one line per object,
`obj_12 cube red medium x=512 y=83 z=0 yaw=12 size=50x50x50` (+ ` held`), and the sentence of decision 1 in the
writer's system prompt. `RecoveryAgent` is unchanged (its context already carries every field).

### A.9 Targets

The parent design is updated: the expert is a search over strategy programs (replacing greedy-with-backtracking
over grasp feasibility); the L3 target is ≥ 90 % solved per family, the rest recorded with their errors. The
recovery summary's `confirmed` count is reported, not gated.
