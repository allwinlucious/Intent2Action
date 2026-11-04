# Phase V2: the program runtime (agent-written programs, fail-fast, recovery ladder)

Part of [demo_to_api.md](../demo_to_api.md) (D11, "Program format and runtime"). The agent writes a short
Python program for an intent. The runtime checks it, dry-runs it, and executes it on `RobotService` with no
model in the loop. On a failure it hands the situation to the agent, which climbs a budgeted recovery
ladder. A rule checker scores the result against Isaac's ground truth, which programs never see.

## Constraints for the worker

*   **No heavy compute:** no Isaac Sim, no Docker, no Bazel, no GPU, no model training. Do not run
    `tests/e2e/*`.
*   **Allowed:**
    *   editing files;
    *   `.venv/bin/python -m pytest tests/ -q`;
    *   `.venv/bin/python -m py_compile …`;
    *   `.venv/bin/python -m program_runtime.cli --help`.
*   **No network calls.** No real model endpoint is used in this phase; tests use fakes.
*   Do not modify `/home/mew/intrinsic-core`. No `git commit`. No `pkill -f`.
*   Make no design decisions beyond this plan. If something is not covered, stop that part and record it in
    `docs/designs/phases/phase_v2_notes.md`.
*   **Code style:** 2-space indent; comments only for the why; standard library only beyond what `.venv`
    already has (no new dependencies).

## Done criteria

*   **Worker:**
    *   `.venv/bin/python -m pytest tests/ -q` passes, including the new `tests/test_program_runtime.py` and
        `tests/test_rules.py`;
    *   `py_compile` passes on every new file;
    *   `python -m program_runtime.cli --help` prints the usage;
    *   `phase_v2_notes.md` is written.
*   **Lead, later, in a sim window:** `tests/e2e/program_check.py` passes:
    *   ≥ 8/9 scripted programs succeed by the rule checker;
    *   the injected-failure check recovers with a scripted agent.

## Files

*   **Create:**
    *   `program_runtime/__init__.py` (empty);
    *   `program_runtime/api.py`;
    *   `program_runtime/check.py`;
    *   `program_runtime/runtime.py`;
    *   `program_runtime/agent.py`;
    *   `program_runtime/rules.py`;
    *   `program_runtime/cli.py`;
    *   `program_runtime/examples/` (3 example programs and intents, see §8);
    *   `tests/test_program_runtime.py`;
    *   `tests/test_rules.py`;
    *   `tests/e2e/program_check.py`;
    *   `docs/designs/phases/phase_v2_notes.md`.
*   **Modify:** nothing else.

## Decisions

### 1. Intent format (JSON; `program_runtime/rules.py` defines and validates it)

```json
{"description": "every cube goes into the red box",
 "rules": [{"select": {"category": "cube"}, "relation": "in", "target": {"category": "box", "color": "red"}}],
 "others": "stay"}
```

*   `select` and `target` are attribute filters. Keys are a subset of `category`, `color` and `size` (a size
    class, §2). Values are strings. An empty `select` means every movable object.
*   `relation` is `"in"` (inside a box) or `"on"` (on top of an object).
*   `others` is always `"stay"`: unselected objects must not move.
*   `validate_intent(d) -> list[str]` returns the problems (an empty list means valid), e.g. unknown keys,
    an unknown relation, or a target that is not a single filter.

### 2. Objects and size classes (`program_runtime/api.py`)

*   **`Obj`, a frozen dataclass:** `id`, `category`, `color`, `size` (class), `x`, `y`, `z`, `yaw`, `sx`, `sy`,
    `sz`, `attached` (bool). Built from a `ListObjectsResponse` object:
    *   pose fields from `pose`, with the yaw from the quaternion;
    *   `sx`, `sy`, `sz` from `size_x`, `size_y`, `size_z`;
    *   `attached = attached_to != ""`.
*   **`size_class(sx, sy, sz)`:** on `m = max(sx, sy, sz)`, returns `"small"` if `m < 0.04`, `"medium"`
    if `m < 0.06`, otherwise `"large"`.
*   **Never movable:** objects with category `table`, `box`, `pillar` or `block_with_hole`. `find_all`
    with an empty filter returns only movable ones.

### 3. The program API (what programs may call; `api.py`)

**`class RobotAPI`** wraps a backend (`GrpcBackend` for the real stack, `FakeBackend` for dry runs and
tests; §4). Every method below is exposed to programs as a plain function.

| Function | Behaviour | Postcondition checked by the runtime |
|---|---|---|
| `find(**filters) -> Obj` | exactly one match in a fresh `list_objects`, else `RobotFailure("perception", …)`: "no object matches {filters}" or "{n} objects match {filters}; add a filter" | — |
| `find_all(**filters) -> list[Obj]` | all matches, sorted by id; may be empty | — |
| `pick(obj)` | backend `pick(obj.id)` | `held(obj)` |
| `place(obj, target)` | backend `place(obj.id, target.id)` | `inside(obj, target)` if `target.category == "box"`, else `on_top(obj, target)` |
| `held(obj) -> bool` | `obj` is attached in a fresh `list_objects` | — |
| `inside(obj, box) -> bool` | in the box frame (yaw), centre `\|dx\| <= box.sx/2`, `\|dy\| <= box.sy/2`, and `box.z <= obj.z <= box.z + box.sz` (fresh poses) | — |
| `on_top(obj, t) -> bool` | `hypot(dx, dy) <= max(t.sx, t.sy)/2` and `\|obj.z - (t.z + t.sz)\| <= 0.01` | — |
| `verify(cond, message="verify failed")` | if not `cond`: `RobotFailure("verify", message)` | — |
| `report(text)` | appended to the run's report lines | — |

**Postconditions are checked immediately after the call** with fresh `list_objects`. A violation raises
`RobotFailure("postcondition", "after {call}: expected {expectation}")`.

**`RobotFailure(Exception)`** has `kind`, `message`, `call` (e.g. `"pick(obj_12)"`) and `line` (the program
line, filled in by the runtime).

**Failure classification of backend errors** (`classify(code_name, message) -> kind`), first match wins,
case-insensitive:

| kind | when |
|---|---|
| `timeout` | code `DEADLINE_EXCEEDED` |
| `fault` | message contains `fault` or `not enabled` |
| `perception` | code `NOT_FOUND` and the message contains `no object with id` |
| `grasp` | message contains `grasp`, `closed on nothing` or `wider than the gripper` |
| `place` | message contains `free collision-free spot`, `box is full` or `place pose` |
| `reachability` | message contains `ik`, `reach` or `unreachable` |
| `collision` | message contains `collision` |
| `other` | anything else |

### 4. Backends (`api.py`)

*   **`GrpcBackend(address="127.0.0.1:18100")`:** `RobotServiceStub` from `gen/robot_service`
    (`sys.path` setup like `tests/e2e/phase3_cuj_check.py`).
    *   `list_objects() -> list[Obj]`.
    *   `pick(id)` and `place(id, target_id)` call `Pick` / `Place` with `wait=True` and an RPC timeout of
        90 s. If the returned operation is not done, call `WaitOperation` (timeout 60 s) up to 2 times.
        An RPC error or an operation error raises `RobotFailure(classify(...), message)`.
    *   `state() -> dict` from `GetState` (`json_format.MessageToDict`).
*   **`FakeBackend(objects: list[Obj])`:** in-memory, for dry runs and tests.
    *   `pick` marks the object attached and raises `grasp` if another object is attached.
    *   `place` moves the object to the target's centre: z = `target.z + 0.01` for a box, `target.z +
        target.sz` otherwise. In a box it offsets each next object by 0.06 m in x, so `inside` stays true for
        up to 3 objects; a 4th raises `RobotFailure("place", "box is full")`. It then marks the object
        unattached.
    *   `inject: dict[str, list[RobotFailure]]`: a queue of failures to raise on the next call of a method
        (tests and failure injection).

### 5. Static check (`program_runtime/check.py`)

**`check(source: str, *, patch: bool = False) -> list[str]`** returns the problems; empty means OK.

*   Parse with `ast.parse`. A `SyntaxError` is one problem: `"syntax error line N: msg"`.
*   **Allowed node types:**
    *   `Module`, `Expr`, `Assign`, `AugAssign`, `For`, `While`, `If`, `Break`, `Continue`, `Pass`;
    *   `Name`, `Load`, `Store`, `Constant`;
    *   `Call`, `keyword`, `Attribute`, `Subscript`, `Slice`;
    *   `Compare` and all compare ops; `BoolOp`, `And`, `Or`; `UnaryOp`, `Not`, `USub`;
    *   `BinOp`, `Add`, `Sub`, `Mult`, `Div`, `Mod`, `FloorDiv`;
    *   `List`, `Tuple`, `Dict`, `ListComp`, `GeneratorExp`, `comprehension`, `IfExp`;
    *   `JoinedStr`, `FormattedValue`;
    *   `Lambda` and `arguments`, `arg` (for `sorted(key=...)`).
*   **`Call.func`** must be a `Name` in `API_FUNCTIONS = {find, find_all, pick, place, held, inside, on_top,
    verify, report}` or in `BUILTINS = {len, sorted, min, max, sum, range, enumerate, abs, any, all, round,
    list, zip, str}`.
*   **`Attribute`** must have an `attr` in `OBJ_FIELDS = {id, category, color, size, x, y, z, yaw, sx, sy,
    sz, attached}`.
*   **Names starting with `_` are rejected.**
*   **With `patch=True` (the delivery check for a patch)** also reject:
    *   any string constant matching `^obj_\d+$` ("hard-coded object id");
    *   any float constant ("hard-coded coordinate or number; resolve objects with find").

### 6. Runtime (`program_runtime/runtime.py`)

```python
@dataclass
class Budget:
  handoffs: int = 3
  patches: int = 1
  steps: int = 20_000      # executed program lines, a guard against endless loops
  calls: int = 200         # robot API calls per run
```

**`run(source, api: RobotAPI, agent | None, log: Callable[[dict], None], budget=Budget()) -> RunResult`**,
where `RunResult` is a dataclass with `ok`, `report`, `failures`, `handoffs`, `patches` and `error`.

1.  **`check(source)`.** Problems → `RunResult(ok=False, error="check: …")`. No execution.
2.  **Dry run:**
    *   execute on a `RobotAPI(FakeBackend(copy of api.backend.list_objects()))`, with the same globals
        rules and step guard;
    *   any exception → `ok=False, error="dry run: {kind}: {message} (line N)"`, except failures that the
        fake cannot model (`place` "box is full" when the real box is bigger), which are ignored and only
        logged;
    *   the dry run does not consult the agent.
3.  **Execute:**
    *   compile with filename `"<program>"`;
    *   globals: `{"__builtins__": {name: builtin for name in BUILTINS}}` plus the API functions bound to
        `api`;
    *   a `sys.settrace` line counter enforces `budget.steps` and records the current program line (frames
        whose `co_filename == "<program>"`);
    *   every API call is counted against `budget.calls`.
4.  **Fail-fast and handoff inside the API call.** When a robot call (`pick`, `place`, or a postcondition)
    raises `RobotFailure`, the runtime's call wrapper:
    1.  sets `failure.line`;
    2.  logs the failure;
    3.  if `agent` is None or `budget.handoffs` is exhausted, re-raises it (the run fails);
    4.  otherwise calls `agent.recover(context)`, where `context` is a dict with:
        *   `program`: the source with line numbers;
        *   `line`, `kind`, `message`, `call`;
        *   `objects`: fresh `list_objects` as dicts;
        *   `state`: `api.backend.state()` (`{}` for the fake);
        *   `history`: the earlier handoffs of this run;
        *   `budget`: the remaining handoffs and patches.
    5.  The agent returns `{"rung": ..., "reason": str, "patch": str | None, "question": str | None}`.
        `handoffs -= 1`. The decision is logged. Then, by rung:
        *   `reobserve` / `retry_step` / `fix_world`: call the same API function again with the same
            arguments, re-resolving `Obj` arguments by id from a fresh `list_objects`, and continue the
            program. A failure of the retried call goes through this wrapper again.
        *   `skip`: return `None` from the call and continue.
        *   `patch`: requires `budget.patches > 0` and `check(patch, patch=True) == []`. Otherwise the
            decision is logged as rejected and treated as `stop`. If accepted, `patches -= 1`; raise the
            internal `_Patch(patch)`. The runtime catches it at top level, dry-runs and executes the patch
            from its start with the remaining budget, and its result is the run's result.
        *   `ask_user`: call `ask(question) -> str` (a callable passed to `run`; default: raise
            `RobotFailure("other", "ask_user is not available")`). Append the answer to the context as
            `"user_answer"`, and call `agent.recover` again **without** consuming another handoff (once per
            handoff).
        *   `stop`: re-raise the original failure.
5.  **`verify` failures** go through the same wrapper. `retry_step` re-evaluates the same `verify` call
    with the condition re-computed: the runtime re-executes the program from the start of the statement
    that contains the `verify` call. If the wrapper cannot re-evaluate (the condition is an already-computed
    bool), `retry_step` on `verify` is treated as `stop`.
6.  **Logging:** `log(event)` for each of these `kind`s, each event with `t` (wall time):
    *   `check`, `dry_run`, `line` (only when the line number changes), `call`, `result`;
    *   `failure`, `handoff`, `decision`, `patch`, `report`, `done`.

    The CLI writes them as JSONL. The demo UI (phase V7) reads this file.

### 7. Agent (`program_runtime/agent.py`)

*   **`ChatClient(base_url, api_key, model)`:** an OpenAI-compatible `POST {base_url}/chat/completions`
    with `urllib.request`. JSON body: `model`, `messages`, `temperature` 0, `max_tokens` 2048, and
    optionally `tools` / `tool_choice`. Timeout 120 s. Returns the first choice's message dict. Configured
    from the environment (`LLM_BASE_URL`, `LLM_API_KEY`, `LLM_MODEL`); missing values raise a clear error.
*   **`ProgramWriter(client)`:**
    *   `write(intent: dict, objects: list[Obj]) -> str`.
    *   **System prompt:**
        *   the program API reference, generated from the docstrings of the §3 functions;
        *   the rules: only the API and builtins, no imports or functions, objects by attributes, never ids
            or coordinates, end with `verify`;
        *   the three example programs from §8.
    *   **User message:** the intent JSON plus a one-line-per-object scene summary (`id category color size`).
    *   The reply's first fenced python block (or the whole reply) is the program.
    *   If `check()` fails, one retry with the problems appended. `ProgramWriter.last_attempts` records the
        number of attempts (for the "valid on the first try" metric).
*   **`RecoveryAgent(client, tools_enabled=True)`:**
    *   `recover(context) -> dict` with the decision format of §6.
    *   **System prompt:**
        *   the ladder: use the lowest rung that can work, and always give a reason;
        *   what each rung does;
        *   the patch rules (the §5 patch check);
        *   the budget;
        *   that the agent may call robot tools before deciding (e.g. `pick`/`place` a blocking object
            aside for `fix_world`).
    *   **Tools:** the MCP tool schemas from `mcp_server.server.Tools(DESCRIPTOR_SET)`, minus `get_image`,
        `detect_objects` and `calibrate_camera`, in OpenAI function format. Calls are executed with
        `mcp_server.server.invoke` on the same gRPC channel, at most 12 per handoff.
    *   **The final answer** must be one JSON object. Parse the first `{…}` in the content; an invalid
        answer → `{"rung": "stop", "reason": "invalid decision: …"}`.
*   **`ScriptedAgent(decisions: list[dict], tool_calls: list[tuple[str, dict]] = [])`:** returns the
    decisions in order, for tests and the e2e check.

### 8. Examples (`program_runtime/examples/`)

The intents and their programs are each a `.json` and `.py` pair, used as few-shot examples, in unit tests
(with `FakeBackend`) and in the e2e check:

1.  **`cubes_in_box`:** every cube → in the red box.

    ```python
    box = find(category="box", color="red")
    for cube in find_all(category="cube"):
      pick(cube)
      place(cube, box)
    verify(all(inside(c, box) for c in find_all(category="cube")), "a cube is not in the red box")
    report(f"{len(find_all(category='cube'))} cubes are in the red box")
    ```

2.  **`sort_by_color`:** red cubes → red box, blue cubes → blue box (two rules), with a loop over
    `[("red", red_box), ("blue", blue_box)]`.
3.  **`small_on_large`:** the small cube → on the red cube (`relation: "on"`).

### 9. Rule checker (`program_runtime/rules.py`; ground truth, evaluation only)

**`check_intent(intent, before: dict[str, TrueObj], after: dict[str, TrueObj]) -> RuleResult`.**

*   `TrueObj` has `id`, `category`, `color`, `size`, `x`, `y`, `z`, `yaw`, `sx`, `sy`, `sz`. It is built from
    `SimService.ListTrueObjects` (pose) plus the catalog's `object.yaml` (category, `color_name`, dims), read
    from `assets/objects/<catalog>/object.yaml`.
*   **Per rule:** the selected objects (by `before` attributes) must satisfy the relation in `after`, using
    the same `inside`/`on_top` geometry as §3 with true poses. The target is the unique object matching
    `target`; zero or several matches makes the intent `invalid_for_scene`.
*   **`others: stay`:** every movable object not selected by any rule must be within 0.01 m of its `before`
    position.
*   **`RuleResult`:** `ok`, `per_object` (`{id: "ok" | "not in target" | "moved"}`), `invalid_for_scene`
    (bool) and `summary`.
*   **`truth_snapshot(sim_stub) -> dict[str, TrueObj]`:** the helper the e2e check uses.

### 10. CLI (`program_runtime/cli.py`)

```
python -m program_runtime.cli run --intent I.json (--program P.py | --write) [--recover] \
    [--address 127.0.0.1:18100] [--log logs/runs/NAME.jsonl]
```

*   `--write` uses `ProgramWriter`; `--recover` uses `RecoveryAgent`. Both need the `LLM_*` environment.
*   Prints the program, then the report, then `OK` or `FAILED: …`. Exit code 0 or 1.
*   SIGTERM → `SystemExit`, as `learn/env.py` does.
*   `--log` defaults to `logs/runs/<UTC timestamp>.jsonl`.

### 11. Unit tests (`tests/test_program_runtime.py`, `tests/test_rules.py`; `FakeBackend` only)

1.  **`check`:**
    *   accepts the 3 examples;
    *   rejects: `import os`, `__import__`, `open(...)`, `obj.__class__`, `def f(): pass`, `eval(...)`, an
        unknown function, `x._y`;
    *   with `patch=True` also rejects `"obj_12"` and `0.45`.
2.  **Each example program** runs to `ok=True` on a fake scene (2 cubes + red box, etc.). The postconditions
    hold.
3.  **A postcondition violation:** a fake whose `place` does not move the object → `RobotFailure` of kind
    `postcondition`.
4.  **`classify`:** one case per row of the §3 table.
5.  **Handoff:**
    *   injected `grasp` failure + `ScriptedAgent([{"rung": "retry_step"}])` → `ok`, 1 handoff;
    *   4 injected failures with `retry_step` each → fails after 3 handoffs (budget);
    *   `skip` → continues;
    *   `patch` with a float literal → rejected → `stop` → fails;
    *   a valid `patch` → runs the patch, and its result is the run's result;
    *   `ask_user` → the callback is called once and does not consume a handoff.
6.  **The step budget:** `while True: pass` stops with a step-budget error.
7.  **The dry run catches a bad `find`** (two red cubes, `find(category="cube", color="red")`) before any
    real call.
8.  **Rule checker:**
    *   all cubes inside → ok;
    *   one cube outside → `"not in target"`;
    *   an unselected cylinder moved by 0.05 → `"moved"`;
    *   no red box in the scene → `invalid_for_scene`.

### 12. E2E check (`tests/e2e/program_check.py`; written, not run)

The structure follows `tests/e2e/pickplace_check.py` (stack start and stop, `ResetScene`, remove the
default movable objects, add objects with `AddObject`, PASS/FAIL lines, exit code).

*   **`scripted`:** the 3 examples × 3 layouts (seeded random positions in x ∈ [0.35, 0.60], y ∈ [0.00, 0.20];
    `red_box` at (0.45, −0.22), `blue_box` at (0.62, −0.05) when needed). Run with `GrpcBackend` and no
    agent. Pass if ≥ 8/9 are `ok` **and** `check_intent` is ok.
*   **`recovery`:**
    *   the `cubes_in_box` program with a `ScriptedAgent([{"rung": "retry_step", "reason": "test"}])`;
    *   the failure is injected by removing the second cube with `SimService.RemoveObject` right after the
        first `place` and adding it back 0.10 m away, so the program's handle is stale;
    *   expected: either the retry succeeds (re-resolved by id), or the run fails with a `perception` or
        `grasp` failure that is logged with a handoff;
    *   pass if a handoff is logged and the run ends with `ok` or a logged failure, never an uncaught
        exception.
*   **Logs:** `logs/runs/program_check_*.jsonl`.

### 13. Notes (`docs/designs/phases/phase_v2_notes.md`)

What was done, the test output, the files changed, and anything the plan did not cover.

## Addendum A (after the review, 2026-10-31)

This addendum **supersedes** the sections it names.

### A.1 `verify` takes a lambda (supersedes §3 `verify` and §6.5)

*   `verify(check, message="verify failed")`: `check` must be a zero-argument `lambda`, enforced by
    `check()` (A.5). The runtime evaluates `check()` itself, so it can re-evaluate it after recovery.
*   On a `verify` failure, the rungs `reobserve`, `retry_step` and `fix_world` mean "re-evaluate the lambda"
    (after the agent's tool calls, for `fix_world`). `skip` means "continue". `patch`, `ask_user` and `stop`
    behave as in §6.

### A.2 Which failures reach the agent (supersedes §6.4)

*   **The wrapper covers every robot-facing call:** `find`, `find_all`, `pick`, `place`, the postcondition
    checks, and `verify`. `held`, `inside` and `on_top` never raise.
*   **A dry-run failure means the program is invalid.** It is returned as `RunResult(ok=False, error="dry
    run: …")` and never goes to the recovery agent. `ProgramWriter` gets one retry with that error. Only
    failures during real execution go to the agent.
*   **Postcondition failures** (`kind == "postcondition"`): `reobserve`, `retry_step` and `fix_world` mean
    "re-check the postcondition with fresh objects". The action is **not** called again.
*   **API-call failures:**
    *   `reobserve`, `retry_step` and `fix_world` mean "call the same function again", with `Obj` arguments
        re-resolved by id from a fresh `list_objects`;
    *   if the id is gone, the retry fails with `perception` "object {id} no longer exists" and goes through
        the wrapper again.
*   A new classification row before `grasp`: `precondition` when the message contains `not holding` or
    `already holding`.

### A.3 `run` signature and results (supersedes §6 header)

```python
def run(source: str, api: RobotAPI, agent=None, log=lambda event: None, budget: Budget | None = None,
        ask: Callable[[str], str] | None = None) -> RunResult
```

*   **Budget:** `budget` defaults to `Budget()`; the runtime works on `dataclasses.replace(budget)`. A patch
    run shares the same counters object.
*   **`RunResult`:**

    ```python
    @dataclass
    class RunResult:
      ok: bool
      report: list[str]            # report() texts in order
      failures: list[dict]         # {"kind","message","call","line"}
      handoffs: list[dict]         # {"failure": {...}, "decision": {...}, "outcome": "resolved"|"failed"|"rejected"}
      patches: int                 # patches executed
      error: str | None            # None when ok
    ```

    A patch's result is merged in: lists concatenated, `patches` summed.
*   **Log events:** every event has `"t"` (wall-clock seconds) and `"kind"`. The extra fields by kind:

    | kind | fields |
    |---|---|
    | `check` | `problems` (list) |
    | `dry_run` | `ok`, `error` |
    | `line` | `line` |
    | `call` | `fn`, `args` (strings, `Obj` as id) |
    | `result` | `fn`, `ok`, `message` |
    | `failure` | `kind`, `message`, `call`, `line` |
    | `handoff` | `n` (1-based) |
    | `decision` | `rung`, `reason`, `patch`, `question` |
    | `patch` | `accepted`, `problems` |
    | `report` | `text` |
    | `done` | `ok`, `error`, `handoffs`, `patches` |

### A.4 Recovery agent protocol (supersedes §7 `RecoveryAgent`, `ScriptedAgent`)

*   **Construction:** `RecoveryAgent(client: ChatClient, stub: RobotServiceStub, tools:
    mcp_server.server.Tools)`. The CLI creates the stub from `--address` and the `Tools` from
    `mcp_server.server.DESCRIPTOR_SET`.
*   **Tool schemas** in OpenAI format: `{"type": "function", "function": {"name", "description",
    "parameters": t.input_schema}}` for every tool except `get_image`, `detect_objects` and
    `calibrate_camera`.
*   **The loop**, with `messages = [system, user(json.dumps(context))]` and at most 12 tool calls in all:
    1.  `msg = client.chat(messages, tools=specs)`.
    2.  If `msg` has `tool_calls`:
        *   append `msg` as returned;
        *   for each call, in order:
            *   parse `function.arguments` as JSON; invalid JSON gives the result `"invalid arguments:
                <error>"`;
            *   a name not in the visible tools gives `"unknown tool: <name>"`;
            *   otherwise run `result, status = mcp_server.server.invoke(tools, stub, name, args)`, and the
                text is the joined `TextContent` texts, prefixed with `"ERROR <status>: "` when
                `status != "OK"`;
            *   append `{"role": "tool", "tool_call_id": call["id"], "content": text[:4000]}`;
        *   repeat from step 1.
    3.  Otherwise, parse the decision from `msg["content"]`.
    4.  After the 12th tool call: append `{"role": "user", "content": "Tool budget used. Answer now with
        the decision JSON only."}` and make one last `client.chat(messages)` without tools.
*   **The decision:**
    *   the first `{…}` parsed with `json.loads`;
    *   `rung` must be one of `reobserve`, `retry_step`, `fix_world`, `skip`, `patch`, `ask_user`, `stop`;
    *   `reason` must be a non-empty str;
    *   anything else gives `{"rung": "stop", "reason": "invalid decision: <why>"}`.
*   **`ScriptedAgent(decisions, tool_calls=None)`:** `tool_calls` maps the 1-based handoff number to a
    list of `(fn, kwargs)`. Before returning the n-th decision it calls `getattr(api_backend, fn)(**kwargs)`,
    where the backend is passed in `context["_backend"]` by the runtime (scripted runs only). When the
    decisions run out it returns `{"rung": "stop", "reason": "scripted decisions exhausted"}`.

### A.5 Program invariants for every program (supersedes §5's patch-only rules)

`check(source)` also rejects, for **all** programs:

*   a string constant, or a constant part of an f-string, that contains `obj_` ("hard-coded object id");
*   a float constant ("hard-coded number; resolve objects with find"); integer constants are allowed;
*   a `verify` call whose first argument is not a `Lambda`;
*   a program whose last statement that is not a `report(...)` call is not a top-level `verify(...)` call
    ("the program must end with verify").

`patch=True` adds nothing beyond these checks; patches pass the same checks.

### A.6 Intent schema exactly (supersedes §1's validation)

*   **Root keys:**
    *   `rules`: required, a list of 1–6 rules;
    *   `description`: optional, a str;
    *   `others`: optional, must equal `"stay"`;
    *   anything else is a problem.
*   **Rule keys:**
    *   exactly `select`, `relation`, `target`;
    *   `relation` ∈ {`in`, `on`};
    *   `select` and `target` are dicts with keys ⊆ {`category`, `color`, `size`} and non-empty string
        values;
    *   `target` has at least one key;
    *   `size` ∈ {`small`, `medium`, `large`}.
*   **Duplicate rules** (equal dicts) are a problem.
*   **"A target that is not a single filter"** is removed from the schema. Whether a target matches exactly
    one object is a property of the scene, checked by `check_intent` (`invalid_for_scene`).

### A.7 Rule checker edge cases (supersedes §9's edge cases)

*   **`invalid_for_scene = True`** when:
    *   a target matches zero or several objects in `before`;
    *   an object is selected by two rules with different targets;
    *   a selected object or a target is missing from `after`.
*   **`per_object`** holds one status per object, the first failing reason in this order: `"missing"`,
    `"not in target"`, `"moved"`, then `"ok"`.
*   `ok` is True when it is not `invalid_for_scene` and every status is `"ok"`.

### A.8 FakeBackend semantics (supersedes §4 `FakeBackend`)

*   **`pick(id)`:** `precondition` "already holding {other}" if another object is attached; otherwise
    attach it.
*   **`place(id, target)`:** `precondition` "not holding {id}" unless `id` is attached. Into a box, the
    object goes to the next free slot of a grid with 0.06 m pitch inside the box's inner area (walls 0.01):
    `nx = floor((sx - 0.02) / 0.06)`, `ny` likewise, which gives 3 × 3 = 9 slots for the V1 boxes. A full
    grid raises `place` "box is full". Onto an object: its top centre. Then detach.
*   **`state()`** returns `{}`; `list_objects()` returns copies.
*   **`inject`** works as in §4. **`GrpcBackend` also gets `inject`** with the same semantics: the
    failure is raised instead of calling the robot. The e2e check uses this for identity-preserving
    transient failures.

### A.9 Examples, exactly (supersedes §8)

*   **`cubes_in_box.json`:**

    ```json
    {"description": "every cube goes into the red box", "rules": [{"select": {"category": "cube"},
      "relation": "in", "target": {"category": "box", "color": "red"}}], "others": "stay"}
    ```

    ```python
    box = find(category="box", color="red")
    for cube in find_all(category="cube"):
      pick(cube)
      place(cube, box)
    report(f"{len(find_all(category='cube'))} cubes placed in the red box")
    verify(lambda: all(inside(c, box) for c in find_all(category="cube")), "a cube is not in the red box")
    ```

*   **`sort_by_color.json`:** two rules, red cube → in the red box, blue cube → in the blue box.

    ```python
    for color in ["red", "blue"]:
      box = find(category="box", color=color)
      for cube in find_all(category="cube", color=color):
        pick(cube)
        place(cube, box)
    verify(lambda: all(inside(c, find(category="box", color=c.color)) for c in find_all(category="cube")
                       if c.color in ["red", "blue"]), "a cube is in the wrong box")
    ```

*   **`yellow_on_red.json`** (replaces `small_on_large`; the catalog has no large cube until V1b): the
    yellow cube goes **on** the red cube.

    ```python
    red = find(category="cube", color="red")
    yellow = find(category="cube", color="yellow")
    pick(yellow)
    place(yellow, red)
    verify(lambda: on_top(find(category="cube", color="yellow"), find(category="cube", color="red")),
           "the yellow cube is not on the red cube")
    ```

### A.10 E2E check, exactly (supersedes §12)

**Prerequisite (lead, in the sim window):** build the boxes' `model.usd` in Isaac, then
`tools/sync_isaac.sh --pull-catalog`, and pass `tests/e2e/pickplace_check.py` first.

*   **Scenes:** `ResetScene`, then remove all default movable objects. The boxes are fixed: `red_box` at
    (0.45, −0.22) yaw 0, `blue_box` at (0.62, −0.05) yaw 0. Cube positions for layout `i` ∈ {0, 1, 2}
    come from `random.Random(i)`: x ∈ [0.35, 0.60], y ∈ [0.00, 0.20], yaw ∈ [−0.3, 0.3], resampled until
    every pair of cubes is ≥ 0.08 m apart.

    | Example | Objects |
    |---|---|
    | `cubes_in_box` | `red_cube`, `blue_cube`, `small_yellow_cube` + `red_box` |
    | `sort_by_color` | `red_cube`, `blue_cube`, `green_cube` + `red_box` + `blue_box` (the green cube must stay) |
    | `yellow_on_red` | `red_cube`, `small_yellow_cube`, `blue_cube` |

*   **`scripted`:** 3 examples × 3 layouts with `GrpcBackend` and no agent. A run counts when `RunResult.ok`
    **and** `check_intent(...).ok`. Pass at ≥ 8/9.
*   **`recovery`:** `cubes_in_box` on layout 0.
    *   `GrpcBackend.inject = {"pick": [None, RobotFailure("grasp", "grasp blocked (injected)")]}`: the
        first pick runs normally, the second raises (`None` entries mean "no injection for this call").
    *   `ScriptedAgent([{"rung": "retry_step", "reason": "transient"}])`.
    *   Pass only if `RunResult.ok`, exactly one handoff with outcome `"resolved"`, and `check_intent(...).ok`.
*   **`stop`:** the same, with `ScriptedAgent([{"rung": "stop", "reason": "test"}])`. Pass if `RunResult.ok`
    is False with one handoff and the failure logged; no uncaught exception.

### A.11 More tests (adds to §11)

*   **`validate_intent`:** each rule of A.6 has one valid and one invalid case.
*   **`Obj`:** built from a `WorldObject`-like dict; yaw from a 90° quaternion; `size_class` boundaries
    0.0399 → small, 0.04 → medium, 0.06 → large; `inside` with a box at yaw 90°.
*   **`GrpcBackend`** with a fake stub:
    *   an operation returned not done → `WaitOperation` is called;
    *   an operation error → `RobotFailure` with the classified kind;
    *   `inject` works.
*   **`ChatClient`** with `urllib.request.urlopen` monkeypatched: the request body and headers are right,
    and a missing `LLM_BASE_URL` raises `ValueError("LLM_BASE_URL is not set")`.
*   **`ProgramWriter`** with a fake client:
    *   extraction from a fenced block;
    *   one retry after a failed `check`;
    *   `last_attempts == 2`.
*   **`RecoveryAgent`** with a fake client and a fake stub:
    *   one tool call, then a decision;
    *   an unknown tool;
    *   invalid argument JSON;
    *   the 12-call limit;
    *   an invalid decision → `stop`.
*   **CLI:** `run --program examples/cubes_in_box.py --intent examples/cubes_in_box.json --fake-scene
    S.json`. `--fake-scene` (a JSON list of `Obj` dicts) uses `FakeBackend` and exists for tests. Exit code
    0, and a JSONL log is written.

### A.12 Implementation order (each step ends with `pytest tests/ -q` green)

1.  `check.py` and its tests.
2.  `api.py`: `Obj`, `size_class`, `classify`, `RobotFailure`, `FakeBackend`, `RobotAPI`.
3.  `runtime.py`.
4.  `rules.py`.
5.  `examples/`.
6.  `agent.py`.
7.  `cli.py`.
8.  `tests/e2e/program_check.py` (compile only).

### A.13 Small points

*   `size_of` in the parent design is replaced by the `Obj.size` field; the design doc is updated.
*   `Obj` is built from each `WorldObject` of a `ListObjectsResponse`.
*   `ChatClient` raises `ValueError("<VAR> is not set")` for each missing variable.
