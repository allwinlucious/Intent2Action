# Phase V2 notes: the program runtime

Plan: [phase_v2.md](phase_v2.md), including Addendum A.

## What was done

An earlier worker run stopped part-way with most files written. This run audited every file against the plan
(protos, the e2e helpers, `mcp_server.server.Tools`/`invoke` and the catalog YAML included) and completed it:

*   `program_runtime/`: `check.py` (§5, A.5), `api.py` (§2–§4, A.2, A.8), `runtime.py` (§6, A.1–A.3),
    `rules.py` (§1, §9, A.6, A.7), `agent.py` (§7, A.4), `cli.py` (§10, A.11), and the three example
    pairs of A.9.
*   Fixes made in this run:
    *   the dry run ignores a fake `place` "box is full" failure and only logs it (§6.2);
    *   `ProgramWriter`'s one retry also covers a dry-run error (A.2), not only `check()` problems;
    *   a step-budget failure raised inside a `verify` lambda is final. Before, it went to the agent, and
        because Python unsets the trace function after it raises, a `retry_step` then ran unguarded and
        returned `ok=True`;
    *   the API functions are bound by closure, so a program cannot override the bound name with a
        `_name=` keyword;
    *   only program frames are traced (API and library frames return `None` from the trace function);
    *   `RecoveryAgent` answers every tool call of a reply, also those past the 12-call limit;
    *   a decision with extra keys is no longer rejected (the plan validates only `rung` and `reason`).
*   Tests added for each fix. The rotated-box test now fails without the yaw. `validate_intent` has valid
    and invalid cases for each A.6 rule.
*   `tests/e2e/program_check.py` is written and compiles, and was not run, as the plan says. Prerequisite
    (A.10): the boxes' `model.usd`, `tools/sync_isaac.sh --pull-catalog`, and a passing
    `tests/e2e/pickplace_check.py`.
*   A.13: `size_of` was already removed from `demo_to_api.md` in commit `cbf9414`; nothing changed there.
*   No Docker, Isaac, Bazel or network was used, and no containers were started.

## Test output

```
$ .venv/bin/python -m pytest tests/ -q
........................................................................ [ 40%]
........................................................................ [ 80%]
...................................                                      [100%]
179 passed in 1.10s
```

`py_compile` passes on every new file, and `.venv/bin/python -m program_runtime.cli --help` prints the usage.

## Files changed

All new; no existing file was modified.

*   `program_runtime/__init__.py`, `api.py`, `check.py`, `runtime.py`, `agent.py`, `rules.py`, `cli.py`
*   `program_runtime/examples/cubes_in_box.{json,py}`, `sort_by_color.{json,py}`, `yellow_on_red.{json,py}`
*   `tests/test_program_runtime.py`, `tests/test_rules.py`, `tests/e2e/program_check.py`
*   `docs/designs/phases/phase_v2_notes.md`

## Not covered by the plan (each implemented as below; review)

1.  **`failure` log event field name.** A.3 gives the `failure` event a `kind` field, which collides with
    the event's own `kind`. The failure's kind is logged as `failure_kind`.
2.  **Dry-run events in the log.** The dry run's own `line`, `call`, `result` and `failure` events go to
    the same log, before its `dry_run` event. The plan does not say whether the dry run is silent. The V7
    UI should either ignore events before the first `dry_run` of each attempt, or the dry run should get a
    no-op log.
3.  **An ignored dry-run failure** is logged as `{"kind": "dry_run", "ok": true, "error": "ignored: place:
    box is full (line N)"}`.
4.  **Where `ProgramWriter` dry-runs.** `write(intent, objects)` dry-runs on `FakeBackend(objects)` itself.
    There is one retry in all, for either a `check()` problem or a dry-run error.
5.  **Tool calls past the 12-call limit** in one reply get the tool reply `"not run: tool budget used"`.
    Chat endpoints reject a conversation with unanswered tool calls.
6.  **`GrpcBackend` operation still not done** after 2 `WaitOperation` calls: `RobotFailure("timeout",
    "operation <name> is not done")`. The `WaitOperation` request asks for 60 s and its RPC deadline is
    90 s.
7.  **`check_intent` with an invalid intent** (non-empty `validate_intent`): `ok=False`,
    `invalid_for_scene=True`, empty `per_object`, and the problems as `summary`.
8.  **`ask_user` without an `ask` callback.** The run fails with `other: ask_user is not available`. The
    decision is logged, but that handoff is not in `RunResult.handoffs`.
9.  **`context["_backend"]`** is passed when the agent's class is `ScriptedAgent`, checked by class name.
    `runtime.py` cannot import `agent.py`, because `agent.py` imports `runtime.dry_run`.
10. **Re-resolving an argument whose id is gone** sets the new failure's `call` to the bare function name
    (e.g. `"pick"`), not `"pick(obj_12)"`.

## Lead review (2026-10-31)

*   **Accepted:** the worker's points 1 and 3–10, and its extra fixes.
*   **Point 2 is changed:** the dry run now logs only its one `dry_run` summary event (no `line`/`call`
    events), so the V7 UI reads a single run's events without filtering.
*   **Still open, for the sim window:**
    *   the boxes' `model.usd`;
    *   `tests/e2e/pickplace_check.py`, then `tests/e2e/program_check.py`.
