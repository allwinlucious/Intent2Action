# Phase L1 implementation notes

## Files changed

- `container/entrypoint.sh`
- `container/run.sh`
- `container/services/robot/plan_tier.py` (new)
- `container/services/robot_service.py`
- `container/services/scene_sync.py`
- `tests/e2e/phase3_hwm_check.py`
- `tests/e2e/plan_tier_check.py` (new)
- `tests/test_plan_tier.py` (new)
- `docs/designs/phases/phase_l1_notes.md` (new)

`container/services/robot_service.py` already contained the user's pre-existing pre-grasp-joints change; the Phase
L1 changes were integrated without removing it. Other pre-existing worktree changes were not modified for Phase L1.

## Check output

```text
$ .venv/bin/python -m pytest tests/ -q
........................................................................ [ 38%]
........................................................................ [ 76%]
.............................................                            [100%]
189 passed in 1.16s
```

The focused Phase L1 unit tests also pass:

```text
$ .venv/bin/python -m pytest tests/test_plan_tier.py -q
.......                                                                  [100%]
7 passed in 0.11s
```

The container starts in the plan tier without Isaac or ICON, and the final container check reached the services in
10.1 seconds. Its result was:

```text
$ .venv/bin/python tests/e2e/plan_tier_check.py
robot_service plan tier ready after 10.1 s
FAIL pick_place_box: RuntimeError: 7/10 placed in the box (need 9)
PASS stack: 5/5 stacked
PASS errors: Place holding nothing -> FAILED_PRECONDITION; Pick while holding -> FAILED_PRECONDITION;
  Place(cube, cube) -> INVALID_ARGUMENT; Pick('obj_999') -> NOT_FOUND; Pick(plate) -> ABORTED
FAIL truth: NOT_FOUND: operations/Place-52: planning failed: Could not solve FinePathIK without excessive change
  in joint config
FAIL speed: RuntimeError: reachability: operations/Pick-56: planning failed: Could not solve FinePathIK without
  excessive change in joint config. (line 3)
elapsed 59.2 s
```

The container was removed after the check and is stopped.

## Uncovered

Repeated episodes expose a reset behavior that the plan does not specify. `ResetScene` restores the truth-table
objects, but `PlanIcon.clear_faults()` is required to do nothing and no plan-tier component is told to restore the
arm joints. The check's first episode starts at `HOME`; every later episode starts at the preceding episode's final
retreat (or failed-path) joints. In the final run, trials 3, 8 and 10 failed when the real planner rejected all three
straight lowering paths with `FinePathIK` joint-continuity errors. The truth check and five-cube program then began
from those accumulated joints and failed for the same reason.

Resetting the virtual arm to `HOME`, adding a homing RPC to each test scene, restarting the container between
episodes, or changing Place's retry policy could each change this result, but the plan selects none of them. No such
behavior was added. Consequently, the speed median was not available and the Phase L1 container done criterion is
not met, while the unit-test criterion is met.
