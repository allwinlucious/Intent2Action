# Phase L3 implementation notes

Status: implemented. Unit tests pass, both CLI commands ran on the plan tier, and the container is stopped.
The ≥ 90 % solved target is met for packing, stacking and reach but **not for cluster (25 %)**. As the plan
requires, the families were not tuned; the errors for the failing tasks are listed below.

Entry condition (A.1): `tests/e2e/plan_tier_check.py` passed all five checks on 2026-10-31 (pick_place_box 9/10,
stack 5/5, errors, truth, speed), in `logs/checks/plan_tier.out`.

## Files changed

* `program_runtime/agent.py`: the writer's scene line now carries geometry, and the system prompt gains the
  coordinates sentence (A.8). `RecoveryAgent` is unchanged.
* `tests/test_program_runtime.py`: new scene-text test.
* `learn/strategies.py` (new): `program(intent, strategy)` and `STRATEGIES`.
* `learn/episodes.py` (new): `Episode`, `connect()`, `run_episode()`, `record()`.
* `learn/search_expert.py` (new): `solve`, `recovery`, the capturing agent, the CLI and the summaries.
* `tests/test_strategies.py` (new), `tests/test_search_expert.py` (new).
* `docs/designs/phases/phase_l3_notes.md` (new).

Generated (git-ignored): `logs/expert/train.jsonl` (80 records), `logs/expert/recovery_train.jsonl` (3 records),
`logs/expert/solve_train.out`, `logs/expert/recovery_train.out`, `logs/expert/plan_tier_container.log`.

## Test output

```text
$ .venv/bin/python -m pytest tests/ -q
........................................................................ [ 29%]
........................................................................ [ 58%]
........................................................................ [ 87%]
...............................                                          [100%]
247 passed in 22.98s
```

`tests/test_strategies.py` (31 tests) covers each family × 20 seeds × each strategy. Every program passes `check()`,
contains no `obj_` and no float literal, and achieves the intent on `FakeBackend` (`check_intent` on the fake
truth). It also checks that `most_free_first` picks an outer cube of a plus first (the centre is listed first; the
`listed` strategy picks the centre, so the test discriminates) and that every strategy runs within the default
`Budget` on each family's largest tasks in the first 100 seeds (cluster: 7 cubes). `tests/test_search_expert.py`
(4 tests) checks that `listed` runs first, the search stops at the first success, `top_first` is skipped without
stacks and run with them, and `expert_strategy` is null when nothing succeeds. It also checks that recovery writes
only confirmed examples, drops `_backend`, and counts `no_handoff` and `held`.

## Solve summary

The plan tier was started with `ROBOT_BACKEND=plan bash container/run.sh` and readiness was checked with
`plan_tier_check._wait_ready()`. The run was on HEAD `473702d`.

```text
$ .venv/bin/python learn/search_expert.py solve --split train --n 20
...
wrote /home/mew/untitled-project/logs/expert/train.jsonl
cluster: tasks=20 solved=25% listed=20% wins listed=4 most_free_first=1 outer_first=0 nearest_target_first=0 top_first=0 mean_s=5.3 mean_calls=4.8
packing: tasks=20 solved=100% listed=100% wins listed=20 most_free_first=0 outer_first=0 nearest_target_first=0 top_first=0 mean_s=4.0 mean_calls=3.0
stacking: tasks=20 solved=100% listed=35% wins listed=7 most_free_first=1 outer_first=1 nearest_target_first=2 top_first=9 mean_s=3.6 mean_calls=2.5
reach: tasks=20 solved=90% listed=75% wins listed=15 most_free_first=2 outer_first=1 nearest_target_first=0 top_first=0 mean_s=7.5 mean_calls=5.4
```

First-failure kinds across all failed episodes: cluster place 21, grasp 29, reachability 10, other 1; stacking
grasp 29, collision 11, reachability 5; reach grasp 6, place 3, reachability 3; packing none.

### Unsolved tasks (cluster 15 of 20, reach 2 of 20)

Each entry gives the error for `listed` / `most_free_first` / `outer_first` / `nearest_target_first`. Abbreviations:
`full` = `place: no free collision-free spot in <box> (the box is full or out of reach; tried 0 spots)`,
`nograsp` = `grasp: no feasible grasp found (goal status 6, MoveIt error code 99999)`,
`collfree` = `grasp: no grasp is collision-free in the robot's world model`,
`ik` = `reachability: planning failed: Could not solve FinePathIK without excessive change in joint config`.

| task | listed | most_free_first | outer_first | nearest_target_first |
|---|---|---|---|---|
| cluster 0 | full | ik | full | full |
| cluster 1 | nograsp | nograsp | nograsp | nograsp |
| cluster 4 | nograsp | collfree | collfree | nograsp |
| cluster 7 | full | ik | full | nograsp |
| cluster 8 | ik | ik | full | nograsp |
| cluster 9 | ik | ik | ik | nograsp |
| cluster 10 | nograsp | `other: MoveIt's /apply_planning_scene did not answer within 5 s` | full | nograsp |
| cluster 12 | nograsp | full | full | nograsp |
| cluster 13 | full | full | full | full |
| cluster 14 | nograsp | full | full | nograsp |
| cluster 15 | ik | ik | ik | nograsp |
| cluster 16 | nograsp | nograsp | nograsp | nograsp |
| cluster 17 | nograsp | full | full | nograsp |
| cluster 18 | full | full | full | nograsp |
| cluster 19 | collfree | nograsp | nograsp | nograsp |
| reach 13 | nograsp | nograsp | nograsp | nograsp |
| reach 17 | nograsp | ik | `place: ... tried 40 spots` | nograsp |

Most `full` failures happen after several cubes are already in the box: cluster puts up to 7 medium cubes into one
box, while `plan_tier_check.py` notes that the place planner fits 4 to 6. The `nograsp` failures on the first pick
come from the tight cluster gaps (10 to 30 mm).

## Recovery summary

```text
$ .venv/bin/python learn/search_expert.py recovery --split train --n 20
...
wrote /home/mew/untitled-project/logs/expert/recovery_train.jsonl
cluster: tasks=1 confirmed=0 not_confirmed=1 no_handoff=0 held=0 mean_s=13.7 mean_calls=7.0
packing: tasks=0 confirmed=0 not_confirmed=0 no_handoff=0 held=0 mean_s=0.0 mean_calls=0.0
stacking: tasks=13 confirmed=3 not_confirmed=7 no_handoff=3 held=0 mean_s=5.4 mean_calls=3.7
reach: tasks=3 confirmed=0 not_confirmed=2 no_handoff=1 held=0 mean_s=7.1 mean_calls=5.7
total: confirmed=3 not_confirmed=10 no_handoff=4 held=0; 3 recovery examples confirmed
```

There were 17 eligible tasks (an expert program exists and `listed` failed). The 3 confirmed examples are stacking 16,
17 and 19: a grasp failure on `pick`, then a `top_first` patch. All 4 `no_handoff` tasks are re-runs where `listed`
succeeded this time (stacking 4, 7, 13 and reach 5): plan-tier outcomes vary between runs. Of the 10
`not_confirmed`, 6 were rejected by the patch's dry run with `precondition: already holding obj_..` (stacking 0, 6,
8, 9, 11, reach 16), and 1 failed for the same reason while executing (reach 15). See the first item under
"Uncovered" below.

## Uncovered by the plan (recorded, not decided)

1.  **A.7's `context["held"]` does not exist.** The runtime's handoff context has the keys `program`, `line`, `kind`,
    `message`, `call`, `objects`, `state`, `history` and `budget`. `GetStateResponse` has no held-object field
    (`gripping` and `gripper_width` only). The check is implemented as written: `context.get("held")` or a failure
    call starting with `place`. Only the second half can fire. Failed picks that leave the object attached (e.g. a
    collision during the lift) therefore count as `not_confirmed`, not `held`; they are the 7 "already holding" cases
    above. Two candidate readings, neither chosen: any `objects[i]["attached"]`, or `state["gripping"]`. Either way no
    unconfirmed example is written, so only the counts are affected.
2.  **Per-episode JSON lines.** The constraints say every episode "is written as one JSON line". Decisions 4 and 5
    instead specify one line per task (episodes inside) and one per confirmed recovery example. I followed decisions
    4 and 5 and wrote no separate per-episode file.
3.  **Status line details.** For `error` I print the runtime error; when the runtime succeeded but the rules failed,
    I print the rule summary instead. `strategy` shows `model` for a model-written program (`None`). `duration_s`
    covers the load, the run and the rule check.
4.  **Recovery `--n` and summary.** `recovery` keeps the input records whose seed is among the split's first `n`
    seeds for the selected families. Its per-family line gives eligible tasks, the four A.7 counts, mean episode
    seconds and mean calls. Solved % and wins do not apply to recovery.
5.  **Shared stack.** Another session ran Isaac `program_check.py` runs on the same machine during this phase. Both
    stacks use the container name `intrinsic` and the same host ports. The first `solve` attempt was aborted when that
    session replaced the plan-tier container: the next RPC failed with UNAVAILABLE and was raised as an infrastructure
    error, as designed. I waited for its runs to finish. The final `solve` and `recovery` runs used one uninterrupted
    plan-tier container (started 19:27:51 UTC, `ROBOT_BACKEND=plan`), which was then removed.
