# Phase V1 notes (worker part)

Implements [phase_v1.md](phase_v1.md) decisions 1-10 with Addendum A. The sim part (boxes' `model.usd`,
`tests/e2e/pickplace_check.py`, `phase3_cuj_check.py`, `phase6_check.py`) is left for the lead's sim window.

## What was done

*   **Proto (decision 1):** `robot_service/robot_service.proto` verified as specified (messages, `Pick` and
    `Place` after `PlanGrasp` with their comments); not changed.
*   **Generated code (decision 2):** `container/gen_protos.sh robot_service/robot_service.proto`. Only
    `gen/robot_service.desc`, `gen/robot_service/robot_service_pb2.py` and `robot_service_pb2_grpc.py` changed;
    nothing under `gen/intrinsic/` or `gen/google/`. The new descriptor set has `pick` and `place` tools; the
    one at HEAD has not (checked with `server.Tools`).
*   **Mount (decision 3):** `container/run.sh` mounts `$ROOT/gen:/opt/app/gen:ro` (the image's `PYTHONPATH`
    already has `/opt/app/gen`).
*   **Boxes (decision 4, A.2, A.5):** `_open_box`, `red_box` and `blue_box` as
    `_open_box(0.25, 0.21, 0.08, 0.01, 0.01)`, `CONTAINERS`, `container: {wall: 0.01, floor: 0.01}` in
    `object.yaml`; `obstacle` is now `("pillar", "grey")`. After the run, `git status` shows only
    `assets/objects/obstacle/object.yaml` changed among the existing objects (`category: box` to `pillar`);
    every other existing `mesh.glb` and `object.yaml` is byte-identical.
*   **Place planner (decision 5, A.3):** `container/services/robot/place.py`.
*   **robot_service (decision 6, A.4):** the refactors (`gripper_action`, `attach`, `detach` with
    `_wait_synced` moved into `Robot`, `held`, `move`), `Pick`, `Place`, `grasp.catalog_entry`.
*   **MCP text (decision 7, A.6)** and **tests (A.8, decision 8's MCP part)**.
*   **E2E check (decision 9, A.9):** `tests/e2e/pickplace_check.py`, written and compiled, **not run**.

## Checks asked for by the plan

*   **Obstacle category (A.5):** `rg category tests learn` and `rg obstacle tests learn` show no use of the
    obstacle's category. `learn/` filters on `cube`, `peg` and `plate` only. `tests/e2e/phase3_cuj_check.py`
    uses the obstacle by its catalog name (`OBSTACLE = "obstacle"`), not by its category.
    `tests/e2e/ids_check.py` compares the category of an added `red_cube` only.
*   **Finger axis (A.3):** `assets/robots/franka/lula_franka_gen.urdf` has `right_gripper`
    `rpy="0 0 2.35619449019"` (parent `panda_link8`) and `panda_hand_joint` `rpy="0 0 -0.785398163397"`, a
    difference of 3.14159 rad (180°) about z, as the plan says.

## Test output

`.venv/bin/python -m pytest tests/ -q` (92 before this phase):

```
........................................................................ [ 71%]
.............................                                            [100%]
101 passed in 1.18s
```

`tests/test_place.py` alone: `8 passed`. It failed at collection before `place.py` existed.

`.venv/bin/python -m py_compile container/services/robot_service.py container/services/robot/place.py`
(and `tests/e2e/pickplace_check.py`): no output, exit 0.

`.venv/bin/python tools/build_catalog_meshes.py`:

```
OK   table: bounds [[-0.6, -0.4, 0.0], [0.6, 0.4, 0.75]], 12 faces, watertight True
OK   red_cube: bounds [[-0.025, -0.025, 0.0], [0.025, 0.025, 0.05]], 12 faces, watertight True
OK   blue_cube: bounds [[-0.025, -0.025, 0.0], [0.025, 0.025, 0.05]], 12 faces, watertight True
OK   green_cube: bounds [[-0.025, -0.025, 0.0], [0.025, 0.025, 0.05]], 12 faces, watertight True
OK   small_yellow_cube: bounds [[-0.0175, -0.0175, 0.0], [0.0175, 0.0175, 0.035]], 12 faces, watertight True
OK   plate: bounds [[-0.09, -0.09, 0.0], [0.09, 0.09, 0.012]], 128 faces, watertight True
OK   peg: bounds [[-0.01, -0.01, 0.0], [0.01, 0.01, 0.08]], 128 faces, watertight True
OK   hole_block: bounds [[-0.04, -0.04, 0.0], [0.04, 0.04, 0.05]], 142 faces, watertight True
OK   calib_board: bounds [[-0.14, -0.1, -0.0025], [0.14, 0.1, 0.0025]], 12 faces, watertight True
OK   obstacle: bounds [[-0.05, -0.05, 0.0], [0.05, 0.05, 0.5]], 12 faces, watertight True
OK   red_box: bounds [[-0.125, -0.105, 0.0], [0.125, 0.105, 0.08]], 28 faces, watertight True
OK   blue_box: bounds [[-0.125, -0.105, 0.0], [0.125, 0.105, 0.08]], 28 faces, watertight True
```

`robot_service.py` cannot be imported on the host (rclpy, ICON), so `Pick` and `Place` have no test until
`pickplace_check.py` runs in the sim.

## Files changed

*   **Modified:** `robot_service/robot_service.proto` (by the lead, verified),
    `gen/robot_service.desc`, `gen/robot_service/robot_service_pb2.py`,
    `gen/robot_service/robot_service_pb2_grpc.py`, `container/run.sh`, `tools/build_catalog_meshes.py`,
    `container/services/robot/grasp.py`, `container/services/robot_service.py`, `mcp_server/server.py`,
    `tests/test_mcp_server.py`, `assets/objects/obstacle/object.yaml`.
*   **Created:** `assets/objects/red_box/{mesh.glb,object.yaml}`, `assets/objects/blue_box/{mesh.glb,object.yaml}`,
    `container/services/robot/place.py`, `tests/test_place.py`, `tests/e2e/pickplace_check.py`,
    `docs/designs/phases/phase_v1_notes.md`.

## Not covered by the plan (what I did, for the lead to confirm)

1.  **Pick's log line has `target=` empty.** A.4 gives one format for both,
    `pick|place object=%s target=%s ...`. Pick has no target, so it logs `target=`.
2.  **`Place`'s detail `at (x, y)`** uses the object's world pose after release and detach (`object_pose`), not
    the planned spot.
3.  **The `object_id == target_id` message** is not given in the plan. It is `cannot place {id} on itself`.
4.  **Place's argument checks look up both catalog names** (`_catalog`: `world.get` plus `object_catalog`).
    A world object without a catalog gets NOT_FOUND (`unknown_object`), as `plan_grasp` does, instead of an
    INTERNAL error later in the operation.
5.  **Gripper ownership race.** A.4 has Pick and Place check for a running gripper operation before the arm
    operation starts, then set `gripper_op = op` inside `work` without checking again. A `Grasp` or `Release`
    RPC that starts between the two is overwritten as `gripper_op`, and both drive the gripper. I implemented it
    as written. A re-check under the lock in `_gripper` would close the race.
6.  **Pick's "opening the gripper" step** sets the operation to EXECUTING with that detail. While it is still
    PLANNING, `Stop` ends the operation at once while the gripper action keeps running, so EXECUTING makes
    `Stop` wait for the worker instead. The closing and release gripper steps have no detail in the plan, so
    they keep the detail of the motion before them ("approaching the grasp pose", "lowering").
7.  **Small helpers and constants** that the plan implies but does not name:
    *   `Robot.require_gripper_free()`, the existing check moved out of `start_gripper` so Pick and Place can
        reuse it with the same message;
    *   `OPEN_TOLERANCE_M = 0.005` (A.4's `- 0.005`);
    *   `GRASP` and `RELEASE` aliases;
    *   `place.BELOW_BOX_M = 0.01` (A.3's `box.z - 0.01`).
8.  **Grid and overlap details in `candidates()`:**
    *   The grid has `int(l / GRID_M)` points each side of the centre.
    *   Overlap is strict, so rectangles that only touch do not overlap.
    *   With these, the A.8 capacity test finds exactly 6 spots for 5 cm cubes (2 rows of 3).
9.  **E2E `errors` order.**
    *   "Place while holding nothing" is `Place(cube, plate)`.
    *   "Pick while holding" picks the cube, then calls `Pick(plate)`.
    *   `Pick(plate)` with the `TOO_WIDE` message runs after the scene is rebuilt, because while the cube is
        held it would get FAILED_PRECONDITION.
    *   With `wait=true`, a failed operation fails the RPC, so the script checks the RPC's code.
10. **`wait=true` blocks at most 60 s** (`WAIT_S`). If a Pick or Place in the sim takes longer, the RPC returns
    a running operation and `pickplace_check.py` reports it as "still running after wait".
11. **Containers.** The plan forbids Docker in this phase, so I neither started nor checked any container.
    Nothing was started by this work.

## Known limitation (A.7)

The grasp planner sees every object as a solid box of its `dims`, including the inside of an open box.
Picking an object out of a box is not supported in V1. The MCP instructions say so.

## Lead review (2026-10-29)

*   Points 1–4 and 6–11 are accepted as implemented.
*   **Point 5 is fixed:** `_gripper` re-checks `require_gripper_free()` under `self.r.lock` before claiming
    `gripper_op`, unless the operation already owns it.
*   **Still open, for the sim window:**
    *   build `red_box`/`blue_box` `model.usd` in Isaac (`sim/tools/build_catalog.py`, then
        `tools/sync_isaac.sh --pull-catalog`);
    *   run `tests/e2e/pickplace_check.py`, `phase3_cuj_check.py` and `phase6_check.py`.
