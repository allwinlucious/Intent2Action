# Phase V1: object-level `Pick` and `Place`, open boxes

Part of [demo_to_api.md](../demo_to_api.md) (owner decisions D11, D12). The model says *what* and *where*
(`pick(obj_12)`, `place(obj_12, obj_07)`); the planners compute *how*. Both calls make one planned
attempt and never retry: on failure the operation's error says why, and the VLM decides what to do.

This phase adds the two RPCs, a place planner, and open boxes in the catalog. More shapes and sizes follow
when the owner's real objects are known (Open Question 1 of the design).

## Constraints for the worker

*   **No heavy compute:**
    *   no Isaac Sim, no Docker (`sg docker`), no Bazel, no GPU, no model training;
    *   do not run any `tests/e2e/*` script.

    Allowed: editing files, `container/gen_protos.sh`, `.venv/bin/python tools/build_catalog_meshes.py`,
    and `.venv/bin/python -m pytest tests/ -q` (unit tests, no sim).
*   Do not modify `/home/mew/intrinsic-core`. No `git commit`. No `pkill -f`.
*   Make no design decisions beyond this plan. If something is not covered, stop that part and record it
    in `docs/designs/phases/phase_v1_notes.md`.
*   Follow the code style of the surrounding files: 2-space indent, comments only for the why, logging
    like the existing RPCs (Observability: every operation logs its status and reason).

## Done criteria

*   **Worker:**
    *   `.venv/bin/python -m pytest tests/ -q` passes, including the new `tests/test_place.py`;
    *   `tools/build_catalog_meshes.py` prints OK for every object, including `red_box` and `blue_box`;
    *   the regenerated `gen/robot_service/robot_service_pb2*.py` and `gen/robot_service.desc` contain
        `Pick` and `Place`;
    *   `tests/test_mcp_server.py` sees the `pick` and `place` tools;
    *   `phase_v1_notes.md` is written.
*   **Lead, later, in an owner-approved sim window:** build the boxes' `model.usd` in Isaac and run
    `tests/e2e/pickplace_check.py`:
    *   ≥ 9/10 pick → place-into-box trials;
    *   4/4 into one box;
    *   ≥ 4/5 stacking;
    *   all error checks pass;
    *   `phase3_cuj_check.py` and `phase6_check.py` still pass.

## Decisions

### 1. API (`robot_service/robot_service.proto`, already edited by the lead; verify, do not change)

*   `PickRequest { string object_id = 1; bool wait = 2; }`
*   `PickResult { string object_id = 1; Pose grasp = 2; double width = 3; double duration_s = 4; }`
*   `PlaceRequest { string object_id = 1; string target_id = 2; bool wait = 3; }`
*   `PlaceResult { string object_id = 1; string target_id = 2; Pose place = 3; Pose object_pose = 4;
    double duration_s = 5; }`
*   `rpc Pick(PickRequest) returns (google.longrunning.Operation);` and
    `rpc Place(PlaceRequest) returns (google.longrunning.Operation);`, directly after `PlanGrasp`, with
    the comments already in the file. The comments become the MCP tool descriptions.

### 2. Generated code

Run `container/gen_protos.sh robot_service/robot_service.proto` from the project root. It rewrites
`gen/robot_service/*` and `gen/robot_service.desc`.

### 3. Container mounts (`container/run.sh`, Modify)

Add `-v $ROOT/gen:/opt/app/gen:ro` next to the existing `container/services` mount, so the container uses
the regenerated stubs without an image rebuild.

### 4. Open boxes (`tools/build_catalog_meshes.py`, Modify)

*   **New mesh function `_open_box(x, y, z, wall, floor)`:** the outer box `_box(x, y, z)` minus an inner
    box of extents `(x - 2*wall, y - 2*wall, z)` translated up by `floor`, made with
    `trimesh.boolean.difference(..., engine="manifold")` like `_hole_block`. The inner box reaches past
    the top, so the box is open and the mesh stays watertight.
*   **New `OBJECTS` entries** (static, so Isaac uses the exact triangle mesh, the walls collide and the
    inside is free):
    *   `"red_box": (_open_box(0.20, 0.15, 0.08, 0.01, 0.01), True, None, (0.8, 0.15, 0.15))`
    *   `"blue_box": (_open_box(0.20, 0.15, 0.08, 0.01, 0.01), True, None, (0.15, 0.25, 0.8))`
*   **`ATTRIBUTES`:** `"red_box": ("box", "red")`, `"blue_box": ("box", "blue")`.
*   **New dict `CONTAINERS = {"red_box": (0.01, 0.01), "blue_box": (0.01, 0.01)}`** (wall, floor, in m).
    In `main()`, for names in it, write `description["container"] = {"wall": wall, "floor": floor}` into
    `object.yaml`. Add a comment: `robot_service`'s place planner reads it.
*   **Run the script.** It writes `assets/objects/red_box/` and `blue_box/` (`mesh.glb`, `object.yaml`).
    The existing objects' files must come out byte-identical; check with `git status`. `model.usd` is
    built later in Isaac by the lead (not in this phase's worker part).

### 5. Place planner (`container/services/robot/place.py`, Create; pure functions, no gRPC, no ROS)

Constants:

```python
GRID_M = 0.015           # spacing of candidate spots inside a box
EDGE_MARGIN_M = 0.01     # held object to inner wall
GAP_M = 0.01             # held object to objects already in the box
RELEASE_CLEARANCE_M = 0.005  # object bottom above the surface at release
ABOVE_M = 0.10           # approach / retreat height above the release pose
```

Types: `Spot = tuple[float, float, float]` (x, y, surface z in the world frame).

*   **`yaw(pose) -> float`:** the yaw of a pose proto's quaternion (`atan2(2(wz + xy), 1 - 2(y² + z²))`).
*   **`held_radius(dims) -> float`:** `hypot(dims[0], dims[1]) / 2`.
*   **`occupied(box_pose, box_dims, others) -> list[tuple[float, float, float]]`:**
    *   `others` is a list of `(pose, dims)` of the world's unattached catalog objects, excluding the box
        and the held object.
    *   Returns `(x, y, held_radius(dims))` for each one whose centre, expressed in the box frame (rotate
        by `-yaw`), has `|dx| <= box_dims[0]/2` and `|dy| <= box_dims[1]/2`.
*   **`candidates(target_pose, target_dims, container, held_dims, taken) -> list[Spot]`:**
    *   **`container` is None** (not a box): return `[(target.x, target.y, target.z + target_dims[2])]`
        (the top centre).
    *   **Box:**
        1.  Inner limits: `hx = target_dims[0]/2 - wall - held_radius(held_dims) - EDGE_MARGIN_M`, `hy`
            likewise. If `hx < 0` or `hy < 0`, return `[]`.
        2.  Grid points `(i*GRID_M, j*GRID_M)` with `|i*GRID_M| <= hx`, `|j*GRID_M| <= hy`, for integer
            i, j, in the box frame. Rotate by `yaw(target_pose)` and translate by the box position.
        3.  Drop points closer than `held_radius + r + GAP_M` to any `(x, y, r)` in `taken`.
        4.  Sort by distance to the box centre, ties by (i, j).
        5.  Surface z: `target.z + floor`.
*   **`release_pose(tcp, held_pose, spot) -> api.Pose`:**
    *   Keep `tcp`'s orientation and move it so the held object's bottom centre lands on the spot,
        `RELEASE_CLEARANCE_M` above it.
    *   x: `spot.x + (tcp.x - held.x)`; y likewise; z: `spot.z + (tcp.z - held.z) + RELEASE_CLEARANCE_M`.
*   **`above(pose) -> api.Pose`:** the same pose with `z + ABOVE_M`.

### 6. `robot_service.py` (`container/services/robot_service.py`, Modify)

*   **Imports:** `from robot import place`.
*   **Constants:** `PICK_LIFT_M = 0.10`, `PLACE_CANDIDATES = 40`.
*   **Refactors in `Robot`.** Behaviour unchanged; existing tests must still pass.
    *   **`gripper_action(self, command, cancel) -> tuple[float, bool, float]`:** the body of
        `_run_gripper` up to computing `width` and `holding`. It returns `(width, holding, sim seconds)`
        and sets `self.last_grasp` as `start_gripper` does. `_run_gripper` calls it and finishes the
        operation as before.
    *   **`attach(self, object_id)`:** the body of `RobotService.AttachObject` after the argument checks
        (world attach plus `SetAttached(True)`).
    *   **`detach(self, object_id)`:** the body of `RobotService.DetachObject` after the checks, including
        `_wait_synced`. Move `_wait_synced` into `Robot`. `AttachObject` and `DetachObject` call these.
    *   **`held(self) -> list[str]`:** ids of attached world objects not in `NOT_SCENE_OBJECTS` (use
        `World.attached()`).
    *   **`move(self, op, pose, motion, detail) -> float`:** `self.execute(op, [segment(api.Waypoint(pose=pose,
        motion=motion), api.TCP)], detail=detail)[0]`, i.e. the sim seconds taken.
*   **`RobotService.Pick(request, context)`:**
    1.  **Checks before the operation:**
        *   `require_scene_object(request.object_id)` and `self.r.world.get(request.object_id)` (NOT_FOUND
            as today);
        *   if `self.r.held()`: `FAILED_PRECONDITION`, "already holding {ids}; place or detach it first";
        *   if the gripper operation is running: `FAILED_PRECONDITION` with the existing message.
    2.  **`work(op)`:**
        1.  `plan = self.r.grasps.plan_grasp(self.r.world, id, self.r.planner.check_collisions)`. Its
            errors pass through unchanged.
        2.  `move(op, plan.pregrasp, api.ANY, "moving to the pre-grasp pose")`.
        3.  `move(op, plan.grasp, api.LINEAR, "approaching the grasp pose")`.
        4.  `width, holding, _ = gripper_action(GRASP, op.cancel)`. If not holding: `ApiError(ABORTED,
            f"the gripper closed on nothing (width {width*1e3:.1f} mm); the object may have moved; the
            gripper is still closed")`.
        5.  `self.r.attach(id)`.
        6.  `move(op, plan.grasp with z + PICK_LIFT_M, api.LINEAR, "lifting")`.
        7.  Finish with `PickResult(object_id, grasp=plan.grasp, width, duration_s=sim now − start)` and
            the detail `f"picked {id} (width {w*1e3:.1f} mm) in {s:.1f} s of sim time"`.
        8.  Log `pick object=… status=… duration_s=… message=…`, as the force calls do.
    3.  `return self.r.reply(self.r.start_arm("Pick", work), request.wait)`.
*   **`RobotService.Place(request, context)`:**
    1.  **Checks:**
        *   `require_scene_object` on both ids;
        *   `world.get` on both;
        *   `object_id == target_id` → `INVALID_ARGUMENT`;
        *   `object_id not in self.r.held()` → `FAILED_PRECONDITION`, "not holding {object_id} (holding:
            {held or 'nothing'}); pick it first";
        *   `target_id in self.r.held()` → `INVALID_ARGUMENT`, "{target_id} is held by the gripper".
    2.  **`work(op)`:**
        1.  `start` = sim now.
        2.  `tcp = planner.fk(icon.arm_joints(), api.TCP)`.
        3.  `held_pose = world.pose(object_id)`; held dims from its catalog via `grasp.object_catalog`
            and `grasp.catalog_dims`.
        4.  Target pose, dims and catalog entry. Read `container` from its `object.yaml`: add
            `grasp.catalog_entry(catalog)` as a public wrapper of `_catalog_entry` that returns the dict.
        5.  `taken = place.occupied(...)` over `world.objects()` (unattached catalog objects, excluding
            the target and the held object).
        6.  `spots = place.candidates(...)[:PLACE_CANDIDATES]`.
        7.  For each spot:
            *   `release = place.release_pose(tcp, held_pose, spot)`, `approach = place.above(release)`;
            *   for both poses, `q = planner.ik(pose, api.TCP, icon.arm_joints())` (an `ApiError` means
                skip this spot) and `planner.check_collisions(q)` (in collision means skip);
            *   take the first spot where both pass.
        8.  **No spot found** → `ApiError(ABORTED, …)`:
            *   for a box: "no free collision-free spot in {target_id} (the box is full or out of reach;
                tried {n} spots)";
            *   otherwise: "no collision-free place pose on top of {target_id}".
        9.  `move(op, approach, api.ANY, "moving above the place spot")`, then `move(op, release,
            api.LINEAR, "lowering")`.
        10. `gripper_action(RELEASE, op.cancel)`; `self.r.detach(object_id)`.
        11. `move(op, approach, api.LINEAR, "retreating")`.
        12. Finish with `PlaceResult(object_id, target_id, place=release, object_pose=world.pose(object_id),
            duration_s)` and the detail `f"placed {object_id} {'in' if box else 'on'} {target_id} at
            ({x:.3f}, {y:.3f}) in {s:.1f} s of sim time"`.
        13. Log like Pick.
    3.  `return self.r.reply(self.r.start_arm("Place", work), request.wait)`.

### 7. MCP server (`mcp_server/server.py`, Modify `INSTRUCTIONS` only)

*   Add `pick` and `place` to the long-running calls list in the second paragraph.
*   Replace the "Typical pick and place: …" sentence with:

    > Typical pick and place: list_objects, choose the ids whose category, color, size and pose match the
    > request, then pick(object_id) → place(object_id, target_id) (a box: inside; any other object: on
    > top) → list_objects to check the result. pick and place make one attempt; if one fails, read its
    > error and decide what to do. The low-level calls (plan_grasp, move_to_pose, grasp, release,
    > attach_object, detach_object) remain for special cases.

### 8. Unit tests (`tests/test_place.py`, Create; pytest, no sim)

Build poses with `robot_service_pb2.Pose` from `gen/` (see `tests/conftest.py` for the path setup). Cases:

1.  An empty 0.20 × 0.15 box (wall 0.01) at (0.5, 0, 0), yaw 0, held 0.05 cube: the first candidate is
    (0.5, 0, 0.01). Every candidate stays within the inner limits.
2.  The same box with `taken = [(0.5, 0, 0.0354)]`: no candidate within 0.0354 + 0.0354 + 0.01 of (0.5, 0).
3.  The box is too small for the held object (held dims 0.2 × 0.2): `[]`.
4.  The box at yaw 90°: candidates stay inside the rotated footprint.
5.  A non-box target (a 0.05 cube at (0.4, 0.1, 0)): `[(0.4, 0.1, 0.05)]`.
6.  `release_pose`: TCP (0.5, 0, 0.2), held bottom (0.51, 0, 0.17), spot (0.3, 0.1, 0.01) → (0.29, 0.1,
    0.045), orientation unchanged.
7.  `occupied`: an object inside the footprint is returned, one outside is not.

Extend `tests/test_mcp_server.py` so that the tool list contains `pick` and `place` with `object_id`
(and `target_id` for place) in their input schemas.

### 9. E2E check (`tests/e2e/pickplace_check.py`, Create; do not run)

Follow the structure of `tests/e2e/phase5c_check.py`:
*   `harness.sync_isaac()`, `phase3_hwm_check._Run`, start Isaac and the container, wait for ENABLED and
    ready;
*   a docstring with the run command;
*   PASS/FAIL per check, a final summary, exit code.

Use `phase3_cuj_check` helpers (`_call`, `_sim`, `_robot`, `ids`, `_wait_ready`) and
`AddObjectRequest(catalog, pose)`.

**Scene per trial:**
1.  `ResetScene`.
2.  Remove the default cubes, then add `red_box` at (0.40, −0.25, 0) yaw 0 and one cube at a random spot
    in x ∈ [0.35, 0.60], y ∈ [−0.05, 0.20] (seeded `random.Random(trial)`), its kind cycling through
    `red_cube`, `blue_cube`, `green_cube`, `small_yellow_cube`.
3.  Wait until `ListObjects` shows the new ids (as `learn/env.py` does).

**Checks:**
1.  **`pick_place_box`:** 10 trials of `Pick(cube)` then `Place(cube, box)`, both `wait=true`. Done
    operations, and the cube's true pose inside the box: in the box frame `|dx| <= 0.09`, `|dy| <= 0.065`,
    and `0.005 <= z <= 0.03`. Pass at ≥ 9/10.
2.  **`fill_box`:** one scene with `red_box` and all 4 cubes; pick and place each into the box. All 4 inside
    (same bounds). Report the spots.
3.  **`stack`:** 5 trials of `small_yellow_cube` onto `red_cube`. The true pose is within 15 mm in x/y of
    the red cube and `dz` is in [0.045, 0.06]. Pass at ≥ 4/5.
4.  **`errors`:**
    *   `Place` while holding nothing → `FAILED_PRECONDITION`;
    *   `Pick` while holding → `FAILED_PRECONDITION`;
    *   `Pick(plate)` → the operation fails with `ABORTED` and the `TOO_WIDE` message;
    *   `Place(cube, cube)` → `INVALID_ARGUMENT`;
    *   `Pick("obj_999")` → `NOT_FOUND`.
5.  **`logs`:** the robot_service log has `pick object=` and `place object=` lines with `status=`.

Stop the container and Isaac at the end. Logs go to `logs/pickplace_check_*.log`.

### 10. Notes (`docs/designs/phases/phase_v1_notes.md`, Create)

What was done, the test output, the files changed, and anything the plan did not cover.

## Addendum A (after the independent review, 2026-10-29)

This addendum **supersedes** the sections it names.

### A.1 Files changed (explicit list)

*   **Modify:**
    *   `robot_service/robot_service.proto` (already done)
    *   `gen/**` (regenerated; changes under `gen/intrinsic/` and `gen/google/` from `gen_protos.sh` are
        expected and kept)
    *   `container/run.sh`
    *   `tools/build_catalog_meshes.py`
    *   `container/services/robot/grasp.py` (`catalog_entry` only)
    *   `container/services/robot_service.py`
    *   `mcp_server/server.py`
    *   `tests/test_mcp_server.py`
    *   `assets/objects/obstacle/object.yaml` (regenerated, see A.5)
*   **Create:**
    *   `assets/objects/red_box/{mesh.glb,object.yaml}`, `assets/objects/blue_box/{mesh.glb,object.yaml}`
    *   `container/services/robot/place.py`
    *   `tests/test_place.py`
    *   `tests/e2e/pickplace_check.py`
    *   `docs/designs/phases/phase_v1_notes.md`
*   **Also allowed:** `.venv/bin/python -m py_compile container/services/robot_service.py
    container/services/robot/place.py`. It is a done criterion: `robot_service.py` cannot be imported on
    the host, so the e2e check in the sim window is its real test.

### A.2 Box size (supersedes §4's dimensions)

*   **Both boxes: `_open_box(0.25, 0.21, 0.08, 0.01, 0.01)`.** The inner area is 0.23 × 0.19 m, which
    holds six 5 cm cubes (2 rows of 3) with the footprint model below.
*   `container` in `object.yaml` is exactly `{"wall": 0.01, "floor": 0.01}`, a dict with these two float
    keys. `candidates()` takes that dict, or `None` for a target that is not a box.

### A.3 Footprints instead of circles (supersedes §5's `held_radius`, `occupied`, `candidates`)

Everything is an axis-aligned rectangle in the **box frame**: the box position, rotated by the box yaw.

*   **Constants:**

    ```python
    GRID_M = 0.01
    EDGE_MARGIN_M = 0.005
    GAP_M = 0.005
    FINGER_HALF_SPAN_M = 0.0475  # open fingers: 0.075 / 2 + pad thickness 0.01
    FINGER_HALF_WIDTH_M = 0.012  # finger pad half width across the finger axis
    ```

*   **`half_extents(dims, angle) -> (ex, ey)`:** the half extents of a `dims[0]` × `dims[1]` rectangle
    rotated by `angle`: `ex = |cos|·dims[0]/2 + |sin|·dims[1]/2`, `ey = |sin|·dims[0]/2 + |cos|·dims[1]/2`.
*   **`held_footprint(held_dims, held_yaw, finger_yaw, box_yaw) -> (ex, ey)`:** the rectangle covering
    the held object *and* the opened fingers at release.
    *   Object: `half_extents(held_dims, held_yaw - box_yaw)`.
    *   Fingers: a rectangle of half sizes `(FINGER_HALF_WIDTH_M, FINGER_HALF_SPAN_M)` along (TCP x, TCP
        y), so `half_extents((2*FINGER_HALF_WIDTH_M, 2*FINGER_HALF_SPAN_M), finger_yaw - box_yaw)`.
    *   Result: the component-wise max of the two.
    *   `finger_yaw` is the yaw of the TCP orientation. The fingers open along the TCP's y axis: the URDF
        has `right_gripper` yaw 2.356 and `panda_hand_joint` yaw −0.785, a 180° difference about z. The
        worker confirms these two values in the URDF under `assets/robots/` and records them in the notes.
*   **`occupied(box_pose, box_dims, others) -> list[(cx, cy, ex, ey)]`:**
    *   `others` is a list of `(pose, dims)` of unattached catalog objects, excluding the box and the held
        object.
    *   Keep one when its centre in the box frame satisfies `|dx| <= box_dims[0]/2` and `|dy| <=
        box_dims[1]/2`, **and** its bottom z is in `[box.z - 0.01, box.z + box_dims[2]]`. The height test
        excludes the table and anything under the box.
    *   Return its centre in the box frame and `half_extents(dims, yaw(obj) - yaw(box))`.
*   **`candidates(target_pose, target_dims, container, held_footprint, taken) -> list[Spot]`:**
    *   **Box:** the inner limits are `lx = target_dims[0]/2 - wall - EDGE_MARGIN_M - fx` (`fx`, `fy` from
        `held_footprint`) and `ly` likewise; `[]` if either is negative.
        *   Grid points at `GRID_M` spacing with `|x| <= lx`, `|y| <= ly`, in the box frame.
        *   Drop a point if its rectangle `(x ± fx, y ± fy)`, grown by `GAP_M`, overlaps any `taken`
            rectangle.
        *   **Order: fill from the box's −x/−y corner**, sorted by (x, then y) ascending in the box frame,
            so objects pack in rows instead of blocking the centre.
        *   Convert to the world frame. Surface z = `target.z + floor`.
    *   **Not a box:** `[(target.x, target.y, target.z + target_dims[2])]`, unchanged.
*   **In `Place`:**
    *   `held_yaw = place.yaw(world.pose(object_id))`;
    *   `finger_yaw = place.yaw(tcp)`;
    *   `box_yaw = place.yaw(target_pose)`.

### A.4 Errors and locking (supersedes the matching steps of §6)

*   **Spot feasibility:** catch `ApiError` from `planner.ik` and `planner.check_collisions`. Skip the spot
    only if `error.code in POSE_REJECTED_CODES` (as `CalibrationArm.feasible` does); re-raise anything
    else, e.g. `UNAVAILABLE`.
*   **Gripper ownership:**
    *   `Pick` and `Place` pre-check, as `start_gripper` does: a running gripper operation →
        `FAILED_PRECONDITION`.
    *   Inside `work`, before each `gripper_action`, set `self.r.gripper_op = op` under `self.r.lock`. A
        concurrent `Grasp` or `Release` RPC is then rejected by the existing check, since the arm
        operation's `worker_done` is not yet set.
    *   The gripper commands are `simple_gripper_pb2.SimpleGripperFixedParams.GRASP` and `.RELEASE`.
*   **Pick opens first.** If `self.r.gripper_width() < grasp.GRIPPER_OPENING_M - 0.005` at the start of
    `work`, run `gripper_action(RELEASE)` before the pre-grasp move, with the detail "opening the
    gripper". This is setup, not a retry.
*   **Logging on every exit:** wrap the body of each `work` in `try/except ApiError` (log, then re-raise)
    and log after `finish`. The line is `log.info/warning("pick|place object=%s target=%s status=%s
    duration_s=%.3f message=%s", …)` with `status` the code name (`OK` on success). Both start with
    `start = self.r.clock.now()`.
*   **`Robot.detach` keeps the existing guards:** world detach only if `parent.name != "root"`, and
    `_wait_synced` only if `mirror_world`.
*   **`grasp.catalog_entry(catalog: str, catalog_dir: Path = CATALOG_DIR) -> dict`:** returns
    `_catalog_entry(...)[1]`.
*   **The "closed on nothing" message** ends "…; the gripper is still closed (call release before trying
    again)".

### A.5 Category clash (supersedes §4's `ATTRIBUTES` note)

*   The existing `obstacle` changes to `("pillar", "grey")` in `ATTRIBUTES`. Rerun the script so
    `assets/objects/obstacle/object.yaml` gets `category: pillar`.
*   Before the change, the worker greps `tests/` and `learn/` for uses of the obstacle's category and
    lists any in the notes.
*   Boxes keep `category: box`.

### A.6 MCP text (supersedes §7's sentence)

Use ASCII `->` like the rest of `INSTRUCTIONS`:

> Typical pick and place: list_objects, choose the ids whose category, color, size and pose match the
> request, then pick(object_id) -> place(object_id, target_id) (a target with category box: put inside;
> any other object: on top) -> list_objects to check the result. pick and place make one attempt; if one
> fails, read its error and decide what to do (e.g. after "closed on nothing", call release, then pick
> again). Objects inside a box cannot be picked again in this version. The low-level calls (plan_grasp,
> move_to_pose, grasp, release, attach_object, detach_object) remain for special cases.

### A.7 Known limitation

`GraspPlanner._apply_scene` gives MoveIt every object as a solid box of its `dims`, so the inside of an open
box is solid for the grasp planner. **Picking out of a box is not supported in V1.** The MCP text says so
(A.6) and the notes record it. Holding an attached object near the fingers is already handled by the
planner: CUJ 1 runs placed held cubes with `move_to_pose` and collision-checked plans.

### A.8 Unit tests (supersede §8)

Path setup as in `tests/test_plan_grasp.py`: append `container/services` to `sys.path`, never prepend.

1.  **Empty box**, 0.25 × 0.21 × 0.08 (wall 0.01, floor 0.01) at (0.5, 0, 0), yaw 0. Held: a 0.05 cube
    with yaws 0, fingers along y (finger yaw 0).
    *   The first candidate is the −x/−y corner spot;
    *   all candidates satisfy the inner limits;
    *   every surface z is 0.01.
2.  **Capacity:** greedily take the first candidate, add it to `taken` (cube half extents 0.025) and
    repeat. At least 6 spots are found before `candidates()` returns `[]`; no two chosen spots' cube
    rectangles overlap.
3.  **Too small:** held dims 0.3 × 0.3 → `[]`.
4.  **Box at yaw 90°:** every candidate, mapped back into the box frame, stays inside the limits.
5.  **Not a box:** a 0.05 cube target at (0.4, 0.1, 0) → `[(0.4, 0.1, 0.05)]`.
6.  **`release_pose`:** unchanged from §8 case 6.
7.  **`occupied`:**
    *   a cube inside the footprint at box z is returned;
    *   one outside the footprint is not;
    *   a table-like object (centre inside the footprint, bottom z −0.75) is not.
8.  **`held_footprint`:**
    *   finger yaw 0 → `ey >= FINGER_HALF_SPAN_M`;
    *   finger yaw 90° → `ex >= FINGER_HALF_SPAN_M`.

### A.9 E2E scenes (supersede §9's scenes)

Every scene starts with `ResetScene` and removes **all** default movable objects: cubes, plate and peg. The
hole block stays at (0.60, 0.30). Each scene then adds exactly:

| Check | Objects added |
|---|---|
| `pick_place_box` | `red_box` at (0.45, −0.22) yaw 0, plus one cube at x ∈ [0.35, 0.60], y ∈ [0.00, 0.20], yaw ∈ [−0.4, 0.4] (`random.Random(trial)`; kinds cycle red, blue, green, small_yellow) |
| `fill_box` | `red_box` at (0.45, −0.22) yaw 0; red (0.38, 0.05), blue (0.52, 0.05), green (0.38, 0.18), small_yellow (0.52, 0.18), yaw 0 |
| `stack` | `red_cube` at (0.50, 0.05), `small_yellow_cube` at (0.40, 0.17), yaw 0 |
| `errors` | `plate` at (0.50, 0.12), `red_cube` at (0.38, −0.02) |

*   **Inside-box test:** in the box frame `|dx| <= 0.115`, `|dy| <= 0.095`, and `0.005 <= z <= 0.03`.
*   **Targets:** `pick_place_box` ≥ 9/10, `fill_box` 4/4, `stack` ≥ 4/5, `errors` all pass.

### A.10 Scope against the design

Size classes and the rule checker (part of V1 in `demo_to_api.md`) move to **V1b**. V1b also adds the
owner's real object kinds once they are known. The design's phase table is updated by the lead.
