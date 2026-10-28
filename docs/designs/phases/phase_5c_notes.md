# Phase 5c implementation notes

Status: **stopped at Step 3 again, after Addendum A.** Addendum A is
applied. Blockers 1 and 2 are gone, and `cube` passes 10/10. `peg` fails
5/10, and the cause is Addendum A.2's 5 mm margin itself (blocker 3). The
fix changes a value that the addendum fixed, so it needs a design decision,
and this run stops here as instructed. A diagnostic build with a 0.5 mm
margin passed every check, peg 10/10 included. The code and the image are
back on 5 mm.

`tests/e2e/phase5c_check.py` with Addendum A
(`logs/phase5c_check_addA_run1.log`):

| Check | Result |
|---|---|
| `planner_up` | PASS (server available 1 s after `robot_service` was ready) |
| `cube` | PASS, 10/10 lifted (rose 99.6–100.0 mm, gripping at 50 mm) |
| `peg` | FAIL, 5/10 (blocker 3) |
| `too_wide` | PASS |
| `logs` | FAIL, from a check bug (it counted the accept handler's "Accepted … goal" lines too); fixed, and PASS in the diagnostic run |
| `errors` | PASS |

The same check on a diagnostic image with a 0.5 mm margin
(`logs/phase5c_diag_margin_0p5mm.log`) passed all six checks: cube 10/10
and peg 10/10.

Regression with Addendum A (5 mm image, `logs/phase5c_addA_regression_*.log`):

- phase 3 cuj: 8/8. `cuj4` (Addendum A.3) passes: "Stop 1 s into a 4.47 s
  MoveJoints → CANCELLED". The first run failed `obstacle` 1/20, the
  known flake; the rerun (`…_phase3_cuj_check2.log`) passed 8/8.
- phase 3 hwm: 7/7
- phase 3 scene: 5/5
- phase 4: 6/6
- `pytest tests/`: 49 passed

No container or Isaac process is left running. `intrinsic-core:dev` is
built from the current tree (5 mm margin).

## Addendum A changes

- **A.1** (`container/services/robot/grasp.py`): every `CollisionObject`
  (`ADD` and `REMOVE`) uses `header.frame_id = "base_link"`
  (`PLANNING_FRAME`). The code never had a static TF; the one in the first
  run was started by hand.
- **A.2** (`grasp.py`): `lowered_m(static, bottom_z, dims)` returns
  0.005 m when the catalog entry is `static: true` and the top
  (`pose.z + dims.z`, object upright) is within 0.005 m of z = 0. The
  MoveIt box's `primitive_poses[0].z` becomes `dims.z/2 − lowered`. Only
  `table` qualifies. The first time per process, `robot_service.log` gets
  "plan_grasp MoveIt box of table lowered by 0.005 m (its top is at the
  robot base plane)". `catalog_static` reads `static` from `object.yaml`.
  New unit test: `test_static_top_at_the_base_plane_is_lowered`.
- **A.3** (`tests/e2e/phase3_cuj_check.py` `cuj4`): `Home`, then joint 1
  +1.5 rad at `SetSpeed(0.2)`. At full speed the move takes under 1 s
  (0.5 rad takes 0.41 s, `misc`), so the 3 s needs the slower speed. The
  check asks `PlanOnly` for the planned duration and fails if it is under
  3 s (it is 4.47 s). Then `MoveJoints` (no wait), `Stop` 1 s later, and
  `SetSpeed(1.0)` in a `finally`.
- **`phase5c_check.py` `logs`:** it now counts only main.cpp's per-goal
  lines (`]: goal ` with ` target=`).

## Blocker 3: the 5 mm margin lets MoveIt pick grasps that are in the table

**Facts** (`logs/phase5c_check_addA_run1.log`):

- In each peg trial, the planner's best grasp was either top-down at
  z = 0.06 (quality about 0.11) or from the side at the peg's mid-height,
  z = 0.04 (quality about 0.115–0.12). The two qualities are close, so the
  choice changes from trial to trial.
- All 5 top-down trials lifted the peg. All 5 side trials failed at the
  first `MoveToPose` (pregrasp at y = 0.21, z = 0.04). Intrinsic's
  planner returned `NOT_FOUND` "IK could not find a collision free
  configuration": all 4 IK solutions have `panda_link6` in collision
  with `table`.
- FK of those solutions with the URDF collision meshes (`/tmp` script,
  not kept) puts `panda_link6` 4.2 mm and `panda_link7` 3.9 mm below the
  table top (z = 0). MoveIt's table top is at −0.005 (A.2), so MoveIt
  sees 0.8 mm of clearance and returns the grasp. Intrinsic uses the real
  table and rejects it correctly. The arm would hit the table.
- Any margin δ lets MoveIt accept poses up to δ inside the table.
  `panda_link0`'s collision mesh reaches only 0.03 mm below z = 0 at
  home, so a much smaller margin still clears the start state.
- Cube grasps are not affected: the best cube grasp was top-down
  (z = 0.03) in all 10 trials.

**Diagnostic** (temporary image with `BASE_PLANE_MARGIN_M = 0.0005`,
reverted and rebuilt afterwards; `logs/phase5c_diag_margin_0p5mm.log`,
`logs/phase5c_diag_margin_0p5mm_container.log`): all checks passed, and
peg 10/10. MoveIt now rejected the side grasps, and every peg trial got
the top-down grasp at z = 0.06.

**Options for the designer:**

- (a) Make the A.2 margin 0.0005 m (the threshold for "top within … of
  z = 0" uses the same constant). Verified above. It is one constant in
  `grasp.py`. It still allows poses up to 0.5 mm into the table, which
  Intrinsic would reject.
- (b) Leave the table box as it is and allow `table`–`panda_link0` in
  the scene diff's `allowed_collision_matrix` (blocker 2 option (b)).
  MoveIt would then see the true table for every other link. Not tried.
  It is not checked that the planner's `/get_planning_scene` request
  includes the ACM.
- (c) In `robot_service`, check each returned grasp's pregrasp and grasp
  with Intrinsic's IK, in quality order, and return the best one that
  passes. This catches any mismatch between the two collision models,
  but it changes decision 6.7 and adds planning time. Not tried.

## Success rates and planning times

PlanGrasp time is measured by the client: the whole RPC, including the
scene update. The planner's own time is `duration_s` in
`grasp_planner.log`.

| Run | Object | Lifted | PlanGrasp median / max | Planner `duration_s` |
|---|---|---|---|---|
| 5 mm margin (Addendum A) | `red_cube` | 10/10 | 0.82 / 0.87 s | 0.51–0.62 s (first 6 goals) |
| 5 mm margin (Addendum A) | `peg` | 5/10 | 1.37 / 1.43 s | not kept (log overwritten by the diagnostic) |
| 0.5 mm margin (diagnostic) | `red_cube` | 10/10 | 0.84 / 0.88 s | median 0.62, max 0.66 s |
| 0.5 mm margin (diagnostic) | `peg` | 10/10 | 2.09 / 2.12 s | median 1.87, max 1.90 s |

With the true table, peg planning takes longer: MoveIt's collision checks
now reject the side candidates.

# First run (before Addendum A)

Status then: stopped at Step 3. Decisions 2–5 are built and the grasp
pipeline links and runs (`planner_up` passes). Decision 6 is implemented
and unit-tested. The planner can't produce a grasp in Isaac's scene,
though, for two reasons that the plan doesn't resolve. Each fix is a design
decision, so this run stops here, as instructed.

1. **MoveIt doesn't know the frame `world`** (decisions 3 and 6.3). Every
   `/apply_planning_scene` call fails.
2. **The start state collides with the table** (decision 6.3, the risk
   noted in the plan). This was found with a hand-started static TF for
   blocker 1 (diagnostic only, not in the code). With that TF, every goal
   fails with "panda_link0 colliding with table".

`tests/e2e/phase5c_check.py` results:

| Check | Result |
|---|---|
| `planner_up` | PASS |
| `cube` | FAIL, 0/10 (blocker 1) |
| `peg` | FAIL, 0/10 (blocker 1) |
| `too_wide` | PASS |
| `errors` | PASS |
| `logs` | FAIL in run 1, from a counting bug in the check (fixed, see "Check output"); not rerun |

Regression on the phase 5c image:

- phase 3 hwm: 7/7
- phase 3 scene: 5/5
- phase 4: 6/6
- `pytest tests/`: 48 passed
- phase 3 cuj: 7/8. `cuj4` fails on every run, and it fails the same way
  on the pre-5c image. The cause is machine timing, not phase 5c. See
  "Regression: cuj4".

No container or Isaac process is left running. `intrinsic-core:dev` is the
phase 5c image.

## Blocker 1: the frame `world` is unknown to MoveIt

**Facts.**

- The SRDF virtual joint `world`→`base_link` is `fixed` (decision 3). For a
  fixed virtual joint, MoveIt keeps the URDF root link as the model
  (planning) frame. In Humble `robot_model.cpp`, `constructJointModel` has
  the comment "for fixed frames we still use the robot root link". So the
  planning frame is `base_link`, and `world` is not a robot frame.
- Decision 3 adds no `robot_state_publisher` and no static TF. Isaac's
  `/tf` has `world` → `table`, `red_cube`, `plate`, `peg`, `hole_block`,
  `Camera_Pseudo_Depth`, `d455_optical` and `panda_hand`, but no
  `base_link` (`logs/phase3_tf.log`).
- `move_group`'s planning scene monitor therefore can't attach any `/tf`
  frame to `base_link`. Every scene update logs one WARN per frame:
  "Unable to transform object from frame 'world' to planning frame
  'base_link'". The `CollisionObject`s with
  `header.frame_id = "world"` (decision 6.3) are rejected with
  "Unknown frame: world", and `/apply_planning_scene` returns
  `success=false`.
- `robot_service` reports this as `UNAVAILABLE` "MoveIt's
  /apply_planning_scene returned success=false" (decision 6.3: failure →
  `UNAVAILABLE`).
- Upstream solves this with a static `world → base_link` TF
  (phase_5c_inventory.md, `moveit_planning_node` `:180-214`). The plan
  dropped it because "Isaac already publishes `/tf`", but Isaac's `/tf` has
  no `base_link`.

**Diagnostic** (`/tmp` script, not kept; output in
`logs/phase5c_frame_probe.out`, `logs/phase5c_frame_probe_container.log`).
I started the phase 5c image unchanged and ran
`ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id base_link`
by hand in the container, on CycloneDDS. After that,
`/apply_planning_scene` succeeded and goals reached the planner. They then
hit blocker 2.

**Options for the designer** (only (b) was tried):

- (a) Use `header.frame_id = "base_link"` in decision 6.3: one line in
  `robot/grasp.py`. This is the same frame, because the plan already treats
  world as the robot base (decision 6.7). Isaac's `/tf` frames would still
  produce about 8 WARN lines per scene update in `grasp_planner.log`.
- (b) Add an identity static TF `world → base_link` to
  `grasp_planner.launch.py`, as upstream does. Verified above. MoveIt then
  also imports Isaac's `/tf` frames, so there are no WARN lines.
- (c) Make the virtual joint non-fixed (for example `floating`), so that
  the model frame becomes `world`. This adds variables that are not in
  Isaac's `/joint_states`. Not tried.

## Blocker 2: the start state collides with the table

**Facts** (from the diagnostic with blocker 1 bypassed):

- All six PlanGrasp goals (3 × `red_cube`, 3 × `peg`) were aborted after
  about 1.6 s with `error_code=99999`.
- `grasp_planner.log` shows the MTC stages: `generate box grasp poses`
  20/20, `compute grasp ik` 96 solutions, and then
  `allow gripper-object collision (0/96): panda_link0 colliding with table`.
- Decision 6.3 puts every catalog object in MoveIt's scene, including
  `table`. The table box is 1.2 × 0.8 × 0.75 m at (0.45, 0, −0.75), so its
  top is at z = 0, where `panda_link0` (the robot base) stands. MoveIt
  counts the contact as a collision. The `Fix Start State In Collision`
  adapter can't move a fixed base.

**Options for the designer** (none tried):

- (a) Leave `table` out of the planning scene.
- (b) Allow `table`–`panda_link0` collisions in the scene diff's
  `allowed_collision_matrix`. SRDF `disable_collisions` only covers robot
  links.
- (c) Shrink or lower the table's collision box by a margin.

Nothing past this stage has run yet: grasp quality, the width filter on
real planner output, MoveToPose to the returned poses, and the lift. More
problems could still appear there.

## Step 1 (decisions 1–5)

- **Vendoring** from `intrinsic-moveit` `v0.0.4`, commit `c117cc5e911c`
  (cloned to `/tmp`):
  - `ros_ws/src/moveit_planning_interfaces/`: identical copy.
  - `ros_ws/src/grasp_planner/`: the three pipeline `.cpp`/`.hpp` pairs,
    with only their includes changed to `.h` (decision 2).
  - A new `main.cpp`, `CMakeLists.txt` and `package.xml`.
  - `ros_ws/src/VENDORED.md` lists the changes.
- **`main.cpp`** is ported from the upstream node, minus Intrinsic:
  - It uses `automatically_declare_parameters_from_overrides(true)`, which
    Humble needs for MoveIt's parameters.
  - `/get_planning_scene` gets its own callback group and executor thread.
  - Goals are rejected when there are no candidates or an id is empty.
  - Each goal logs one line: `goal <succeeded|aborted|cancelled>
    target=… candidates=N grasps=N duration_s=… error_code=…`.
- **`franka_grasp_config`** follows decision 3. `ros_ws/prepare.sh` writes
  the URDF and meshes at image build time: it drops the `camera_*` links
  and joints and rewrites mesh URIs to `package://franka_grasp_config/…`.
  The generated files are gitignored.
- **Image** follows decision 4. The colcon build links. `move_group` and
  `grasp_planner` start, and the OMPL and KDL plugins load ("You can start
  planning now!").
- **Catalog dimensions** follow decision 5. `tools/build_catalog_meshes.py`
  writes `dims` (mesh extents). It was rerun: all 7 `object.yaml` files
  gained `dims`, and no `mesh.glb` changed. `scene_sync` sets
  `user_data["catalog"]`.
- **`planner_up`:** `pgrep` finds both processes. A CycloneDDS `rclpy`
  `ActionClient` in the container finds `/grasp_planning/plan_grasps`
  about 1 s after `robot_service` is ready.

## Step 2 (decisions 6 and 7)

- **`container/services/robot/grasp.py`:**
  - Pure helpers: `catalog_dims`, `require_graspable`, `extent_along`,
    `deepened`, `choose`, `matrix_pose` and `response`.
  - `GraspPlanner`: imports ROS lazily, owns its own node and executor
    thread, and uses a lock so only one PlanGrasp runs at a time. It
    remembers which objects it added to MoveIt, so it can send `REMOVE`
    for objects that have left `world`.
- **`robot_service.py`:** creates the `GraspPlanner` in `Robot.__init__`.
  `PlanGrasp` calls `plan_grasp`. Status and message come through
  `ApiError`, and the existing `LoggingInterceptor` logs every call.
- **`tests/test_plan_grasp.py`** has 17 cases, all passing:
  - the cube's side grasp at yaw 0° and 30° (grasp, pregrasp and joint
    order)
  - finger-axis extent 0.075 kept, 0.076 dropped, and the best fitting
    grasp chosen
  - the plate rejected at step 2
  - depth 0.02 (cube) and 0.015 (a box 0.025 m tall)
  - catalog lookups and a missing IK joint (`INTERNAL`)
  - `matrix_pose` round trips, including 180° rotations
- **Decision 7** was already in place:
  `ResetScene` → `_reset` detaches every attached object and releases the
  gripper (`robot_service.py:906-920`). No change.

## Deviations and choices

- **The `UNAVAILABLE` message with `GRASP_PLANNER=off`.** With the planner
  off, `move_group` doesn't run either, so step 3 (the scene) fails before
  step 4 (the action) can. Both "not available" messages begin with
  "grasp planner not running:". The actual reply is "grasp planner not
  running: MoveIt's /apply_planning_scene is not available after 5 s".
- **Timeouts.** The 5 s scene timeout covers the service wait and the call
  together. The 30 s result timeout starts when the goal is sent.
- **Launch:** both nodes use `on_exit=Shutdown()`, so if one exits, the
  launch (and its `grasp_planner.log` stream) ends.
- **`run.sh`** passes `GRASP_PLANNER` through to the container (default
  `on`). `phase5c_check.py` uses this for the `errors` check.
- **`main.cpp`** uses `rmw_qos_profile_services_default` for the
  `/get_planning_scene` client. Humble's `create_client` doesn't take
  `rclcpp::ServicesQoS`.
- **`package.xml` maintainer email:** the git user email. catkin rejects
  `@localhost`.
- **Base image:** `ros:humble-ros-base` was re-pulled for this build, so
  apt package versions may differ slightly from the inventory.
- **`phase5c_check.py`** waits up to 10 s after `ResetScene` until
  `world`'s pose of the target is within 5 mm of Isaac's. This keeps it
  from planning on a stale pose. It is a check-side wait only.

## Regression: cuj4

Resolved by Addendum A.3 (see "Addendum A changes").

`phase3_cuj_check` `cuj4` sends `MoveJoints` (joint 1 to 1.0 rad), sleeps
1.0 s, then calls `Stop` and expects the move to end `CANCELLED`. It failed
on all three runs on the phase 5c image, once with `GRASP_PLANNER=off`, and
once on an image built from the pre-5c tree (`git HEAD` copy, all layers
cached).

The cause is a race:

- The move starts at joint 1 = 0.17 rad, left there by `cuj7`'s `Jog`.
- The planned trajectory is 0.565 s, and ICON finishes it about 0.96 s
  after the `MoveJoints` RPC.
- The `Stop` arrives about 40 ms later.

Phases 3 and 4 passed this check only because the move finished just
after the Stop arrived (`phase_4_notes.md:520`: "next Home done in
0.65 s", meaning the arm was nearly at 1.0 rad). Phase 5c doesn't touch
this behaviour. The check would need a longer move or a shorter sleep;
that wasn't changed, because it is outside this phase.

## Check output

`logs/phase5c_check_run1.log`:

```
PASS planner_up: move_group pid 423, grasp_planner pid 425; /grasp_planning/plan_grasps available 1 s after robot_service was ready
  red_cube trial 1..10: FAILED UNAVAILABLE: MoveIt's /apply_planning_scene returned success=false
FAIL cube: RuntimeError: 0/10 lifted; PlanGrasp median 0.11 s, max 0.11 s (need 9)
  peg trial 1..10: FAILED UNAVAILABLE: MoveIt's /apply_planning_scene returned success=false
FAIL peg: RuntimeError: 0/10 lifted; PlanGrasp median 0.11 s, max 0.11 s (need 9)
PASS too_wide: PlanGrasp('plate') -> ABORTED 'object wider than the gripper opening (0.075 m) in every horizontal direction'
FAIL logs: RuntimeError: 21 'rpc PlanGrasp' lines (21 with status and ms) for 21 calls; 0 'goal ' lines in grasp_planner.log for 20 goals
PASS errors: PlanGrasp('nope') -> NOT_FOUND "no object 'nope' in world"; GRASP_PLANNER=off: PlanGrasp('red_cube') -> UNAVAILABLE "grasp planner not running: MoveIt's /apply_planning_scene is not available after 5 s"
```

The `logs` failure came from the check, which counted goals that were
never sent: they all stopped at `UNAVAILABLE` in step 3. The check now
counts a goal only when PlanGrasp got past step 3. In the diagnostic run,
`grasp_planner.log` had one `goal aborted …` line per goal.

Planning times: none yet for a successful grasp. In the diagnostic, an
aborted goal took 1.6–1.7 s.

## Files

Created:

- `ros_ws/prepare.sh`
- `ros_ws/src/VENDORED.md`
- `ros_ws/src/moveit_planning_interfaces/` (vendored, unchanged)
- `ros_ws/src/grasp_planner/` (`CMakeLists.txt`, `package.xml`,
  `src/main.cpp`, the vendored pipeline in `src/` and
  `include/moveit_planning_service/`)
- `ros_ws/src/franka_grasp_config/` (`CMakeLists.txt`, `package.xml`,
  `srdf/franka.srdf`, `config/{kinematics,joint_limits,ompl_planning}.yaml`,
  `launch/grasp_planner.launch.py`)
- `container/services/robot/grasp.py`
- `tests/test_plan_grasp.py`
- `tests/e2e/phase5c_check.py`
- `docs/designs/phases/phase_5c_notes.md`

Modified:

- `.gitignore`
- `container/Dockerfile`
- `container/entrypoint.sh`
- `container/run.sh`
- `container/services/robot_service.py`
- `container/services/scene_sync.py`
- `tools/build_catalog_meshes.py`
- `assets/objects/*/object.yaml` (7 files: `dims`)

Changed in the Addendum A run (all already in the lists above, except
`phase3_cuj_check.py`):

- `container/services/robot/grasp.py` (A.1, A.2)
- `tests/test_plan_grasp.py` (A.2 test)
- `tests/e2e/phase5c_check.py` (`logs` count)
- `tests/e2e/phase3_cuj_check.py` (A.3, `cuj4`)
- `docs/designs/phases/phase_5c_notes.md`
