# Phase L1: the plan tier (`ROBOT_BACKEND=plan`)

Part of [learning_from_experience.md](../learning_from_experience.md). A container mode without Isaac and without
ICON in which the **unchanged** robot_service RPCs (ListObjects, PlanGrasp, Pick, Place, Move*, ResetScene,
AddObject, RemoveObject, ListTrueObjects, ...) work: trajectories and grasps are planned for real by Intrinsic's
motion planner and the MoveIt grasp planner; execution is replaced by jumping to the trajectory's end. It makes
graded episodes cheap (target ≤ 10 s per 5-cube episode) for the expert and data generation (phases L3, L4).

## Constraints for the worker

*   Allowed: editing files, Docker via `sg docker -c` (`container/run.sh`, `docker logs`, `docker rm -f`), the
    unit tests (`.venv/bin/python -m pytest tests/ -q`) and the new `tests/e2e/plan_tier_check.py`.
    Not allowed: Isaac Sim, any other `tests/e2e/*` script, Bazel, GPU, `git commit`, `pkill -f`, modifying
    `/home/mew/intrinsic-core`. Leave the container stopped at the end.
*   Do not change behaviour of the `isaac` and `loopback` backends. Every change for `plan` is behind the backend
    switch.
*   Make no design decisions beyond this plan; anything uncovered goes to `docs/designs/phases/phase_l1_notes.md`.
*   Code style of the surrounding files (2-space indent, comments only for the why). Observability: the new
    processes log every RPC like scene_sync does (`_logged`), and every jump and gripper action at INFO
    (`plan_tier move from=[..] to=[..] planned_s=..`, `plan_tier gripper command=GRASP width_m=.. object=..`).

## Done criteria

*   `.venv/bin/python -m pytest tests/ -q` passes, including the new `tests/test_plan_tier.py` (decision 7).
*   `ROBOT_BACKEND=plan container/run.sh` starts with no Isaac running; `tests/e2e/plan_tier_check.py` passes
    (decision 8) and prints the median episode time.
*   `phase_l1_notes.md` lists files changed, the check output and anything uncovered.

## Decisions

### 1. Entrypoint (`container/entrypoint.sh`, `container/run.sh`)

*   `ROBOT_BACKEND` accepts `isaac | loopback | plan`.
*   `plan` starts: zenohd, localcas, stubs, geometry, world, planner, the two loaders, the grasp planner (when
    `GRASP_PLANNER=on`, same command as for isaac), `scene_plan` (decision 3) instead of scene_sync, and
    robot_service with `--backend plan`. It does **not** start ros2_hwm, the loopback HWM, ICON, camera_sim or
    calibration.
*   `container/run.sh` passes `ROBOT_BACKEND` through (it already does) and needs no other change.

### 2. robot_service with `--backend plan` (`container/services/robot_service.py`, new `container/services/robot/plan_tier.py`)

*   New argument `--backend {isaac,plan}`, default `isaac`. With `plan`, `Robot.__init__` uses
    `plan_tier.PlanIcon(...)` for `self.icon` and `plan_tier.PlanClock()` for `self.clock`. Nothing else in
    robot_service changes, except where a method below cannot be satisfied (then record it in the notes).
*   `PlanClock.now()`: `time.monotonic()` since start plus the accumulated virtual seconds that PlanIcon adds
    (decision 2, trajectory and gripper durations). `advancing()` is always True.
*   `PlanIcon` implements the `Icon` methods robot_service calls, with the same signatures:
    *   `operational()` → `("ENABLED", "")`; `require_enabled()` passes; `wait_state(want, t)` → `("ENABLED", "")`;
        `clear_faults()`, `set_speed()`, `set_payload()` store the values and do nothing else.
    *   `status()` returns a `GetStatusResponse` with `part_status[ARM].joint_states[i].position_sensed` (the
        current joints, start = robot_service's `HOME`), and `part_status[GRIPPER].linear_gripper_state.sensed_width`.
        `arm_joints()` as in Icon. `joint_max_velocities()` returns the values from the Franka's Lula limits that
        `robot_model` already reads (`read_lula_limits`), clamped as Icon does.
    *   `run_action(parts, action, stop_when, cancel, on_started=None, condition=None)`:
        *   `intrinsic.trajectory_tracking`: the joints become the last state of the action's trajectory;
            `PlanClock` advances by the trajectory duration; `on_started()` is called before, `stop_when(True)` once
            after.
        *   `intrinsic.simple_gripper` GRASP: the width becomes the closing width (decision 4); +0.5 virtual s.
            RELEASE: width `grasp.GRIPPER_OPENING_M`; +0.5 virtual s.
        *   the stop action (`stop_utils.create_stop_action`): no-op, `stop_when(True)`.
        *   anything else (force sequences, tare, jogging): `ApiError(UNIMPLEMENTED, "<action type> is not
            available in the plan tier")`. `run_sequence` and `jog` raise the same.
    *   After every change of the joints, PlanIcon writes them to the `franka` object in `world` and `sim_world`
        (ObjectWorldService `UpdateObjectJoints`) and publishes `sensor_msgs/JointState` on `/joint_states`
        (`panda_joint1..7`, plus `panda_finger_joint1/2` = width / 2) from robot_service's rclpy node; it also
        republishes the last state at 20 Hz so MoveIt's state monitor is always current.

### 3. `scene_plan`: the SimLink stand-in (new `container/services/scene_plan.py`)

*   Serves `robot_service/simlink.proto` SimLink on 127.0.0.1:18101 with the same request/response semantics and
    error codes as scene_sync's SimLink, so robot_service's SimService RPCs are unchanged.
*   Reuses scene_sync's `Sync` (object creation with geometry and collision parts, ID allocation, deletion) by
    feeding it from an in-process truth table instead of ROS: a class with the attributes and methods `Sync` reads
    from its `ros` argument (`lock`, `poses`, `pending`, `take_pending()`, the clock), and no ROS import. Refactor
    scene_sync only by moving code so that both processes import it (no behaviour change for isaac).
*   Truth table: prim → (catalog, pose). `Spawn(catalog, pose)` adds a prim; `Delete` removes it; `Reset` restores
    the default scene, i.e. the same prims, catalogs and poses `sim/scene.py` builds in Isaac (read from the same
    source of truth; if `sim/scene.py` cannot be imported without Isaac, move the scene list into a plain data
    module both use). `GetTruePose`, `ListTrue` read the table. `GetClock` returns the PlanClock time of
    robot_service (scene_plan keeps its own monotonic clock; equality is not required).
*   `SetAttached(id, true)`: while attached, the truth pose follows the object's pose in `world` (read every
    sync tick, 20 Hz). `SetAttached(id, false)`: the object **drops** (decision 5) and the table entry is updated;
    the next sync moves it in both worlds as for isaac.
*   `AttachBoard`, `DetachBoard`: `UNIMPLEMENTED` ("no calibration board in the plan tier").

### 4. Closing width

On GRASP, PlanIcon asks the world for the catalog objects whose oriented box contains the TCP point (FK of the
current joints for the TCP, via the planner's FK that robot_service already uses), with 5 mm tolerance. If there
is exactly one, the width is that object's extent along the TCP y axis (the fingers' closing direction), minus
0.5 mm; if none, 0.0 (closed on nothing; Pick then reports "the gripper closed on nothing" as with Isaac); if
several, the smallest extent and a WARNING log line.

### 5. Drop on release

The released object keeps x, y and yaw; roll and pitch are set to 0; z becomes the top of the highest support
under its footprint centre: the table (z = 0) or another catalog object (its top: pose z + height) or, for a
container, its floor top (catalog `container` floor thickness). Objects are never pushed: the plan tier does not
model disturbance. One INFO log line: `plan_tier drop object=.. from_z=.. to_z=.. support=..`.

### 6. What the plan tier does not model

Written at the top of `plan_tier.py` and `scene_plan.py` as the module docstring: no dynamics (slips, toppling,
pushed neighbours, contact forces), no force sequences, no camera, no calibration, joints jump instantly.

### 7. Unit tests (`tests/test_plan_tier.py`, no container)

With fakes for the world and planner stubs: trajectory jump sets joints and advances the clock; GRASP width for
one object, none, two; RELEASE width; unsupported actions raise UNIMPLEMENTED; drop onto the table, onto a cube,
into a box floor.

### 8. Container check (`tests/e2e/plan_tier_check.py`)

Model it on `tests/e2e/pickplace_check.py` but start the container with `ROBOT_BACKEND=plan` and no Isaac
(harness: skip `start_isaac`/`sync_isaac`). Checks:
*   `pick_place_box`: ≥ 9/10 cubes into a box (same layouts as pickplace_check);
*   `stack`: ≥ 4/5;
*   `errors`: the same five error cases and codes as pickplace_check;
*   `truth`: after a place into a box, `ListTrueObjects` shows the cube inside the box footprint, z on the floor;
*   `speed`: a 5-cube "all cubes in the red box" program through `program_runtime` (GrpcBackend) finishes, and
    the median wall time per cube over the run is printed; target ≤ 2 s per cube (print, not fail).

## Addendum A (review resolutions; supersedes the decisions above where they differ)

### A.1 Scope

Create: `container/services/robot/plan_tier.py`, `tests/test_plan_tier.py`, `tests/e2e/plan_tier_check.py`.
Modify: `container/entrypoint.sh`, `container/run.sh`, `container/services/robot_service.py`,
`container/services/scene_sync.py`, `tests/e2e/phase3_hwm_check.py`. No `scene_plan.py` (A.4 replaces it).

### A.2 Entrypoint reaches the container

`entrypoint.sh` is baked into the image (`Dockerfile:35`). `run.sh` additionally mounts
`$ROOT/container/entrypoint.sh:/opt/app/container/entrypoint.sh:ro`, so no image rebuild is needed. The `plan`
branch starts scene_sync with `--backend plan` (same FASTDDS command as isaac) and robot_service with
`--backend plan` (same CycloneDDS command as isaac).

### A.3 robot_service wiring (`--backend plan`)

*   `Robot.__init__(self, args, mirror_world)` reads `args.backend`. For `plan`, in this order:
    `self.planner = Planner(args.planner)`, `self.world = World(args.world)`, `self.clock = plan_tier.PlanClock()`,
    `self.icon = plan_tier.PlanIcon(self.planner, self.world, self.clock)`. The camera, calibration and simlink stubs
    are created as today. For `isaac` nothing changes.
*   `PlanClock` (needs `rclpy.init()` done, as robot_service's main already does for SimClock): own node
    `robot_service_plan` with an executor thread; `now()` = monotonic seconds since start + virtual seconds;
    `advance(seconds)`; `advancing(within_s)` → True; publishes `rosgraph_msgs/Clock` on `/clock` with `now()` at
    50 Hz (MoveIt's `use_sim_time` needs it; nothing else publishes `/clock` without Isaac);
    `publish_joints(arm: list[float], width: float)` stores the state, which a 20 Hz timer publishes as
    `sensor_msgs/JointState` (`panda_joint1..7`, `panda_finger_joint1/2` = width / 2).
*   `PlanIcon` copies the **exact** parameter lists of the `Icon` methods it replaces (including
    `clear_faults(self, enabled_timeout_s=10.0)` and `run_action`'s parameter order and defaults). `on_started`,
    when given, is called once before `stop_when` for **every** action kind (the gripper completion code depends
    on it); then `stop_when(True)` once.
*   Virtual time per trajectory: `duration_s(trajectory) / speed` (the stored SetSpeed factor), as `Robot.execute`
    computes the planned duration. Gripper actions: 0.5 s.
*   `joint_max_velocities()` returns `[math.radians(30.0)] * 7`: Icon clamps to 30 deg/s and every Franka joint
    limit is above that, so the result equals the isaac backend's. No URDF or Lula reading.
*   Joints in the worlds: after every jump, `UpdateObjectJoints` on `franka` in `world` and `sim_world`, then
    `clock.publish_joints(...)`. Gripper width changes also call `publish_joints`.
*   Unsupported RPCs fail at once with `UNIMPLEMENTED` and the message `"<Rpc> is not available in the plan tier
    (no camera, force sensor or calibration)"`, checked at the top of each RPC when `args.backend == "plan"`:
    GetImage, GetPointCloud, CalibrateCamera, DetectObjects, MoveToContact, MoveCompliant, TareWrench, GetWrench,
    Jog. GetState reports a zero wrench.

### A.4 Scene: scene_sync with a stand-in for Isaac (replaces decision 3)

`Sync` and `SimLink` stay unchanged and are reused: scene_sync gets `--backend {isaac,plan}`; with `plan`,
`main()` creates `plan_tier.PlanIsaac(world_stub, scene_yaml)` instead of `Ros()` and skips `rclpy.init()`.
`PlanIsaac` provides exactly the members `Sync` and `SimLink` use (`lock`, `poses`, `pending`, `tf_arrived`,
`clock`, `epoch`, `take_pending()`, `call(context, service, request, request_renders)`), with these semantics:
*   Truth table prim → pose tuple `(x, y, z, qx, qy, qz, qw)`, initialised from `sim/scenes/default.yaml`
    (mounted at `/opt/app/config/default.yaml`, `SCENE_PATH`) objects; `epoch` starts at 1; `clock` = monotonic
    seconds since start, updated each tick.
*   `call(...)` emulates Isaac's services and returns objects shaped like their responses (`.result.result`,
    `.result.error_message` for spawn/delete; `.success`, `.message` for Trigger):
    `/spawn_entity` adds prim `name.rsplit("/", 1)[1]` at `initial_pose` (RESULT_OK; an existing prim →
    the `simulation_interfaces` "name not unique" result code that `SPAWN_CODES` maps); `/delete_entity` removes
    it (unknown → the "not found" code); `/sim/reset` restores the default table and increments `epoch`
    (success); `/sim/attach_board`, `/sim/detach_board` → success False, message "no calibration board in the
    plan tier".
*   A 20 Hz tick thread: for each prim, the pose to publish is the truth pose, except for prims in
    `sync.attached` (PlanIsaac gets the `Sync` via `bind(sync)` after construction), whose pose is read from
    `world` (`GetTransform` root → object) and written into the truth table. A prim that was attached on the
    previous tick and is not any more is **dropped** (decision 5) from its pose read from `world` at that moment
    (synchronously, not from the previous tick). All poses go to `poses` and `pending`; `tf_arrived` is set.
*   Reset lifecycle: `Sync.reset()` clears `poses`/`pending`; the next tick republishes the restored table, so
    the objects are re-created as with Isaac's /tf.

### A.5 Drop (decision 5, made exact)

Pure function in `plan_tier.py`: `drop(pose, dims, supports) -> (z, support_name)`, `supports` = list of
`(name, pose, dims, container_floor_m | None)` of every other catalog object. Support candidates: objects whose
footprint (yaw-rotated rectangle) contains the dropped object's footprint centre; the top is `z + height`, or for a
container `z + floor thickness`. The highest top wins; none → table, z = 0. Roll and pitch become 0, yaw is kept.

### A.6 Check harness

`phase3_hwm_check._Run.start_container(self, backend: str = "isaac")` passes `ROBOT_BACKEND=backend`.
`plan_tier_check.py` starts the container with `backend="plan"` and no Isaac, waits for TCP ports 18100 and
18101 only (not ICON's 18003), then until `ListObjects` returns the default scene's objects (timeout 180 s), and
never calls `_require_state`. Its scenes are loaded with `ResetScene`, `RemoveObject`, `AddObject` as in
`pickplace_check.py`.

### A.7 Unit tests (`tests/test_plan_tier.py`, extends decision 7)

`plan_tier.py` imports `rclpy` and ROS messages only inside `PlanClock` (tests pass a fake clock). Tests, with fake
world/planner stubs that record calls: trajectory jump sets joints, advances virtual time by duration / speed and
writes `UpdateObjectJoints` to both worlds; `on_started` is called before `stop_when` for trajectory, gripper and
stop actions; GRASP width for one object, none, two; RELEASE width; unsupported actions raise UNIMPLEMENTED;
`advancing(5.0)` is True; `clear_faults()` and `clear_faults(3.0)` work; PlanIsaac: spawn/delete/reset results and
epoch, duplicate and unknown prims, attached prim follows the world pose, detach drops from the synchronous world
read, reset republishes the default table; `drop` onto the table, a cube, a box floor, the highest of two.
