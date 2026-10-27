# Phase 3: Intrinsic drives the Isaac Franka (`ros2_hwm`, `scene_sync`, `camera_sim`, `robot_service`)

Part of [robot_platform.md](../robot_platform.md). Facts with citations:
[phase_3_inventory.md](phase_3_inventory.md) ("Inv §x"); phase 1 inventory
is cited as "P1 §x". Isaac side: [phase_2.md](phase_2.md) and its notes.
This phase also covers the former 3b (`camera_sim`).

Terms: **home** = `[0, -0.785, 0, -2.356, 0, 1.571, 0.785]` ("ready",
`robot_model/franka_scene_object.py` `HOME`); **default_q** =
`[0, -1.3, 0, -2.87, 0, 2.0, 0.75]` (Lula); joints are numbered 1–7 in the
API and 0–6 in code.

The API contract is fixed in `robot_service/robot_service.proto` and
`robot_service/simlink.proto` (already written; implement them exactly).

The work is three parts, implemented and verified in order. Parts share the
container, Isaac and Bazel, so they never run in parallel.

## Decisions

### Process layout

1.  **`robot_service` is plain Python 3.10 in the image**, not a Bazel
    `py_binary` (staging breaks its interpreter symlink, Inv §B.4). It uses
    only ICON's `_session.Session` with raw stubs (as `tests/e2e/phase1_check.py`
    does), never `icon_api` or `create_action_utils` (they import modules
    outside the vendored set). Vendored modules (copied by
    `container/stage_vendor.sh`, Create, into `container/stage/vendor/intrinsic/icon/{python,actions}/`;
    the script imports each one with `PYTHONPATH=stage/vendor:gen` in the
    venv and fails if any import fails): `python/_session.py`,
    `python/actions.py`, `python/reactions.py`, `python/errors.py`,
    `python/state_variable_path.py`, `actions/point_to_point_move_utils.py`,
    `actions/trajectory_tracking_action_utils.py`, `actions/stop_utils.py`,
    `actions/tare_force_torque_sensor_utils.py` (add any module these
    import only if it is a plain proto import available in `gen/`). The
    image adds `absl-py` (pip) and `COPY container/stage/vendor /opt/app/vendor`.
2.  **Container start order** (`entrypoint.sh`): zenohd, localcas, stubs,
    geometry, world, planner, Franka loader, then (Isaac backend)
    `ros2_hwm` immediately followed by ICON (the HWM's `RealtimeClock`
    requires ICON to join within 1 minute, Inv §A.6), then `scene_sync`,
    `camera_sim`, `robot_service`. Launch lines:
    *   `ros2_hwm`: `(cd /opt/intrinsic/bin/ros2_hwm_main.runfiles/_main && CYCLONEDDS_URI=file:///opt/app/container/dds/cyclonedds.xml exec /opt/intrinsic/bin/ros2_hwm_main --module_config_file=$CFG/ros2_hwm.txtpb --logger_connection_timeout=0s)`
        — no ROS setup sourced (Bazel's own ROS 2; the RMW is loaded
        relative to the working directory, Inv §A.10).
    *   Python ROS nodes (`scene_sync`, `camera_sim`): `bash -c 'set +u; source /opt/ros/humble/setup.bash; source /opt/ros_ext/setup.bash; set -u; RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=/opt/app/container/dds/fastdds.xml exec python3 <script>'`.
    *   `robot_service`: same, with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
        `CYCLONEDDS_URI` (for `/clock` and, in phase 5c, the grasp planner),
        `PYTHONPATH=/opt/app:/opt/app/gen:/opt/app/vendor`.
3.  **Backends:** `ROBOT_BACKEND=isaac|loopback` (default `isaac`);
    `container/run.sh` passes `-e ROBOT_BACKEND=${ROBOT_BACKEND:-isaac}`.
    `loopback`: loopback HWM + ICON with `container/config/icon_loopback.txtpb`
    (Create = the current `container/config/icon.txtpb` verbatim); no
    `ros2_hwm`, `scene_sync`, `camera_sim`, `robot_service`. `isaac`:
    `container/config/icon.txtpb` (Modify, decision 7).
4.  **Protos:** `container/proto_includes.sh` (Modify) adds the project root
    as an include root; `container/gen_protos.sh` (Modify) accepts paths
    under the project root. Generate: `robot_service/robot_service.proto`,
    `robot_service/simlink.proto`,
    `intrinsic/icon/actions/simple_gripper.proto`,
    `intrinsic/icon/actions/tare_force_torque_sensor.proto`,
    `intrinsic/icon/actions/joint_jogging.proto`,
    `intrinsic/icon/proto/v1/jogging_service.proto`,
    the proto holding `Icon2PositionPart` (find with `rg "message Icon2PositionPart" ~/intrinsic-core`),
    `intrinsic/perception/proto/v1/camera_service.proto`,
    `google/longrunning/operations.proto`.

### `ros2_hwm` (Part 1)

5.  **Location and build:** `ext/ros2_hwm/` in this repo; `container/build_intrinsic.sh`
    (Modify) creates `~/intrinsic-core/ext → <repo>/ext` if missing, adds
    `ext` to `~/intrinsic-core/.git/info/exclude`, builds
    `//ext/ros2_hwm:ros2_hwm_main`, and with `--test` runs
    `bazel test //ext/ros2_hwm:ros2_hwm_logic_test`. Targets:
    `ros2_hwm_logic` (`cc_library`: pure functions, no ROS/ICON),
    `ros2_hwm_logic_test` (`cc_test`), `ros2_hwm` (`cc_library`,
    `alwayslink`, `REGISTER_HARDWARE_MODULE`), `ros2_hwm_main`
    (`hardware_module_binary`, Inv §A.1), `ros2_hwm_config_proto`
    (`proto_library` + `cc_proto_library`; rule loads as
    `intrinsic_control/intrinsic/icon/hardware_modules/kuka_rsi/BUILD:15-17`).
    ROS deps and copts as `intrinsic_perception/intrinsic/perception/cameras/BUILD:470-519`
    (`-fexceptions`, `-use_header_modules`). `container/stage_intrinsic.sh`
    (Modify) stages it.
6.  **Config proto** `ext/ros2_hwm/ros2_hwm_config.proto`, package
    `ros2_hwm`, message `Ros2HwmConfig`: `repeated string arm_joint_names`
    (7), `string finger_joint_name`, `string joint_states_topic`,
    `string wrench_topic`, `string joint_command_topic`,
    `string gripper_command_topic`, `string epoch_topic`,
    `double stamp_tolerance_s` (must equal `config/timing.yaml`
    `stamp_match_tolerance_s`; comment says so). Packed into
    `HardwareModuleConfig.module_config` in `container/config/ros2_hwm.txtpb`
    (Create; `name: "robot_hwm"`, `drives_realtime_clock: true`,
    `control_frequency_hz: 250`).
7.  **ICON config** (`container/config/icon.txtpb`): parts `arm` (as in the
    current file, plus `payload_command`, `payload_state`,
    `joint_torque_state`), `gripper` and `ft_sensor` exactly as Inv §C.10's
    snippet with: gripper `default_grasp_width` 0.0,
    `default_release_width` 0.075, `min_width` 0.0, `max_width` 0.08,
    `default_force` 30, `min_force` 0, `max_force` 70, `default_speed` 0.1,
    `min_speed` 0, `max_speed` 0.2 (release width 0.075 because Isaac's
    open fingers reach 0.0762, phase 2 notes; RELEASE is done at width ≥
    release width, Inv §0.6); F/T `target_link_name` `panda_link8`,
    `ft_sensor_link_name` `panda_hand`, `num_acceptable_constant_readings`
    4294967295.
8.  **Interfaces** (names exactly): `joint_position_command` (strict),
    `joint_position_state`, `joint_velocity_state`,
    `joint_acceleration_state` (finite difference of velocity over the
    cycle), `joint_torque_state` (`/joint_states` effort),
    `payload_command`, `payload_state` (state = last command),
    `safety_status` (AUTOMATIC, NORMAL_OPERATION, buttons NOT_AVAILABLE),
    `gripper_status` (width = 2 × finger position, `max_width` 0.08,
    `in_error` false) and `gripper_command` (non-strict; type strings
    registered in the module, Inv §A.4), `force_torque_status` and
    `force_torque_command` (non-strict; the part writes it with `UpdatedAt`).
9.  **Wrench:** Isaac's `/wrist_wrench` is in Isaac's `panda_hand` frame,
    which is the flange rotated −π/4 about z. The module rotates force
    and torque into the flange frame: v_flange = Rz(−π/4) · v. It writes
    the result as the raw reading without changing its sign: Isaac already
    publishes the load on the sensor, and ICON negates raw readings
    (Inv §A.5). `status_code` = Ok and `enabled` = true every cycle.
10. **Taring:** start with `retare_completed = true`. On a rising edge of
    `retare` (false → true) in `ApplyCommand`: set
    `retare_completed = false`, then average the next `num_taring_cycles`
    rotated readings in `ReadStatus`; afterwards subtract that bias from
    every reading and set `retare_completed = true`.
11. **Ticking** (Gazebo pattern, Inv §A.6): the ROS callback thread pairs
    `/joint_states` and `/wrist_wrench` by stamp (|Δ| ≤ tolerance) and
    pushes `(t, state, wrench)` into a queue of depth 2 (when full, drop the
    oldest). The tick thread pops the oldest, calls
    `TickBlockingWithTimeout(t, 10 s)`; on error: `Reset`, sleep 100 ms,
    retry the same entry up to 3 times, then drop it. `ReadStatus` copies
    the entry being ticked into the interfaces. `ApplyCommand` copies the
    arm command and gripper command into an `AsyncBuffer` tagged with t.
    After a tick returns, the ROS thread publishes `/joint_command`
    stamped t **only if `ApplyCommand` ran in that tick**; otherwise
    nothing is published and Isaac free-runs on its held command
    (phase 2). `/gripper_command` is published when the gripper command
    differs from the last published one (`position` = width/2, `effort` =
    force); before the first `ApplyCommand` nothing is published. No heap
    allocation in `ReadStatus`/`ApplyCommand` (Inv §A.2, §A.9).
12. **Epoch:** subscribe `/sim/epoch` (reliable, transient-local). The
    first value received is the baseline. A different value makes the next
    `ReadStatus` return an error `"simulation reset"`; the module's
    `ClearFaults()` adopts the latest epoch as the new baseline and
    returns OK.

### Isaac side change (Part 1)

13. `sim/run_scene.py`/`sim/graphs.py` (Modify): add `/World/franka/panda_hand`
    to the `/tf` targets (the source of the true hand pose for tests and
    `GetTruePose`).

### `scene_sync`, `camera_sim`, registry (Part 2)

14. **Registry** (`container/services/platform_stubs.py`, Modify): one
    `ResourceInstance` `robot` whose `resource_handle` has `name` `robot`,
    capability names `intrinsic_proto.icon.v1.IconApi` and
    `Icon2PositionPart` (`intrinsic/icon/equipment/equipment_utils.{h,cc}`),
    `resource_data["Icon2PositionPart"]` = Any(`Icon2PositionPart{object_names{"arm": "franka"}}`),
    `connection_info.grpc.address` = `localhost:18003`
    (`skill_utils.cc:40-51`). `ListResourceInstances` honours
    `strict_filter.capability_names` (an instance matches if it has all of
    them); with no filter it returns all instances.
15. **`scene_sync`** (`container/services/scene_sync.py`, Create; Fast
    DDS):
    *   Catalog map name → catalog: from `/opt/app/config/default.yaml`
        (copy of `sim/scenes/default.yaml`), updated in memory by `Spawn`;
        `calib_board` → catalog `calib_board`. Unknown names are ignored
        (logged once).
    *   `/tf` frames with parent `world`, except `Camera_Pseudo_Depth` and
        `panda_hand`, are objects. On first sight of a name in this
        process: `DeleteObject` if it exists (`force` false), then
        `CreateGeometry` from `assets/objects/<catalog>/mesh.glb` (cached
        per catalog) and `CreateObject`
        (`create_single_entity_object`, `name_is_global_alias` true,
        `user_data["source"]` = `google.protobuf.StringValue("ground_truth")`)
        in `sim_world`, and in `world` if `belief_source` is
        `GROUND_TRUTH`. `sim_world` is created on start-up with
        `CloneWorld(world_id="world", cloned_world_id="sim_world")` if
        `ListWorlds` lacks it, then every object except `franka` and
        `franka_hand` is deleted from it.
    *   A name absent from `/tf` for 2 s → `DeleteObject` in both worlds.
    *   Poses: every `/tf` message → one `UpdateWorldResources` batch of
        `update_transform`s per world, skipping names marked attached.
    *   `belief_source` from `/opt/app/config/robot_service.yaml`
        (`config/robot_service.yaml`, Create: `belief_source: GROUND_TRUTH`).
    *   Serves `SimLink` on 127.0.0.1:18101 (`simlink.proto`): `Spawn` →
        `simulation_interfaces/SpawnEntity` (name `/World/objects/<name>`,
        uri `C:/isaac-sim-work/repo/assets/objects/<catalog>/model.usd`);
        `Delete` → `DeleteEntity`; `Reset` → `/sim/reset`, then wait (5 s)
        for the new `/sim/epoch` and return it; `AttachBoard`/`DetachBoard`
        → `/sim/attach_board`/`/sim/detach_board`; `SetAttached`;
        `GetTruePose` → latest `/tf` pose of the frame (`NOT_FOUND` if
        unknown); `GetClock` → latest `/clock`. While any
        `simulation_interfaces` call is pending, publish `/camera/request`
        (stamp = latest `/clock`) every 0.25 s (phase 2).
16. **`calib_board` catalog entry:** `tools/build_catalog_meshes.py`
    (Modify) also writes `assets/objects/calib_board/mesh.glb` (box
    0.21 × 0.15 × 0.005 m, origin at the bottom centre) and `object.yaml`;
    and `assets/objects/obstacle/` (static box 0.1 × 0.1 × 0.5 m, grey),
    including `model.usd` via `sim/tools/build_catalog.py`.
17. **`camera_sim`** (`container/services/camera_sim.py`, Create; Fast
    DDS): `CameraService` on 127.0.0.1:18005, one camera
    `ros { device_id: "d455" }`. `ListAvailableCameras`; `DescribeCamera`:
    sensor 1 `"color"` (INTENSITY, 848×480) and sensor 4 `"depth"`
    (DEPTH, 848×480), `factory_camera_params` from the latest
    `/camera/camera_info` (`UNAVAILABLE` until one has arrived); `Capture`:
    publish `/camera/request` stamped (latest `/clock` + 0.05 s), wait for
    `/camera/rgb` and `/camera/depth` stamped ≥ that (timeout: the
    request's `timeout`, or 5 s if unset; then `DEADLINE_EXCEEDED`), return
    `CaptureResult` with an Rgb8u and a Depth32f `SensorImage` (raw,
    INTERLEAVED, `camera_params`, `acquisition_time` = stamp; Inv §E.1).
    Other RPCs: `UNIMPLEMENTED`.

### `robot_service` (Part 3)

18. **Server:** `container/services/robot_service.py` (Create) plus helpers
    in `container/services/robot/` (Create: `icon.py` ICON sessions and
    status, `motion.py` planning requests, `ops.py` operation table,
    `gripper.py`, `png.py` PNG encoder with `zlib`/`struct`), serving
    `RobotService`, `SimService` and `google.longrunning.Operations` on
    127.0.0.1:18100.
19. **Sessions:** each motion opens an ICON session on `["arm"]` (force
    calls in phase 5b: `["arm", "ft_sensor"]`), runs one action, and ends
    the session; gripper calls use a session on `["gripper"]`. `Stop` sends
    `intrinsic.stop` in a new session on `["arm"]` after cancelling the
    running operation's session.
20. **Sim time:** `robot_service` subscribes `/clock` (CycloneDDS
    `rclpy`). Timeouts (2 × planned duration + 2 s) and the gripper stall
    detector use it.
21. **Motions:** planning starts from ICON's current joints (`GetStatus`);
    tool frame `tcp` (object `franka_hand`) or `flange` (object `franka`);
    the planner's `discretized` trajectory runs with
    `intrinsic.trajectory_tracking`. `Home` = `MoveJoints(home)`.
22. **Gripper:** `Grasp` starts `intrinsic.simple_gripper` GRASP and
    completes when the sensed width changed < 0.1 mm over 0.2 s of sim
    time or 3 s of sim time passed (completes OK either way);
    `GripperResult.gripping` = width > 0.005 m (an empty close measures
    ≈ 0.004 m). `Release`: RELEASE, completes when width ≥ 0.075 or 3 s.
    `IsGripping`: last command GRASP and width > 0.005.
23. **Jog:** ICON `JoggingService.JogRobot` stream, part `arm`,
    `normalized_velocity` = velocity / joint max velocity (from
    `GetStatus` limits), a command every 0.1 s wall time for `duration_s`
    of sim time.
24. **Faults and recovery:** a watcher (2 Hz) reads ICON's operational
    status; when `FAULTED` for > 2 s and no `ResetScene` is in progress, it
    calls `ClearFaults`, then re-applies the stored payload and speed
    factor (tare is not re-applied; `GetState.wrench` is untared until
    `TareWrench`). Motion calls while faulted return `FAILED_PRECONDITION`
    with the fault reason.
25. **`ResetScene`:** cancel the running operation; `DetachObject` every
    attached object in `world` and `Release` the gripper; `SimLink.Reset` (gets
    the new epoch); wait until ICON reports `FAULTED` (up to 5 s);
    `ClearFaults`; wait until `READY` (10 s); clear payload; speed 1.0.
26. **`GetImage`/`GetPointCloud`:** `CameraService.Capture` via
    `camera_sim`; PNG output per the proto; the point cloud back-projects
    depth with the image's intrinsics (finite, > 0 only); `camera_pose`
    from `world`'s `d455` object if present, else identity with
    `camera_calibrated` false (calibration is phase 4).
27. **Not yet implemented** (return `UNIMPLEMENTED`): `CalibrateCamera`
    (phase 4), `DetectObjects` (phase 5), `MoveToContact` and
    `MoveCompliant` (phase 5b), `PlanGrasp` (phase 5c).

### Logging and status (all parts)

28. Follow the design's **Observability** requirement
    (`robot_platform.md`, Non-Functional Requirements). Concretely:
    *   `robot_service`: a gRPC server interceptor logs each RPC
        (`method`, request summary ≤ 200 chars, duration ms, status code,
        message). Every error path sets a specific status code and a
        message naming the cause (ICON fault text, planner failure reason,
        missing object name, timeout with the waited quantity). Operation
        metadata `OperationInfo` gains `string state = 3` (`PLANNING`,
        `EXECUTING`, `DONE`, `FAILED`, `CANCELLED`) and
        `string detail = 4` (last progress message); update
        `robot_service/robot_service.proto` accordingly.
    *   `ros2_hwm`: `LOG(INFO)` on enable/disable, fault (with reason),
        epoch change, ClearFaults, taring start/end, lockstep ↔
        no-state transitions (> 1 s without a state), and once per 10 s a
        summary (ticks, dropped entries); never per cycle in
        `ReadStatus`/`ApplyCommand`.
    *   `scene_sync`, `camera_sim`: one line per object create/delete,
        world created, SimLink call (+ result), capture (stamp, wait ms).
    *   `container/entrypoint.sh`: the `run` helper also appends to
        `/var/lib/intrinsic/logs/<name>.log`, rotated at 10 MB × 3 (a
        small `container/logpipe.py` reading stdin).
    *   Tests: `phase3_cuj_check.py` asserts that a failing call (e.g.
        `MoveToPose` to an unreachable pose) returns a non-OK code with a
        non-empty message, and that `robot_service.log` has a line for it.

## Parts and checks

All e2e checks reuse `tests/e2e/phase2_check.py`'s Isaac harness (move
its Isaac start/stop and container-probe helpers into
`tests/e2e/harness.py`, Create, and import them in both). Probes that need
ROS run in the image like phase 2's.

**Part 1** — files: `ext/ros2_hwm/*` (Create), `container/build_intrinsic.sh`,
`stage_intrinsic.sh`, `entrypoint.sh`, `run.sh`, `Dockerfile` (Modify),
`container/config/{icon.txtpb (Modify), icon_loopback.txtpb, ros2_hwm.txtpb (Create)}`,
`sim/run_scene.py`, `sim/graphs.py` (Modify), `tests/e2e/harness.py`,
`tests/e2e/phase3_hwm_check.py` (Create). Checks (ICON driven with
`_session` from WSL, as `phase1_check.py`):
*   `move`: point-to-point to home + 0.3 rad on joint 1 → Isaac `/joint_states` within 0.005 rad.
*   `trajectory`: planner trajectory home → default_q; final error < 0.005 rad.
*   `gripper`: GRASP → width < 0.005; RELEASE → width ≥ 0.075.
*   `wrench`: `ft_sensor` part status has a wrench; after `intrinsic.tare_force_torque_sensor` its force norm < 0.5 N.
*   `stop`: `intrinsic.stop` during a 5 s move stops within 0.5 s sim time.
*   `isaac_restart`: kill Isaac; after 25 s ICON reports `FAULTED`; restart Isaac; `ClearFaults` → `READY`; `move` works.
*   `epoch`: `/sim/reset` → ICON `FAULTED` ("simulation reset"); `ClearFaults` → `READY`.
*   Plus `container/build_intrinsic.sh --test` passes and `phase1_check.py` passes with `ROBOT_BACKEND=loopback`.

**Part 2** — files: `container/services/{scene_sync.py, camera_sim.py}`
(Create), `platform_stubs.py` (Modify), `config/robot_service.yaml`,
`tools/build_catalog_meshes.py` (Modify), Dockerfile (Modify: `COPY assets/objects`,
`COPY config`, the copy of `default.yaml`), `entrypoint.sh` (Modify),
`tests/e2e/phase3_scene_check.py` (Create). Checks: `sim_world` and
`world` contain the five default objects within 5 mm of Isaac's `/tf`;
`SimLink.Spawn` of `extra_cube` appears in both within 5 s and `Delete`
removes it; after a `move`, `world`'s `franka` joints are within 0.01 rad
of ICON's; `camera_sim` `DescribeCamera` matches decision 17 and
`Capture` returns 848×480 RGB + depth, `fx` 420.08, stamped after the
request; restarting the container keeps exactly one copy of each object.

**Part 3** — files: `robot_service/*.proto` (exist), `container/services/robot_service.py`,
`container/services/robot/*` (Create), `container/stage_vendor.sh` (Create),
Dockerfile, entrypoint (Modify), `tests/test_robot_service.py` (Create:
operation table, stall detector, PNG encoder), `tests/e2e/phase3_cuj_check.py`
(Create). Checks over gRPC from WSL:
*   CUJ 2: `MoveToPose` LINEAR to a reachable pose; `GetTruePose("panda_hand")` composed with the hand→`tcp` offset within 1 mm / 0.5° of the target.
*   CUJ 4: `MoveJoints(wait=false)` then `Stop` → operation ends `CANCELLED`; next motion accepted.
*   CUJ 5: trigger a fault by calling `SimLink.Reset` directly (not `ResetScene`), then a motion within 1 s (before the watcher's 2 s auto-clear) returns `FAILED_PRECONDITION`; `ClearFaults` → motion works.
*   CUJ 7: `PlanOnly` + `MoveWaypoints` (duration within 5 %); `Jog(joint=1, 0.1 rad/s, 1 s)` → joint 1 moved 0.1 ± 0.01; `ComputeFk` vs `GetState.tool_pose` within 0.1 mm.
*   Grasp: `MoveToPose` above `red_cube`, down, `Grasp` → `gripping` true; `AttachObject("red_cube")`; lift 5 cm → cube's true z rose ≥ 4 cm; `Release`, `DetachObject`.
*   Obstacle: `AddObject` catalog `obstacle` (decision 16) between two poses; `PlanOnly` succeeds and `CheckCollisions` is false at 20 evenly spaced points of the returned path; `RemoveObject`.
*   `Home`, `SetPayload(0.5)` then `GetState` ok, `SetSpeed(0.5)` doubles a move's duration ± 10 %, `ResetScene`, `ListObjects` (sources `ground_truth`), `GetImage` RGB and DEPTH decode as 848×480 PNG, `GetPointCloud` has > 100 000 points with z in (0.2, 4) m.

## Constraints

*   Bazel only through `container/build_intrinsic.sh` (now also `--test`);
    one Bazel command at a time. No `git commit`. Docker via
    `sg docker -c`. No `pkill -f`.
*   Do not modify `/home/mew/intrinsic-core` except the `ext` symlink and
    `.git/info/exclude` (decision 5).
*   If a decision proves impossible, stop and record it in
    `phase_3_notes.md`.
