# Phase 3: Intrinsic drives the Isaac Franka (`ros2_hwm`, `scene_sync`, `camera_sim`, `robot_service`)

Part of [robot_platform.md](../robot_platform.md). Facts with citations:
[phase_3_inventory.md](phase_3_inventory.md) ("Inv §x"). Isaac side:
[phase_2.md](phase_2.md) and its notes. This phase also covers 3b
(`camera_sim`).

The work is split into three parts, implemented and verified in order
(each part's checks must pass before the next starts). Parts share the
container, Isaac and Bazel, so they never run in parallel.

## Decisions made in this plan (design doc updated accordingly)

1.  **`robot_service` is plain Python in the image (Python 3.10), not a
    Bazel `py_binary`.** Staging a Bazel `py_binary` breaks its
    interpreter symlink (Inv §B.4), and phase 1 already drives ICON from
    plain Python with `_session.Session` (`tests/e2e/phase1_check.py`).
    The image gets a copy of `intrinsic_control/intrinsic/icon/python/` and
    `intrinsic_control/intrinsic/icon/actions/*_utils.py` under
    `/opt/app/vendor/` (namespace packages, Inv phase 1 §4.5) plus the
    `gen/` stubs. Its ROS 2 client (for the grasp planner, phase 5c) is the
    image's `rclpy`.
2.  **ICON parts:** `arm` (`HalArmPart`, as now, plus `payload_command`,
    `payload_state`, `joint_torque_state`), `gripper`
    (`HalLinearGripperPart`), `ft_sensor` (`HalForceTorqueSensorPart`,
    attached mode), all on module `robot_hwm` (Inv §C.10 snippet, with the
    values below).
3.  **Gripper:** ICON's only gripper action is GRASP/RELEASE to configured
    widths (Inv §0.6). Config: `default_grasp_width` 0.0,
    `default_release_width` 0.08, `min_width` 0.0, `max_width` 0.08,
    `default_force` 30 N, `min_force` 0, `max_force` 70, `default_speed`
    0.1, `min_speed` 0, `max_speed` 0.2. GRASP never reaches "done" when an
    object stops the fingers, so `robot_service.Grasp` starts the GRASP
    action and returns when the sensed width has changed < 0.1 mm over
    0.2 s of sim time (or 3 s sim time passed); `IsGripping` = sensed width
    > 1 mm and the last command was GRASP. Isaac holds the finger target
    after the session ends. `SetGripper(width, force)` is dropped (no ICON
    action supports it); `Grasp` takes no force argument.
4.  **Jog is joint-only** through ICON's `JoggingService` (Cartesian
    jogging is not implemented in this Intrinsic version, Inv §C.9). CUJ 7
    becomes "jog joint 1 by 0.1 rad".
5.  **Wrench into ICON:** `ros2_hwm` rotates Isaac's `/wrist_wrench`
    (frame Isaac `panda_hand` = flange·Rz(−π/4)) into the flange frame:
    F_flange = Rz(−π/4)·F, same for torque; it is written unchanged in sign
    as the raw F/T reading (Isaac publishes the load on the sensor; ICON
    negates raw readings into its "robot exerts on world" convention, Inv
    §A.5). `ft_sensor_link_name` `panda_hand` (entity of `franka_hand`, at
    the flange), `target_link_name` `panda_link8`,
    `num_acceptable_constant_readings` 4294967295 (Inv §0.4),
    `status_code` Ok every cycle (Inv §0.5). Taring is done in the module:
    when `retare` is set, average the next `num_taring_cycles` readings,
    subtract that bias afterwards, then set `retare_completed`.
6.  **Ticking:** the Gazebo pattern (Inv §A.6): the ROS thread pushes
    `(t, state, wrench)` when both messages for stamp t (within 1 µs) have
    arrived; a tick thread calls `TickBlockingWithTimeout(t, 10 s)`; on
    error `Reset` + 100 ms + retry. `ReadStatus` copies the newest pushed
    state into the interfaces; `ApplyCommand` copies the arm and gripper
    commands into an `AsyncBuffer`; the ROS thread publishes
    `/joint_command` stamped t after each tick, and `/gripper_command` when
    the gripper command changed. No allocation in `ReadStatus` /
    `ApplyCommand` (Inv §A.2; `AsyncBuffer`, Inv §A.9).
7.  **Epoch / reset:** `ros2_hwm` subscribes `/sim/epoch` (CycloneDDS,
    transient-local, which receives the latched value, phase 2 notes); on
    a change after start-up it returns an error from the next `ReadStatus`
    ("simulation reset") → module faulted → `robot_service.ResetScene`
    calls `ClearFaults` (Inv §A.8).
8.  **`ros2_hwm` code location:** `ext/ros2_hwm/` in this repo, linked into
    `~/intrinsic-core/ext` by `container/build_intrinsic.sh` (create the
    symlink if missing; add `ext` to `~/intrinsic-core/.git/info/exclude`).
    Built with `hardware_module_binary` (Inv §A.1); ROS libraries as in
    `intrinsic_perception/.../cameras/BUILD:470-519` (`-fexceptions`,
    `-use_header_modules`, Inv §A.10). The entrypoint starts it with the
    working directory `<runfiles>/_main` (RMW loaded by a relative path,
    Inv §A.10).
9.  **Robot joints in `world`:** `platform_stubs.py` lists one
    `ResourceInstance` named `robot` with the ICON capability and
    `resource_data["Icon2PositionPart"]` = `Icon2PositionPart{object_names{"arm": "franka"}}`
    and connection info `localhost:18003`, so the world service's
    `WorldUpdater` follows ICON's joints
    (`intrinsic/world/service/updater/world_updater.cc:1050-1140`; the
    connection fields are read by `skills::GetConnectionParamsFromHandle`;
    the capability names come from
    `icon::Icon2ResourceSelectorBuilder().WithPositionControlledPart()`).
10. **`scene_sync`** (Fast DDS, image `rclpy`): subscribes `/tf`; objects =
    frames whose parent is `world` except `Camera_Pseudo_Depth`,
    `d455_optical`. Catalog name: from `config/objects.yaml` (written by
    `robot_service` through `SimLink.AddObject`, plus the default scene
    from `sim/scenes/default.yaml` copied into the image). New object →
    `CreateGeometry` (GLB) + `CreateObject` (`create_single_entity_object`,
    `name_is_global_alias`) in `sim_world` and, in ground-truth mode, in
    `world`; gone → `DeleteObject`; poses → one `UpdateWorldResources`
    batch per `/tf` message (Inv §D.1, §D.3), skipping objects marked
    attached. `sim_world` is created with `CreateWorld` at start-up if
    missing. Serves `SimLink` gRPC on 127.0.0.1:18101:
    `Spawn(catalog, name, pose)`, `Delete(name)`, `Reset()`,
    `AttachBoard()`, `DetachBoard()`, `SetAttached(name, bool)` — calls
    `simulation_interfaces` / `/sim/*`, publishing `/camera/request` every
    0.25 s while a `sim_control` call is pending (phase 2).
11. **`camera_sim`** (Fast DDS, image `rclpy`): serves `CameraService` on
    127.0.0.1:18005 for one camera, identifier `ros { device_id: "d455" }`:
    `ListAvailableCameras`, `DescribeCamera` (sensor id 1 `"color"`
    INTENSITY 848×480, sensor id 4 `"depth"` DEPTH 848×480, both with
    `factory_camera_params` from `/camera/camera_info`), `Capture`: publish
    `/camera/request` with stamp = current `/clock`, wait (timeout = request
    timeout, default 5 s) for `/camera/rgb` and `/camera/depth` stamped ≥
    it, return a `CaptureResult` with an `Rgb8u` and a `Depth32f`
    `SensorImage` (raw, INTERLEAVED, `camera_params` set; Inv §E.1). Other
    RPCs return `UNIMPLEMENTED`.
12. **`RobotService` proto** `robot_service/robot_service.proto` (package
    `robot.v1`) with the calls of the design's API table as amended above,
    long-running calls returning `google.longrunning.Operation`, and
    `SimService`. Generated with `container/gen_protos.sh` into `gen/`.
13. **Container processes** added to `entrypoint.sh` after ICON:
    `ros2_hwm` (before ICON; ICON waits up to 120 s for it), `scene_sync`,
    `camera_sim`, `robot_service` (127.0.0.1:18100). Fast DDS nodes get
    `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `FASTRTPS_DEFAULT_PROFILES_FILE`;
    `ros2_hwm` gets `CYCLONEDDS_URI`. ROS setup is sourced inside each
    process's launch line with `set +u` (sourcing under `set -u` aborts,
    phase 2 review).
14. **Loopback stays available:** `config/icon.txtpb` and
    `config/loopback_hwm.txtpb` are kept as `icon_loopback.txtpb`; the
    entrypoint picks with env `ROBOT_BACKEND=isaac|loopback` (default
    `isaac`). Phase 1's checks run with `loopback`.

## Part 1: `ros2_hwm` and ICON with arm, gripper and F/T

Target files:
*   `ext/ros2_hwm/BUILD`, `ros2_hwm.h`, `ros2_hwm.cc`,
    `ros2_hwm_config.proto` (Create): module per decisions 5–7; config
    proto fields `joint_names` (7), `finger_joint_name`, `topics` (5 names),
    `stamp_tolerance_s`. Interfaces (names exactly): `joint_position_command`
    (strict), `joint_position_state`, `joint_velocity_state`,
    `joint_acceleration_state` (finite difference of velocity),
    `joint_torque_state` (from `/joint_states` effort), `payload_command`,
    `payload_state` (copy command → state), `safety_status` (AUTOMATIC,
    NORMAL_OPERATION), `gripper_status` / `gripper_command` (non-strict,
    type strings registered in the module, Inv §A.4; width = 2 × finger
    position), `force_torque_status` / `force_torque_command`.
*   `ext/ros2_hwm/ros2_hwm_test.cc` (Create): unit tests for the wrench
    rotation, the stamp pairing and the taring average (no ROS, no ICON).
*   `container/build_intrinsic.sh` (Modify): symlink (decision 8); add
    `//ext/ros2_hwm:ros2_hwm_main` and `//ext/ros2_hwm:ros2_hwm_test`.
*   `container/stage_intrinsic.sh` (Modify): stage `ros2_hwm_main`.
*   `container/config/ros2_hwm.txtpb`, `container/config/icon.txtpb`
    (Modify/Create per decisions 2, 3, 5, 14).
*   `container/entrypoint.sh`, `container/Dockerfile` (Modify, decisions
    13, 14).
*   `tests/e2e/phase3_hwm_check.py` (Create; WSL): starts Isaac
    (`run_scene.py` as `phase2_check.py` does) and the container; via ICON
    (`_session.Session` as in `phase1_check.py`): `move` point-to-point to
    home + 0.3 rad on joint 1 → Isaac's `/joint_states` within 0.005 rad;
    `trajectory` a planner trajectory home → `default_q` executes, final
    error < 0.005 rad; `grasp` GRASP closes the fingers, RELEASE opens them;
    `wrench` `GetStatus` `ft_sensor` part reports a wrench; after
    `intrinsic.tare_force_torque_sensor` its force norm < 0.5 N;
    `stop` `intrinsic.stop` during a long move stops within 0.5 s sim
    time; `pause` Isaac stopped for 25 s → ICON reports `FAULTED`;
    after Isaac resumes, `ClearFaults` → `READY`, a move works;
    `epoch` `/sim/reset` → module fault; `ClearFaults` clears it.
Done when `bazel test //ext/...` and `phase3_hwm_check.py` pass, and
`phase1_check.py` still passes with `ROBOT_BACKEND=loopback`.

## Part 2: `scene_sync`, `camera_sim`, registry

Target files: `container/services/scene_sync.py`, `camera_sim.py`
(Create), `platform_stubs.py` (Modify, decision 9), `simlink.proto`
(`robot_service/`, Create), entrypoint/Dockerfile additions (copy
`sim/scenes/default.yaml` to `/opt/app/config/`), `tests/e2e/phase3_scene_check.py`
(Create): `sim_world` contains the five default objects with poses within
5 mm of Isaac's `/tf`; `world` too (ground-truth mode); a `SimLink.Spawn`
appears in both worlds within 5 s and `Delete` removes it; `world`'s
`franka` joints follow ICON (after a move, `GetObject(franka)` joints
within 0.01 rad of ICON); `camera_sim` `DescribeCamera` matches decision
11 and `Capture` returns 848×480 RGB + depth with `fx` 420.08 stamped
after the request.

## Part 3: `robot_service` and the CUJs

Target files: `robot_service/robot_service.proto` (Create),
`container/services/robot_service.py` (Create; decision 1, every call of
the API table except `CalibrateCamera`, `DetectObjects`, `PlanGrasp`,
`MoveToContact`, `MoveCompliant`, which return `UNIMPLEMENTED` until
phases 4, 5, 5c, 5b), Dockerfile (vendor copy), entrypoint, and
`tests/e2e/phase3_cuj_check.py` (Create): CUJ 2 (`MoveToPose` LINEAR,
Isaac hand pose via `/tf`-equivalent ground truth within 1 mm / 0.5°),
CUJ 4 (`Stop` with `wait=false`), CUJ 5 (fault → `FAILED_PRECONDITION`
→ `ClearFaults`), CUJ 7 (joint jog), `Grasp`/`IsGripping` on the red
cube (lift 5 cm: cube's Isaac z rises ≥ 4 cm), `ResetScene`,
`AddObject`/`RemoveObject`, `ListObjects`, `GetImage`, `GetPointCloud`
(points within the table/objects box), `SetSpeed` 0.5 doubles a move's
duration ± 10 %. Unit tests `tests/test_robot_service.py` for the
operation bookkeeping and the stall detector.

## Constraints

*   Bazel only through `container/build_intrinsic.sh`; one Bazel command
    at a time. No `git commit`. Docker via `sg docker -c`. No `pkill -f`.
*   Do not modify `/home/mew/intrinsic-core` except the `ext` symlink and
    `.git/info/exclude` (decision 8).
*   Never mention anything outside the technical content of this project
    in code or docs.
*   If a decision here proves impossible, stop and record it in
    `phase_3_notes.md`.
