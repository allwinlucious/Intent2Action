# Phase 3 inventory: `ros2_hwm`, `robot_service`, ICON actions, `scene_sync`, `camera_sim`

Part of [robot_platform.md](../robot_platform.md) ("Adapters",
"`robot_service` and `mcp_server`", "Timing", "RobotService API", "DDS per
process"). Topics are those of [phase_2.md](phase_2.md); results of phase 2
are in [phase_2_notes.md](phase_2_notes.md). This file is a factual
inventory; it proposes nothing beyond one-line consequences.

Path prefixes:

| Prefix | Path |
|---|---|
| `ic/` | `/home/mew/intrinsic-core` (branch `fix/so3-from-matrix-reject-reflections`, HEAD `b2f9336`) |
| `project/` | `/home/mew/untitled-project` |
| `$OB/` | `/home/mew/.cache/bazel/_bazel_mew/c588f6cf2ea4110d87635daafacf38aa/external` (Bazel output base, already fetched) |
| `ICON/` | `ic/intrinsic_control/intrinsic/icon` |
| `APIS_ICON/` | `ic/intrinsic_apis/intrinsic/icon` |

Method: source reads, `rg`, `strings`/`ldd` on already-built binaries, and
three read-only experiments (below). No Bazel command was run (not even
`bazel query`), no file other than this one was changed, Isaac Sim was not
started. The key findings in §0 were re-read at the cited lines; other line
numbers come from one read pass.

Experiments:

| ID | What | Command |
|---|---|---|
| X1 | ROS 2 Python packages in the image | `docker run --rm --entrypoint bash intrinsic-core:dev -c 'source /opt/ros/humble/setup.bash; python3 -c "import rclpy, tf2_ros, tf2_msgs.msg, geometry_msgs.msg"; dpkg -l …'` |
| X2 | Copying a py_binary's runfiles with symlinks dereferenced (as `cp -rL` does) | host `tar -ch` of the exec-config py_binary `rosidl_adapter_app` (+ runfiles) piped into `docker run --rm --network none intrinsic-core:dev`, then run it; then restore one symlink with `ln -sf` and run again |
| X3 | RMW library path compiled into rclcpp users | `rg RMW_LIBRARY_PATH $OB/…ros2_rmw_implementation/BUILD.bazel`; patch read |

---

## 0. Key findings

1.  **The build macro is `hardware_module_binary`** (there is no
    `icon_hardware_module`; `rg` finds none). It wraps a `cc_binary` whose
    only source is ICON's own `hardware_module_main`; the module is a
    `cc_library` (`alwayslink = 1`) with `REGISTER_HARDWARE_MODULE(T)`
    (`ICON/hal/bzl/hardware_module_binary.bzl:25-77`, `:41`;
    `ICON/hardware_modules/loopback/BUILD:29-81`;
    `ICON/hal/hardware_module_registry.h:46-57`). §A.1.
2.  **Ticking on an external event is an existing pattern.** Gazebo's HWM
    queues the sim time from its physics callback and a dedicated thread
    calls `TickBlockingWithTimeout(sim_time, 10 s)`; on failure it calls
    `Reset`, sleeps 100 ms and retries the same time
    (`ic/intrinsic/simulation/gazebo/plugins/gazebo_hwm.cc:561-576`,
    `:857-916`). There is no `TickBlocking`; the API is
    `TickBlockingWithDeadline` / `TickBlockingWithTimeout` / `Reset`
    (`ICON/control/realtime_clock_interface.h:45-55`). §A.6.
3.  **When ticks stop, ICON's main loop exits after 20 s (`--non_realtime`),
    and `ClearFaults` rebuilds ICON.** `BeginCycle` waits
    `hard_cycle_time_timeout_` = `kNonRealtimeHardCycleTimeout` = 20 s
    (`ICON/server/main_loop.cc:70`, `:129-136`, `:313-331`); the container
    starts ICON with `--non_realtime` (`project/container/entrypoint.sh:31`;
    `ICON/server/icon_main.cc:157-163`). The loop then deactivates the
    modules (`main_loop.cc:238-241`, `:287-304`), `IconService()` returns
    the exit status (`ICON/server/icon_main_loop_factory.cc:87-92`),
    `GetOperationalStatus` reports `FAULTED`, and `ClearFaults` calls
    `RebuildIconImpl()` (`ICON/server/grpc_envelope.cc:238-273`). The
    process exit was not observed (not run). §A.7.
4.  **The F/T part faults on constant readings.** With the sensor
    `enabled` and taring complete, more than `num_acceptable_constant_readings`
    (default 10) consecutive wrenches whose change is < 1e-9 (infinity norm)
    return `InternalError("Force sensor reporting constant values …")`
    (`ICON/control/parts/feature_interfaces/force_torque_sensor.cc:82`,
    `:698-726`; `ICON/control/parts/hal/force_torque_sensor_part/force_sensor_controller_migration_utils.h:49-70`).
    The field is a `uint32` and can be raised
    (`APIS_ICON/control/parts/hal/force_torque_sensor_part/hal_force_torque_sensor_part_config.proto:90-94`).
    Whether Isaac's resting wrench is bit-constant is not known. §A.5.
5.  **`ForceTorqueStatus.status_code` defaults to `GenericError`**
    (`ICON/hal/interfaces/force_torque.fbs:42-43`;
    `ICON/hal/interfaces/force_torque_utils.cc:35-38`); the module must set
    `Ok` every cycle or the part errors (`force_torque_sensor.cc:580-596`).
6.  **The gripper: only `HalLinearGripperPart`, only GRASP/RELEASE.**
    `intrinsic.simple_gripper` takes `command ∈ {GRASP, RELEASE}` only
    (`APIS_ICON/actions/simple_gripper.proto:24-31`) and moves to the
    configured `default_grasp_width` / `default_release_width` with the
    default force (`ICON/control/parts/feature_interfaces/linear_gripper.h:68-82`).
    It is done only when `width ≤ default_grasp_width` (GRASP) or
    `≥ default_release_width` (RELEASE)
    (`ICON/control/actions/simple_gripper_action.cc:102-105`). No action in
    `ICON/control/actions` uses the `LinearGripper` (width/force) feature
    (`rg`). §C.4, §F.
7.  **Cartesian jogging is not implemented in this tree.**
    `has_cartesian_jogging_signature = false` (`ICON/server/jogging_service.cc:571`);
    only joint jogging works. §C.9.
8.  **`wrench_at_ft` is on the F/T part's status, not the arm's**
    (`ICON/control/realtime_part_status_helpers.cc:159-166`). The arm part
    has no F/T fields (`HalArmPartConfig` reserves 8, 9, 17,
    `APIS_ICON/control/parts/hal/arm_part/hal_arm_part_config.proto:124`).
    Force actions take two slots, `arm` and `ft_sensor`. §C.
9.  **ROS 2 in Bazel: plain `cc_library`/`cc_binary` with
    `-fexceptions` and `-use_header_modules`;** no target in `ic/` uses
    `ros2_cpp_binary` (`ic/intrinsic_sdk/intrinsic/platform/pubsub/BUILD:91-111`;
    `ic/intrinsic_perception/intrinsic/perception/cameras/BUILD:470-519`).
    **The RMW library is loaded from `$(rootpath …)`, a path relative to the
    working directory** (X3, §A.10). No Intrinsic code ever creates an
    `rclcpp::Node` (`rg` count 0).
10. **Staging a `py_binary` with `cp -rL` breaks it** (X2): the venv
    interpreter `_<bin>.venv/bin/python3` is a relative symlink that only
    resolves inside `.runfiles/`; dereferenced, Python cannot find its
    stdlib. Restoring that one symlink makes it run. Intrinsic's own
    `python_oci_image` macro rewrites the same entry
    (`ic/bazel/python_oci_image.bzl:112-118`). §B.4.
11. **Python ICON action utils are not all visible to `//ext`.**
    `point_to_point_move_utils`, `trajectory_tracking_action_utils`,
    `tare_force_torque_sensor_utils`, `wait_for_settling_utils` are visible
    only to `//intrinsic_control/intrinsic/icon/python:__subpackages__`;
    `stop_utils` and `cartesian_admittance_utils` only to
    `//intrinsic:intrinsic_internal` (`ICON/actions/BUILD:421-483`). The public
    `//intrinsic_control/intrinsic/icon/python:create_action_utils` re-exports
    all of them (`ICON/python/BUILD:17`, `:70-83`;
    `ICON/python/create_action_utils.py:41-77`). There are no Python utils
    for `simple_gripper` or `joint_jogging`. §B.3, §C.
12. **`sim_world` does not exist at start-up**; the world binary creates
    only `world` and `init_world`
    (`ic/intrinsic/world/binary/world_binary.cc:143-147`). `SyncObject`
    creates a missing target world (`ic/intrinsic/world/service/object_world_service.cc:3460-3466`).
    §D.
13. **`CreateGeometry` takes a GLB inline** (`GeometryData.gltf_bytes = 3`,
    `ic/intrinsic_apis/intrinsic/geometry/proto/geometry_service_types.proto:95-126`),
    as the Franka loader already does (`project/robot_model/franka_scene_object.py:420-431`).
    §D.2.
14. **CameraService v1 is stateless (no handles).** Every RPC is keyed by a
    `CameraIdentifier`; C++ readers reject an image with unset
    `packing_type` (`ic/intrinsic_perception/intrinsic/perception/proto_conversion/v1/sensor_image.cc:43-45`);
    the calibration service and the pose estimator take the intrinsics from
    each image's `sensor_config.camera_params`. §E.
15. **The calibration service's `CaptureData` re-reads the `CameraConfig`
    from the AssetInstances service** (`config.hardware_device.service.service_config`)
    before each `Capture`
    (`ic/intrinsic_perception/intrinsic/perception/calibration/services/v1/calibration_service_impl.cc:650-667`,
    `:1980-2016`). §E.2.
16. **`tf2_ros`, `tf2_msgs` and `rclpy` import in `intrinsic-core:dev`**
    (X1). §D.4.

---

## A. `ros2_hwm`

### A.1 BUILD macros and targets to copy

`ICON/hardware_modules/loopback/BUILD`:

```text
load("//bazel:cc_macros.bzl", "cc_library")                                                   # :15
load("//intrinsic_control/intrinsic/icon/hal/bzl:hardware_module_binary.bzl", "hardware_module_binary")  # :16
load("//intrinsic_control/intrinsic/icon/hal/bzl:hardware_module_image.bzl", "hardware_module_image")    # :17

cc_library(name = "loopback_hardware_module", srcs, hdrs, deps = [...], alwayslink = 1)       # :29-76
hardware_module_binary(name = "loopback_hardware_module_main",
                       hardware_module_lib = ":loopback_hardware_module")                     # :78-81
hardware_module_image(...)                                                                    # :83-86 (OCI image; not needed here)
```

- Library deps to copy (`loopback/BUILD:33-73`): HAL
  `hardware_interface_handle`, `hardware_interface_registry`,
  `hardware_interface_traits`, `hardware_module_init_context`,
  `hardware_module_interface`, `hardware_module_registry`, `module_config`;
  interface schemas + utils `joint_command_fbs_cc/_utils`,
  `joint_limits_fbs_cc/_utils`, `joint_state_fbs_cc/_utils`,
  `payload_command_fbs_cc/_utils`, `payload_state_fbs_cc/_utils`,
  `robot_payload_utils`; safety `control/safety:safety_messages_fbs_cc`,
  `:safety_messages_fbs_utils`, `control/safety/extern:safety_status_fbs_cc`;
  `control:realtime_clock_interface`; `utils:core_time`, `realtime_guard`,
  `realtime_status`, `realtime_status_macro`, `time`; thread utils; the
  config proto `@intrinsic_apis//intrinsic/icon/hardware_modules/loopback:loopback_config_cc_proto`.
- `hardware_module_binary` (`ICON/hal/bzl/hardware_module_binary.bzl:25-77`)
  supplies `main()` from `//intrinsic_control/intrinsic/icon/hal:hardware_module_main`
  (`:41`). `main` locks memory (`LockMemory<256 KiB, 256 KiB>`,
  `ICON/hal/hardware_module_main.cc:193-194`), creates the module through the
  registry (`:152-159`) and creates a `RealtimeClock` only if
  `drives_realtime_clock` is set (`ICON/hal/hardware_module_main_util.cc:182-185`).
- One module per binary: `REGISTER_HARDWARE_MODULE` makes two a duplicate
  symbol (`ICON/hal/hardware_module_registry.h:23-35`, `:46-57`).
- `//bazel:cc_macros.bzl`'s `cc_library` sets `strip_include_prefix`, so
  includes are `intrinsic/icon/...` (`ic/bazel/cc_macros.bzl:112-125`).
- Module config: `HardwareModuleConfig` (`APIS_ICON/hal/proto/hardware_module_config.proto:27-90`):
  `name=1`, `module_config=2` (Any), `drives_realtime_clock=3`,
  `simulation_server_address=4`, `simulation_module_config=5`,
  `realtime_cores=6`, `disable_malloc_guard=7`, oneof
  `control_frequency_hz=8` / `control_period_ns=9`, `context_name=10`,
  `intrinsic_runtime=11`. The module's own config is a proto packed in
  `module_config` (loopback: `LoopbackConfig`,
  `APIS_ICON/hardware_modules/loopback/loopback_config.proto:21-34`); the
  current project config is `project/container/config/loopback_hwm.txtpb:3-8`.

Visibility for a `//ext` package (checked): `ICON/hal/BUILD:19-23`,
`ICON/hal/interfaces/BUILD:18`, `ICON/utils/BUILD:19-22` and
`ICON/control/safety/BUILD:18-22` include `//visibility:public`;
`control:realtime_clock_interface` and `safety/extern:safety_status_fbs_cc`
list `//intrinsic:public_api_users` (`ICON/control/BUILD:1098-1104`;
`ICON/control/safety/extern/BUILD:30-35`), which is `packages = ["public"]`
(`ic/intrinsic/BUILD:17-20`). Not every single dependency target was checked.

Other modules for reference: Axia80 F/T module (a separate-sensor template,
`ic/intrinsic_hardware/intrinsic/hardware/force_torque/axia80_net/BUILD:77-133`),
KUKA RSI (`ICON/hardware_modules/kuka_rsi/BUILD:226-273`), UR
(`ICON/hardware_modules/universal_robots/BUILD:295-309`), Gazebo
(`ic/intrinsic/simulation/gazebo/plugins/gazebo_hwm.h:72`). There is no ROS,
`ros2_control` or `icon_hwm_controller` module in `ic/` (`rg`); the
`icon_hwm_controller` is external and only described in
`ic/developer_resources/learn/tutorials/interactive_tutorials/icon/ros2_bridge_tutorial.md:15-35`.

### A.2 `HardwareModuleInterface` lifecycle

`ICON/hal/hardware_module_interface.h`:

| Method | Returns | Contract (line) |
|---|---|---|
| `Init(HardwareModuleInitContext&)` | `absl::Status` | not RT; advertise interfaces; should not fail if the hardware is down (retry in `ClearFaults`) (`:144-153`); error → `kInitFailed` (`hardware_module_runtime.cc:963-975`) |
| `Prepare()` | `absl::Status` | may block; a clock-driving module should `Reset` its clock here; `Reset` must not run in parallel with `TickBlockingWith*` (`:155-181`) |
| `Activate()` / `Deactivate()` | `RealtimeStatus` | RT loop starts / stops; failure is fatal (`:183-204`) |
| `EnableMotion()` / `DisableMotion()` | `absl::Status` | not RT, asynchronous to `ReadStatus` (`:206-252`) |
| `Enabled()` / `Disabled()` | `RealtimeStatus` | first enabled / disabled cycle; must not block (`:222-240`) |
| `ClearFaults()` | `absl::Status` | may block; OK only once cleared (`:254-260`) |
| `Shutdown()` | `absl::Status` | must unblock blocking calls (`:262-269`) |
| `ReadStatus()` | `RealtimeStatus` | cyclic, update sensor values (`:271-279`) |
| `ApplyCommand()` | `RealtimeStatus` | cyclic, **only while motion-enabled** (`:281-286`) |

- `ReadStatus`/`ApplyCommand` run on the runtime's own threads (RT priority
  and affinity when `is_realtime`, `ICON/hal/hardware_module_runtime.cc:1043-1057`),
  wrapped in `INTRINSIC_CHECK_REALTIME_SAFE` (`:259`, `:328`). A heap
  allocation inside either moves the module to `kFaulted` (`:306-322`,
  `:366-382`) unless `init_context.DisableMallocGuard()` is used
  (`ICON/hal/hardware_module_init_context.h:93-101`).
- Consequence of `ApplyCommand` running only while enabled: while ICON is
  disabled or faulted, no command is produced in a tick; Isaac then
  free-runs after `free_run_timeout_s` (`project/config/timing.yaml:3`)
  unless `ros2_hwm` publishes something itself.

### A.3 Advertising interfaces; what loopback exports

`ICON/hal/hardware_interface_registry.h`:

| Call | Module gets | `must_be_used` | Use for |
|---|---|---|---|
| `AdvertiseInterface<T>(name, args…)` (`:59-74`) | read-only handle | false | commands ICON writes, module reads |
| `AdvertiseMutableInterface<T>` (`:76-100`) | writable handle | false | state the module writes |
| `AdvertiseStrictInterface<T>` (`:102-123`) | read-only; `Value()` fails unless ICON wrote it this cycle (`hardware_interface_handle.h:129-149`, `:167-175`) | true | commands that must be fresh |
| `AdvertiseMutableStrictInterface<T>` (`:125-147`) | writable; writer calls `UpdatedAt()` (`hardware_interface_handle.h:239-241`) | true | |

- A `must_be_used` interface not used by the ICON config makes
  `MainLoop::Start` fail (`ICON/server/main_loop.h:129-145`).
- Type strings must match on both sides (`ICON/hal/get_hardware_interface.h:63-70`);
  types are registered with `INTRINSIC_ADD_HARDWARE_INTERFACE`
  (`ICON/hal/hardware_interface_traits.h:37-47`); the default set is in
  `ICON/hal/default_hardware_interfaces.h` (`JointTorqueState` etc. `:93-107`,
  F/T `:113-119`, `"intrinsic_fbs.SafetyStatus"` `:121-123`). Loopback
  registers its own traits (`loopback_hardware_module.cc:280-324`,
  `.h:127-133`) instead of including that header.

Loopback (`ICON/hardware_modules/loopback/loopback_hardware_module.cc:120-159`):

| Name | Type | Kind |
|---|---|---|
| `joint_position_command` | `JointPositionCommand` (position, velocity_feedforward, acceleration_feedforward; `ICON/hal/interfaces/joint_command.fbs:17-45`) | strict |
| `joint_position_state` / `joint_velocity_state` / `joint_acceleration_state` | `JointPositionState` / `…Velocity…` / `…Acceleration…` | mutable |
| `payload_command` | `PayloadCommand` | read-only |
| `payload_state` | `PayloadState` | mutable |
| `joint_system_limits` | `JointLimits` | strict, only with `advertise_system_limits_interface` (`:145-150`) |
| `safety_status` | `SafetyStatusMessage` (`AUTOMATIC`, `UNKNOWN`×3) | mutable |

- Loopback's `ReadStatus` is a no-op (`:273-276`); `ApplyCommand` reads the
  strict command with `Value()` (`:254-271`), which errors (→ `kFaulted`,
  `hardware_module_runtime.cc:353-361`) if ICON did not write it that cycle.
- Joint torque state: `JointTorqueState{torque}` (`ICON/hal/interfaces/joint_state.fbs:17-37`),
  builder `BuildJointTorqueState(num_dof)` (`ICON/hal/interfaces/joint_state_utils.h:24-32`);
  `HalArmPartConfig.joint_torque_state = 7`
  (`APIS_ICON/control/parts/hal/arm_part/hal_arm_part_config.proto:30-125`, list in
  `phase_1_inventory.md` §4.1).
- Joint limits: `JointLimits{min/max_position, has_/max_velocity,
  has_/max_acceleration, has_/max_jerk, has_/max_effort}`
  (`ICON/hal/interfaces/joint_limits.fbs:17-28`), `BuildJointLimits`,
  `ParseProtoJointLimits` (`joint_limits_utils.h:30-52`). The arm config
  says of `joint_system_limits = 23`: "The system limits are written to
  this hardware interface" (`hal_arm_part_config.proto:53-54`); loopback
  advertises it strict (read-only for the module). The arm's limits come
  from the world's kinematic object (`phase_1_inventory.md` §4.1;
  `phase_1_notes.md:9-39` for the ≤ 95 % rule).
- Payload: `payload_command` / `payload_state` with
  `full_payload:OptionalRobotPayload{has_value, value{mass_kg, tip_t_cog, inertia}}`
  (`ICON/hal/interfaces/payload_state.fbs:19-22`, `payload_command.fbs:19-22`,
  `robot_payload.fbs:20-39`); loopback copies command → state in
  `EnableMotion` (`:216-232`). `HalArmPartConfig.payload_command = 21`,
  `payload_state = 22` (`hal_arm_part_config.proto:49-52`).
- Safety status: `SafetyStatusMessage{mode_of_safe_operation,
  estop_button_status, enable_button_status, requested_behavior}`
  (`ICON/control/safety/safety_messages.fbs:33-45`); enums
  `ModeOfSafeOperation{UNKNOWN, AUTOMATIC, TEACH_PENDANT_1, …}`,
  `RequestedBehavior{UNKNOWN, NORMAL_OPERATION, SAFE_STOP_1_TIME_MONITORED,
  SAFE_STOP_2_TIME_MONITORED, SAFE_STOP_0, PAUSE}`
  (`ICON/control/safety/extern/safety_status.fbs:25-117`),
  `ButtonStatus{UNKNOWN, NOT_AVAILABLE, ENGAGED, DISENGAGED}`
  (`safety_messages.fbs:19-31`); helpers `BuildSafetyStatusMessage` /
  `SetSafetyStatusMessage` (`ICON/control/safety/safety_messages_utils.h:31-48`).
  ICON reads it through `RealtimeControlConfig.safety_message_handler_config`
  (`APIS_ICON/server/config/realtime_control_config.proto:92-106`); SS1/SS0
  end sessions, SS2/PAUSE pause them
  (`ICON/control/rtcl_session_manager.cc:573-590`).

### A.4 Gripper interface and `HalLinearGripperPart`

- Schema `ICON/hal/interfaces/gripper.fbs:17-35`:
  `GripperCommand{width:double, speed:double, force:double, open_direction:OpenDirection=NONE}`,
  `GripperStatus{width, temperature, max_width, in_error:bool}`, `OpenDirection{NONE, OPEN, CLOSE}`.
  Builders `BuildGripperCommand()`, `BuildGripperStatus()` (`gripper_utils.h:23-24`).
  Targets `gripper_fbs_cc`, `gripper_fbs_utils` (`ICON/hal/interfaces/BUILD:198-210`).
- The type strings `"intrinsic_fbs.GripperStatus"`/`"…GripperCommand"` are
  registered only in the part (`ICON/control/parts/hal/linear_gripper_part/hal_linear_gripper_part.cc:138-146`),
  not in `default_hardware_interfaces.h`; the module must register them
  too.
- The part reads status non-strict and writes the command through a
  mutable non-strict handle (`hal_linear_gripper_part.cc:64-74`), writes only
  width, force and speed and never calls `UpdatedAt`
  (`ICON/control/parts/feature_interfaces/linear_gripper.cc:126-128`), so the
  module should advertise the command with `AdvertiseInterface` (non-strict).
  It refuses commands while `in_error` (`linear_gripper.cc:98-101`).
- It is the only gripper part (`ICON/control/parts/hal/` holds adio, arm,
  force_torque_sensor, imu, laser_tracker, linear_gripper, rangefinder).
  Config: §C.10.

### A.5 F/T sensor interface and `HalForceTorqueSensorPart`

- Schema `ICON/hal/interfaces/force_torque.fbs:21-57`:
  `ForceTorqueStatus{enabled, retare_completed, retare_suggested, wrench:Wrench,
  status_code:ForceSensorStatusCode = GenericError, raw_status_code}`,
  `ForceTorqueCommand{retare:bool, num_taring_cycles:int = 1}`; builders
  `CreateFbsForceTorqueStatus()` / `CreateFbsForceTorqueCommand()`
  (`force_torque_utils.h:25-26`); type strings in
  `default_hardware_interfaces.h:113-119`; targets `force_torque_fbs_cc`,
  `force_torque_fbs_utils` (`ICON/hal/interfaces/BUILD:163-179`).
- `Wrench{x,y,z,rx,ry,rz}` "represents the force and torque that the robot
  is exerting on the world" unless noted (`ICON/flatbuffers/transform_types.fbs:69-74`).
  The part computes `wrench_at_ft = −raw + support_mass_wrench − bias`
  (`force_torque_sensor.cc:732-738`).
- Taring is done by the device: the part writes `retare = true` and
  `num_taring_cycles` (`force_torque_sensor.cc:762-797`) and reads
  completion from `retare_completed` (`:690-697`). `TaringData` in
  `ICON/utils/sensor_utils.h:55-80` is an RT-safe helper (Axia80 uses it,
  `axia80_net_lib.cc:507-524`).
- Constant-reading fault and `status_code`: §0 items 4–5.
- Two modes (`ICON/control/parts/hal/force_torque_sensor_part/hal_force_torque_sensor_part.cc`):
  without `joint_position_state` it is standalone (`:92-134`) and only
  provides `STANDALONE_FORCE_TORQUE_SENSOR`, which tare, admittance and
  force_primitive do not accept; with it, it needs the WorldService
  (`:240-245`) and the robot in the world, and provides `FORCE_TORQUE_SENSOR`
  (`:282-292`).
- Links: `target_link_name` must be an entity of the robot object
  (`force_torque_sensor.cc:310-327`); `ft_sensor_link_name` is looked up in
  the skeleton including child objects (`:329-332`, `:411-430`) and must be
  rigidly attached to the target link (`:432-441`). In the project's model,
  the hand object `franka_hand` has one link entity `panda_hand` placed at
  the flange with the flange→hand rotation baked into the shapes
  (`project/robot_model/franka_scene_object.py:382-387`); Isaac's
  `panda_hand` frame is the flange rotated −45° about z
  (`phase_1_inventory.md` §5.4). So the Intrinsic `panda_hand` entity frame
  and Isaac's `/wrist_wrench` frame (`phase_2.md` decision 8) differ by that
  rotation.
- The F/T part does not need to be on the arm's module: each interface is
  resolved by its own `module_name` (`hal_force_torque_sensor_part.cc:104-119`, `:150-237`).

### A.6 Driving the realtime clock; ticking per ROS message

- API (`ICON/control/realtime_clock_interface.h:28-55`):
  `TickBlockingWithDeadline(Time, absl::Time)`, `TickBlockingWithTimeout(Time, Duration)`
  (converts to a deadline, `realtime_clock_interface.cc:24-27`), `Reset(Duration)`.
  One tick = `ReadStatus` on all modules, ICON's Sense/Control, `ApplyCommand`
  on all modules (`:28-40`); callable from any thread, not re-entrant (`:41`).
- Implementation `ICON/hal/realtime_clock.cc`: the tick writes the given
  time into `cycle_start_nanoseconds`, releases ICON, and blocks until the
  cycle ends or the deadline passes (`:64-87`); ICON uses that time as the
  cycle start (`ICON/server/main_loop.cc:328-329`). `Reset` cancels and
  re-arms the lockstep (`realtime_clock.cc:88-95`). **The constructor
  `CHECK`s that ICON joins within 1 minute** (`:41`, `:43-54`).
- The module gets the clock from `ModuleConfig::GetRealtimeClock()` (nullptr
  unless `drives_realtime_clock`) (`ICON/hal/module_config.h:91`). ICON
  selects `kHardwareModuleDrivenClock` from
  `IconMainConfig.hardware_module_that_drives_clock`
  (`APIS_ICON/server/config/icon_main_config.proto:94-96`;
  `ICON/server/create_icon_main_loop.cc:275-278`); the project config has it
  (`project/container/config/icon.txtpb:6`).
- Loopback: a clock thread ticks with `TickBlockingWithTimeout(now, InfiniteDuration)`
  while `kActive`/`kMotionEnabled`, calls `Reset` on error, and sleeps for
  the rest of the period (`loopback_hardware_module.cc:70-98`, `:163-198`).
- Event-driven ticking, existing examples:
  - Gazebo: `RequestIconTick(sim_time)` pushes onto a queue, non-blocking,
    ignored while not `active_` (`gazebo_hwm.cc:561-576`); a thread waits
    on the queue (10 s timeout, loops), calls
    `TickBlockingWithTimeout(sim_time, kTickTimeout)` while holding the
    interfaces mutex, pops only on success, otherwise `Reset` + 100 ms sleep
    + retry (`:857-916`). `Prepare()` stops the thread, resets the clock and
    restarts it (`:625-684`).
  - KUKA RSI: one tick per received UDP packet with deadline = cycle +
    tolerance; if the stream stops it keeps ticking on a timer and pushes a
    fault through an `AsyncBuffer<RealtimeStatus>`
    (`ICON/hardware_modules/kuka_rsi/kuka_rsi_communicator.cc:125-137`,
    `:192-198`, `:311-355`).

### A.7 What happens to ICON when ticks pause

- Wait per cycle: `lockstep->StartOperationBWithTimeout(hard_cycle_time_timeout_)`
  (`ICON/server/main_loop.cc:313-331`). Timeout 20 s with `--non_realtime`
  (`:70`, `:129-136`); in `kNormal` mode 100 control periods. Mode:
  `--non_realtime` → `kNonRealtime`; a runtime context `REALITY` →
  `kNormal`, `PHYSICS_SIM` → `kNonRealtime` (`ICON/server/icon_main.cc:157-163`,
  `:198-211`).
- HWM read/write timeout: 20 s in non-RT (`kNonRealtimeReadWriteTimeout`,
  `ICON/server/create_icon_main_loop.cc:73`); `hardware_module_read_write_timeout_seconds`
  is used only in `kNormal` (`:212-216`).
- On timeout: `CyclicUpdate` returns the error, `RunRt` exits, `Run` stores
  `exit_status_` and calls `Shutdown()` which deactivates the modules and
  cancels the clock (`main_loop.cc:238-241`, `:287-304`). Then
  `GetOperationalStatus` = `FAULTED` with the reason, and `ClearFaults`
  rebuilds ICON (`grpc_envelope.cc:238-273`); during the rebuild modules in
  `kFatallyFaulted`/`kInitFailed` are restarted
  (`icon_main_loop_factory.cc:187-244`). ClearFaults returns
  `RESOURCE_EXHAUSTED` while a fatal clear is in progress
  (`APIS_ICON/proto/v1/service.proto:705`, `phase_1_inventory.md` §4.3).
- Pauses seen in phase 2: renders 59–534 ms (`phase_2_notes.md:128-131`),
  RTF 0.22–0.44 (`:120-121`); both far below 20 s.

### A.8 Reporting a fault or disabled state

- No `RaiseFault`/`ReportFault` API exists
  (`ICON/hal/hardware_module_init_context.h:41-164`). A non-OK return from
  `ReadStatus`, `ApplyCommand`, `EnableMotion`, `DisableMotion` or
  `ClearFaults` → `kFaulted`; `kAborted` → `kFatallyFaulted`; failure of
  `Prepare`/`Activate`/`Deactivate` → `kFatallyFaulted`
  (`ICON/hal/hardware_module_runtime.cc:138-251`, `:287-302`, `:353-362`).
  The message is truncated to 256 bytes (`ICON/hal/hardware_module_state.fbs:45`);
  `StateCode` values at `:26-40`.
- `ReadStatus` errors are ignored while `kClearingFaults`
  (`hardware_module_runtime.cc:287-289`); `ClearFaults` → `kClearingFaults`
  → `instance->ClearFaults()` → `kActivated` or faulted again (`:238-256`).
- ICON aggregates module faults as `'<module>': msg`
  (`ICON/hal/hardware_module_manager.cc:630-667`); a fatal fault shuts ICON
  down via `FailedPrecondition` (`ICON/control/realtime_control_manager.cc:659-740`).
- In-band: `GripperStatus.in_error` (gripper), `ForceTorqueStatus.status_code`
  / `enabled` (F/T), safety status (A.3).

### A.9 `async_buffer`

- `intrinsic::AsyncBuffer<T>` (`ICON/utils/async_buffer.h:65-116`): SPSC
  triple buffer, "real-time safe", no copies (`:27-33`); `GetFreeBuffer()`,
  `CommitFreeBuffer()` (false if none checked out), `bool GetActiveBuffer(T**)`
  (true when new data) (`:72-95`, `:151-203`). Allocates only in the
  constructor (`:118-124`). The state is a `std::atomic` used with default
  (seq_cst) operations (`:111`, `:162-197`).
- Target `//intrinsic_control/intrinsic/icon/utils:async_buffer`
  (`ICON/utils/BUILD:407-411`, package public).
- Users: KUKA (`kuka_rsi_communicator.h:36`, `:234-256`), UR and fake modules,
  the HWM runtime (`hardware_module_runtime.cc:450-453`).

### A.10 rclcpp from Bazel

- Module: `bazel_dep(name = "com_github_mvukov_rules_ros2", version = "0.0.0-20260118-78d85ff")`
  (`ic/MODULE.bazel:457`); repos via `use_repo` (`:959-972`): `ros2_common_interfaces`,
  `ros2_geometry2`, `ros2_rcl_interfaces`, `ros2_rclcpp`, `ros2_rclpy`,
  `ros2_rcutils`, `ros2_rmw`, `ros2_rosbag2`, `ros2_rosidl`,
  `ros2_rosidl_typesupport`. Humble, CycloneDDS 0.10.5, rmw_cyclonedds 1.3.4
  (`phase_0_5_inventory.md` §6).
- Every `@ros2_rclcpp//:rclcpp` user (4): `ic/intrinsic_sdk/intrinsic/platform/pubsub/BUILD:109`,
  `ic/intrinsic/world/BUILD:152`,
  `ic/intrinsic_perception/intrinsic/perception/cameras/BUILD:515`, `:550`.
  All are `cc_library` from `//bazel:cc_macros.bzl` with
  `copts = ["-fexceptions"]` ("ROS can throw exceptions") and
  `features = ["-use_header_modules"]` (`pubsub/BUILD:91-111`;
  `cameras/BUILD:470-471`, `:527-528`). `world_binary` is a plain `cc_binary`
  reaching rclcpp through `//intrinsic/world` (`ic/intrinsic/world/binary/BUILD:15-37`).
- Message labels (from `$OB/com_github_mvukov_rules_ros2++non_module_deps+ros2_common_interfaces/BUILD.bazel`):
  `@ros2_common_interfaces//:cpp_std_msgs` (`:31`), `:cpp_geometry_msgs`
  (`:80`), `:cpp_sensor_msgs` (`:136`); `@ros2_rcl_interfaces//:cpp_builtin_interfaces`
  (`…ros2_rcl_interfaces/BUILD.bazel:102`); tf2: `@ros2_geometry2//:cpp_tf2_msgs`,
  `:tf2_ros` (`…ros2_geometry2/BUILD.bazel:31`, `:65`). The camera library
  also uses `@ros2_rmw//:rmw`, `@ros2_rcutils//:rcutils`,
  `@ros2_rosidl//:rosidl_runtime_cpp` (`cameras/BUILD:512-518`).
- `ros2_cpp_binary` is optional: without `set_up_ament`/`idl_deps` it is a
  plain `cc_binary` with `ROS_PACKAGE_NAME` (`$OB/com_github_mvukov_rules_ros2+/ros2/cc_defs.bzl:69-76`);
  its launcher only sets `AMENT_PREFIX_PATH` (and log dirs for tests)
  (`ros2/launch.sh.tpl:5-21`).
- **RMW loading (X3):** `rmw_implementation` is compiled with
  `RMW_LIBRARY_PATH="$(rootpath @ros2_rmw_cyclonedds//:rmw_cyclonedds)"`
  (`$OB/com_github_mvukov_rules_ros2++non_module_deps+ros2_rmw_implementation/BUILD.bazel:19`);
  the patch replaces the `RMW_IMPLEMENTATION` lookup with
  `rcpputils::SharedLibrary(RMW_LIBRARY_PATH)`
  (`$OB/com_github_mvukov_rules_ros2+/repositories/patches/rmw_implementation_library_path.patch`).
  A `rootpath` of an external repo has the form `../<repo>/<file>`, which
  `dlopen` resolves against the working directory, so (inferred) a node
  must start with cwd = `<bin>.runfiles/_main`. The current entrypoint
  starts binaries without `cd` (`project/container/entrypoint.sh:30-33`).
  `CYCLONEDDS_URI` is still honoured (`phase_0_5_inventory.md` §6).
- No Intrinsic code calls `rclcpp::init`/`Node`/`spin` (`rg` count 0);
  `pubsub_ros` only uses `rclcpp::SerializedMessage`
  (`ic/intrinsic_sdk/intrinsic/platform/pubsub/pubsub_ros.cc:31-51`).

### A.11 Building a package that is a symlinked `//ext` directory

- `ic/ext` does not exist (`ls`); `project/container/build_intrinsic.sh:1-17`
  does not create it (the design says it does, `robot_platform.md:548-552`).
- Repo config: `ic/.bazelignore:16-43` has no `ext` entry; `ic/.bazelrc`
  has no `--deleted_packages` or symlink flags; `ic/BUILD:15-23` only has
  `exports_files`. `ext` is not in `ic/.gitignore`, so the symlink would
  show as untracked in intrinsic-core.
- Flags that apply to `//ext` C++: `-c opt` (`ic/.bazelrc:96`),
  `--config=haswell` (`:216-218`), `--dynamic_mode=off` (`:27`),
  `--copt=-Werror` with `-Wno-everything` only for `external/` and
  `third_party/` (`:169-172`, `:191-192`), `-std=c++20` (`:34`),
  `--incompatible_disable_autoloads_in_main_repo` (`:53`, so `py_*`/`cc_*`
  rules must be `load()`ed).
- `--override_module=tinygltf=…` is required for every build
  (`project/container/build_intrinsic.sh:9-11`).
- Bazel follows a symlinked directory in the source tree as a package
  (general knowledge, **not tested here**: creating `ic/ext` would modify
  intrinsic-core).

---

## B. `robot_service` as a Bazel `py_binary`

### B.1 Toolchain

- `PYTHON_VERSION = "3.11"`, `rules_python` 2.3.3,
  `python.toolchain(python_version = PYTHON_VERSION)` (`ic/MODULE.bazel:228-239`).
- `pip.parse(hub_name = "ai_intrinsic_sdks_pip_deps", requirements_lock = "//:requirements.txt")`
  (`:246-255`); pins `grpcio==1.74.0` (`ic/requirements.txt:2007`),
  `googleapis-common-protos[grpc]==1.73.1` (`:1920`).
- The hermetic interpreter (fetched) is 3.11.15; `ldd` shows only glibc
  libraries (subagent run). The image's system Python is 3.10.12 (X1).

### B.2 Rules and how imports work

- BUILD files load `py_binary`/`py_library` from `//bazel:python.bzl`
  (wraps `@rules_python//python:defs.bzl`, `ic/bazel/python.bzl:23`). The
  wrapper adds `imports` roots only for packages under
  `GOOGLE3_OR_IOC_LOCATIONS` (`python.bzl:26-49`; list
  `ic/bazel/migration_locations.bzl:22-34`); `ext` is not in it. Whether
  `intrinsic.*` imports resolve in `//ext` through the deps' own `imports`
  was not tested.
- Example Python gRPC server `py_binary`:
  `//intrinsic/assets/services/examples/calcserver:calc_server_main_py`
  (`ic/intrinsic/assets/services/examples/calcserver/BUILD:168-182`), using
  `grpc.server(futures.ThreadPoolExecutor(...))` (`calc_server_main.py:51-54`).

### B.3 Labels

| What | Label | Source |
|---|---|---|
| ICON client (`icon_api`, `_session`, `actions`, `reactions`, `errors`) | `//intrinsic_control/intrinsic/icon/python:icon` | `ICON/python/BUILD:19-48`, package public `:17` |
| ICON action utils (public re-export) | `//intrinsic_control/intrinsic/icon/python:create_action_utils` | `ICON/python/BUILD:70-83` |
| ICON service stubs | `@intrinsic_apis//intrinsic/icon/proto/v1:service_py_pb2_grpc`, `:service_py_pb2` | `APIS_ICON/proto/v1/BUILD:182-191` |
| Action param protos | `@intrinsic_apis//intrinsic/icon/actions:<name>_py_pb2` (`point_to_point_move`, `trajectory_tracking_action`, `simple_gripper`, `tare_force_torque_sensor`); `//intrinsic_control/intrinsic/icon/actions:<name>_py_pb2` (`force_primitive`, `cartesian_admittance_action`, `joint_jogging`) | `APIS_ICON/actions/BUILD:104-157`; `ICON/actions/BUILD:156-157`, `:264-265`, `:511-512` |
| World client | `//intrinsic_sdk/intrinsic/world/python:object_world_client` | `ic/intrinsic_sdk/intrinsic/world/python/BUILD:45-73` |
| ObjectWorldService | `//intrinsic/world/proto:object_world_service_py_pb2_grpc` | `ic/intrinsic/world/proto/BUILD:68-79` |
| MotionPlannerService | `//intrinsic_motion_planning/intrinsic/motion_planning/proto/v1:motion_planner_service_py_pb2_grpc` | its `BUILD:97-106` |
| GeometryService | `//intrinsic/geometry/proto:geometry_service_py_pb2_grpc` | `ic/intrinsic/geometry/proto/BUILD:54-63` |
| CameraService | `@intrinsic_apis//intrinsic/perception/proto/v1:camera_service_py_pb2_grpc` | `ic/intrinsic_apis/intrinsic/perception/proto/v1/BUILD:657-666` |
| `google.longrunning` messages | `@com_google_googleapis//google/longrunning:operations_py_proto` | used at `ic/intrinsic_sdk/intrinsic/assets/install/BUILD:33`; defined `$OB/googleapis+/google/longrunning/BUILD.bazel:222-225` |
| `google.longrunning` gRPC stub | `:operations_py_grpc` exists (`BUILD.bazel:227-231`) but is a fail macro under Bzlmod (`$OB/…com_google_googleapis_imports/imports.bzl:34`); Intrinsic code imports `google.longrunning.operations_pb2_grpc` (`installed_assets_client.py:23`) with only `operations_py_proto` + `@ai_intrinsic_sdks_pip_deps//grpcio_status` as deps (provider of the module not verified) | |
| grpcio | `@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio` | e.g. `ICON/python/BUILD:35` |

- `@intrinsic_apis//` is a separate local module (`ic/MODULE.bazel:1012-1016`)
  whose path is bazelignored in the main repo (`ic/.bazelignore:19`).
- Outside Bazel the project already has `gen/google/longrunning/operations_pb2.py`
  and `operations_pb2_grpc.py` (`ls project/gen/google/longrunning`).
- Python ICON client outside Bazel: `phase_1_inventory.md` §4.4–4.5.

### B.4 Packaging a py_binary + runfiles into the container

- Intrinsic packages Python with `python_oci_image` / `python_layers`
  (`ic/bazel/python_oci_image.bzl:56-313`), which rewrites the venv
  interpreter entry to a symlink (`:112-118`); no `--build_python_zip` or
  zipapp use was found (`rg`).
- Layout (inspected on the exec-config `rosidl_adapter_app` stub; no
  target-config py_binary is built in `ic/bazel-bin`): shebang
  `#!/usr/bin/env python3` (system Python runs the stub);
  `PYTHON_BINARY = '<pkg>/_<bin>.venv/bin/python3'`,
  `PYTHON_BINARY_ACTUAL = "rules_python++python+python_3_11_x86_64-unknown-linux-gnu/bin/python3"`;
  runfiles found relative to the stub; the venv `python3` is a relative
  symlink into the interpreter repo.
- X2: after a dereferencing copy the venv `python3` is a 54 MB regular file
  and start-up fails with "Could not find platform independent libraries";
  after `ln -sf` of that one symlink the binary runs. The copied `MANIFEST`
  lists host `/home/mew/.cache/...` paths (inferred to break `Rlocation`,
  not imports).
- The image sets `PYTHONPATH=/opt/app:/opt/app/gen`
  (`project/container/Dockerfile:28`), which the hermetic interpreter
  inherits (X2 verbose output); `gen/` holds `google/`, `intrinsic/`
  packages that can shadow the Bazel-built ones.
- Current scripts: `build_intrinsic.sh` builds a fixed target list
  (`:10-17`); `stage_intrinsic.sh` has a fixed `BINARIES` list, copies with
  `cp -L` / `cp -rL` and checks C++ libs with `ldd` (`:9-28` of that
  script; full text read); `Dockerfile` copies `stage/bin` and `stage/lib`
  (`:16-17`); `entrypoint.sh` starts each process explicitly (`:18-33`).

---

## C. ICON actions and parts

Aliases: `CA/` = `ICON/actions/`, `AA/` = `APIS_ICON/actions/`, `CCA/` =
`ICON/control/actions/`. All actions below are in the default set
`rtcl_controller_default_actions` (`CCA/BUILD:23-48`), linked into ICON
through `realtime_control_manager` (`ICON/control/BUILD:961-1002`). A single
`part_name` is accepted only for single-slot actions; multi-slot actions
need a slot map (`ICON/server/service.cc:170-197`;
`ICON/python/actions.py:70-85`).

| Action | Params (package) | Python creator | Slots → required feature interfaces |
|---|---|---|---|
| `intrinsic.trajectory_tracking` (`CA/trajectory_tracking_action_info.h:32`) | `TrajectoryTrackingActionFixedParams{trajectory=1: JointTrajectoryPVA}` (`intrinsic_proto.icon.actions`, `AA/trajectory_tracking_action.proto:17-30`) | `create_trajectory_tracking_action(action_id, part, trajectory)`; start within 0.1 rad / 0.15 rad/s (`CA/trajectory_tracking_action_utils.py:26-37`) | `arm`: JOINT_POSITION, JOINT_VELOCITY_ESTIMATOR, JOINT_LIMITS (`CA/trajectory_tracking_action_signature.cc:29-39`) |
| `intrinsic.point_to_point_move` (`CA/point_to_point_move_info.h:30`) | `PointToPointMoveFixedParams{goal_position=1, goal_velocity=2, optional joint_limits=3}` (`intrinsic_proto.icon.actions.proto`, `AA/point_to_point_move.proto:17-39`) | `create_point_to_point_move_action(action_id, joint_position_part_name, goal_position, goal_velocity=None, joint_limits=None)` (`CA/point_to_point_move_utils.py:28-34`) | `arm`: JOINT_POSITION, JOINT_POSITION_SENSOR, JOINT_VELOCITY_ESTIMATOR, JOINT_LIMITS, MOVE_OK (`CA/point_to_point_move_signature.cc:34-51`) |
| `intrinsic.stop` (`ICON/common/builtins.h:24`) | none (`RegisterNoParameters`, `CCA/rtcl_joint_stop_action_register.cc:31-32`) | `create_stop_action(action_id, joint_position_part_name)`; state `is_settled` (`CA/stop_utils.py:26-31`) | `position_part`: JOINT_POSITION, JOINT_POSITION_SENSOR, JOINT_LIMITS, JOINT_VELOCITY_ESTIMATOR (`CCA/rtcl_joint_stop_action.cc:148-185`) |
| `intrinsic.simple_gripper` (`CA/simple_gripper_info.h:23`) | `SimpleGripperFixedParams{command=1: UNKNOWN/GRASP/RELEASE}` (`AA/simple_gripper.proto:17-31`) | none; build `actions.Action(id, "intrinsic.simple_gripper", "gripper", params)` (`ICON/python/actions.py:33`) | `gripper`: SIMPLE_GRIPPER (`CA/simple_gripper_signature.cc:47-51`) |
| `intrinsic.tare_force_torque_sensor` (`CA/tare_force_torque_sensor_info.h:24-25`) | `TareForceTorqueSensorParams{optional num_taring_cycles=1}` (`AA/tare_force_torque_sensor.proto:17-24`) | `create_tare_force_torque_sensor_action(action_id, force_torque_sensor_part_name, num_taring_cycles=None)` (`CA/tare_force_torque_sensor_utils.py:25-29`) | `ft_sensor`: FORCE_TORQUE_SENSOR (`CA/tare_force_torque_sensor_signature.cc:33-37`) |
| `intrinsic.force_primitive` (`CA/force_primitive_info.h:25`) | `ForcePrimitiveFixedParams{force_primitive=1, force_control_settings=2, task_params=3, disable_reference_lowpass_filter=4}`, `TaskParams{robot_tip_t_robot_tool=1, robot_base_t_task=4, environment_stiffness=3, joint_limits=5, cartesian_limits=6}` (`CA/force_primitive.proto:17-68`) | `create_make_contact_action(action_id, joint_position_part_name, ft_sensor_part_name, primitive, force_control_settings, task_params, …)`, `build_task_params(...)` (`CA/force_primitive_utils.py:27-83`) | same as admittance (signature copied, `CCA/force_primitive_register.cc:26-34`); impedance variant if `use_impedance_control` (`CCA/force_primitive.cc:34-48`) |
| `intrinsic.cartesian_admittance_action` (`CA/cartesian_admittance_info.h:24-25`; "tare immediately before", `:37-41`) | `CartesianAdmittanceActionFixedParams{cartesian_impedance_parameters=1}`, streaming status `CartesianAdmittanceStatus` (`CA/cartesian_admittance_action.proto:17-72`) | `create_cartesian_admittance_action(action_id, joint_position_part_name, ft_sensor_part_name, *, virtual_cartesian_inertia, cartesian_stiffness, …)`, `create_approach_contact_action`, `create_stabilize_contact_action` (`CA/cartesian_admittance_utils.py:82-101`, `:282`, `:358`) | `arm`: JOINT_POSITION, JOINT_POSITION_SENSOR, JOINT_VELOCITY_ESTIMATOR, CARTESIAN_LIMITS, JOINT_LIMITS, MANIPULATOR_KINEMATICS; `ft_sensor`: FORCE_TORQUE_SENSOR (`CCA/cartesian_admittance_action.cc:734-774`); uses IK (`:688`) |
| `intrinsic.joint_jogging` (`CA/joint_jogging_info.h:24`; watchdog 0.25 s `:55`) | `JointJoggingFixedParams{optional joint_limits=1}`, streaming `JointJoggingStreamingParams{goal_velocity=1}` rad/s (`CA/joint_jogging.proto:17-40`) | none | `arm`: JOINT_POSITION, JOINT_POSITION_SENSOR, JOINT_VELOCITY_ESTIMATOR, JOINT_LIMITS, MOVE_OK (`CA/joint_jogging_signature.cc:31-45`) |

Python creators are reachable from `//ext` only through
`create_action_utils` (§0 item 11). `create_action_utils` re-exports
`create_trajectory_tracking_action`, `create_tare_force_torque_sensor_action`,
`create_point_to_point_move_action`, `create_stop_action`,
`create_cartesian_admittance_action`, `create_approach_contact_action`,
`create_stabilize_contact_action`, `build_task_params`,
`create_make_contact_action`, `create_wait_for_settling_action`
(`ICON/python/create_action_utils.py:41-77`).

### C.1–C.8 Notes per action

- Speed override scales point-to-point and joint jogging velocities
  (`CCA/rtcl_joint_position_action.cc:350-355`; `CCA/joint_jogging_action.cc:242-243`, `:274`).
- `HalArmPart` gets Cartesian limits from the world, else from the safety
  config, else only `LOG(ERROR)` (`ICON/control/parts/hal/arm_part/hal_arm_part.cc:522-567`);
  whether the part then lacks `CARTESIAN_LIMITS` (blocking admittance) was
  not checked.
- Cartesian IK in the current config is allowed through
  `allow_non_real_time_inverse_kinematics: true` (`project/container/config/icon.txtpb:10`;
  `phase_1_inventory.md` §4.1).

### C.9 `JoggingService`

- `ic/intrinsic_apis/intrinsic/icon/proto/v1/jogging_service.proto`:
  `JogRobot(stream JoggingRequest) returns (stream JoggingResponse)` (`:202`),
  `GetAvailableParts` (`:206-211`). First request carries
  `InitialJoggingData{part_name=1, initial_joint_jogging_spec=2 | initial_cartesian_jogging_spec=3, client_name=4}`
  (`:54-81`); then `JoggingCommand` with `JointJoggingCommand{joint_index=1, normalized_velocity=2 ∈ [-1,1]}`
  (`:129-152`).
- Always started inside ICON (`ICON/control/realtime_control_manager.cc:539-596`)
  and exposed on ICON's port through `GrpcEnvelope::JoggingWrapperService`
  (`ICON/server/grpc_envelope.cc:583-609`, `:858`). Without a WorldService a
  dummy returns Unimplemented (`ICON/server/jogging_service.cc:186-216`,
  `:552-556`). It opens a session with `intrinsic.joint_jogging`; velocity =
  normalized × part max velocity, ceiling 30°/s (`:80`, `:158`, `:456-525`);
  needs `hardware_resource_name` and a matching world object (`:615-625`).
- Cartesian: not implemented (`:440-447`, `:571`, `:591`;
  `APIS_ICON/actions/BUILD:47-78` names a `cartesian_jogging.proto` that is
  absent).

### C.10 Parts and the `IconMainConfig` snippet

- `RealtimePartConfig{part_type_name=1, config=3 (Any), safety_action_type_name=4, hardware_resource_name=6}`
  (`APIS_ICON/control/parts/proto/v1/realtime_part_config.proto:31-54`);
  `HardwareInterface{module_name=1, interface_name=2}` (`APIS_ICON/control/parts/hal/v1/hal_part_config.proto:22-27`).
- `HalLinearGripperPartConfig` (`APIS_ICON/control/parts/hal/linear_gripper_part/hal_linear_gripper_part_config.proto:23-54`):
  `status_interface=1`, `command_interface=2`, `optional default_grasp_width=4`,
  `optional default_release_width=5` (both required,
  `hal_linear_gripper_part.cc:76-81`), `min_width=6`, `optional max_width=7`,
  `min_force=8`, `optional max_force=9`, `min_speed=10`, `optional max_speed=11`,
  `default_force=12`, `default_speed=13`; validation
  `min_width ≤ grasp < release ≤ max_width` (`linear_gripper.cc:46-79`).
  Type name `"HalLinearGripperPart"` (`hal_linear_gripper_part.h:39`). No
  example config for it exists in `ic/` (`rg`).
- `HalForceTorqueSensorPartConfig` (`…/force_torque_sensor_part/hal_force_torque_sensor_part_config.proto:25-98`):
  oneof `force_torque_state=1` / `external_joint_torque_state=15`,
  `optional force_torque_command=2`, `world_robot_collection_name=3`,
  `optional joint_position_state=4`, `optional joint_velocity_state=5`, oneof
  `joint_acceleration_state=6` / `linear_joint_acceleration_filter_config=7`,
  `target_link_name=8`, `ft_sensor_link_name=9`, `support_mass=10`,
  `repeated ft_t_cog=11` (must have 3 values, `force_torque_sensor.cc:334-338`),
  `estimate_post_sensor_dynamic_load=12`,
  `optional num_acceptable_constant_readings=13`, `force_control_settings=14`.
  Type name `"HalForceTorqueSensorPart"`; payload properties
  `mounted_payload`, `grasped_payload` (`hal_force_torque_sensor_part.h:73-75`).
  Example: `ic/developer_resources/learn/tutorials/interactive_tutorials/client_api/move_to_contact_tutorial.md:30-71`
  (it says `hardware_config.parts_by_name`, but `parts_by_name` is in
  `realtime_control_config`, `APIS_ICON/server/config/icon_main_config.proto:90`).
- `PartStatus` (`APIS_ICON/proto/part_status.proto:64-107`):
  `gripper_state=5`, `base_t_tip_sensed=6`, `wrench_at_ft=7`,
  `wrench_at_tip=8`, `base_twist_tip_sensed=9`, `linear_gripper_state=10`,
  `wrench_at_ft_uncompensated=15`; `RobotStatus.speed_override=7` (`:174`).
- ICON's auto-config uses part keys `arm` / `ft_sensor` and safety actions
  `intrinsic.stop` / `intrinsic.empty` (`ICON/server/auto_config_helpers.cc:311`, `:552`, `:606-645`).

Snippet, built from the current `project/container/config/icon.txtpb` plus
verified field names. Interface instance names (`gripper_status`, …) are
whatever `ros2_hwm` advertises; link names are not decided (A.5):

```textproto
realtime_control_config {
  parts_by_name { key: "arm" value { ... as in container/config/icon.txtpb:13-28,
    plus in HalArmPartConfig:
      payload_command { module_name: "robot_hwm" interface_name: "payload_command" }   # :49 (enables "full_payload")
  } }
  parts_by_name {
    key: "gripper"
    value {
      part_type_name: "HalLinearGripperPart"
      safety_action_type_name: "intrinsic.empty"
      config {
        [type.googleapis.com/intrinsic_proto.icon.HalLinearGripperPartConfig] {
          status_interface  { module_name: "robot_hwm" interface_name: "gripper_status" }
          command_interface { module_name: "robot_hwm" interface_name: "gripper_command" }
          default_grasp_width: 0.0      # GRASP is "done" only at width <= this
          default_release_width: 0.08
          max_width: 0.08
          default_force: 20.0
        }
      }
    }
  }
  parts_by_name {
    key: "ft_sensor"
    value {
      part_type_name: "HalForceTorqueSensorPart"
      safety_action_type_name: "intrinsic.empty"
      hardware_resource_name: "franka"
      config {
        [type.googleapis.com/intrinsic_proto.icon.HalForceTorqueSensorPartConfig] {
          force_torque_state   { module_name: "robot_hwm" interface_name: "force_torque_status" }
          force_torque_command { module_name: "robot_hwm" interface_name: "force_torque_command" }
          joint_position_state { module_name: "robot_hwm" interface_name: "joint_position_state" }  # attached mode
          world_robot_collection_name: "franka"
          target_link_name: "panda_link8"       # must be an entity of "franka" (A.5)
          ft_sensor_link_name: "panda_hand"     # entity of child object franka_hand (A.5)
          support_mass: 0.0
          ft_t_cog: [0.0, 0.0, 0.0]
          num_acceptable_constant_readings: 4294967295   # §0 item 4
        }
      }
    }
  }
}
```

### C.11 Speed override and payload

- `SetSpeedOverride{override_factor ∈ [0,1]}` / `GetSpeedOverride`
  (`APIS_ICON/proto/v1/service.proto:95-99`, `:739-767`; server
  `ICON/server/service.cc:1199-1208`); Python `set_speed_override(f)`,
  `get_speed_override()` (`ICON/python/icon_api.py:514-538`).
- `SetPayload{part_name=1, payload_name=2, payload=3: world.RobotPayload}`
  (`service.proto:456-463`, `:808-821`): fails with an open session
  (`service.cc:1309-1314`), needs ICON enabled and not faulted, disables
  and re-enables to apply (`:1361-1378`). Names: `full_payload` on the arm
  (only with `payload_command`, `hal_arm_part.cc:1024-1044`),
  `mounted_payload` / `grasped_payload` on the F/T part. Python
  `set_payload(payload, part_name, payload_name)` / `get_payload`
  (`icon_api.py:613-652`).

---

## D. `scene_sync`

### D.1 ObjectWorldService

Protos: `ic/intrinsic/world/proto/object_world_service.proto` ("svc") and
`ic/intrinsic/world/proto/object_world_updates.proto` ("upd"); references
in `ic/intrinsic_apis/intrinsic/world/proto/object_world_refs.proto`.

| RPC (svc line) | Request fields |
|---|---|
| `CreateObject` (224) | `world_id=1`, `name=2`, `parent_object=3` (`ObjectReferenceWithEntityFilter`, must match one entity), `parent_object_t_created_object=4`, `name_is_global_alias=9`, `action_for_object_entities=11`, oneof `create_single_entity_object=5{entity_local_name=4, geometry_component=3}` / `clone_from_existing_object=10` / `create_from_scene_object=12`, `user_data=13` (map<string, Any>), `view=7` (upd :159-248); duplicate name = error (:203-204) |
| `DeleteObject` (236) | `world_id=1`, `object=2`, `force=3`, `reparent_child_objects=4` (upd :251-278) |
| `UpdateTransform` (451) | `world_id=1`, `node_a=2`, `node_a_filter=7`, `node_b=3`, `node_b_filter=8`, `node_to_update=4`, `a_t_b=5`, `view=6`; without `node_to_update`, a/b must be parent/child (upd :674-723) |
| `UpdateWorldResources` (468) | atomic batch, e.g. many `update_transform`s with empty `world_id`; response holds **all objects** (`ic/intrinsic/world/service/object_world_service.cc:2441-2475`) |
| `UpdateObject` (254, experimental) | `update_from_scene_object{…, allow_missing=3}` creates a missing object; IDs not kept (upd :288-323) |
| `ReparentObject` (275) | keeps the global pose (upd :348-376) |
| `UpdateObjectProperties` (314) | `user_data=10` with INSERT/…/CLEAR_ALL policies (upd :493-521; `ic/intrinsic_apis/intrinsic/world/proto/user_data.proto:24-50`) |
| `GetObject` (202), `ListObjects` (213) | `view` BASIC=1 / FULL=2 (upd :92-104); `user_data` only in FULL (svc :776-778) |
| `SyncObject` (553) | `from_world_id=1`, `to_world_id=2`, oneof `object=3` / `frame=4` (svc :1412-1427) |
| `CreateWorld` (138), `CloneWorld` (88), `DeleteWorld` (112), `ListWorlds` (127) | `CloneWorld{world_id, cloned_world_id=5, allow_overwrite=6, …}` (svc :967-990) |

- `SyncObject` semantics: copies the whole object from one world to another
  keeping IDs and parent; overwrites the target, deletes it if absent in the
  source; fails if the parent is missing in the target or the ID is taken
  (svc :543-552). Server: source ≠ target (`object_world_service.cc:3444-3447`),
  missing target world is created (`:3460-3466`), the target world is cloned
  for the update (`:3492`, `:3518`). Matching is by object ID
  (`ic/intrinsic/world/objects/object_world.cc:597-623`).
- Clone targets are restricted to `world`, `sim_world`,
  `pre-execution-snapshot`, `init_world`, `save_monitor`
  (`ic/intrinsic/world/service/world_storage.cc:272-279`).
- References: `ObjectReference` oneof `id=1` / `by_name=2`; "strongly prefer
  by_name" (`object_world_refs.proto:65-78`); by-name use needs
  `name_is_global_alias` (svc :738-748). Names: letters, digits, `_`,
  starting with a letter.
- No `ground_truth`/`perceived` field exists (`rg`); `user_data` is the only
  per-object tag (upd :238-241; server stores it,
  `ic/intrinsic/world/service/objects/object_world_updates_utils.cc:476-482`).
- Every change marks the world for saving; the flusher runs every 100 ms
  and disk saving is on by default (`world_storage.cc:60`, `:252`, `:445-450`).
- The world service's `WorldUpdater` (30 ms, `ic/intrinsic/world/service/updater/world_updater.h:87`)
  subscribes on Intrinsic pubsub (Zenoh) to `/tf` (written into `world`) and
  `/worlds/<sim_world>/world_updates` (`PubsubWorldUpdates`, into
  `sim_world`) (`world_updater.cc:241-242`, `:549-590`, `:939-988`;
  `ic/intrinsic/world/proto/v1/pubsub_world_updates.proto:25-41`). These are
  Zenoh topics, not ROS 2.

### D.2 Geometry from a GLB

- `CreateGeometry(CreateGeometryRequest{data=1: GeometryData}) → GeometryWithMetadata`
  (`ic/intrinsic/geometry/proto/geometry_service.proto:62-71`, `:134-137`);
  `GeometryData` oneof includes `gltf_bytes=3` and `stl_bytes=5`
  (§0 item 13). The server loads it with assimp (`LoadMeshBufferToGeometry(..., "glb")`,
  `ic/intrinsic/geometry/service/util/geometry_data.cc:60-61`;
  `ic/intrinsic/geometry/api/file_io.cc:245-284`).
- Response `geometry_storage_refs=4` (v1 `GeometryStorageRefs{exact_geometry_ref, renderable_ref, keep_renderable}`,
  `ic/intrinsic_apis/intrinsic/geometry/proto/v1/geometry_storage_refs.proto:27-39`),
  stored in CAS and cached by fingerprint
  (`ic/intrinsic/geometry/storage/cas_storage.cc:101-134`, `:264-283`).
- Referencing it: `GeometryComponent.named_geometries["Intrinsic_Visual"|"Intrinsic_Collision"]`
  → `TransformedGeometry{geometry{geo_ref}, ref_t_shape}`
  (`ic/intrinsic_apis/intrinsic/world/proto/geometry_component.proto:67-95`;
  `…/geometry/proto/v1/transformed_geometry.proto:23-30`). The project does
  exactly this (`project/robot_model/franka_scene_object.py:239-273`,
  `:420-431`).
- Known pitfall: a GLB with degenerate faces failed with
  `INTERNAL: Unsupported number of vertices in a face`; removing zero-area
  faces fixed it (`phase_1_notes.md:124-136`).
- SDK helpers exist (`create_object`, `update_transform`, `register_geometry_v1`,
  `object_world_client.py:540-601`, `:1328-1439`) but the module imports much
  of the SDK (`:21-51`); the project calls stubs directly
  (`phase_1_inventory.md` §2.2).

### D.3 Pose updates at ~30 Hz

- One object: `UpdateTransform(node_a = root, node_b = by_name object, a_t_b, view = BASIC)`;
  takes the world writer lock (`object_world_service.cc:2388-2419`).
- Many objects: `UpdateWorldResources` (atomic; transform-only batches skip
  the structure hash, `object_world_updates_utils.cc:1229-1267`), but its
  response contains all objects. No `UpdateTransforms` RPC exists.

### D.4 `/tf` in Python in the image (X1)

```text
ROS_DISTRO=humble; /usr/bin/python3 = Python 3.10.12
OK rclpy, tf2_ros (/opt/ros/humble/lib/python3.10/site-packages/tf2_ros), tf2_msgs.msg, geometry_msgs.msg
ros-humble-rclpy 3.3.22 | ros-humble-tf2-ros-py 0.25.24 | ros-humble-tf2-msgs 0.25.24
ros-humble-rmw-fastrtps-cpp 6.2.10 | ros-humble-rmw-cyclonedds-cpp 1.3.5
grpc 1.85.0 protobuf 7.36.2
tf2_ros.Buffer, tf2_ros.TransformListener present
```

Phase 2 found that Fast DDS (data-sharing AUTO) late joiners do not receive
the transient-local `/sim/epoch` from Isaac; turning data-sharing off fixed
it, but `container/dds/fastdds.xml` was not changed
(`phase_2_notes.md:95-116`). `scene_sync` and `camera_sim` are Fast DDS
nodes (`robot_platform.md:296-301`).

---

## E. `camera_sim`

### E.1 `camera_service.proto` (v1)

Files in `ic/intrinsic_apis/intrinsic/perception/proto/v1/`.

| RPC | Request → Response | Lines |
|---|---|---|
| `ListAvailableCameras` | `{}` → `{camera_identifiers=1}` | `camera_service.proto:38-43`, `160-166` |
| `DescribeCamera` | `{camera_identifier=1}` → `{sensors=1: SensorInformation}` | `46-57`, `266-273` |
| `Capture` | `CaptureRequest` → `CaptureResponse` | `60-77`, `168-206` |
| `ReadCameraSettingAccess` / `ReadCameraSettingProperties` / `ReadCameraSetting` / `UpdateCameraSetting` | `{camera_identifier, name}` / `{camera_identifier, setting}` | `81-152`, `208-264` |

- `CaptureRequest{camera_config=1, timeout=2 (default 500 ms), sensor_ids=3 (empty = all), post_processing_by_sensor_id=4, optional capture_result_location=5 (KV store), optional context=6}`
  (`:168-199`); `CaptureResponse` oneof `capture_result=1` /
  `capture_result_location=2` (`:201-206`).
- `SensorInformation{id=1, display_name=2, factory_camera_params=3, camera_t_sensor=4, supported_pixel_types=5, dimensions=6, disabled=7}` (`:275-292`).
- `CaptureResult{capture_at=1, sensor_images=2, capture_duration=3}` (`capture_result.proto:27-37`)
  → `SensorImage{sensor_config=1, acquisition_time=2, buffer=3}` (`sensor_image.proto:26-33`)
  → `ImageBuffer{encoding=1, pixel_type=2, num_channels=3, type=4, dimensions=5 (required if raw), data=6, packing_type=7}` (`image_buffer.proto:70-92`).
- Enums (`image_buffer.proto`): `Encoding` UNSPECIFIED (raw)=0, JPEG, PNG, WEBP,
  YUV420P (`:25-31`); `DataType` UINT8=1 … FLOAT32=7, FLOAT64=8 (`:34-44`);
  `PixelType` INTENSITY=1, DEPTH=2, POINT=3, NORMAL=4 (`:49-55`);
  `PackingType` INTERLEAVED=1, PLANAR=2 (`:59-66`).
- RGB8 = raw, INTENSITY, 3 channels, UINT8, INTERLEAVED; depth = DEPTH, 1
  channel, FLOAT32, metres (`ic/intrinsic_perception/intrinsic/perception/core/image_traits.h:97-98`).
  No depth unit/scale field exists. C++ `FromProto(SensorImage)` accepts only
  8U or 32F and rejects unset packing
  (`…/proto_conversion/v1/sensor_image.cc:43-45`, `:61-123`).
- Intrinsics: `CameraConfig{identifier=1, camera_settings=2, sensor_configs=3}`
  (`camera_config.proto:28-42`) → `SensorConfig{id=1, camera_t_sensor=2, camera_params=3}`
  (`sensor_config.proto:27-36`) → `CameraParams{intrinsic_params=1, distortion_params=2}`
  (`camera_params.proto:26-35`) → `IntrinsicParams{focal_length_x=1, focal_length_y=2, principal_point_x=3, principal_point_y=4, dimensions=5}`
  (`intrinsic_params.proto:24-43`); `DistortionParams{k1, k2, p1, p2, …}`
  (`distortion_params.proto:22-64`).
- `CameraIdentifier` oneof `file_camera=1`, `genicam=3`, `ros=4{device_id=1, optional driver_type=2}`,
  `fake_genicam=5` (`camera_identifier.proto:27-37`; `camera_drivers.proto:34-40`).

### E.2 What consumers call

- **Calibration service** (`ic/intrinsic_perception/intrinsic/perception/calibration/services/v1/calibration_service_impl.cc`):
  `Initialize` reads each camera resource handle's `CameraConfig` and
  connection, calls `DescribeCamera`, requires a sensor with
  `PIXEL_INTENSITY` (else `FAILED_PRECONDITION`), uses its `dimensions`
  (`:389`, `:447-489`). `CaptureData` gets the `CameraConfig` from the
  AssetInstances service, then `Capture{camera_config, timeout 5 s, post_processing_by_sensor_id}`,
  reads `capture_result` inline, takes the first Rgb8u/Gray8u/Gray32f image
  and stores its `camera_params` (`:569`, `:650-680`; missing params fail
  later, `:976-980`).
- **Pose estimator**: `PoseEstimationService.RunPoseEstimation` takes a
  `CaptureResult` or KV locations and never calls CameraService
  (`…/pose_estimation_service.proto:31-59`). The IOC estimator needs one
  INTENSITY and one float32 DEPTH image; intrinsics from the intensity
  image's `sensor_config.camera_params`; depth in metres
  (`ic/intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/service/ioc_pose_estimator_service.py:200-268`;
  `pose_estimator_model.py:246`). The multi-view skill looks for sensor
  `"P0"` (ros) / `"color"` (genicam), falling back to `"RGB0"`
  (`…/service/python/pose_estimation_client_utils.py:160-169`;
  `…/service/multi_view/camera_capture_utils.py:56-70`).
- **Python SDK client** (`ic/intrinsic_perception/intrinsic/perception/client/v1/python/camera/`):
  constructor always calls `DescribeCamera` (`cameras.py:358-376`);
  `capture()` sends `CaptureRequest{camera_config, timeout, sensor_ids}` and
  reads `capture_result` (`camera_client.py:113-205`);
  `serialize_image_buffer` / `deserialize_image_buffer` in
  `client/v1/python/image_utils.py:192-304`.
- **`capture_images` skill** always sets `capture_result_location` and
  requires it back (`…/skills/capture_images.cc:138-155`, `:237`).

### E.3 Gazebo reference implementation

`ic/intrinsic/simulation/gazebo/plugins/cameras/`:

- `GazeboCameraGrpcService` on `$CAMERA_SERVICE_ADDRESS`, default
  `0.0.0.0:15333` (`camera_services.cc:56-75`); cameras keyed by
  `CanonicalString(identifier)` (`camera_service.cc:435-437`;
  `ic/intrinsic_perception/intrinsic/perception/cameras/camera_identifier.cc:28-55`).
- `DescribeCamera` returns a pre-built description: one sensor `"color"` id 0,
  or RGBD intensity id 1 + depth id 4; every sensor gets
  `factory_camera_params`; intrinsics from SDF, no distortion
  (`camera_service.cc:598-612`; `camera_plugin.cc:184-208`, `:350-382`;
  `camera_service.h:64-67`; `simulation_camera_utils.cc:39-51`).
- `Capture` (`camera_service.cc:426-487`): requires an identifier; verifies
  requested intrinsics against the sim's (`VerifyIntrinsicParams`); waits for
  frames newer than the request with an internal 120 s timeout (request
  timeout ignored, `camera_service.h:116`); `capture_at` = first image's
  acquisition time; post-processing; `EncodeAndStoreCaptureResult` returns
  inline or writes the KV store (`…/cameras/services/v1/camera_service_utils.cc:32-67`).
  Images: RGB_INT8 → Rgb8u, R_FLOAT32 → Depth32f, each with `camera_params`;
  timestamps `absl::Now()` at arrival (`camera_plugin.cc:461`, `:486-535`).
- ROS image source (real hardware path): `CameraInfo` → fx=k[0], fy=k[4],
  cx=k[2], cy=k[5]; `rgb8` → Rgb8u, `32FC1` → Depth32f
  (`ic/intrinsic_perception/intrinsic/perception/cameras/ros_image_source.cc:255-304`, `:361-394`).
- In k8s sim, `cameraserviceproxy.go` forwards CameraService to
  `camera-sim.<ns>.svc.cluster.local:8080`
  (`ic/intrinsic_perception/intrinsic/perception/cameras/services/cameraserviceproxy.go:15-19`, `:133-150`).

### E.4 Camera config textprotos

Only templates exist: `identifier { ros { device_id, driver_type } }`
(`…/cameras/services/ros_camera_default_config.pbtxt.tpl:17-19`), GenICam
(`genicam_gige_vision_camera_default_config.pbtxt.tpl:17-19`), Orbbec
(`hardware_devices/orbbec_gemini_335le_data_manifest.textproto:32-36`). No
textproto in `ic/` sets `sensor_configs` for a sim camera (`rg`).

Isaac side: `/camera/rgb` `rgb8` 848×480, `/camera/depth` `32FC1` metres,
`frame_id` `d455_optical` (`phase_2.md:25`); measured
K = [[420.0821, 0, 424], [0, 420.0822, 240], [0, 0, 1]]; the bridge's
CameraInfo forces fy = fx (`phase_2_notes.md:136-139`).

---

## F. Facts that conflict with or refine the design text

| Design text | Fact |
|---|---|
| `Jog(mode=CARTESIAN)` via `JoggingService` (`robot_platform.md:440`) | Cartesian jogging is not implemented here (C.9) |
| `SetGripper(width, force)` via `intrinsic.simple_gripper` (`:441`) | the action takes only GRASP/RELEASE with configured defaults; no action uses the width/force feature (§0 item 6) |
| `Grasp` ends when fingers stop | GRASP is done only at `width ≤ default_grasp_width` (§0 item 6) |
| `GetState` wrench from ICON `GetStatus` (`:425`) | `wrench_at_ft` is on the `ft_sensor` part's status (§0 item 8) |
| `/wrist_wrench` → F/T interface (`:433`) | sign: ICON negates the raw reading (A.5); frame: Intrinsic `panda_hand` entity ≠ Isaac `panda_hand` frame (A.5); constant readings fault (§0 item 4) |
| `build_intrinsic.sh` creates the `ext` symlink (`:550-551`) | it does not (A.11) |
| `robot_service` as Bazel `py_binary` in the container | `cp -rL` staging breaks it (B.4); image `PYTHONPATH` leaks into it |
| `ros2_hwm` on Bazel's ROS 2 | RMW loaded relative to cwd (A.10) |
| "After 20 s without a tick ICON faults" (`:371-379`) | confirmed in code (A.7); recovery = `ClearFaults` → rebuild |

---

## G. Not verified

- Nothing was compiled or run under Bazel. In particular: an `rclcpp` node
  inside a `hardware_module_binary` (link, `-Werror` on ROS headers, the
  cwd dependence of the RMW load); a symlinked `//ext` package; `//ext`
  Python imports of `intrinsic.*`; a target-config `py_binary` layout.
- Whether the ICON process stays up after the 20 s cycle timeout and recovers
  with `ClearFaults` (code path only).
- Whether Isaac's resting `/wrist_wrench` is bit-constant (would trip the
  constant-reading check).
- Whether the rclcpp executor threads in a HWM process are malloc-guarded
  (only the runtime's RT threads are known to be,
  `ICON/hal/hardware_module_main_util.cc:214-216`).
- The `RealtimeClock` constructor `CHECK` (1 min for ICON to join) with the
  project's start order; loopback with the same order works (phase 0/1).
- `HalArmPart`'s full feature list per config (whether it advertises
  `CARTESIAN_LIMITS` without world Cartesian limits); slot requirements of
  the impedance variant of `force_primitive`; `ForceControlSettings` fields.
- `SetPayload` with the F/T part's `mounted_payload`/`grasped_payload`.
- Where `google.longrunning.operations_pb2_grpc` comes from in a Bazel
  py_binary (pip wheel not fetched).
- World: `CreateWorld` with an empty `World`; real latency of
  `UpdateTransform`/`UpdateWorldResources` at 30 Hz with the 100 ms disk
  flush; `SyncObject` with same-name/different-ID objects; whether the
  catalog GLBs contain degenerate faces.
- Camera: how the platform fills `ResourceHandle.resource_data["CameraConfig"]`;
  what the calibration service's post-processing does; the Python client's
  integer-second deadline for sub-second timeouts.
- Line numbers not listed in §0 were taken from one read pass and not all
  re-read.
