# Robot platform: Intrinsic Core + Isaac Sim + MCP

*   **Author:** mew (lead: an MCP client; review: a coding agent, a frontier hosted model)
*   **Status:** Approved by owner (plan v7, 2026-10-27); in implementation
*   **Date:** 2026-10-27

--------------------------------------------------------------------------------

## Overview

A simulated Franka Emika Panda arm that a vision-language model (VLM) can
operate through an MCP server. Every robot function (motion planning, inverse
kinematics, control, gripper, camera, calibration, perception, force control,
grasp planning) is done by Intrinsic Core services. The physics run in NVIDIA
Isaac Sim 5.1. This is the base layer of a larger product, so it is built to
the finished-product standard: no needed feature is deferred, and the
simulation is made realistic (real sensor model, sensor noise) where that
helps later work.

## The Problem

Isaac Sim ships a Franka and a scripted pick-and-place demo, but no robot
software stack: its demo uses a hand-written state machine and differential
IK. A VLM needs a stable, high-level robot API (plan, move, grasp, see) whose
results are trustworthy. Intrinsic Core provides that stack, but it normally
runs on a k3s cluster with Gazebo or real hardware, a real-time kernel, and
an executive that runs behavior trees. None of that fits here.

The solution: run the Intrinsic services as plain processes in one Docker
container, replace the platform services they expect with small
stand-ins, connect them to Isaac Sim over ROS 2, and put one thin gRPC API
(`RobotService`) and then an MCP server on top.

### Glossary

*   **VLM**: vision-language model (here the frontier model). **MCP**: Model Context
    Protocol, the standard way an AI model calls external tools.
*   **ICON**: Intrinsic's robot control service. It runs a fixed-rate
    control loop and executes *actions* (e.g. follow a trajectory, stop,
    move until contact) on *parts* (a controllable piece of hardware: the
    arm, the gripper, the force-torque sensor). A client opens a *session*
    on parts and starts actions in it. `--non_realtime` runs it without a
    real-time kernel.
*   **Hardware module (HWM)**: the process that connects ICON to a robot.
    ICON and the HWM exchange commands and state through shared memory
    every control cycle. `loopback` is Intrinsic's example HWM that echoes
    commands back as state (a robot with no physics).
*   **World / `world` / `sim_world`**: Intrinsic's world service holds a
    scene (objects, frames, kinematics). `world` is the belief scene the
    planner uses; `sim_world` holds the true poses from Isaac.
*   **CAS / `localcas`**: content-addressed storage for large files (meshes);
    `localcas` is Intrinsic's local implementation.
*   **Zenoh / `zenohd`**: the publish-subscribe system Intrinsic services
    use among themselves; `zenohd` is its router.
*   **`runtime_config.pb`**: a per-process configuration file that k3s
    normally writes for each Intrinsic service; here we encode it ourselves.
*   **ResourceRegistry / InstalledAssetsReader**: platform services that
    list the installed hardware and assets; served by our
    `platform_stubs.py`.
*   **design** (lead–review model): the development process for this
    project. The *lead* (an MCP client, full context) writes designs;
    *review* (fresh a coding agent sessions with no history) review them
    and implement them.

## Goals

*   **G1.** One Docker container in WSL runs the Intrinsic layer: world,
    geometry, motion planner, ICON (non-real-time, with arm, gripper and
    force-torque parts), calibration service, pose estimator with Triton,
    grasp planner, the platform stand-ins, and our adapters (including
    `camera_sim`, which serves Intrinsic's `CameraService` API).
*   **G2.** All traffic between the container and Isaac Sim is ROS 2. No
    custom TCP/UDP or gRPC link to Isaac.
*   **G3.** The same Franka model (Isaac's
    `exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/franka/lula_franka_gen.urdf`)
    is used by
    Isaac and by Intrinsic. Forward kinematics of both agree within 1 mm on
    20 random joint configurations.
*   **G4.** `RobotService` (gRPC) exposes every call in the table under
    "RobotService API", and each call is implemented only by calling
    Intrinsic APIs.
*   **G5.** The camera is Isaac's Intel RealSense D455 model
    (`Isaac/Sensors/Intel/RealSense/rsd455.usd`) with RGB and depth. Depth
    comes from Isaac's stereo-disparity depth sensor
    (`isaacsim.sensors.camera.SingleViewDepthSensor`), which produces
    realistic D455 noise and holes; noise-free depth is published separately
    as ground truth.
*   **G6.** Camera-to-robot calibration through Intrinsic's
    `CalibrationService` reaches < 2 mm and < 0.2° against Isaac's true pose.
*   **G7.** `DetectObjects` through Intrinsic's pose estimator matches ground
    truth (thresholds set in the phase 5 plan). If the published models fall
    short on Isaac images, they are retrained on Isaac-rendered data.
*   **G8.** Force/contact: `MoveToContact` stops on contact within a set
    force; a guarded peg-in-hole insertion works (CUJ 5).
*   **G9.** Grasp planning: `PlanGrasp(object)` returns grasps that succeed
    in Isaac. The grasp planner's ROS 2 service is called directly by
    `robot_service`; Intrinsic's grasp *skill* is not used.
*   **G10.** An MCP server exposes every `RobotService` call as a tool. The
    tools are generated from the `RobotService` proto (names, descriptions
    and argument schemas come from the proto and its comments), so the API
    is defined once. the frontier model picks and places a cube from a chat prompt,
    sequencing the calls itself.

## Non-Goals

*   k3s, Kubernetes, Helm, or the Intrinsic cluster gateway.
*   A real-time kernel or real-time guarantees. ICON runs `--non_realtime`.
*   The Intrinsic executive, skills, behavior trees, or Solution Building
    Library. The VLM does all sequencing.
*   Real hardware. (The design must not block it later; see Future
    Improvements.)
*   Gazebo.

## Open Questions

*   **Q1 (phase 0.5).** Does Intrinsic's Bazel ROS 2 (Humble, CycloneDDS)
    exchange messages with Isaac's ROS 2 bridge (Humble, Fast DDS on Windows)
    across Windows → WSL (mirrored networking) → Docker (host network)?
    Fallbacks, in order: force UDP transport (disable Fast DDS shared
    memory); explicit unicast peer lists; build `ros2_hwm` against the
    system's Humble with Fast DDS instead of Bazel's ROS 2 (this last
    fallback reopens D1 and goes back to the owner).
*   **Q2 (answered).** The D455 asset is `Isaac/Sensors/Intel/RealSense/rsd455.usd`,
    loaded with `SingleViewDepthSensorAsset` (Isaac's own test does this).
*   **Q3 (phase 4).** Which platform services `CalibrationService` needs at
    startup; each gets a stand-in in `platform_stubs`.
*   **Q4 (phase 5).** Whether RF-DETR and FoundationPose weights used by
    `ioc_pose_estimator` are public. If not, train with Intrinsic's
    `TrainService` / the published training recipes on Isaac-rendered data.
*   **Q5 (phase 5c).** The grasp planner: Intrinsic's MoveIt-based grasp
    planning (`developer_resources/.../moveit_grasp_planning.md`) lives in
    a separate repository, not in `~/intrinsic-core`. Phase 5c finds it,
    checks that it builds on ROS 2 Humble and runs as a ROS 2 service
    without Intrinsic's skill and ROS bridge, and fixes the service
    interface `robot_service` calls.
*   **Q6 (phase 0).** zenohd (from Intrinsic's image) and the zenoh-c
    version linked by the services must be compatible.
*   **Q7 (phase 2).** Does `SingleViewDepthSensor` work with the Direct3D
    12 renderer (`--/app/vulkan=false`)? If not: find another fix for the
    Vulkan crash (driver version, render settings).

### Owner decisions

*   **D1 (2026-10-27): `ros2_hwm` is built in Intrinsic's Bazel with Bazel's
    ROS 2 (Humble, CycloneDDS).** It gets every interface (arm, gripper, F/T,
    payload, clock). Phase 0.5 must prove CycloneDDS ↔ Isaac's Fast DDS
    interop; Q1 lists the fallbacks. Rejected: `icon_hwm_controller`
    (upstream exports only position/velocity, targets ROS 2 Kilted) and a
    plain system-ROS node (separate build outside Intrinsic).

## Critical User Journeys (CUJs)

### CUJ 1: VLM picks and places a cube

*   **Given**: Isaac Sim is running the scene (Franka, table, cube, D455
    camera, calibration done); the container is up; the MCP server is
    registered with the frontier model.
*   **When**: The user asks the frontier model: "put the red cube on the plate".
*   **Then**: the frontier model calls `GetImage`, `DetectObjects`, `PlanGrasp`,
    `MoveToPose`, `Grasp`, `MoveToPose`, `Release` in an order it chooses.
    The cube ends on the plate in Isaac. Every call returns a status the VLM
    can act on (success, or an error that says what failed).

### CUJ 2: Developer moves the arm through the API

*   **Given**: The container and Isaac are running.
*   **When**: A Python client calls `RobotService.MoveToPose(pose,
    motion=LINEAR)`.
*   **Then**: The planner plans a collision-free Cartesian path, ICON
    executes it, the Isaac Franka moves along it, and `GetState` reports the
    final pose within 1 mm of the target.

### CUJ 3: Calibrate the camera

*   **Given**: The D455 is fixed in the scene, looking at the table
    (eye-to-hand); the stack is running.
*   **When**: `CalibrateCamera()` is called.
*   **Then**: `robot_service` runs the calibration flow (see "Calibration
    flow"): board on the hand, sampled poses, `CaptureData` at each,
    `Calibrate`, `Save`. The camera pose in `world` matches Isaac's true
    pose within 2 mm / 0.2°, and the board is gone afterwards.

### CUJ 4: Safety stop

*   **Given**: The arm is executing a long motion.
*   **When**: `Stop()` is called.
*   **Then**: ICON brings the arm to a controlled stop and the motion call
    returns `CANCELLED`. The next motion call is accepted. (Separately: if
    ICON reports a fault, every motion call returns `FAILED_PRECONDITION`
    with the fault message until `ClearFaults()` succeeds.)

### CUJ 5: Guarded peg-in-hole insertion

*   **Given**: A peg is held in the gripper (`Grasp`, then `AttachObject` and
    `SetPayload` so the planner and ICON account for it); a block with a hole
    (2 mm clearance) is on the table.
*   **When**: The VLM moves the peg 2 cm above the hole with `MoveToPose`,
    calls `TareWrench()` (removes the peg's weight from the reading), then
    `MoveToContact(direction=down, max_force=15 N, speed=10 mm/s)`, then
    `MoveCompliant(target=3 cm deeper, stiffness=low, max_force=20 N)`.
*   **Then**: `MoveToContact` stops on touch with measured force ≤ 15 N
    (`GetWrench` confirms); `MoveCompliant` slides the peg into the hole;
    ICON reports no fault.

### CUJ 6: Developer drives the arm by hand

*   **Given**: The stack is running.
*   **When**: A developer calls `PlanOnly` for a set of `MoveWaypoints`,
    checks duration and swept volume, executes it, then uses `Jog` to nudge
    the tool and `ComputeFk` to read the resulting pose.
*   **Then**: The executed path matches the plan; jogging moves the tool in
    the requested direction; `ComputeFk` agrees with `GetState`.

## Technical Plan (Architecture)

```text
 VLM ─MCP─> mcp_server ─gRPC─> ┌ Docker "intrinsic" (host network) ─────────────────────────────┐ [WSL]
                               │ robot_service   thin gRPC API, calls only Intrinsic APIs        │
                               │ Intrinsic       motion_planner · world (+ sim_world) · geometry  │
                               │                 ICON --non_realtime (parts: arm, gripper, ft)    │
                               │                 calibration service · pose estimator + Triton    │
                               │                 grasp planner                                    │
                               │ Stand-ins       platform_stubs · localcas · zenohd · runtime pb  │
                               │ Adapters (ours) ros2_hwm   ICON hardware module ↔ ROS 2 topics   │
                               │                 camera_sim  serves CameraService from ROS 2 topics│
                               │                 scene_sync  Isaac scene → sim_world              │
                               └───────────────────────────────┬─────────────────────────────────┘
                                                               │ ROS 2 (DDS), mirrored networking
                               Isaac Sim 5.1 (Windows): Franka · D455 · scene · built-in ROS 2 bridge
```

### Process model

*   Each Intrinsic service runs as a plain process started by
    `container/entrypoint.sh`. If any process exits, the container exits
    (no silent failures).
*   Each process that expects a k3s `runtime_config.pb` gets one encoded
    from a checked-in textproto by `container/encode_runtime.sh`.
*   `platform_stubs.py` serves the platform gRPC services Intrinsic expects
    (ResourceRegistry, InstalledAssetsReader, and any found later).
*   The container uses the host network. Every gRPC server binds to
    127.0.0.1. Ports: world 18000, geometry 18001, planner 18002, ICON
    18003, camera (`camera_sim`) 18005, calibration 18006, pose estimator
    18007, robot_service 18100, stubs 18010, localcas 18020, Triton 18030
    (HTTP) / 18031 (gRPC) / 18032 (metrics), zenohd 7447. (18004 and 18008
    are unused: the gripper is an ICON part and the grasp planner is a ROS 2
    service.)
*   **Every ROS 2 node in the container uses CycloneDDS** (consequence of
    D1): `ros2_hwm` through Bazel's ROS 2, and the Python nodes and the
    grasp planner through the image's ROS 2 Humble with
    `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. The only DDS-vendor boundary is
    container ↔ Isaac, which phase 0.5 tests.
*   Each process's flags: the phase 0 plan lists every address flag of
    every binary, and states for each one whether it is pointed at a
    stand-in, at a real service, or left empty (feature off).

### Isaac Sim side (built-in ROS 2 bridge only)

| Channel | ROS 2 interface | Isaac Sim 5.1 component |
|---|---|---|
| Robot state ↑ | `/joint_states` (position, velocity, effort) every physics step | ROS2 Publish Joint State |
| Robot command ↓ | `/joint_command` (7 arm joints, position + velocity feed-forward), stamped with the state stamp it answers | ROS2 Subscribe Joint State → Articulation Controller (applied by `sim/run_scene.py`, see lockstep) |
| Gripper command ↓ | `/gripper_command` (`control_msgs/msg/GripperCommand`: total width, max effort) | `sim/run_scene.py`: both finger joints to width/2; finger drive max force = max effort / 2 |
| Camera ↑ | `/camera/rgb`, `/camera/depth` (32FC1, meters, stereo-disparity noise), `/camera/depth_gt` (noise-free), `/camera/camera_info`; rendered at 30 Hz | `rsd455.usd` via `SingleViewDepthSensorAsset` + ROS2 Camera Helper |
| Wrench ↑ | `/wrist_wrench` (`geometry_msgs/WrenchStamped`, frame `panda_hand`, force on the hand from the flange joint) every physics step | `sim/run_scene.py` reads the articulation's measured joint force at the `panda_hand_joint` and publishes it with an OmniGraph `ROS2Publisher` node |
| Scene ↕ | `simulation_interfaces` services | `isaacsim.ros2.sim_control` |
| Ground truth ↑ | `/tf` (objects, true camera pose) | ROS2 Publish Transform Tree |
| Clock ↑ | `/clock` | ROS2 Publish Clock |

*   **Lockstep between Isaac physics and ICON.** Isaac is run by our
    script `sim/run_scene.py`, which owns the simulation loop (it calls the
    physics step itself; rendering runs every Nth step for 30 Hz camera
    output). Each cycle:
    1.  Isaac steps physics once and publishes `/joint_states` stamped with
        simulation time *t*.
    2.  `ros2_hwm` receives it, ticks ICON's clock once
        (`drives_realtime_clock`), and ICON's resulting command is published
        on `/joint_command` with stamp *t*.
    3.  `run_scene.py` waits until a `/joint_command` stamped *t* arrives,
        applies it, and steps again.
    *   ICON therefore runs on simulation time: latency slows the whole
        simulation down, but a command is never late and trajectories never
        run ahead of the arm. Pausing Isaac pauses ICON.
    *   **No controller attached:** if no command stamped *t* arrives within
        1 s of wall time, `run_scene.py` holds the last command and keeps
        stepping (free-running), so Isaac works without the container. It
        re-enters lockstep on the next matching command.
    *   **Lost or duplicated messages:** both topics use reliable QoS, depth
        1. `ros2_hwm` ignores a `/joint_states` whose stamp is not newer
        than the last one (duplicate). A lost state message is covered by
        the 1 s rule above.
    *   **Rate:** the physics rate (target 120–250 Hz) is ICON's control
        rate; phase 0.5 measures the round trip and phase 2 fixes the rate
        at or below 1 / (p99 round trip).
    *   **Timeouts in `robot_service`** are wall-clock and generous: 10 ×
        the planned duration + 5 s, so a slow simulation does not fail
        motions.
*   Isaac runs its ROS 2 bridge in Humble mode with Fast DDS; the exact
    environment (ROS distro variable, library path, DDS profile, domain ID,
    QoS) is fixed in the phase 0.5 plan.
*   Isaac is started headless by `sim/run_scene.py` with
    `--/app/vulkan=false`. Isaac's Vulkan renderer crashes in its
    `rtx.scenedb` plugin with the installed NVIDIA GPU driver (616.92);
    with this flag Isaac uses its Direct3D 12 renderer. RGB rendering is
    verified on it; the depth sensor is checked in phase 2 (Q7).

### RobotService API

The MCP tools mirror these calls one-to-one.

| Call | Intrinsic API | Internal path |
|---|---|---|
| `GetState` | ICON `GetStatus` + `ObjectWorldService` | joints, gripper width, wrench and faults from ICON; tool pose from the world. The world service's own `WorldUpdater` copies ICON joints into the belief world `world` every 30 ms (it needs only the ICON resource listed in the registry stand-in) |
| `MoveToPose(pose, motion=ANY\|LINEAR)` | `MotionPlannerService.PlanTrajectory`, then ICON trajectory tracking | planner → `JointTrajectoryPVA` → ICON session on part `arm` → `ros2_hwm` → `/joint_command` |
| `MoveJoints(q)`, `Home()` | same; `Home` uses the `home` named configuration | same |
| `MoveWaypoints([...], blending)` | `PlanTrajectory` with segments + `BlendingParameters` | same |
| `PlanOnly(...)` | `PlanTrajectory` without executing | returns duration + swept volume |
| `ComputeIk(pose)`, `ComputeFk(q)` | `MotionPlannerService.ComputeIk / ComputeFk` | — |
| `CheckCollisions(q or pose)` | `MotionPlannerService.CheckCollisions` | — |
| `GetWrench()`, `TareWrench()` | ICON force-torque part status; `intrinsic.tare_force_torque_sensor` | `/wrist_wrench` → `ros2_hwm` force-torque interface → ICON |
| `MoveToContact(direction, max_force, speed)` | ICON `intrinsic.force_primitive` (move along a direction until the force threshold) | stops when the measured force along `direction` reaches `max_force` |
| `MoveCompliant(target_pose, stiffness, max_force)` | ICON `intrinsic.cartesian_admittance` | used for insertion: moves toward the target while yielding to contact forces |
| `PlanGrasp(object)` | grasp planner (ROS 2 service, see Q5), then `PlanTrajectory` | object mesh + pose from the belief world |
| `Stop()` | ICON `intrinsic.stop` action | ends the current ICON session with a controlled stop. Not a fault: the next motion call is accepted |
| `SetSpeed(factor)` | ICON speed override | scales all following motions |
| `ClearFaults()` | ICON `ClearFaults` | when ICON reports a fault (e.g. limit or following-error violation), every motion call returns `FAILED_PRECONDITION` with ICON's fault message until `ClearFaults` succeeds |
| `Jog(joint\|cartesian, axis, velocity)` | joint: ICON `intrinsic.joint_jogging` action; Cartesian: ICON jogging service | ICON |
| `Grasp(force)`, `Release()`, `SetGripper(width, force)`, `IsGripping()` | ICON gripper part (`HalLinearGripperPart`, `intrinsic.simple_gripper` action) | ICON → `ros2_hwm` gripper interface (`hal/interfaces/gripper.fbs`) → `/gripper_command`; Isaac limits finger drive force to the command. `IsGripping` = fingers stopped more than 1 mm wider than commanded |
| `AttachObject`, `DetachObject`, `SetPayload` | `ObjectWorldService.ReparentObject` (belief world), ICON payload | planner treats a held object as part of the arm |
| `AddObject`, `RemoveObject` | Isaac `SpawnEntity` / `DeleteEntity` only | `scene_sync` then mirrors Isaac into `sim_world` and, in ground-truth mode, into `world` (see below) |
| `ListObjects` | `ObjectWorldService.ListObjects` on `world` | each object marked `ground_truth` or `perceived` |
| `ResetScene` | ICON `intrinsic.stop`, Isaac `ResetSimulation`, ICON `ClearFaults` | sequence: stop the ICON session, reset Isaac (arm teleports home), clear faults. Detaches held objects and sets payload to empty. Camera calibration is kept |
| `GetImage(rgb\|depth)` | `CameraService.Capture` | `camera_sim` serves it from the ROS 2 camera topics (noise already in the Isaac depth) |
| `GetPointCloud()` | `CameraService.Capture` (depth + intrinsics) | `robot_service` back-projects depth with the camera intrinsics; points in the camera frame, plus the camera's pose in `world` |
| `CalibrateCamera()` | `CalibrationService` (`Initialize`, `CaptureData`, `Calibrate`, `Save`) + Intrinsic's `pose_sampling` library | see "Calibration flow" |
| `DetectObjects()` | `PoseEstimationService.RunPoseEstimation` (RF-DETR → FoundationPose, Triton) | `robot_service` writes each estimate into `world` (create or update, marked `perceived`) |

**Ground truth vs. belief.**
*   Isaac is the only place objects are created. `scene_sync` mirrors it
    into `sim_world` (true poses).
*   `world` is the belief world that the planner uses. `robot_service` has a
    setting `belief_source`:
    *   `ground_truth` (default until phase 5): `scene_sync` also copies
        every object from `sim_world` into `world` (create with
        `SyncObject` when new, update the pose on every change, delete when
        gone). Objects are marked `ground_truth`.
    *   `perception` (after phase 5): `scene_sync` leaves `world` alone;
        only `DetectObjects` writes objects into `world` (marked
        `perceived`).
*   `AttachObject`/`DetachObject` act on `world`. While attached, the
    ground-truth copy does not update that object's pose.

**Calibration flow** (eye-to-hand: the D455 is fixed in the scene).
1.  `CalibrateCamera()` spawns a ChArUco board in Isaac rigidly fixed to
    the hand, offset so the fingers stay clear, and adds it to `world` as
    attached to the flange (so the planner avoids it).
2.  It calls `CalibrationService.Initialize`, generates 15–25 board poses
    with Intrinsic's `pose_sampling` library (facing the camera, inside
    reach and view), moves to each reachable one with `MoveToPose`, and
    calls `CaptureData` there.
3.  `Calibrate`, then `Save`. The camera pose in `world` is updated and the
    result is stored in the container volume
    (`/var/lib/intrinsic/calibration/`), reloaded at start-up.
4.  The board is removed from Isaac and `world`.
The phase 4 plan fixes how the service's "execute" and "initial" worlds map
to `world` and `sim_world`.

### Adapters

*   **`ros2_hwm`** (C++, ICON hardware module, based on the `loopback`
    module, built in Intrinsic's Bazel per D1): exports to ICON joint
    position command/state, joint velocity/torque state, joint limits,
    safety status, payload, a gripper interface (`hal/interfaces/gripper.fbs`)
    and a force-torque sensor (from `/wrist_wrench`). It drives ICON's clock
    (lockstep, above). `ReadStatus` and `ApplyCommand` must not allocate;
    ROS I/O runs on separate threads with `icon/utils/async_buffer.h`.
*   **`camera_sim`** (Python): serves `CameraService` from the ROS 2 camera
    topics, as a relay (noise comes from Isaac's depth sensor). Publishes
    nothing back to Isaac.
*   **`scene_sync`** (Python): Isaac → `sim_world` only. `GetEntities`
    gives the object list (spawn/delete); `/tf` gives poses.

### Franka model (same robot on both sides)

1.  Source: Isaac's `lula_franka_gen.urdf` (path in G3). It is a Lula file
    with extra fixed frames (wrist `camera_*` chain, `panda_face_*`,
    fingertip points); the phase 1 plan lists which frames are kept.
2.  Convert to SDF; add Intrinsic tags (`intrinsic:ik_solver
    kinematic_chain`, flange/tool frames, limits including jerk).
3.  Build a SceneObject with `sdf_scene_object` + `updates.pbtxt` (limits,
    `home` configuration), following the UR robot definitions.
4.  The loader stages geometry into CAS/geometry, then creates the object
    with the SDK's `ObjectWorldClient.create_object(geometry_spec=SceneObject)`.
5.  Check: FK on 20 random configurations, Intrinsic vs. Isaac, < 1 mm.

### Languages and code location

*   C++ only where Intrinsic requires it (the ICON hardware module, built as
    decided in D1). Python for every other adapter, stand-in,
    `robot_service`, and the MCP server.
*   The hardware module talks to ICON over ICON's shared-memory IPC, so it
    runs in the same container as ICON.
*   Code that must build with Intrinsic's Bazel lives in this repository
    under `ext/` and is linked into the Intrinsic workspace as
    `~/intrinsic-core/ext` (symlink created by `container/build_intrinsic.sh`),
    so it builds as `//ext/...` without changing Intrinsic's sources.
*   Python code that needs Intrinsic's Python clients (ICON) is built as a
    Bazel `py_binary` under `ext/`. Python code that only needs gRPC stubs
    uses `gen/` (from `container/gen_protos.sh`).
*   Python ROS 2 nodes in the container use the image's `rclpy` (Humble).

## Alternatives Considered

### Alternative A: Run Intrinsic on k3s as designed

*   **Pros**: Supported path; all platform services present.
*   **Cons**: Heavy; pulls in the cluster gateway, Gazebo, executive.
*   **Reason for Rejection**: Owner decision: no k3s.

### Alternative B: gRPC or TCP/UDP link between container and Isaac

*   **Pros**: Simple to write; no DDS interop risk.
*   **Cons**: Custom protocol; not reusable with other ROS tools or real
    robots.
*   **Reason for Rejection**: Owner decision: the bridge is ROS 2 only.

### Alternative C: Use Intrinsic's executive, skills and behavior trees

*   **Pros**: Ready-made pick/place and calibration skills.
*   **Cons**: Pulls in the executive and skill runtime; hides the API the
    VLM should compose.
*   **Reason for Rejection**: Owner decision: the VLM sequences raw API calls.

### Alternative D: Isaac's own controllers (Lula, RMPflow, differential IK)

*   **Pros**: Already works in the demo.
*   **Cons**: Not the Intrinsic stack; no calibration, perception, force
    control or grasp planning services.
*   **Reason for Rejection**: Owner wants the Intrinsic suite.

### Alternative E: Intrinsic's pinch gripper service (`GenericGripper`, sim mode) + a `PinchGripperServer` adapter

*   **Pros**: Uses Intrinsic's gripper service API.
*   **Cons**: Three parts (service, adapter, merge into the joint command)
    where one ICON gripper part does the same; one more process and port.
*   **Reason for Rejection**: ICON's gripper part is also Intrinsic and is
    simpler; gripper commands then share ICON's safety and stop handling.

### Alternative F: Intrinsic's `ros_camera_service`

*   **Pros**: Intrinsic's own `CameraService` for ROS cameras; the path a
    real D455 would take.
*   **Cons**: It reaches the camera driver over Intrinsic pubsub (Zenoh),
    not DDS, so it would need a Zenoh–DDS bridge to Isaac.
*   **Reason for Rejection**: Breaks the ROS-2-only rule for the Isaac link.

### Alternative G: A hand-written D455 noise model in `camera_sim`

*   **Pros**: Full control over the noise.
*   **Cons**: Hand-tuned; Isaac's stereo-disparity sensor already produces
    D455 noise and holes from the scene geometry.
*   **Reason for Rejection**: Isaac's sensor is more realistic for less code.

--------------------------------------------------------------------------------

## Non-Functional Requirements

*   **Security**: gRPC services have no authentication. With WSL mirrored
    networking, a port bound on all interfaces would be reachable from the
    LAN, so every gRPC server binds to 127.0.0.1. DDS on both sides is
    restricted to the loopback interface (CycloneDDS network interface
    `lo`; Fast DDS interface allowlist 127.0.0.1, UDP only, no multicast
    beyond loopback); mirrored mode with `hostAddressLoopback=true` lets
    Windows and WSL reach each other on loopback. Phase 0.5 verifies this;
    if loopback does not carry DDS between Windows and WSL, DDS uses the
    WSL interface and a Windows firewall rule blocks the DDS ports from
    the LAN. No credentials in the repository.
*   **Performance**: ICON ticks on Isaac's physics steps (lockstep), so
    latency slows the simulation instead of breaking control. Round-trip
    latency (Isaac publishes state → ICON command arrives in Isaac) is
    measured in phase 0.5; its 99th percentile sets the achievable physics
    rate (rate ≤ 1 / p99 round trip).
*   **Reproducibility**: Bazel targets and Docker base images are pinned;
    `container/build_intrinsic.sh`, `container/stage_intrinsic.sh` and
    `docker build` recreate the image from a clean checkout.

--------------------------------------------------------------------------------

### Additional Resources

*   Intrinsic Core: `~/intrinsic-core` (`AGENTS.md`, `developer_resources/`).
*   ICON bring-up tutorial: `developer_resources/learn/tutorials/interactive_tutorials/icon/robot_bringup.md`.
*   Gazebo camera reference: `intrinsic/simulation/gazebo/plugins/cameras/camera_service.cc`.
*   Isaac Sim 5.1: `C:\isaac-sim` (`/mnt/c/isaac-sim`).

### Future Improvements

*   **Real Franka**: needs a hardware module with the same interfaces as
    `ros2_hwm` (arm, gripper, force-torque, payload): either extend
    Intrinsic's `icon_hwm_controller` (today position/velocity only) on top
    of `franka_ros2`, or write a Franka module. Everything above ICON is
    unchanged.
*   **Isaac on Linux**: would allow CycloneDDS on both sides.

--------------------------------------------------------------------------------

## Detailed Implementation Plan

Each phase gets its own zero-decision plan in `docs/designs/phases/`,
written when the phase starts (later phases depend on what earlier phases
find). Process per phase:

1.  Lead investigates and writes `docs/designs/phases/phase_<n>.md`
    (target files, exact changes, tests, done criteria).
2.  Review review (a coding agent, a frontier hosted model, fresh session, design
    implementation-readiness persona). Findings resolved in the plan.
3.  Review workers (a coding agent, a frontier hosted model, fresh session each)
    implement the plan.
4.  Lead verifies the done criteria by running them, and a review
    reviews the code diff. Then commit.

| # | Phase | Plan | Done when |
|---|---|---|---|
| 0 | Build + container skeleton: ICON, loopback, localcas, zenohd, image | `phases/phase_0.md` | All services start in the container; planner and world answer |
| 0.5 | ROS 2 interop test (after `wsl --shutdown` for mirrored networking) | `phases/phase_0_5.md` | Messages flow both ways between Docker and Isaac; latency measured |
| 1 | Franka in Intrinsic: SDF scene object, loader, ICON with loopback | `phases/phase_1.md` | IK/FK work, FK check passes, ICON moves the Franka in loopback |
| 2 | Isaac scene + ROS 2 bridge: D455, wrench sensor, sim_control, physics rate | `phases/phase_2.md` | From the container: joint states, command moves arm, RGB + depth arrive, `GetEntities` lists the cube |
| 3 | `ros2_hwm` (arm + gripper + clock), `scene_sync`, `robot_service` motion/gripper/scene/safety calls | `phases/phase_3.md` | `MoveToPose` moves the Isaac Franka; `Grasp` reports gripping; planner avoids a box; `Stop` works |
| 3b | `camera_sim` relay | `phases/phase_3b.md` | `GetImage` returns D455 frames through `CameraService`; depth error vs. `/camera/depth_gt` within Intel's D455 spec (≤ 2% RMS at 4 m) |
| 4 | Calibration via `CalibrationService` | `phases/phase_4.md` | < 2 mm, < 0.2° vs. true pose |
| 5 | Perception: Triton, models (train if needed), FoundationPose | `phases/phase_5.md` | `DetectObjects` matches ground truth; belief world filled |
| 5b | Force/contact | `phases/phase_5b.md` | `MoveToContact` stops within set force; guarded insert works |
| 5c | Grasp planning | `phases/phase_5c.md` | Planned grasps succeed in Isaac |
| 6 | MCP server | `phases/phase_6.md` | the frontier model picks and places a cube from a chat prompt |

### Verification Plan

*   **Build**: `container/build_intrinsic.sh` (Bazel build of the Intrinsic
    binaries and `//ext/...`), `container/stage_intrinsic.sh` (copies those
    binaries, their runfiles and Bazel-built shared libraries into
    `container/stage/`, failing on any unresolved library), then
    `docker build -f container/Dockerfile -t intrinsic-core:dev .`
*   **Unit tests**: `pytest tests/` for Python code; `bazel test //ext/...`
    for Bazel code.
*   **Lint/format**: `pyink --line-length=80 --indent-spaces=2`, `isort
    --profile=google`, `clang-format`, `buildifier` (Intrinsic's style).
*   **Integration**: each phase's done criteria, run as scripts in `tests/e2e/`.

--------------------------------------------------------------------------------

*Drafted with design*
