# Robot platform: Intrinsic Core + Isaac Sim + MCP

*   **Author:** mew (lead: an MCP client; review: a coding agent, a frontier hosted model)
*   **Status:** Approved by owner (plan v7, 2026-10-27); review stages 5–7 applied
*   **Date:** 2026-10-27

--------------------------------------------------------------------------------

## Overview

A simulated Franka Emika Panda arm that a vision-language model (VLM) can
operate through an MCP server. Robot functions (motion planning, inverse
kinematics, control, gripper, camera, calibration, perception, force
control, grasp planning) are done by Intrinsic software. The physics run in
NVIDIA Isaac Sim 5.1. This is the base layer of a larger product built on
top of it (owner decision D6): every feature in the Goals is built now, and
the simulation uses a real sensor model (Intel RealSense D455) with its
noise.

Work is split into phases (0 to 6); the phase table is under "Detailed
Implementation Plan".

## The Problem

Isaac Sim ships a Franka and a scripted pick-and-place demo, but no robot
software stack: the demo uses a hand-written state machine and differential
IK, and has no planner, calibration, perception or force control. A VLM
needs a high-level robot API (plan, move, grasp, see) whose calls either
succeed or report what failed. Intrinsic Core provides that stack, but it
normally runs on a k3s (Kubernetes) cluster with Gazebo or real hardware, a
real-time Linux kernel, and an executive that runs behavior trees (see
Non-Goals for why these are excluded).

The solution: run the Intrinsic services as plain processes in one Docker
container, replace the platform services they expect with stand-ins,
connect them to Isaac Sim over ROS 2, and put one gRPC API
(`RobotService`) and then an MCP server on top.

## Glossary

*   **VLM**: vision-language model (here the frontier model). **MCP**: Model Context
    Protocol, a protocol through which an AI model calls external tools.
*   **ICON**: Intrinsic's robot control service. It runs a fixed-rate
    control loop and executes *actions* (follow a trajectory, stop, move
    until contact) on *parts* (a controllable piece of hardware: the arm,
    the gripper, the force-torque sensor). A client opens a *session* on
    parts and starts actions in it. `--non_realtime` runs it without a
    real-time kernel.
*   **Hardware module (HWM)**: the process that connects ICON to a robot.
    ICON and the HWM exchange commands and state through shared memory
    every control cycle. `loopback` is Intrinsic's example HWM that echoes
    commands back as state (a robot without physics).
*   **World service, `world`, `sim_world`**: Intrinsic's world service
    holds scenes (objects, frames, kinematics, collision geometry). `world`
    is the *belief world* the planner uses; `sim_world` holds the true
    poses from Isaac.
*   **CAS / `localcas`**: content-addressed storage for large files
    (meshes); `localcas` is Intrinsic's local implementation.
*   **Zenoh / `zenohd`**: the publish-subscribe system Intrinsic services
    use among themselves; `zenohd` is its router.
*   **`runtime_config.pb`**: a per-process configuration file that k3s
    normally writes for each Intrinsic service; here we encode it ourselves.
*   **Stand-ins**: our minimal implementations of Intrinsic platform
    services (ResourceRegistry, InstalledAssetsReader: lists of installed
    hardware and assets), served by `container/services/platform_stubs.py`.
*   **ROS 2, DDS, QoS**: ROS 2 is the robot middleware; DDS is its network
    layer (CycloneDDS and Fast DDS are two implementations); QoS settings
    control delivery (reliable or best-effort, queue depth).
*   **Triton**: NVIDIA's model-serving server. **RF-DETR**: an object
    detector. **FoundationPose**: a model that estimates an object's 6D pose
    from RGB-D and its CAD mesh.
*   **IK / FK**: inverse / forward kinematics. **Force-torque (F/T)**: the
    6-axis force and torque at the wrist ("wrench").
*   **URDF / SDF**: robot description formats. **Lula**: Isaac's motion
    library, whose Franka URDF we use. **ChArUco**: a checkerboard with
    markers used for camera calibration. **`JointTrajectoryPVA`**:
    Intrinsic's trajectory format (positions, velocities, accelerations).
*   **design** (lead–review model): the development process for this
    project. The *lead* (an MCP client, full context) writes designs; the
    *review* (fresh a coding agent sessions with no history) review and
    implement them.

## Goals

*   **G1.** One Docker container in WSL runs the Intrinsic layer: world,
    geometry, motion planner, ICON (non-real-time, with arm, gripper and
    force-torque parts), calibration service, pose estimator with Triton,
    grasp planner, the stand-ins, and our adapters (including `camera_sim`,
    which serves Intrinsic's `CameraService` API).
*   **G2.** All traffic between the container and Isaac Sim is ROS 2. No
    custom TCP/UDP or gRPC link to Isaac.
*   **G3.** The same Franka model (Isaac's
    `exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/franka/lula_franka_gen.urdf`)
    is used by Isaac and by Intrinsic. FK of both agrees within 1 mm and
    0.1° on 20 random joint configurations.
*   **G4.** `RobotService` (gRPC) exposes the calls in "RobotService API".
    Each call gets its data and performs its actions only through Intrinsic
    APIs (Intrinsic Core services, ICON, and Intrinsic's grasp planner);
    `robot_service` may post-process results (for example, turning a depth
    image into a point cloud). Simulation-only operations (spawning
    objects, resetting the scene) are in a separate `SimService` and never
    reach the robot functions.
*   **G5.** The camera is Isaac's D455 model with RGB and depth. Depth
    comes from Isaac's stereo-disparity depth sensor, with error against
    noise-free depth within Intel's D455 specification (≤ 2% RMS at 4 m) and
    with missing depth at depth edges.
*   **G6.** Camera-to-robot calibration through Intrinsic's
    `CalibrationService` reaches < 2 mm and < 0.2° against Isaac's true
    camera pose.
*   **G7.** `DetectObjects` through Intrinsic's pose estimator, on catalog
    objects that are at least 70% visible: recall ≥ 95%, position error
    < 5 mm and rotation error < 5° for ≥ 95% of detections. If the
    published models fall short on Isaac images, they are retrained on
    Isaac-rendered data.
*   **G8.** Force/contact: `MoveToContact` stops with measured force within
    ±20% of the requested limit; the peg-in-hole insertion of CUJ 5
    succeeds in ≥ 9 of 10 trials.
*   **G9.** Grasp planning: grasps from `PlanGrasp` lift the target object
    in ≥ 9 of 10 trials for each catalog object.
*   **G10.** An MCP server exposes every `RobotService` call as a tool,
    generated from the `RobotService` proto (names, descriptions and
    argument schemas come from the proto and its comments). the frontier model
    completes CUJ 1 in ≥ 4 of 5 trials.

## Non-Goals

*   k3s, Kubernetes, Helm, or the Intrinsic cluster gateway (D2).
*   A real-time kernel or real-time guarantees. ICON runs `--non_realtime`.
*   The Intrinsic executive, skills, behavior trees, or Solution Building
    Library. The VLM does all sequencing (D4).
*   Real hardware. (The design must not block it later; see Future
    Improvements.)
*   Gazebo.
*   More than one arm or more than one camera.

## Owner Decisions

| ID | Date | Decision | Reason |
|---|---|---|---|
| D1 | 2026-10-27 | `ros2_hwm` is built in Intrinsic's Bazel with Bazel's ROS 2 (Humble, CycloneDDS) | Gets every interface (arm, gripper, F/T, payload, clock) and builds with the rest of Intrinsic. Phase 0.5 must prove CycloneDDS ↔ Fast DDS interop (Q1) |
| D2 | 2026-10-27 | No k3s; services run as plain processes in one container | k3s brings the cluster gateway, Helm charts, Gazebo and the executive, none of which this system uses, and hides the processes behind cluster networking |
| D3 | 2026-10-27 | The container ↔ Isaac link is ROS 2 only | A standard robot middleware: the same topics work with ROS tools, recording, and later with real robots; no custom protocol to maintain |
| D4 | 2026-10-27 | No executive, skills or behavior trees | The VLM is the sequencer; it should compose the underlying API calls directly |
| D5 | 2026-10-27 | Use Intrinsic's stack, not Isaac's controllers (Lula, RMPflow, differential IK) | Intrinsic provides planning, ICON control, calibration, perception and force control as services; Isaac's controllers cover motion only |
| D6 | 2026-10-27 | Build for the finished product: no needed feature deferred; realistic sensors | This is the base of a larger product |

## Open Questions

*   **Q1 (phase 0.5).** Does Intrinsic's Bazel ROS 2 (Humble, CycloneDDS)
    exchange messages with Isaac's ROS 2 bridge (Humble, Fast DDS on
    Windows) across Windows → WSL (mirrored networking) → Docker (host
    network)? Fallbacks, in order: explicit unicast peer lists on both
    sides; DDS on the WSL network interface instead of loopback; build
    `ros2_hwm` against the system's Humble with Fast DDS (this reopens D1
    and goes back to the owner).
*   **Q3 (phase 4).** Which platform services `CalibrationService` needs at
    start-up; each gets a stand-in.
*   **Q4 (phase 5).** Are the RF-DETR and FoundationPose weights used by
    `ioc_pose_estimator` public, and does their licence allow use in a
    commercial product? (FoundationPose's original weights are believed to
    be under a non-commercial NVIDIA licence; to be confirmed.) If either
    answer is no: train on Isaac-rendered data with Intrinsic's
    `TrainService` / published training recipes, or choose another model
    the pose estimator supports.
*   **Q5 (phase 5c).** The grasp planner: Intrinsic's MoveIt-based grasp
    planning is described in the local tutorial
    `developer_resources/learn/tutorials/moveit_grasp_planning.md`, but its
    code is in a separate Intrinsic repository. Phase 5c finds it, checks
    that it builds on ROS 2 Humble and runs as a ROS 2 service without
    Intrinsic's skill and ROS bridge, and defines the service interface
    `robot_service` calls.
*   **Q6 (phase 0).** zenohd (from Intrinsic's image) and the zenoh-c
    version linked by the services must be compatible.
*   **Q7 (phase 2).** Does `SingleViewDepthSensor` work with the Direct3D
    12 renderer (`--/app/vulkan=false`)? If not: find another fix for the
    Vulkan crash (driver version, render settings).

## Critical User Journeys (CUJs)

### CUJ 1: VLM picks and places a cube

*   **Given**: Isaac runs the default scene (Franka, table, red cube, plate,
    D455 fixed above the table); calibration is done; the container is up;
    the MCP server is registered with the frontier model.
*   **When**: The user asks the frontier model: "put the red cube on the plate".
*   **Then**: the frontier model calls `GetImage`, `DetectObjects`, `PlanGrasp`,
    `MoveToPose`, `Grasp`, `MoveToPose`, `Release` in an order it chooses.
    The cube rests on the plate in Isaac (center within the plate's
    outline). Every call returns success or an error that says what
    failed.

### CUJ 2: Developer moves the arm through the API

*   **Given**: The container and Isaac are running.
*   **When**: A Python client calls `RobotService.MoveToPose(pose,
    motion=LINEAR)`.
*   **Then**: The planner plans a collision-free straight-line path, ICON
    executes it, and the Isaac Franka moves along it. Isaac's true hand pose
    (from `SimService.GetTruePose("panda_hand")`) is within 1 mm and 0.5° of
    the target.

### CUJ 3: Calibrate the camera

*   **Given**: The D455 is fixed in the scene, looking at the table
    (eye-to-hand). A developer calls `SimService.AttachCalibrationBoard()`
    (in a real cell, a person mounts the board).
*   **When**: `RobotService.CalibrateCamera()` is called.
*   **Then**: The flow in "Calibration flow" runs. The camera pose in
    `world` is within 2 mm / 0.2° of Isaac's true camera pose
    (`SimService.GetTruePose("camera")`). The developer then calls
    `SimService.DetachCalibrationBoard()`.

### CUJ 4: Stop a motion

*   **Given**: The arm is executing a motion started with `wait=false`.
*   **When**: `Stop()` is called.
*   **Then**: ICON brings the arm to rest within 0.5 s of simulation time.
    The motion's task reports `CANCELLED`. The next motion call is accepted.

### CUJ 5: Recover from a fault

*   **Given**: ICON reports a fault (for example, a joint limit violation).
*   **When**: The VLM calls a motion.
*   **Then**: The call returns `FAILED_PRECONDITION` with ICON's fault text.
    After `ClearFaults()`, motion calls are accepted again.

### CUJ 6: Guarded peg-in-hole insertion

*   **Given**: A peg is held in the gripper (`Grasp`, then `AttachObject`
    and `SetPayload`); a block with a hole (2 mm clearance) is on the table.
*   **When**: The VLM moves the peg 2 cm above the hole with `MoveToPose`,
    calls `TareWrench()` (removes the peg's static weight from the
    reading), then `MoveToContact(direction=down, max_force=15 N,
    speed=10 mm/s)`, then `MoveCompliant(target=3 cm lower, stiffness=200
    N/m, max_force=20 N)`.
*   **Then**: `MoveToContact` stops with measured force between 12 and
    18 N (`GetWrench`); `MoveCompliant` ends with the peg ≥ 25 mm inside the
    hole; no ICON fault.

### CUJ 7: Developer drives the arm by hand

*   **Given**: The stack is running.
*   **When**: A developer calls `PlanOnly` for a `MoveWaypoints` request,
    executes the same request, then `Jog`s the tool 5 cm along +x and
    calls `ComputeFk` on the resulting joints.
*   **Then**: Executed duration is within 5% of the planned duration;
    the tool moved 5 cm ± 2 mm along +x; `ComputeFk` agrees with
    `GetState`'s tool pose within 0.1 mm.

## Technical Plan (Architecture)

```text
 VLM ─MCP─> mcp_server ─gRPC─> ┌ Docker "intrinsic" (host network) ──────────────────────────────┐ [WSL]
            (WSL, stdio)       │ robot_service   RobotService + SimService (gRPC)                 │
                               │ Intrinsic       motion_planner · world (world + sim_world)       │
                               │                 geometry · ICON --non_realtime (arm/gripper/F-T) │
                               │                 calibration service · pose estimator + Triton    │
                               │                 grasp planner (ROS 2 service)                    │
                               │ Stand-ins       platform_stubs · localcas · zenohd · runtime pb  │
                               │ Adapters (ours) ros2_hwm    ICON hardware module ↔ ROS 2 topics  │
                               │                 camera_sim  serves CameraService from ROS 2      │
                               │                 scene_sync  Isaac scene → sim_world (+ world)    │
                               └────────────────────────────────┬─────────────────────────────────┘
                                                                │ ROS 2 (DDS), mirrored networking
                               Isaac Sim 5.1 (Windows): sim/run_scene.py · Franka · D455 · scene
```

### Process model

*   Each Intrinsic service runs as a plain process started by
    `container/entrypoint.sh`. If any process exits, the container exits.
    A process that hangs is detected by `robot_service`'s health check
    (each dependency's gRPC health or a trivial call every 5 s); after 3
    failures `robot_service` reports `UNAVAILABLE` for calls that need it.
*   Each process that expects a k3s `runtime_config.pb` gets one encoded
    from a checked-in textproto by `container/encode_runtime.sh`.
*   `platform_stubs.py` serves the platform gRPC services Intrinsic
    expects. The phase 0 plan lists every address flag of every binary, and
    for each one whether it points at a stand-in, a real service, or is
    left empty (feature off).
*   The container uses the host network. Fixed ports: world 18000,
    geometry 18001, planner 18002, ICON 18003, `camera_sim` 18005,
    calibration 18006, pose estimator 18007, `robot_service` 18100 (both
    services), stubs 18010, localcas 18020, Triton 18030 (HTTP) / 18031
    (gRPC) / 18032 (metrics), zenohd 7447. Other listening ports exist
    (world's second port, HWM gRPC, metrics exporters, Zenoh peers on
    random ports); the phase 0 plan lists them all. Exposure is controlled
    as described under Security.
*   **Every ROS 2 node in the container uses CycloneDDS** (consequence of
    D1): `ros2_hwm` through Bazel's ROS 2; the Python nodes and the grasp
    planner through the image's ROS 2 Humble with
    `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. The only DDS-vendor boundary is
    container ↔ Isaac.
*   **GPU budget** (one RTX 3080, 10 GB, shared by Windows and WSL): Isaac
    rendering ≤ 6 GB (camera 848×480, the D455's native depth resolution);
    Triton ≤ 3.5 GB (RF-DETR and FoundationPose in FP16). Phase 5 measures
    actual use; if it exceeds the budget, FoundationPose runs with a lower
    batch size first.

### Isaac Sim side

Isaac is run by our script `sim/run_scene.py` (Isaac's Python, headless),
which builds the scene, sets up Isaac's built-in ROS 2 bridge (OmniGraph
nodes), and owns the simulation loop.

| Channel | ROS 2 interface | Producer / consumer in Isaac |
|---|---|---|
| Robot state ↑ | `/joint_states` (7 arm + 2 finger joints: position, velocity, effort), stamped with simulation time | ROS2 Publish Joint State |
| Wrench ↑ | `/wrist_wrench` (`geometry_msgs/WrenchStamped`, frame `panda_hand`), same stamp | `run_scene.py` reads the measured joint force of `panda_hand_joint` (kept as a separate joint, not merged) and publishes it through a `ROS2Publisher` node |
| Arm command ↓ | `/joint_command` (7 arm joints: position + velocity feed-forward), stamped with the state stamp it answers | `run_scene.py` applies it to the articulation |
| Gripper command ↓ | `/gripper_command` (`control_msgs/msg/GripperCommand`: total width, max effort) | `run_scene.py`: both finger joints to width/2; finger drive max force = max effort / 2 |
| Camera ↑ | `/camera/rgb`, `/camera/depth` (32FC1, meters, with sensor noise), `/camera/depth_gt` (noise-free), `/camera/camera_info`; 30 Hz of simulation time | `rsd455.usd` (`Isaac/Sensors/Intel/RealSense/rsd455.usd`) via `SingleViewDepthSensorAsset` + ROS2 Camera Helper |
| Scene ↕ | `simulation_interfaces`: `SpawnEntity`, `DeleteEntity`, `GetEntities`, `GetEntitiesStates` | `isaacsim.ros2.sim_control` |
| Sim control ↕ | `/sim/reset`, `/sim/attach_board`, `/sim/detach_board` (`std_srvs/srv/Trigger`) | `run_scene.py` |
| Clock ↑ | `/clock` | ROS2 Publish Clock |

#### Timing: lockstep between Isaac and ICON

*   **One timing constant.** `config/timing.yaml` holds the ICON control
    rate (250 Hz) and physics substeps per control cycle (4, i.e. physics at
    1 kHz). `run_scene.py`, the ICON config and `ros2_hwm` all read it. The
    phase 0.5 latency measurement can lower the control rate; the
    substep count keeps physics at ≥ 1 kHz for contact accuracy.
*   **Cycle** (normal operation):
    1.  `run_scene.py` runs 4 physics substeps, then publishes
        `/wrist_wrench` and `/joint_states`, both stamped with simulation
        time *t*.
    2.  `ros2_hwm` waits until it has both messages for *t*, ticks ICON's
        clock once (`drives_realtime_clock`), and publishes ICON's command
        on `/joint_command` (and `/gripper_command` when it changes),
        stamped *t*.
    3.  `run_scene.py` waits for `/joint_command` stamped *t*, applies it,
        and continues.
    *   Rendering for the camera runs every 33 ms of simulation time.
*   **Consequence.** ICON runs on simulation time. Round-trip latency
    limits the real-time factor (how fast simulation runs against the wall
    clock), not control accuracy: a slow link makes the simulation run
    slower, but each command is applied at the step it was computed for.
*   **No controller attached (free-running).** If no matching command
    arrives within 1 s of wall time, `run_scene.py` holds the last position
    command with zero velocity feed-forward and keeps stepping, waiting up
    to 5 ms each cycle for a command stamped with the latest published *t*.
    When one arrives, it is back in lockstep. Isaac therefore works without
    the container, and a lost message costs at most 1 s.
*   **Simulation absent.** ICON faults if not ticked for 20 s. If no state
    arrives for 200 ms, `ros2_hwm` ticks ICON from a wall-clock timer at
    the control rate and reports the robot as disabled ("simulation
    absent"), so ICON rejects motions. When states return, `ros2_hwm`
    resumes lockstep and reports enabled; `robot_service` notices the
    change, calls `ClearFaults`, and re-applies payload, tare and speed
    override (which a full ICON restart resets). Start-up order of Isaac
    and the container therefore does not matter.
*   **Reset.** Only `run_scene.py` resets the simulation (`/sim/reset`);
    `sim_control`'s `ResetSimulation` is not used because it stops and
    restarts Isaac's timeline behind `run_scene.py`. A reset starts a new
    *epoch*: simulation time restarts at 0, so `ros2_hwm` treats a stamp
    lower than the last one as a new epoch (not a duplicate), reports a
    fault ("simulation reset"), and accepts the new stamps. `SimService
    .ResetScene` then calls `ClearFaults`, which re-initializes ICON's
    setpoints from the measured joints.
*   **Duplicates and QoS.** Both directions use reliable QoS, depth 1.
    Within an epoch, a stamp not newer than the last is ignored.
*   **Timeouts in `robot_service`** are in simulation time: 2 × the planned
    duration + 2 s. A stalled simulation is caught by "simulation absent"
    above, not by timeouts.

#### Isaac start-up

*   Isaac runs its ROS 2 bridge in Humble mode with Fast DDS; the exact
    environment (ROS distro variable, library path, DDS profile, domain ID,
    QoS) is defined in the phase 0.5 plan.
*   `run_scene.py` starts Isaac headless with `--/app/vulkan=false`.
    Isaac's Vulkan renderer crashes in its `rtx.scenedb` plugin with the
    installed NVIDIA GPU driver (version 616.92); with this flag Isaac uses
    its Direct3D 12 renderer. RGB rendering worked with it (the pick-and-
    place screenshots taken on 2026-10-27); depth is Q7.

### RobotService API

Every call returns a gRPC status: `OK`, `FAILED_PRECONDITION` (ICON fault
or robot disabled, with the reason), `INVALID_ARGUMENT`, `NOT_FOUND`
(unknown object), `ABORTED` (planning failed, with the planner's reason),
`CANCELLED` (stopped), `UNAVAILABLE` (a dependency is down).

**Long-running calls** (motions, `Grasp`/`Release`, `CalibrateCamera`,
`DetectObjects`) take `wait` (default true). With `wait=true` the call
blocks for at most 60 s of wall time; if not finished, it returns `OK`
with `state=RUNNING` and a `task_id`. With `wait=false` it returns the
`task_id` at once. `WaitTask(task_id, timeout_s)` and `GetTask(task_id)`
report the task's state; `Stop()` cancels the running motion. Only one
motion task runs at a time; a second one returns `FAILED_PRECONDITION`.
Typical durations: a motion 1–10 s, `CalibrateCamera` 2–5 min,
`DetectObjects` 1–3 s (simulation at real-time factor 1).

| Call | Intrinsic API | Internal path |
|---|---|---|
| `GetState` | ICON `GetStatus`, `ObjectWorldService` | joints, gripper width, wrench, faults and enabled state from ICON; tool pose from `ComputeFk` on ICON's joints. (The world service's own `WorldUpdater` also copies ICON's joints into `world` every 30 ms.) |
| `MoveToPose(pose, motion=ANY\|LINEAR, wait)` | `MotionPlannerService.PlanTrajectory`, ICON `intrinsic.trajectory_tracking` | planning starts from ICON's current joints; trajectory → ICON session on part `arm` → `ros2_hwm` → `/joint_command` |
| `MoveJoints(q, wait)`, `Home(wait)` | same; `Home` uses the `home` named configuration | same |
| `MoveWaypoints([...], blending, wait)` | `PlanTrajectory` with segments + `BlendingParameters` | same |
| `PlanOnly(request)` | `PlanTrajectory` without executing | returns duration + swept volume |
| `ComputeIk(pose)`, `ComputeFk(q)` | `MotionPlannerService.ComputeIk / ComputeFk` | — |
| `CheckCollisions(q or pose)` | `MotionPlannerService.CheckCollisions` | — |
| `GetWrench()`, `TareWrench()` | ICON F/T part status; ICON `intrinsic.tare_force_torque_sensor` | `/wrist_wrench` → `ros2_hwm` F/T interface → ICON. Tare removes static load only (not acceleration effects) |
| `MoveToContact(direction, max_force, speed, wait)` | ICON `intrinsic.force_primitive` | moves along `direction` until the measured force along it reaches `max_force` |
| `MoveCompliant(target_pose, stiffness, max_force, wait)` | ICON `intrinsic.cartesian_admittance_action` | moves toward the target while yielding to contact forces |
| `PlanGrasp(object)` | Intrinsic grasp planner (ROS 2 service, Q5), then `PlanTrajectory` | object mesh + pose from `world` |
| `Stop()` | ICON `intrinsic.stop` | controlled stop; ends the session; the motion task ends `CANCELLED`; not a fault |
| `SetSpeed(factor)` | ICON speed override | scales following motions; re-applied after ICON recovery |
| `ClearFaults()` | ICON `ClearFaults` | see CUJ 5 |
| `Jog(mode=JOINT\|CARTESIAN, axis, velocity, duration)` | joint: ICON `intrinsic.joint_jogging`; Cartesian: ICON `JoggingService` (`intrinsic_apis/intrinsic/icon/proto/v1/jogging_service.proto`) | ICON |
| `Grasp(force, wait)`, `Release(wait)`, `SetGripper(width, force, wait)`, `IsGripping()` | ICON gripper part (`HalLinearGripperPart`), `intrinsic.simple_gripper` | ICON → `ros2_hwm` gripper interface (`intrinsic_control/intrinsic/icon/hal/interfaces/gripper.fbs`) → `/gripper_command`. `IsGripping` = fingers stopped more than 1 mm wider than commanded |
| `AttachObject(name)`, `DetachObject(name)`, `SetPayload(mass, com, inertia)` | `ObjectWorldService.ReparentObject` on `world`; ICON payload | the planner treats a held object as part of the arm |
| `ListObjects` | `ObjectWorldService.ListObjects` on `world` | each object marked `ground_truth` or `perceived` |
| `GetImage(rgb\|depth)` | `CameraService.Capture` | returns only a frame rendered at or after the request's simulation time + 50 ms (arm settled) |
| `GetPointCloud()` | `CameraService.Capture` (depth + intrinsics) | `robot_service` back-projects depth; points in the camera frame, plus the camera's pose in `world` |
| `CalibrateCamera(wait)` | `CalibrationService` (`Initialize`, `CaptureData`, `Calibrate`, `Save`), Intrinsic's `pose_sampling` library | see "Calibration flow"; precondition: a board is attached |
| `DetectObjects(wait)` | `PoseEstimationService.RunPoseEstimation` | `robot_service` writes each estimate into `world` (create or update, marked `perceived`) |
| `WaitTask`, `GetTask` | — | task bookkeeping in `robot_service` |

**`SimService`** (same process, same port; for developers and test
scripts; not exposed to the VLM):

| Call | Path |
|---|---|
| `AddObject(catalog_name, name, pose)`, `RemoveObject(name)` | Isaac `SpawnEntity` / `DeleteEntity` with the catalog entry's USD |
| `ResetScene()` | ICON stop → `/sim/reset` → wait for the new epoch → `ClearFaults`. Removes objects added with `AddObject` (the default scene is rebuilt), detaches held objects, clears payload and tare. Camera calibration is kept |
| `GetTruePose(entity)` | `GetEntitiesStates` (ground truth, for tests) |
| `AttachCalibrationBoard()`, `DetachCalibrationBoard()` | `/sim/attach_board`, `/sim/detach_board`: `run_scene.py` creates/removes the board rigidly fixed to `panda_hand`, offset clear of the fingers; `robot_service` adds/removes it in `world` attached to the flange |
| `SetBeliefSource(GROUND_TRUTH\|PERCEPTION)` | see "Ground truth and belief" |

### Object catalog

`assets/objects/<catalog_name>/` holds, per object type: `model.usd` (for
Isaac), `mesh.glb` (collision and visual geometry for Intrinsic, CAD for
FoundationPose), and `object.yaml` (mass, grasp width hints). The default
scene (`sim/scenes/default.yaml`) lists the table, red cube, plate, peg and
hole block by catalog name and pose. Only catalog objects are mirrored
into Intrinsic; the robot, the camera and the ground are not.

### Ground truth and belief

*   Isaac is the only place objects are created. `scene_sync` polls
    `GetEntities` and `GetEntitiesStates` at 30 Hz of wall time and mirrors
    catalog objects into `sim_world` (geometry from the catalog via CAS).
*   `world` is the belief world the planner uses. `belief_source`:
    *   `GROUND_TRUTH` (default until phase 5): `scene_sync` also mirrors
        catalog objects into `world` (create with `SyncObject` when new,
        update the pose on change, delete when gone), marked
        `ground_truth`. `DetectObjects` then only returns estimates and
        does not write.
    *   `PERCEPTION`: `scene_sync` leaves `world` alone; `DetectObjects`
        writes objects into `world`, marked `perceived`.
*   While an object is attached (`AttachObject`), its pose in `world` is
    not updated from ground truth.
*   The camera's pose in `world` comes only from calibration, never from
    ground truth.

### Calibration flow

Eye-to-hand: the D455 is fixed in the scene.
1.  Precondition: a board is attached (`AttachCalibrationBoard` in sim).
2.  `CalibrationService.Initialize`; 20 board poses from Intrinsic's
    `pose_sampling` library (board facing the camera, inside reach and
    view); for each reachable pose: `MoveToPose`, then `CaptureData` (with
    an image taken after the arm settled, as in `GetImage`).
3.  `Calibrate`, then `Save`. The camera pose in `world` is updated, and
    the result is stored in the container volume
    (`/var/lib/intrinsic/calibration/`) and reloaded at start-up.
The phase 4 plan defines how the service's "execute" and "initial" worlds
map to `world` and `sim_world`.

### Adapters

*   **`ros2_hwm`** (C++, ICON hardware module based on `loopback`, built in
    Intrinsic's Bazel per D1): exports joint position command/state, joint
    velocity/torque state, joint limits, safety/enabled status, payload, a
    gripper interface and an F/T sensor (from `/wrist_wrench`). Drives
    ICON's clock as described in "Timing". `ReadStatus` and `ApplyCommand`
    must not allocate; ROS I/O runs on separate threads with
    `intrinsic_control/intrinsic/icon/utils/async_buffer.h`.
*   **`camera_sim`** (Python): serves `CameraService` from the ROS 2 camera
    topics (noise comes from Isaac's sensor). Keeps the latest frames with
    their stamps; publishes nothing back to Isaac.
*   **`scene_sync`** (Python): see "Ground truth and belief".

### `robot_service` and `mcp_server`

*   **`robot_service`** (Python, Bazel `py_binary` under `ext/` so it can
    use Intrinsic's ICON Python client): serves `RobotService` and
    `SimService` on 127.0.0.1:18100 (our own code, so it binds loopback).
    Holds the task table, the health checks, and the ICON recovery logic.
    Its proto is `ext/robot_service/robot_service.proto`.
*   **`mcp_server`** (Python, runs in WSL outside the container, started by
    the MCP client over stdio): generates one tool per `RobotService` RPC
    from the proto descriptors (tool description = RPC comment; input
    schema = request message). Connects to 127.0.0.1:18100.

### Franka model (same robot on both sides)

1.  Source: Isaac's `lula_franka_gen.urdf` (path in G3). It is a Lula file
    with extra fixed frames (wrist `camera_*` chain, `panda_face_*`,
    fingertip points); the phase 1 plan lists which frames are kept.
2.  Convert to SDF; add Intrinsic tags (`intrinsic:ik_solver
    kinematic_chain`, flange/tool frames, limits including jerk).
3.  Build a SceneObject with `sdf_scene_object` + `updates.pbtxt` (limits,
    `home` configuration), following the UR definitions under
    `intrinsic/` (the phase 1 plan gives the path).
4.  The loader stages geometry into CAS/geometry, then creates the object
    with the SDK's `ObjectWorldClient.create_object(geometry_spec=SceneObject)`.
5.  Check: FK on 20 random configurations, Intrinsic vs. Isaac (G3).

### Languages and code location

*   C++ only for the ICON hardware module. Python for everything else.
*   The hardware module talks to ICON over shared memory, so it runs in the
    same container as ICON.
*   Code that builds with Intrinsic's Bazel lives in this repository under
    `ext/` and is linked into the Intrinsic workspace as
    `~/intrinsic-core/ext` (symlink created by
    `container/build_intrinsic.sh`), so it builds as `//ext/...` without
    changing Intrinsic's sources.
*   Python that only needs gRPC stubs uses `gen/` (from
    `container/gen_protos.sh`). Python ROS 2 nodes use the image's `rclpy`.
*   Isaac-side code is in `sim/` and runs with Isaac's Python.

## Risks and Mitigations

| Risk | Effect | Mitigation |
|---|---|---|
| CycloneDDS ↔ Fast DDS across Windows/WSL/Docker fails | No link to Isaac | Phase 0.5 first; Q1 fallbacks |
| Round trip too slow | Simulation runs slower than real time | Lockstep keeps control correct; lower control rate (substeps keep physics ≥ 1 kHz) |
| ICON's 20 s tick timeout | Fatal ICON fault | "Simulation absent" wall-clock ticking; automatic recovery |
| Isaac reset or restart | Time goes backwards; ICON setpoint jumps | Epoch handling; reset only via `run_scene.py`; `ClearFaults` re-initializes |
| GPU memory (10 GB shared) | Isaac or Triton out of memory | Budget above; measured in phase 5 |
| Pose estimator weights missing or not licensed for product use | G7 blocked | Q4: retrain or alternative model |
| Grasp planner does not build on Humble | G9 blocked | Q5 in phase 5c; build in its own image stage |
| Hidden platform dependencies of services not yet started | Service will not start | Stand-ins, as for registry and assets |
| zenoh version mismatch | Services cannot exchange updates | Q6 in phase 0 |
| Contact simulation accuracy (2 mm clearance at 1 kHz PhysX) | CUJ 6 unreliable | Phase 5b tunes contact offsets and substeps; CUJ 6 trial count measures it |
| A service hangs without exiting | Calls stall | `robot_service` health checks → `UNAVAILABLE` |
| Limits | One arm, one camera; simulation speed bound by round trip | Accepted (Non-Goals) |

## Alternatives Considered

### Alternative A: Run Intrinsic on k3s

*   **Pros**: Supported path; all platform services present.
*   **Cons**: Brings cluster gateway, Helm, Gazebo, executive.
*   **Reason for Rejection**: D2.

### Alternative B: gRPC or TCP/UDP link between container and Isaac

*   **Pros**: No DDS interop risk.
*   **Cons**: Custom protocol; not usable with ROS tools or real robots.
*   **Reason for Rejection**: D3.

### Alternative C: Intrinsic's executive, skills and behavior trees

*   **Pros**: Ready-made pick/place and calibration skills.
*   **Cons**: Hides the calls the VLM should compose.
*   **Reason for Rejection**: D4.

### Alternative D: Isaac's controllers (Lula, RMPflow, differential IK)

*   **Pros**: Already works in the demo.
*   **Cons**: Motion only; no calibration, perception or force control.
*   **Reason for Rejection**: D5.

### Alternative E: `icon_hwm_controller` (ros2_control) for the HWM

*   **Pros**: Intrinsic's documented ROS 2 route; same path as a real
    Franka with `franka_ros2`.
*   **Cons**: Exports only position/velocity (no gripper, F/T, payload);
    targets ROS 2 Kilted while Isaac 5.1 has Humble/Jazzy.
*   **Reason for Rejection**: D1.

### Alternative F: A plain C++ ROS 2 node on system ROS 2 (Fast DDS) for the HWM

*   **Pros**: Same DDS as Isaac.
*   **Cons**: Separate build outside Intrinsic's Bazel.
*   **Reason for Rejection**: D1 (kept as Q1's last fallback).

### Alternative G: Intrinsic's pinch gripper service (`GenericGripper`) + adapter

*   **Pros**: Uses Intrinsic's gripper service API.
*   **Cons**: Service + adapter + a merge into the joint command, where one
    ICON gripper part does the same; gripper outside ICON's stop handling.
*   **Reason for Rejection**: ICON's gripper part is simpler and shares
    ICON's safety and stop.

### Alternative H: Intrinsic's `ros_camera_service`

*   **Pros**: Intrinsic's own `CameraService` for ROS cameras.
*   **Cons**: Reaches the driver over Zenoh, not DDS.
*   **Reason for Rejection**: D3.

### Alternative I: Hand-written D455 noise in `camera_sim`

*   **Pros**: Full control.
*   **Cons**: Hand-tuned; Isaac's stereo-disparity sensor derives noise and
    holes from scene geometry.
*   **Reason for Rejection**: Isaac's sensor models the physical cause.

### Alternative J: ICON on a wall clock, Isaac paced to real time

*   **Pros**: No per-step handshake; simpler `run_scene.py`.
*   **Cons**: When Isaac runs slower than real time (camera rendering,
    contacts), ICON's trajectory runs ahead of the arm and following-error
    faults follow; commands arrive at varying physics steps.
*   **Reason for Rejection**: Lockstep keeps control correct at any speed.

### Alternative K: Wrist force from Isaac's contact or effort sensors

*   **Pros**: Ready-made sensor nodes.
*   **Cons**: Contact sensors report contact forces on one body, not the
    wrench through the wrist; the effort sensor reads one joint's scalar
    effort.
*   **Reason for Rejection**: The measured joint force at the hand joint
    is the 6-axis wrench a wrist F/T sensor measures.

### Alternative L: One world instead of `world` + `sim_world`

*   **Pros**: Fewer moving parts.
*   **Cons**: Perception and calibration could not be scored against
    ground truth, and the planner would see true poses a real robot never
    has.
*   **Reason for Rejection**: Two worlds is also Intrinsic's own sim setup.

### Alternative M: Blocking-only or polling-only MCP calls

*   **Pros**: Simpler (blocking) or uniform (polling).
*   **Cons**: Blocking-only: the VLM cannot stop a motion it started, and
    long calls hit client timeouts. Polling-only: every motion needs extra
    tool calls.
*   **Reason for Rejection**: `wait` flag with a 60 s cap gives both.

--------------------------------------------------------------------------------

## Non-Functional Requirements

*   **Security**: Intrinsic's binaries listen on all interfaces and have no
    flag to change it, and the gRPC services have no authentication. With
    WSL mirrored networking, ports in WSL could be reachable from the LAN.
    Exposure is controlled by WSL's Hyper-V firewall, whose default for
    inbound traffic to WSL is block; phase 0.5 checks the setting
    (`Get-NetFirewallHyperVVMSetting`) and the project never adds inbound
    allow rules. Our own servers (`robot_service`) bind 127.0.0.1. DDS on
    both sides is restricted to loopback where possible (Q1). No
    credentials in the repository.
*   **Performance**: see "Timing". Phase 0.5 measures the round-trip time;
    the real-time factor at 250 Hz control is 1 / (250 × round trip).
*   **Reproducibility**: Bazel targets and Docker base images are pinned;
    `container/build_intrinsic.sh`, `container/stage_intrinsic.sh` and
    `docker build` recreate the image from a clean checkout.

## Additional Resources

*   Intrinsic Core: `~/intrinsic-core` (`AGENTS.md`, `developer_resources/`).
*   ICON bring-up tutorial: `developer_resources/learn/tutorials/interactive_tutorials/icon/robot_bringup.md`.
*   ICON ROS 2 tutorial: `developer_resources/learn/tutorials/interactive_tutorials/icon/ros2_bridge_tutorial.md`.
*   Gazebo camera reference: `intrinsic/simulation/gazebo/plugins/cameras/camera_service.cc`.
*   Isaac Sim 5.1: `C:\isaac-sim` (`/mnt/c/isaac-sim`).
*   Review reports: `docs/designs/reviews/`.

## Future Improvements

*   **Real Franka**: needs a hardware module with the same interfaces as
    `ros2_hwm` (arm, gripper, F/T, payload): extend `icon_hwm_controller`
    on top of `franka_ros2`, or write a Franka module. `camera_sim` is
    replaced by a real camera driver, `scene_sync` and `SimService` are
    dropped, and the belief world is filled only by perception. The
    `RobotService` API is unchanged.
*   **Isaac on Linux**: would allow CycloneDDS on both sides.

--------------------------------------------------------------------------------

## Detailed Implementation Plan

Each phase gets its own zero-decision plan in `docs/designs/phases/`,
written when the phase starts (later phases depend on what earlier phases
find). Process per phase:

1.  The lead investigates and writes `docs/designs/phases/phase_<n>.md`
    (target files, exact changes, tests, done criteria).
2.  Review review (a coding agent, a frontier hosted model, fresh session, design
    implementation-readiness, codebase-details and style personas).
    Findings resolved in the plan.
3.  Review workers (a coding agent, a frontier hosted model, fresh session each)
    implement the plan.
4.  The lead runs the done criteria; a review reviews the code diff.
    Then commit.

| # | Phase | Plan | Done when |
|---|---|---|---|
| 0 | Build + container skeleton: ICON, loopback, localcas, zenohd, image; full flag/port inventory | `phases/phase_0.md` | All services start in the container; planner and world answer |
| 0.5 | ROS 2 interop + latency + firewall check (after `wsl --shutdown` for mirrored networking) | `phases/phase_0_5.md` | Messages flow both ways Docker ↔ Isaac; round trip measured; Hyper-V firewall inbound = block |
| 1 | Franka in Intrinsic: SDF scene object, loader, ICON with loopback | `phases/phase_1.md` | IK/FK work, G3 check passes, ICON moves the Franka in loopback |
| 2 | Isaac: `run_scene.py`, scene + catalog, D455, wrench, sim services, timing | `phases/phase_2.md` | From the container: joint states, commands move the arm, RGB + depth arrive, `GetEntities` lists the cube; Q7 answered |
| 3 | `ros2_hwm` (arm, gripper, F/T, clock, lockstep), `scene_sync`, `robot_service` motion/gripper/scene/safety + `SimService` | `phases/phase_3.md` | CUJ 2, CUJ 4, CUJ 5 and CUJ 7 pass; `Grasp` reports gripping; planner avoids a box |
| 3b | `camera_sim` | `phases/phase_3b.md` | `GetImage` returns D455 frames; G5 holds |
| 4 | Calibration via `CalibrationService` | `phases/phase_4.md` | CUJ 3 / G6 |
| 5 | Perception: Triton, models (train if needed), FoundationPose | `phases/phase_5.md` | G7; belief world filled in `PERCEPTION` mode |
| 5b | Force/contact | `phases/phase_5b.md` | CUJ 6 / G8 |
| 5c | Grasp planning | `phases/phase_5c.md` | G9 |
| 6 | MCP server | `phases/phase_6.md` | CUJ 1 / G10 |

### Verification Plan

*   **Build**: `container/build_intrinsic.sh` (Bazel build of the Intrinsic
    binaries and `//ext/...`), `container/stage_intrinsic.sh` (copies those
    binaries, their runfiles and Bazel-built shared libraries into
    `container/stage/`, failing on any unresolved library), then
    `docker build -f container/Dockerfile -t intrinsic-core:dev .`
*   **Unit tests**: `pytest tests/` for Python code; `bazel test //ext/...`
    for Bazel code.
*   **Lint/format** (Intrinsic's style): `pyink --line-length=80
    --indent-spaces=2`, `isort --profile=google`, `clang-format`,
    `buildifier`.
*   **Integration**: each phase's done criteria, as scripts in `tests/e2e/`.

--------------------------------------------------------------------------------

*Drafted with design*
