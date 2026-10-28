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
the simulation uses a real sensor model (Intel RealSense D455:
resolution and optics; depth without noise, D8).

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
*   **G5.** The camera is Isaac's D455 model (1280×720) with RGB and
    depth. Depth is Isaac's clean rendered depth, with no noise model (D8).
*   **G6.** Camera-to-robot calibration through Intrinsic's
    `CalibrationService` reaches < 2 mm and < 0.2° against Isaac's true
    camera pose.
*   **G7.** `DetectObjects` through Intrinsic's pose estimator, on catalog
    objects that are at least 70% visible: recall ≥ 95%, position error
    < 5 mm and rotation error < 5° for ≥ 95% of detections. If the
    published models fall short on Isaac images, they are retrained on
    Isaac-rendered data.
*   **G8.** Force/contact: `MoveToContact` stops with measured force within
    ±20% of the requested limit; the peg-in-hole insertion of CUJ 6
    succeeds in ≥ 9 of 10 trials; a jammed `MoveCompliant` ends `ABORTED`
    "settled before reaching the target". Checked by
    `tests/e2e/phase5b_check.py` (`contact_face`, `insert`, `jam`;
    [phase_5b.md](phases/phase_5b.md)).
*   **G9.** Grasp planning: grasps from `PlanGrasp` lift the target in
    ≥ 9 of 10 trials for each graspable catalog object (`red_cube`,
    `peg`); objects wider than the 0.075 m gripper opening are rejected
    with a clear status. The grasp planner (Intrinsic's MoveIt grasp
    pipeline, ROS 2 action) is called directly by `robot_service`;
    Intrinsic's grasp *skill* is not used.
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
| D7 | 2026-10-27 | Isaac I/O only through the ROS 2 bridge's own components; script `rclpy` only where the bridge has none (listed in phase 2) | One integration surface in Isaac; nothing custom serving data from the simulator |
| D8 | 2026-10-27 | No depth noise model: depth stays clean unless a prebuilt noise model becomes available | Isaac's prebuilt D455 sensor is unusable (phase 2), and a hand-written model is not wanted |
| D9 | 2026-10-27 | Perception uses Intrinsic's current pose-estimation path (`nvidia_pose_estimator`: SAM 3 + FoundationStereo + FoundationPose on TensorRT), not the older `ioc_pose_estimator` + Triton | Current upstream path; FoundationPose weights under the NVIDIA Open Model License (commercial use); the older path's weights are under a restrictive EULA and the path is deprecated (phase 5 inventory) |

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
*   **Q5 (answered).** The planner is in `intrinsic-ai/intrinsic-moveit`
    (Apache-2.0); it is a ROS 2 action. Its Intrinsic-free pipeline is
    vendored and built for Humble with a new `main` (phase 5c).
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
*   **When**: `RobotService.CalibrateCamera(wait=false)` is called and the
    developer polls `GetOperation` until it is done (a few minutes).
*   **Then**: The flow in "Calibration flow" runs and uses at least 12
    captures. The `sensor` frame of `d455` in `world` is within 2 mm /
    0.2° of Isaac's true camera pose
    (`SimService.GetTruePose("Camera_Pseudo_Depth")`), and
    `GetPointCloud.camera_calibrated` is true, also after a container
    restart. The developer then calls `SimService.DetachCalibrationBoard()`.

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
*   **When**: The VLM moves the peg 2 cm above the block top and 3 mm off
    the hole axis with `MoveToPose`, calls `TareWrench()` (removes the
    peg's static weight from the reading), then `MoveToContact(direction=down,
    max_force=15 N, speed=10 mm/s)`, which stops on the top face. It then
    searches with `MoveCompliant(target=the contact TCP pose moved onto the
    hole axis and 2 mm lower, stiffness=2000 N/m, max_force=30 N)`, which
    slides the peg over the hole, and inserts with
    `MoveToContact(direction=down, max_force=15 N, speed=10 mm/s)`.
*   **Then**: The first `MoveToContact` stops on the top face with measured
    force between 12 and 18 N (`GetWrench`); after the second the peg is
    ≥ 25 mm inside the hole; no ICON fault.

### CUJ 7: Developer drives the arm by hand

*   **Given**: The stack is running.
*   **When**: A developer calls `PlanOnly` for a `MoveWaypoints` request,
    executes the same request, then `Jog`s joint 1 by 0.1 rad and
    calls `ComputeFk` on the resulting joints.
*   **Then**: Executed duration is within 5% of the planned duration;
    joint 1 moved 0.1 ± 0.01 rad; `ComputeFk` agrees with
    `GetState`'s tool pose within 0.1 mm.

## Technical Plan (Architecture)

```text
 VLM ─MCP─> mcp_server ─gRPC─> ┌ Docker "intrinsic" (host network) ──────────────────────────────┐ [WSL]
            (WSL, stdio)       │ robot_service   RobotService + SimService (gRPC)                 │
                               │ Intrinsic       motion_planner · world (world + sim_world)       │
                               │                 geometry · ICON --non_realtime (arm/gripper/F-T) │
                               │                 calibration service · pose estimator + Triton    │
                               │                 grasp planner (ROS 2 action)                    │
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
    A process that hangs is caught by gRPC deadlines: every call
    `robot_service` makes has one, and `DEADLINE_EXCEEDED` is returned to
    the caller as `UNAVAILABLE`.
*   Each process that expects a k3s `runtime_config.pb` gets one encoded
    from a checked-in textproto by `container/encode_runtime.sh`.
*   `platform_stubs.py` serves the platform gRPC services Intrinsic
    expects. The phase 0 plan lists every address flag of every binary, and
    for each one whether it points at a stand-in, a real service, or is
    left empty (feature off).
*   The container uses the host network. Fixed ports: world 18000,
    geometry 18001, planner 18002, ICON 18003, `camera_sim` 18005,
    calibration 18006, pose estimator 18007, `SimLink` (`scene_sync`) 18101, `robot_service` 18100 (both
    services), stubs 18010, localcas 18020, Triton 18030 (HTTP) / 18031
    (gRPC) / 18032 (metrics), zenohd 7447. Other listening ports exist
    (world's second port, HWM gRPC, metrics exporters, Zenoh peers on
    random ports); the phase 0 plan lists them all. Exposure is controlled
    as described under Security.
*   **DDS per process** (phase 0.5): ROS 2 *services* and large messages
    are only safe between the same DDS vendor (rmw_cyclonedds puts a
    request header inside service payloads), and Isaac on Windows has only
    Fast DDS.
    *   `ros2_hwm`: CycloneDDS (D1, Bazel's ROS 2); topics only
        (`/joint_states`, `/wrist_wrench`, `/joint_command`,
        `/gripper_command`), which cross vendors.
    *   `scene_sync` and `camera_sim`: Fast DDS, through the image's ROS 2
        Humble (`rmw_fastrtps_cpp`, the image's default). `scene_sync`
        makes every ROS 2 *service* call to Isaac (`simulation_interfaces`,
        `/sim/*`) and serves them to `robot_service` over an internal gRPC
        API (`SimLink`, 127.0.0.1:18101); `camera_sim` receives the camera
        topics vendor-to-vendor.
    *   `robot_service`: no direct ROS 2 link to Isaac; Isaac operations go
        through `SimLink`. Its only ROS 2 peer is the grasp planner.
    *   Grasp planner: CycloneDDS (apt `rmw_cyclonedds_cpp`), the same
        vendor as `robot_service`'s `rclpy` (CycloneDDS).
    *   Discovery on both sides: unicast to 127.0.0.1, multicast off
        (multicast does not cross Windows ↔ WSL both ways). Configuration in
        `container/dds/` (phase 0.5).
*   **GPU budget** (one RTX 3080, 10 GB, shared by Windows and WSL): Isaac
    rendering ≤ 6 GB (camera 1280×720, the D455's native depth resolution
    and a supported colour resolution, rendered at native resolution with
    DLAA);
    Triton ≤ 3.5 GB (RF-DETR and FoundationPose in FP16). Phase 5 measures
    actual use; if it exceeds the budget, FoundationPose runs with a lower
    batch size first.

### Isaac Sim side

Isaac is run by our script `sim/run_scene.py` (Isaac's Python, headless),
which builds the scene, sets up Isaac's built-in ROS 2 bridge (OmniGraph
nodes), and owns the simulation loop.

**Owner decision D7 (2026-10-27): all Isaac I/O goes through the bridge's
own components** (OmniGraph publish/subscribe nodes, Camera Helper,
`isaacsim.ros2.sim_control`). Nothing else serves data from Isaac. The
script's own `rclpy` is allowed only where the bridge has no component for
the job; each such use is listed by name in the phase 2 plan with the
reason.

| Channel | ROS 2 interface | Producer / consumer in Isaac |
|---|---|---|
| Robot state ↑ | `/joint_states` (7 arm + 2 finger joints: position, velocity, effort), stamped with simulation time | ROS2 Publish Joint State |
| Wrench ↑ | `/wrist_wrench` (`geometry_msgs/WrenchStamped`, frame `panda_hand`), same stamp | `run_scene.py` reads the measured joint force of `panda_hand_joint` (kept as a separate joint, not merged) and publishes it through a `ROS2Publisher` node |
| Arm command ↓ | `/joint_command` (7 arm joints: position + velocity feed-forward), stamped with the state stamp it answers | `run_scene.py` applies it to the articulation |
| Gripper command ↓ | `/gripper_command` (`sensor_msgs/JointState`: `panda_finger_joint1`, position = width/2, effort = max force N) | bridge Subscribe Joint State; `run_scene.py` sets the finger drive target and max force (the second finger mimics the first) |
| Camera request ↓ | `/camera/request` (`std_msgs/Header`, stamp = minimum sim time) | bridge generic Subscriber; `run_scene.py` renders on request |
| Camera ↑ | `/camera/rgb`, `/camera/depth` (32FC1, metres, ground truth), `/camera/camera_info` | `rsd455.usd` camera + bridge Camera Helper |
| Object poses ↑ | `/tf` (parent `world`; objects under `/World/objects`, `Camera_Pseudo_Depth`, `d455_optical`), 31.25 Hz sim time | bridge Publish Transform Tree + Raw Transform Tree |
| Epoch ↑ | `/sim/epoch` (`std_msgs/UInt32`, transient-local) | bridge generic Publisher |
| Scene ↕ | `simulation_interfaces`: `SpawnEntity`, `DeleteEntity`, `GetEntities` | `isaacsim.ros2.sim_control` (served during renders) |
| Sim control ↕ | `/sim/reset`, `/sim/attach_board`, `/sim/detach_board` (`std_srvs/srv/Trigger`) | `run_scene.py`'s own `rclpy` (D7 exception: the bridge's service nodes reply before the script can act) |
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
    *   Rendering happens only on a camera request and every 0.5 s of wall
        time (renders cost 100–200 ms; phase 2).
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
*   **Simulation paused or absent.** While no state arrives, ICON is not
    ticked, so it pauses with the simulation. After 20 s without a tick
    ICON faults (`kNonRealtimeHardCycleTimeout`). Recovery is the same as
    for any fault: when `ros2_hwm` receives states again, `robot_service`
    (which watches ICON's status) calls `ClearFaults` and re-applies
    payload, tare and speed override, because clearing this fault rebuilds
    ICON's internal state. Start-up order of Isaac and the container
    therefore does not matter. Phase 3 verifies that the ICON process
    survives the timeout.
*   **Reset.** Only `run_scene.py` resets the simulation (`/sim/reset`);
    `sim_control`'s `ResetSimulation` is not used because it stops and
    restarts Isaac's timeline behind `run_scene.py`. A reset increments
    `/sim/epoch`; simulation time continues. `ros2_hwm` treats an epoch
    change as a reset: it reports a fault ("simulation reset").
    `SimService.ResetScene` then calls `ClearFaults`, which re-initializes
    ICON's setpoints from the measured joints.
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
`DetectObjects`) use the standard `google.longrunning` pattern that
Intrinsic's own services use. Each takes `wait` (default true). With
`wait=true` the call blocks for at most 60 s of wall time and returns the
finished `Operation`, or the still-running one. With `wait=false` it
returns the `Operation` at once. `robot_service` also serves
`google.longrunning.Operations` (`GetOperation`, `WaitOperation`,
`CancelOperation`); `Stop()` is the same as cancelling the running motion.
Only one motion runs at a time; a second one returns
`FAILED_PRECONDITION`.
Typical durations: a motion 1–10 s, `CalibrateCamera` 2–5 min,
`DetectObjects` 1–3 s (simulation at real-time factor 1).

| Call | Intrinsic API | Internal path |
|---|---|---|
| `GetState` | ICON `GetStatus` | one call: joints, tool pose (`base_t_tip_sensed`), wrench (`wrench_at_ft`, tared and payload-compensated), gripper width, faults, enabled state. (The world service's own `WorldUpdater` separately copies ICON's joints into `world` every 30 ms.) |
| `MoveToPose(pose, motion=ANY\|LINEAR, wait)` | `MotionPlannerService.PlanTrajectory`, ICON `intrinsic.trajectory_tracking` | planning starts from ICON's current joints; trajectory → ICON session on part `arm` → `ros2_hwm` → `/joint_command` |
| `MoveJoints(q, wait)`, `Home(wait)` | same; `Home` uses the `home` named configuration | same |
| `MoveWaypoints([...], blending, wait)` | `PlanTrajectory` with segments + `BlendingParameters` | same |
| `PlanOnly(request)` | `PlanTrajectory` without executing | returns duration + swept volume |
| `ComputeIk(pose)`, `ComputeFk(q)` | `MotionPlannerService.ComputeIk / ComputeFk` | — |
| `CheckCollisions(q or pose)` | `MotionPlannerService.CheckCollisions` | — |
| `GetWrench()`, `TareWrench()` | ICON F/T part status; ICON `intrinsic.tare_force_torque_sensor` | `/wrist_wrench` → `ros2_hwm` F/T interface → ICON. Tare removes static load only (not acceleration effects) |
| `MoveToContact(direction, max_force, speed, wait)` | ICON `intrinsic.force_primitive` | moves along `direction` until the measured force along it reaches `max_force` |
| `MoveCompliant(target_pose, stiffness, max_force, wait)` | ICON `intrinsic.cartesian_admittance_action` | moves toward the target while yielding to contact forces |
| `PlanGrasp(object)` | Intrinsic's MoveIt grasp pipeline (ROS 2 action `/grasp_planning/plan_grasps`) | object pose from `world`, box dims from the catalog; graspable check, width filter, depth offset (phase 5c) |
| `Stop()` | ICON `intrinsic.stop` | controlled stop; ends the session; the motion task ends `CANCELLED`; not a fault |
| `SetSpeed(factor)` | ICON speed override | scales following motions; re-applied after ICON recovery |
| `ClearFaults()` | ICON `ClearFaults` | see CUJ 5 |
| `Jog(joint, velocity, duration)` | ICON `JoggingService` (joint jogging; Cartesian jogging does not exist in this Intrinsic version) | ICON |
| `Grasp(wait)`, `Release(wait)`, `IsGripping()` | ICON gripper part (`HalLinearGripperPart`), `intrinsic.simple_gripper` (GRASP/RELEASE to configured widths, force 30 N) | ICON → `ros2_hwm` gripper interface → `/gripper_command`. `Grasp` returns when the fingers stop; `IsGripping` = fingers stopped open > 1 mm after a GRASP (phase 3) |
| `AttachObject(name)`, `DetachObject(name)`, `SetPayload(mass, com, inertia)` | `ObjectWorldService.ReparentObject` on `world`; ICON payload | the planner treats a held object as part of the arm |
| `ListObjects` | `ObjectWorldService.ListObjects` on `world` | each object marked `ground_truth` or `perceived` |
| `GetImage(rgb\|depth)` | `CameraService.Capture` | returns only a frame rendered at or after the request's simulation time + 50 ms (arm settled) |
| `GetPointCloud()` | `CameraService.Capture` (depth + intrinsics) | `robot_service` back-projects depth; points in the camera frame, plus the camera's pose in `world` |
| `CalibrateCamera(wait)` | `CalibrationService` (`Initialize`, `CaptureData`, `Calibrate`, `Save`), Intrinsic's `pose_sampling` library | see "Calibration flow"; precondition: a board is attached |
| `DetectObjects(wait)` | `PoseEstimationService.RunPoseEstimation` | `robot_service` writes each estimate into `world` (create or update, marked `perceived`) |
| `GetOperation`, `WaitOperation`, `CancelOperation` | — | `google.longrunning.Operations` served by `robot_service` |

**`SimService`** (same process, same port; for developers and test
scripts; not exposed to the VLM):

| Call | Path |
|---|---|
| `AddObject(catalog_name, name, pose)`, `RemoveObject(name)` | Isaac `SpawnEntity` / `DeleteEntity` with the catalog entry's USD |
| `ResetScene()` | ICON stop → `/sim/reset` → wait for the new epoch → `ClearFaults`. Removes objects added with `AddObject` (the default scene is rebuilt), detaches held objects, clears payload and tare. Camera calibration is kept |
| `GetTruePose(entity)` | latest `/tf` pose from Isaac (ground truth, for tests) |
| `AttachCalibrationBoard()`, `DetachCalibrationBoard()` | `/sim/attach_board`, `/sim/detach_board`: `run_scene.py` creates/removes the board rigidly fixed to `panda_hand`, offset clear of the fingers; `robot_service` adds/removes it in `world` attached to the flange |

### Object catalog

`assets/objects/<catalog_name>/` holds, per object type: `mesh.glb` (the
single source of geometry: collision and visual geometry for Intrinsic,
CAD for FoundationPose), `object.yaml` (mass, color, physics material), and
`model.usd`, generated from the two by `sim/tools/build_catalog.py` with
Isaac's `omni.kit.asset_converter` (never edited by hand). The default
scene (`sim/scenes/default.yaml`) lists the table, red cube, plate, peg and
hole block by catalog name and pose. Only catalog objects are mirrored
into Intrinsic; the robot, the camera and the ground are not.

### Ground truth and belief

*   Isaac is the only place objects are created. `scene_sync` follows
    `/tf` (which objects exist and their poses) and mirrors catalog objects
    into `sim_world` (geometry from the catalog via CAS); details in
    phase 3 decision 15.
*   `world` is the belief world the planner uses. `belief_source` is a
    start-up setting (`config/robot_service.yaml`, read by `scene_sync` and
    `robot_service`; changing it means restarting the container):
    *   `GROUND_TRUTH` (default until phase 5): `scene_sync` also mirrors
        catalog objects into `world` (create when new,
        update the pose on change, delete when gone), marked
        `ground_truth`. `DetectObjects` then only returns estimates and
        does not write.
    *   `PERCEPTION`: `scene_sync` leaves `world` alone; `DetectObjects`
        writes objects into `world`, marked `perceived`.
*   While an object is attached (`AttachObject`), its pose in `world` is
    not updated from ground truth.
*   The camera's pose in `world` (object `d455`, optical frame `sensor`) is
    never Isaac's true pose. Until the first calibration it is a nominal
    seed (Isaac's nominal camera pose moved 3 cm and turned 3°), flagged
    `user_data["calibrated"] = false`; after it, the calibrated pose,
    flagged `true`. `GetPointCloud` reports both (`camera_pose`,
    `camera_calibrated`).

### Calibration flow

Eye-to-hand: the D455 is fixed in the scene. Details in phase 4 decisions
5–11.
1.  Precondition: the board is attached (`AttachCalibrationBoard`: Isaac
    fixes a 7 × 5 ChArUco board, 40 mm squares, 30 mm `DICT_5X5_100`
    markers, 0.12 m along the hand's z; `robot_service` parents
    `calib_board` to `franka`'s flange in `world`). Otherwise
    `FAILED_PRECONDITION`.
2.  Move the flange to a start pose at (0.60, 0, 0.50) m with its z (the
    board's textured face) toward the camera position in `world`.
3.  `CalibrationService.Initialize` (ChArUco detection, camera `d455` at
    `camera_sim`, arm `franka`).
4.  Sample 200 flange poses with Intrinsic's `pose_sampling` library
    (`sample_poses` CLI: box ±(0.08, 0.12, 0.08) m, tilt 15°, roll 20°,
    board facing the camera) and keep, in order, the first 20 with a
    collision-free IK solution and a planned trajectory.
5.  For each kept pose: `MoveToPose` (a failed move is skipped), wait
    0.2 s of sim time and until `world`'s joints equal ICON's, then
    `CaptureData`. Fewer than 12 board detections → `ABORTED`.
6.  `Calibrate` (camera-to-robot, stationary camera; it writes the new
    camera pose into `world`), then `Save` (it writes only `init_world`),
    set `calibrated` in both worlds, `Home`. The result carries the
    camera pose, the RMS translation and rotation residuals and the number
    of captures used.

`init_world` is persisted by the world service on the container's volume.
At start-up the camera loader takes `d455`'s pose and flag from
`init_world` if it has one (else the seed), creates it in `world` and
clones `world` into `init_world`, so the robot and flange IDs that
`Save` sends match in both worlds.

### Adapters

*   **`ros2_hwm`** (C++, ICON hardware module based on `loopback`, built in
    Intrinsic's Bazel per D1): exports joint position command/state, joint
    velocity/torque state, joint limits, safety/enabled status, payload, a
    gripper interface and an F/T sensor (from `/wrist_wrench`). Drives
    ICON's clock as described in "Timing". `ReadStatus` and `ApplyCommand`
    must not allocate; ROS I/O runs on separate threads with
    `intrinsic_control/intrinsic/icon/utils/async_buffer.h`.
*   **`camera_sim`** (Python): serves `CameraService` from the ROS 2 camera
    topics as they arrive (clean depth, D8). Keeps the latest frames with
    their stamps; publishes nothing back to Isaac.
*   **`scene_sync`** (Python): see "Ground truth and belief".

### `robot_service` and `mcp_server`

*   **`robot_service`** (plain Python 3.10 in the image, using a vendored
    copy of Intrinsic's ICON Python client and the `gen/` stubs; a Bazel
    `py_binary` does not survive staging, phase 3): serves `RobotService`
    and `SimService` on 127.0.0.1:18100 (our own code, so it binds
    loopback). Holds the operation table and the ICON recovery logic.
    Its proto is `robot_service/robot_service.proto`.
*   **`mcp_server`** (Python, runs in WSL outside the container, started by
    the MCP client over stdio): generates one tool per `RobotService` RPC
    from the proto descriptors (tool description = RPC comment; input
    schema = request message). Connects to 127.0.0.1:18100.

### Franka model (same robot on both sides)

1.  Source: Isaac's `lula_franka_gen.urdf` (path in G3). It is a Lula file
    with extra fixed frames (wrist `camera_*` chain, `panda_face_*`,
    fingertip points); phase 1 lists the dropped frames.
2.  A Python builder (`robot_model/`) creates the SceneObjects `franka`
    and `franka_hand` from the URDF and uploads meshes with
    `GeometryService.CreateGeometry`; see phase 1.
3.  Check: FK on 20 random configurations, Intrinsic vs. Isaac (G3).

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
| ICON's 20 s tick timeout | ICON fault | Automatic `ClearFaults` + re-apply payload/tare/speed when states return |
| Isaac reset or restart | Time goes backwards; ICON setpoint jumps | Epoch handling; reset only via `run_scene.py`; `ClearFaults` re-initializes |
| GPU memory (10 GB shared) | Isaac or Triton out of memory | Budget above; measured in phase 5 |
| Pose estimator weights missing or not licensed for product use | G7 blocked | Q4: retrain or alternative model |
| Grasp planner does not build on Humble | G9 blocked | Q5 in phase 5c; build in its own image stage |
| Hidden platform dependencies of services not yet started | Service will not start | Stand-ins, as for registry and assets |
| zenoh version mismatch | Services cannot exchange updates | Q6 in phase 0 |
| Contact simulation accuracy (2 mm clearance at 1 kHz PhysX) | CUJ 6 unreliable | Phase 5b tunes contact offsets and substeps; CUJ 6 trial count measures it |
| A service hangs without exiting | Calls stall | gRPC deadlines → `UNAVAILABLE` |
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

### Alternative I: Isaac's stereo-disparity depth sensor for noisy depth

*   **Pros**: Noise derived from scene geometry.
*   **Cons**: Measured in phase 2: 12–15% valid pixels, only along intensity
    edges (a real D455 projects an IR pattern and gives dense depth),
    values ≈ 10⁴ × metres, and the bridge's Camera Helper cannot publish
    it.
*   **Reason for Rejection**: Not a D455-like output. Clean depth is used
    (D8).

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
*   **Reason for Rejection**: `wait` flag with a 60 s cap gives both,
    using `google.longrunning` operations.

### Alternative N: SDF + Bazel `sdf_scene_object`

*   **Pros**: The path Intrinsic's own robot definitions (UR, KUKA, …)
    use; the converter validates the result.
*   **Cons**: Needs a hand-written SDF and a Bazel build, and the
    resulting `.gzf` must be unpacked, its geometry uploaded to CAS and
    re-referenced before `CreateObject`, which only Intrinsic's Go tools do.
*   **Reason for Rejection**: The Python builder uploads meshes with
    `GeometryService.CreateGeometry` and uses the same URDF Isaac uses, so
    there is one source of truth (G3).

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
*   **Observability (owner requirement, 2026-10-27)**:
    *   Every API reply carries a status: gRPC code plus a human-readable
        message saying what failed and why (for example
        `FAILED_PRECONDITION: robot faulted: simulation reset`,
        `ABORTED: no collision-free path to the target`). Long-running
        operations report `done`, `error` (code + message) or the result,
        and their metadata carries the current state (`PLANNING`,
        `EXECUTING`, `DONE`, `FAILED`, `CANCELLED`). No call fails silently
        or returns an empty error.
    *   Every process we write logs one line per event to stdout in the
        form `<time> <LEVEL> <component> <message> key=value…`
        (Python `logging`; C++ via Intrinsic's `LOG`). `robot_service`
        logs every RPC: method, key arguments, duration, resulting status;
        `ros2_hwm` logs state changes (enabled, faulted, epoch, taring,
        free-run/lockstep transitions) but never per cycle; `scene_sync`
        and `camera_sim` log each object/world change and each capture.
    *   The entrypoint prefixes each process's lines with its name (as
        now) and also writes them to `/var/lib/intrinsic/logs/<name>.log`
        (persistent volume, rotated at 10 MB × 3), so logs survive a
        crash and a restart. `docker logs intrinsic` shows all of them.
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
| 1 | Franka in Intrinsic: SceneObjects from the URDF, loader, ICON with loopback | `phases/phase_1.md` | IK/FK work, G3 check passes, ICON moves the Franka in loopback |
| 2 | Isaac: `run_scene.py`, scene + catalog, D455, wrench, sim services, timing | `phases/phase_2.md` | From the container: joint states, commands move the arm, RGB + depth arrive, `GetEntities` lists the cube; Q7 answered |
| 3 | `ros2_hwm` (arm, gripper, F/T, clock, lockstep), `scene_sync`, `robot_service` motion/gripper/scene/safety + `SimService` | `phases/phase_3.md` | CUJ 2, CUJ 4, CUJ 5 and CUJ 7 pass; `Grasp` reports gripping; planner avoids a box |
| 3b | `camera_sim` (part of phase 3) | `phases/phase_3.md` | `GetImage` returns D455 RGB and depth frames through `CameraService` |
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
