# Plan v7: the Intrinsic suite in one container, Isaac Sim over ROS 2, MCP on top

Status: 2026-10-27. Replaces v1–v6. v7: built as the base of a larger product, so nothing needed is deferred (force/contact and grasp planning pulled in, realistic camera with noise, real calibration service). Includes the findings of an independent audit of intrinsic-core.

**Done so far**
- Isaac Sim's built-in Franka pick-and-place runs headless with screenshots.
- Docker Engine installed in WSL. `.wslconfig` with mirrored networking written (takes effect after `wsl --shutdown`).
- **Intrinsic `geometry`, `world` and `motion_planner` built and running standalone** (no k3s). Each answers gRPC calls, using:
  - `container/services/platform_stubs.py`: stand-ins for the ResourceRegistry and InstalledAssetsReader.
  - `container/encode_runtime.sh`: the per-process `runtime_config.pb` files that k3s normally writes.
- Tooling: `container/build_intrinsic.sh` (Bazel, with a tinygltf checksum override), `container/gen_protos.sh` (Python stubs for any Intrinsic API).

**Principles**
- **Every robot function goes through an Intrinsic API.** Our code only (a) adapts Isaac Sim to Intrinsic's hardware, camera and gripper backends, (b) stands in for missing platform services, and (c) exposes one gRPC API to the MCP.
- **No k3s, no real-time kernel** (ICON runs with `--non_realtime`).
- **Foundation for a larger product.** Nothing that's needed gets deferred; where sim realism helps later work (sensor noise, real sensor models), it goes in now.
- **No executive, skills or behavior trees** (decision 2026-10-27). The *APIs underneath* the skills are exposed directly, and **the VLM does the sequencing**.

## The picture

```
 VLM ─MCP─> mcp_server ─gRPC─> ┌ Docker: intrinsic ─────────────────────────────────────────────────────┐ [WSL]
                               │ robot_service: thin gRPC API, calls only Intrinsic APIs                 │
                               │                                                                         │
                               │ Intrinsic   motion_planner · world (+ sim_world) · geometry             │
                               │             ICON --non_realtime (arm part)                              │
                               │             pinch_gripper resource (serves GenericGripper, sim mode)    │
                               │             calibration service · pose estimator (+ Triton)             │
                               │ Platform    platform_stubs (registry, installed assets) · localcas ·    │
                               │ stand-ins   zenohd · runtime_config.pb per process                      │
                               │ Adapters    ros2_hwm        ICON hardware module ↔ /joint_command, /joint_states │
                               │ (ours)      gripper_sim     PinchGripperServer sim backend ↔ finger joints │
                               │             camera_sim      CameraService ↔ camera topics (RGB + depth)  │
                               │             scene_sync      simulation_interfaces/tf → sim_world; ICON joints → world │
                               └──────────────────────────────┬──────────────────────────────────────────┘
                                                              │ ROS 2 (DDS, mirrored networking)
                               Isaac Sim 5.1 built-in ROS 2 bridge: Franka · camera · scene     [Windows]
```

## Isaac Sim side: built-in ROS 2 bridge only

| Channel | ROS 2 interface | Built into Isaac Sim 5.1 |
|---|---|---|
| Robot state ↑ | `/joint_states`, every physics step | ROS2 Publish Joint State |
| Robot command ↓ | `/joint_command` (7 arm + 2 finger joints) | ROS2 Subscribe Joint State → Articulation Controller |
| Camera ↑ | `image_raw`, `camera_info`, **`depth` (32FC1)**, from Isaac's **Intel RealSense D455** asset (real intrinsics, resolution, FOV) mounted in the scene | ROS2 Camera Helper |
| Wrench ↑ | `/joint_efforts` (measured joint torques; Franka has torque sensing in every joint) + wrist F/T from `isaacsim.sensors.physics` | ROS2 Publish Joint State (effort) + small OmniGraph publisher |
| Scene ↕ | `simulation_interfaces` (`GetEntities`, `Get/SetEntityState`, `Spawn/DeleteEntity`, `ResetSimulation`, `StepSimulation`) | `isaacsim.ros2.sim_control` |
| Ground truth ↑ | `/tf` (objects, true camera pose) | ROS2 Publish Transform Tree |
| Clock ↑ | `/clock` | ROS2 Publish Clock |

- **Physics rate:** raised from the 60 Hz default to match ICON's control rate (target 120–250 Hz).

**ROS 2 distro and middleware (biggest open risk).**
- **The mismatch:** Intrinsic's Bazel ROS 2 is **Humble with CycloneDDS only**, so `ros2_hwm` is built against that. Isaac Sim on Windows ships Humble *and* Jazzy, but **Fast DDS only**.
- **Plan:** run Isaac's bridge in **Humble** mode (removes the distro mismatch) and rely on Cyclone↔Fast DDS interoperability for standard messages.
- **Test first:** phase 0.5 checks this before any adapter is written.

## What `RobotService` exposes, and how each call is wired

The MCP tools mirror these calls one-to-one. The VLM composes them; there are no built-in multi-step behaviors.

| `RobotService` call | Intrinsic API | Internal path |
|---|---|---|
| `GetState` | ICON `GetStatus` + `ObjectWorldService` | Joints from ICON. The world's robot joints are kept current from ICON (registry stub lists the ICON resource so the world service syncs joints itself, or `scene_sync` calls `UpdateObjectJoints`) |
| `MoveToPose(pose, motion=ANY\|LINEAR)` | `MotionPlannerService.PlanTrajectory`, then ICON `intrinsic.trajectory_tracking` | planner → `JointTrajectoryPVA` → ICON session on part `arm` → `ros2_hwm` → `/joint_command` |
| `MoveJoints(q)`, `Home()` | same; `Home` uses the robot's `home` named configuration | same |
| `MoveWaypoints([...], blending)` | `PlanTrajectory` with several segments + `BlendingParameters` | same |
| `PlanOnly(...)` (dry run) | `PlanTrajectory` without executing; duration + swept volume | — |
| `ComputeIk(pose)` / `ComputeFk(q)` | `MotionPlannerService.ComputeIk / ComputeFk` | — |
| `CheckCollisions(q or pose)` | `MotionPlannerService.CheckCollisions` | — |
| `GetWrench()`, `MoveToContact(direction, max_force)` | ICON wrench state + ICON contact/admittance actions | Isaac torque/F/T → `ros2_hwm` (force state interface) → ICON |
| `PlanGrasp(object)` | `intrinsic-moveit` grasp planner, then `PlanTrajectory` | object mesh + pose from the belief world |
| `Stop()`, `SetSpeed(factor)` | ICON `intrinsic.stop`, `SetSpeedOverride`; `ClearFaults` | ICON |
| `Jog(...)` | ICON jogging service / `joint_jogging` | ICON |
| `Grasp()`, `Release()`, `SetGripper(width, force)`, `IsGripping()` | **`GenericGripper`**, served by Intrinsic's own `pinch_gripper_server_async_callback_resource_main` in `PHYSICS_SIM` mode | that service → our `gripper_sim` (4 RPCs: Create/Reset/Command/GetStatus) → finger joints → `ros2_hwm` merges them into `/joint_command` |
| `AttachObject(name)` / `DetachObject(name)`, `SetPayload` | `ObjectWorldService.ReparentObject`, ICON `SetPayload` | planner then treats a held object as part of the arm |
| `AddObject`, `RemoveObject`, `ListObjects`, `ResetScene` | `ObjectWorldService` Create/Delete/List + Isaac `SpawnEntity`/`DeleteEntity`/`ResetSimulation` | `scene_sync` keeps both sides equal |
| `GetImage(rgb\|depth)`, `GetPointCloud()` | `CameraService.Capture` | our `camera_sim` serves `CameraService` from the ROS 2 camera topics. That's the same pattern as Intrinsic's own Gazebo camera; reference: `gazebo/plugins/cameras/camera_service.cc` |
| `CalibrateCamera()` | Intrinsic **`CalibrationService`** (`perception/proto/v1/calibration_service.proto`: ChArUco detection, pose sampling, camera-to-robot). The three platform services it needs get stand-ins, same as the registry | poses via planner + ICON; images via `CameraService` |
| `DetectObjects()` | `PoseEstimationService.RunPoseEstimation` (RF-DETR → FoundationPose, served through Triton). **Needs depth** | camera → pose estimator → results written to the belief world |

**Ground truth vs. belief.**
- Isaac's true object poses go into a **separate `sim_world`**, like Intrinsic's own sim setup does.
- The planner's belief world is filled from `DetectObjects` once perception exists (phase 5).
- Before that, belief = ground truth, clearly flagged.

## The adapters, with what Intrinsic requires of them

- **`ros2_hwm`** (from Intrinsic's `loopback` hardware module):
  - **Required interfaces:** `JointPositionCommand`, `JointPositionState`. **Also provided:** joint velocity/torque state, joint limits, safety status (AUTOMATIC), payload.
  - **Rule:** `ReadStatus`/`ApplyCommand` must not allocate memory (ICON faults the module otherwise). ROS publishing and subscribing happens on separate threads, with a lock-free buffer (`icon/utils/async_buffer.h`), as in Intrinsic's Gazebo and KUKA modules.
  - **Clock:** ICON's `control_frequency_hz` must equal what Isaac delivers.
  - **Config:** `allow_non_real_time_inverse_kinematics: true` (Franka uses the numeric `kinematic_chain` IK). The ICON config is written by hand, following `developer_resources/learn/tutorials/interactive_tutorials/icon/robot_bringup.md`.
- **`gripper_sim`:** implements the small `PinchGripperServer` sim backend that Intrinsic's own gripper service talks to.
- **`camera_sim`:** serves `CameraService` (RGB, depth, intrinsics) and applies a **D455-style noise model** to everything it serves: depth noise growing with distance², invalid pixels/holes at edges and on dark or shiny surfaces, slight RGB noise. Clean ground truth stays available through `sim_world`, so calibration and perception can be scored against it.
- **`scene_sync`:** Isaac scene → `sim_world`; ICON joints → world (if the world service doesn't do it itself).

## Franka model (same robot on both sides)

- **Why not USD:** the USD lacks what Intrinsic's world, planner and ICON need (IK solver tag, flange/tool frames, joint limits including **jerk**, named configurations). Intrinsic also has no URDF importer.
- **Route:**
  1. Take Isaac's Franka URDF (`lula_franka_gen.urdf`, the URDF for this exact robot).
  2. Convert it to SDF.
  3. Add the Intrinsic tags (`intrinsic:ik_solver kinematic_chain`, frames, limits).
  4. Build a SceneObject with `sdf_scene_object` + `updates.pbtxt` (limits, `home` configuration), copying the UR robot definitions' layout.
  5. Load it into the world with a small loader: geometry into CAS/geometry, then `CreateObject`.
- **Check:** FK for about 20 random joint configurations, Intrinsic against Isaac, within 1 mm.

## Phases

| # | Phase | Done when |
|---|---|---|
| 0 | **Build + standalone skeleton.** *Done:* geometry, world, planner, platform stubs. *Left:* ICON + loopback build (running), `localcas`, `zenohd`, one container image | Everything starts in the container, and planner/world answer (**passes outside Docker today**) |
| 0.5 | **ROS 2 interop test**: a Bazel-built (Humble/Cyclone) talker/listener ↔ Isaac Sim bridge (Humble, Fast DDS), after WSL restart with mirrored networking | Messages flow both ways between WSL/Docker and Isaac |
| 1 | **Franka in Intrinsic**: SDF scene object, loader, ICON config with `loopback` | IK/FK work, FK check passes, ICON moves the Franka in loopback (no Isaac) |
| 2 | **Isaac Sim ROS 2 bridge** (action graph + `sim_control`, depth enabled, physics rate raised) | From the container: joint states visible, a test command moves the arm, RGB + depth arrive, `GetEntities` lists the cube |
| 3 | **`ros2_hwm`, `gripper_sim`, `scene_sync`** + `robot_service` move, gripper, scene, safety calls | `MoveToPose` → planner → ICON → Isaac Franka moves (screenshot); `Grasp` reports gripping; planner avoids a box; `Stop` works |
| 3b | **`camera_sim`** (RGB, depth, point cloud, noise model) | `GetImage` returns noisy D455 frames through Intrinsic's `CameraService`; depth error vs. ground truth matches the D455 spec (≈2% at 2 m) |
| 4 | **Calibration** (ChArUco board in Isaac, `CalibrationService`) | Calibrated camera pose matches Isaac's true pose (< 2 mm, < 0.2°) |
| 5 | **Perception** (Triton, models, object registration, FoundationPose). **Largest phase** | `DetectObjects` matches ground truth; belief world is filled from detections |
| 5b | **Force/contact**: wrench into ICON (`move_to_contact`, admittance) from the Isaac torque/F/T channel | `MoveToContact` stops on touch within set force; a guarded insert works |
| 5c | **Grasp planning** (`intrinsic-moveit` grasp planner) feeding `PlanGrasp(object)` | Planned grasps on detected objects succeed in Isaac |
| 6 | **MCP server** (last) | the frontier model picks and places a cube from a chat prompt, sequencing the calls itself |

## Risks

1. **ROS 2 middleware:** Humble/Cyclone (Intrinsic) ↔ Fast DDS (Isaac on Windows) across Windows/WSL/Docker. Phase 0.5 tests this first.
2. **Perception dependencies:** Triton, model files, data assets, CAS, and hard-coded cluster DNS names in the pose estimator (fix with `/etc/hosts` aliases or a small patch).
3. **Non-real-time ICON over DDS:** timing jitter between ICON's cycle and Isaac's physics. Mitigation: matched rates, lock-free buffers, ICON speed override.
4. **Hidden platform dependencies** in services not yet started (Conductor, calibration, gripper resource). Handled the same way as so far: small stubs and runtime configs.
5. **Unverified:** zenohd/zenoh-c version compatibility; whether the DataLogger is closed source (every client treats it as optional).
