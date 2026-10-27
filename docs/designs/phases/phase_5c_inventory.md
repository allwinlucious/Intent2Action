# Phase 5c inventory: Intrinsic's MoveIt grasp planner (Q5)

This is a factual inventory for [robot_platform.md](../robot_platform.md)
Q5 (`robot_platform.md:165-171`), G9 (`:116-117`), the `PlanGrasp` row
(`:436`) and the risk "Grasp planner does not build on Humble" (`:567`).
It answers:

- where the grasp planner lives
- its ROS 2 distro and packages
- its exact interface
- how Intrinsic's skill and bridge call it, and what we would call instead
- its required inputs for a Franka
- whether it can run on Humble with CycloneDDS in a container
- its licence

**Citation conventions.**

| Shorthand | Path |
|---|---|
| `IM/` | `/tmp/intrinsic-moveit/`, a clone of <https://github.com/intrinsic-ai/intrinsic-moveit> at `c117cc5e911c30cea7310cb9ceb854fa6f2af343` (tag `v0.0.4`, 2026-10-22) |
| `SR/` | `/tmp/sdk-ros/`, a clone of <https://github.com/intrinsic-ai/sdk-ros> checked out at `06907d67964eaa55c4123b9d968faa27b4f37c70`, the commit `IM/` pins (`IM/lyrical.repos:2-6`, `IM/jazzy.repos:2-6`) |
| `RR/` | `/home/mew/.cache/bazel/_bazel_mew/c588f6cf2ea4110d87635daafacf38aa/external/com_github_mvukov_rules_ros2+/` (intrinsic-core's Bazel external directory, read only) |
| `ic/` | `/home/mew/intrinsic-core/` |
| `project/` | `/home/mew/untitled-project/` |
| `TUT` | `ic/developer_resources/learn/tutorials/moveit_grasp_planning.md` |

"Observed (eN)" means the output of experiment N in §9. Each experiment ran
in a throwaway `docker run --rm` container of the existing `intrinsic-core:dev`
image, which is `ros:humble-ros-base` (`project/container/Dockerfile:6`).
None used the host network, bound a port, or used the GPU. This is the only
repository file written. Bazel was not run, Isaac Sim was not started, and
nothing was committed.

---

## 0. Key findings

1.  **Repository: <https://github.com/intrinsic-ai/intrinsic-moveit>**,
    Apache-2.0 (`IM/LICENSE`, `IM/README.md:296-298`). It is linked from
    `TUT:3,19` and `ic/README.md:50`. Tags are `v0.0.1` to `v0.0.4`. HEAD is
    `v0.0.4`, dated 2026-10-22 (§1).
2.  **The interface is a ROS 2 *action*, not a service.**
    `/grasp_planning/plan_grasps` has type
    `moveit_planning_interfaces/action/PlanGrasps`
    (`IM/moveit_planning_interfaces/action/PlanGrasps.action:15-34`,
    `IM/moveit_planning_service/src/moveit_planning_node.cpp:688-708`).
    Up to `v0.0.2` it was the service `PlanGrasps.srv`. Commit `f0ee7c9`
    ("Migrate grasp planning from ROS 2 service to action", 2026-10-13,
    tag `v0.0.3`) changed it to an action (§3). The design calls it a
    "ROS 2 service" (`robot_platform.md:258,436`). `TUT:53` still downloads
    `v0.0.2`, the service version.
3.  **It is a box-face grasp generator verified by MoveIt Task Constructor
    (MTC).** Every object is reduced to a box. The box dimensions come from
    the request, from the object's BOX primitive, or from the bounding box of
    its mesh vertices (`IM/moveit_planning_service/src/object_geometry.cpp:24-90`).
    Candidate tool poses sit at the centres of the six faces × N rotations
    (`IM/moveit_planning_service/src/generate_box_grasp_poses.cpp:170-247`).
    MTC then checks each candidate with IK, a Cartesian approach and an OMPL
    connection from the current state (`IM/moveit_planning_service/src/grasp_planning_pipeline.cpp:387-524`).
    `TUT:5-6` says the same: cuboid grasp planning only, with motion and
    execution left to Intrinsic.
4.  **Distro.** The upstream code targets ROS 2 Jazzy and Lyrical
    (`IM/README.md:42-46`, `IM/Dockerfile.service:21`, `TUT:12`). Its
    Intrinsic dependency `sdk-ros` supports Jazzy only (`SR/README.md:9`).
    **The upstream `moveit_planning_node` does not build on Humble as is.**
    It links `flowstate_ros_bridge` and `intrinsic_sdk_cmake` from `sdk-ros`
    (`IM/moveit_planning_service/CMakeLists.txt:110,125,162-165`), and
    `intrinsic_sdk_cmake` is built with Bazel (`SR/intrinsic_sdk_cmake/CMakeLists.txt:16-17,35`).
    It also uses rclcpp's `EventsExecutor` (`moveit_planning_node.cpp:42,719`),
    which Humble's rclcpp does not have (observed, e1).
5.  **The grasp pipeline itself does build on Humble** (observed, e4). The
    three files `grasp_planning_pipeline.cpp`, `generate_box_grasp_poses.cpp`
    and `object_geometry.cpp` have no Intrinsic dependency (§8.2). They
    compiled against apt MoveIt 2.5.10 and MTC 0.1.3 with zero warnings once
    three `#include` paths were renamed from `.hpp` to `.h`.
    `moveit_planning_interfaces` built on Humble unmodified. Nothing was
    linked into an executable or run.
6.  **The Franka needs its own MoveIt config.** The planner hard-requires:
    - an end effector
    - an IK-frame link
    - named gripper states `open` and `closed`

    (`grasp_planning_pipeline.cpp:392-401,559-579`). Humble's apt Panda
    config names its closed state `close`, not `closed`, and its URDF has no
    TCP link (observed, e3). The only hardware config shipped is UR5e plus
    Robotiq Hand-E (`IM/third_party/robot_hardware_moveit_config/config/robot_hardware.srdf:47-83`).
7.  **Calling it from Bazel `robot_service` needs `moveit_msgs`.**
    `PlanGrasps` uses `moveit_msgs/CollisionObject`, `moveit_msgs/Grasp` and
    `moveit_msgs/MoveItErrorCodes`. The `moveit_msgs` package depends on
    `object_recognition_msgs` and `octomap_msgs` (observed, e5). None of the
    three is in intrinsic-core's `rules_ros2` (§8.4). `rclcpp_action` and
    `.action` generation are present (`RR/repositories/rclcpp.BUILD.bazel:99-107`,
    `RR/ros2/interfaces.bzl:51`).

---

## 1. Where it lives

| Fact | Evidence |
|---|---|
| Repository URL | `TUT:3` links `https://github.com/intrinsic-ai/intrinsic-moveit`; `TUT:19` `gh repo clone intrinsic-ai/intrinsic-moveit`; `ic/README.md:50` |
| Not in `~/intrinsic-core` | `grep -rln 'intrinsic-moveit\|moveit_plan_grasp\|flowstate_ros_bridge'` over `ic/` matched only `README.md`, `TUT` and `swap_assets.md` (one `:flowstate_ros_bridge` Bazel label, `ic/developer_resources/learn/tutorials/swap_assets.md:122`) |
| History | 12 commits, `677d5f8` "first commit" to `c117cc5` (#14) (`git log --oneline`) |
| Tags | `v0.0.1` (`362aca2`, 2026-09-01), `v0.0.2` (`b649ffe`, 2026-09-17), `v0.0.3` (`f0ee7c9`, 2026-10-13), `v0.0.4` (`c117cc5`, 2026-10-22) (`git log -1 <tag>`) |
| Release assets | `v0.0.4`: `moveit_plan_grasp_skill.bundle.tar`, `flowstate_ros_bridge_config.binarypb`. `v0.0.2`: only `moveit_plan_grasp_skill.bundle.tar` (`gh release view`). `TUT:53-55` asks `v0.0.2` for both files |
| Author | Intrinsic Innovation LLC (`IM/AUTHORS.txt`). Maintainers in `package.xml`: Aaron Chong (`IM/moveit_planning_service/package.xml:24`), Ivan David Martinez Baron (`IM/moveit_planning_interfaces/package.xml:24`) |
| Status | "still undergoing active development" (`IM/README.md:9-10`). Supports only the OMTS hardware (`TUT:169`) |

Packages (`IM/README.md:14-25`):

| Package | Role |
|---|---|
| `moveit_planning_service` | Node `moveit_planning_node`. Serves `motion_planning/get_motion_plan`, the `grasp_planning/plan_grasps` action and `~/sync_collision_objects`, and syncs the World scene in-process (`IM/moveit_planning_service/README.md:472-477`) |
| `moveit_planning_interfaces` | `PlanGrasps.action`, `GraspCandidate.msg`, `RankedGrasp.msg` (`IM/moveit_planning_interfaces/CMakeLists.txt:149-154`) |
| `moveit_plan_grasp_skill` | Intrinsic C++ skill; ROS 2 action client (§5.1) |
| `moveit_plan_motion_skill` | Intrinsic C++ skill for motion planning |
| `robot_hardware_description` | UR5e + Robotiq Hand-E + Orbbec Gemini 335Le URDF/xacro and meshes |
| `third_party/robot_hardware_moveit_config` | MoveIt config (SRDF, kinematics, OMPL/STOMP/Pilz) for that hardware |

`motion_planning/get_motion_plan` is a placeholder. It returns `SUCCESS`
with no trajectory: `// TODO: implement actual free-space or Cartesian
motion planning, replace dummy code`
(`IM/moveit_planning_service/src/moveit_planning_node.cpp:221-231`).

---

## 2. ROS 2 distro and dependencies

| Item | Evidence |
|---|---|
| Tutorial distro | "ROS lyrical" (`TUT:12,21-25`) |
| README distros | Lyrical (`lyrical.repos`) or Jazzy (`jazzy.repos`) (`IM/README.md:42-46`); builds on "Ubuntu 26 / Distrobox" (`IM/README.md:29`) |
| Container default | `ARG ROS_DISTRO=jazzy`, `FROM ros:${ROS_DISTRO}` (`IM/Dockerfile.service:21,26`) |
| Default RMW | `rmw_zenoh_cpp` (`IM/Dockerfile.service:31,149`; `TUT:76-77`); the launch test forces it (`IM/moveit_planning_service/CMakeLists.txt:305-310`) |
| `lyrical.repos` source deps | `sdk-ros` `06907d6` (v1.35.20260720), plus moveit2, moveit_msgs, moveit_resources, **moveit_task_constructor**, ompl, osqp_vendor and warehouse_ros_sqlite at pinned commits (`IM/lyrical.repos:1-34`) |
| `jazzy.repos` | only `sdk-ros` (`IM/jazzy.repos:1-6`); MoveIt and MTC come from rosdep/apt |
| `sdk-ros` distro | "compatible with ROS 2 Jazzy Jalisco" (`SR/README.md:9`); CI `ROS_DISTRO: jazzy` (`SR/.github/workflows/deploy_container_images.yaml:16`) |
| `intrinsic_sdk_cmake` build | `include(cmake/bazel_setup.cmake)` and "Extract tool binaries from bazel build of sdk" (`SR/intrinsic_sdk_cmake/CMakeLists.txt:16-17,35`); `buildtool_depend bazelisk_vendor` (`SR/intrinsic_sdk_cmake/package.xml:21`). Known build issues: system `libabsl*` conflicts, and the LLVM source build needs `libxml2.so.2` (`IM/README.md:244-258`) |

`moveit_planning_service` dependencies
(`IM/moveit_planning_service/package.xml:27-69`). Build/link:

- MoveIt and MTC: `moveit_core`, `moveit_ros_planning_interface`,
  `moveit_task_constructor_core`, `moveit_msgs`, `geometric_shapes`,
  `moveit_planning_interfaces`
- ROS 2 core: `rclcpp`, `rclcpp_action`, `std_srvs`, `tf2_ros`, `tf2_msgs`,
  `geometry_msgs`, `sensor_msgs`, `shape_msgs`, `ament_index_cpp`
- Intrinsic: `flowstate_ros_bridge`, `intrinsic_sdk_cmake`
- Other: `abseil_cpp_vendor`, `eigen`, `liblttng-ust-dev`

Exec: `robot_hardware_moveit_config`, `moveit_configs_utils`,
`moveit_ros_move_group`, `robot_state_publisher`, `rviz2`,
`controller_manager`.

**MoveIt Task Constructor: yes.** The pipeline builds an `mtc::Task`. The
stages in order are:

| Stage | Name in code | Lines |
|---|---|---|
| `CurrentState` | `current_state` | `grasp_planning_pipeline.cpp:407-413` |
| `MoveTo` (joint interpolation) | "open hand" | `:415-425` |
| `Connect` (`PipelinePlanner`, 5 attempts) | "connect current state to pre-grasp" | `:427-439` |
| `MoveRelative` (`CartesianPath`, 1 cm steps, along tool +Z) | "approach object" | `:441-470` |
| `ModifyPlanningScene` (allows end-effector ↔ object collisions) | "allow gripper-object collision" | `:472-487` |
| `ComputeIK` (≤ 16 solutions, 0.2 s) wrapping the custom generator `GenerateBoxGraspPoses` | "compute grasp ik" | `:489-520` |

Then `task.init()` and `task.plan(max_num_grasps)` run (`:522-524`).

---

## 3. Interface (exact definitions, `v0.0.4`)

### 3.1 `moveit_planning_interfaces/action/PlanGrasps.action`

`IM/moveit_planning_interfaces/action/PlanGrasps.action:15-34`, licence
header omitted:

```text
# Goal
string group_name
string end_effector_group
string tool_frame
# Total planning timeout in seconds, divided evenly across candidates
float64 planning_timeout_sec
float64 gripper_motion_duration_sec
float64 retract_dist_m
int32[] surfaces
int32 num_rotations
int64 max_num_grasps
moveit_planning_interfaces/GraspCandidate[] candidates
---
# Result
moveit_planning_interfaces/RankedGrasp[] ranked_grasps
moveit_msgs/MoveItErrorCodes error_code
---
# Feedback
uint32 completed_candidates
uint32 total_candidates
```

### 3.2 `msg/GraspCandidate.msg` (`IM/moveit_planning_interfaces/msg/GraspCandidate.msg:15-22`)

```text
# The collision object in the MoveIt planning scene
moveit_msgs/CollisionObject target

# The dimensions of the object in meters
geometry_msgs/Vector3 obj_dims_in_meters

# The pose of the center of the object in the target frame
geometry_msgs/Pose obj_t_obj_center
```

### 3.3 `msg/RankedGrasp.msg` (`IM/moveit_planning_interfaces/msg/RankedGrasp.msg:15-20`)

```text
uint32 candidate_index
string target_id
moveit_msgs/Grasp grasp
geometry_msgs/PoseStamped pre_grasp_pose
sensor_msgs/JointState grasp_ik_solution
sensor_msgs/JointState pregrasp_ik_solution
```

### 3.4 Previous service, `v0.0.1`–`v0.0.2` (`git show v0.0.2:moveit_planning_interfaces/srv/PlanGrasps.srv`, lines 15-31)

```text
string group_name
string end_effector_group
string tool_frame
float64 planning_timeout_sec
float64 gripper_motion_duration_sec
float64 retract_dist_m
int32[] surfaces
int32 num_rotations
geometry_msgs/Vector3 obj_dims_in_meters
geometry_msgs/Pose obj_t_obj_center
moveit_msgs/CollisionObject target
---
moveit_msgs/Grasp[] grasps
geometry_msgs/PoseStamped[] pre_grasp_poses
sensor_msgs/JointState[] grasp_ik_solutions
sensor_msgs/JointState[] pregrasp_ik_solutions
moveit_msgs/MoveItErrorCodes error_code
```

It was served as `grasp_planning/plan_grasps`
(`git show v0.0.2:moveit_planning_service/src/moveit_planning_node.cpp`,
lines 524-525).

### 3.5 Server behaviour (`v0.0.4`)

| Behaviour | Evidence |
|---|---|
| Name `grasp_planning/plan_grasps` (relative; `/grasp_planning/plan_grasps` with the node in the root namespace, as in `IM/README.md:155`) | `moveit_planning_node.cpp:688-689` |
| Goal rejected if the collision scene is not ready, if `candidates` is empty, or if any `target.id` is empty | `moveit_planning_node.cpp:77-96` |
| "Scene ready": set at once with `use_mock_hardware`, or with `expect_collision_objects:=false`; otherwise when `/get_planning_scene` shows ≥ 1 collision object and robot joints | `moveit_planning_node.cpp:275-280,309-314,337-348` |
| Candidates planned **sequentially**; feedback after each; cancel checked between candidates | `moveit_planning_node.cpp:124-149`; `IM/moveit_planning_service/README.md:426` |
| Any candidate failing aborts the whole goal with `FAILURE` | `moveit_planning_node.cpp:137-141` |
| Results across candidates sorted by `grasp.grasp_quality`, truncated to `max_num_grasps` (≤ 0 → 1); empty → abort `PLANNING_FAILED` | `moveit_planning_node.cpp:151-172` |
| Defaults: `end_effector_group` "hand", `tool_frame` "hande_tcp", timeout 5 s, gripper motion 0.5 s, `retract_dist_m` 0.05, surfaces all six, `num_rotations` 4 | `grasp_planning_pipeline.cpp:157-161,288-298,353-368,456-457` |
| Per-candidate timeout = `planning_timeout_sec` / number of candidates | `grasp_planning_pipeline.cpp:155-165` |
| `group_name` has no default (empty → `INVALID_GROUP_NAME`) | `grasp_planning_pipeline.cpp:372-377` |
| Before each goal, re-syncs collision objects from Intrinsic's World if the synchronizer is active | `moveit_planning_node.cpp:182-194` |

### 3.6 Meaning of the outputs

- **Only `target.id` is read from the `CollisionObject`.** The object must
  already exist in MoveIt's planning scene. The id is resolved against
  scene ids: exact match, then leading `/` stripped, then `<id>/whole`,
  then `<id>/…` prefix (`grasp_planning_pipeline.cpp:111-153,270-281`;
  `generate_box_grasp_poses.cpp:124-136`). Without `obj_dims_in_meters`
  (all three > 0), the geometry is fetched from the scene with
  `/get_planning_scene` (`grasp_planning_pipeline.cpp:303-351`).
- **Box reduction.** The first valid BOX primitive is used with its pose.
  Otherwise an axis-aligned bounding box is taken over all mesh vertices in
  the object frame (`object_geometry.cpp:28-89`).
- **Surface ids** 0..5 are +X, −X, +Y, −Y, +Z, −Z. The code cites
  `intrinsic/manipulation/grasping/grasp_annotations.proto#L88`
  (`generate_box_grasp_poses.cpp:174-218`). Ids outside 0..5 are dropped
  (`grasp_planning_pipeline.cpp:353-361`).
- **`grasp.grasp_pose`.** The pose has `header.frame_id` = object id and
  `pose` = object_T_tool, with the tool frame at the face centre
  (`generate_box_grasp_poses.cpp:225-233`). The approach runs along the
  tool frame's **+Z** (`grasp_planning_pipeline.cpp:460-466,636-638`), and
  the pre-grasp sits at `retract_dist` along tool −Z (`:708-711`).
- **`pre_grasp_pose`** is in the object frame (`grasp_planning_pipeline.cpp:727-733`).
- **`grasp_ik_solution` / `pregrasp_ik_solution`** hold the arm group's
  active joints only (`grasp_planning_pipeline.cpp:675-739`).
- **`grasp_quality`** = 1 / (1 + MTC solution cost) (`grasp_planning_pipeline.cpp:746-747`).
- **`grasp_posture` / `pre_grasp_posture`** come from the SRDF `closed` /
  `open` states (`grasp_planning_pipeline.cpp:559-631,743-744`).
  `max_contact_force` is 0 (`:751`).
- **Gripper width.** I found no width check in the code. The generator puts
  no limit on box size, and collisions between the end effector and the
  object are allowed during the approach (`grasp_planning_pipeline.cpp:472-487`).
  `grep -i width` finds only a URDF comment: "adding 1mm additional opening
  to grasp 50mm objects" (`IM/robot_hardware_description/urdf/robot.urdf.xacro:42`).
  `TUT:168` says no grasp is found for objects wider than 50 mm on all
  sides. I did not determine how that limit is enforced.

---

## 4. Other endpoints and topics the node uses

| Endpoint | Direction | Evidence |
|---|---|---|
| `/get_planning_scene` (`moveit_msgs/srv/GetPlanningScene`), served by `move_group` | client | `moveit_planning_node.cpp:643-646` |
| `/collision_object` (`moveit_msgs/msg/CollisionObject`) | publisher (World → MoveIt) | `IM/moveit_planning_service/src/moveit_scene_synchronizer.cpp:183-188` |
| `~/sync_collision_objects` (`std_srvs/srv/Trigger`) | server | `IM/moveit_planning_service/README.md:476` |
| Intrinsic World, over gRPC ingress `localhost:17080` and Zenoh `tcp/localhost:7447` (`GetObjects`, `GetGltf`, TF subscription) | client, only when `use_mock_hardware` is false | `moveit_planning_node.cpp:585-635`; `moveit_scene_synchronizer.cpp:191,365,448` |
| `/etc/intrinsic/runtime_config.pb` (optional, falls back to defaults) | file read | `moveit_planning_node.cpp:393-416` |

`service.launch.py` also starts:

- `robot_state_publisher` and `move_group` from `robot_hardware_moveit_config`
  (`IM/moveit_planning_service/launch/service.launch.py:98-101,164-178`)
- with `use_mock_hardware`: a mock `ros2_control_node`, controllers and a
  static `world → base_link` TF (`:180-214`)
- optionally RViz and the Intrinsic `status_monitor` (`:227-273`)

---

## 5. How Intrinsic calls it

### 5.1 `moveit_plan_grasp_skill`

- **Manifest.** Skill `ai.intrinsic.moveit_plan_grasp_skill`, C++, supports
  cancellation
  (`IM/moveit_plan_grasp_skill/src/moveit_plan_grasp_skill/moveit_plan_grasp_skill.manifest.textproto:18-45`).
- **Parameters (`GraspPlanningParams`).**
  - `candidate_objects` (World `ObjectReference`)
  - `tool_frame`
  - `box_grasp_annotations` (`intrinsic_proto.grasping.BoxShapedGraspAnnotationParams`)
  - optional `output_grasp_frame` / `output_pregrasp_frame`, `retract_dist_m`,
    `group_name` (default "ur_manipulator"), `timeout_ms` (default 15000),
    `end_effector_group`, `plan_id`, `max_num_grasps`,
    `gripper_motion_duration_sec`

  (`…/moveit_plan_grasp_skill.proto:23-73`).
- **Result.** `GraspPlanningResult { repeated PoseBasedGrasp grasps; plan_id; success }`
  (`:75-84`).
- **Execute.**
  1. Converts the params to a `PlanGrasps` goal. Object references become
     `target.id`; the box annotation dims and centre are copied into every
     candidate (`moveit_plan_grasp_skill.cpp:48-158`).
  2. Starts rclcpp inside the skill process
     (`InitRos`, `moveit_plan_grasp_skill.cpp:204-212`).
  3. Creates an action client node `grasp_planning_skill_client` on
     `grasp_planning/plan_grasps`: wait for the server ≤ 5 s, send the goal,
     wait for the result up to timeout + 5 s, cancel on timeout
     (`:222-284,319-323`).
  4. Maps `MoveItErrorCodes` to `absl::Status` (`:160-201`).
  5. Writes the top grasp and pre-grasp into the pre-allocated World frames
     with `object_world.UpdateTransform` relative to the target object
     (`:387-455`).
  6. Returns `PoseBasedGrasp`s with IK solutions (`:457-532`).
- **Transport.** ROS 2 over Zenoh to the cluster router
  (`IM/README.md:267`; `IM/Dockerfile.service:149-151`).

### 5.2 `flowstate_ros_bridge`

The bridge is an `sdk-ros` service already running in Intrinsic Core. It
must be reconfigured (`TUT:48-61`;
`IM/docs/flowstate_ros_bridge_configuration.md:49-61`) to:

- stream `/joint_states` from ICON instance `icon`, with the UR joint names
  overridden
- publish the robot base frame `ur_module/base_link`
- enable F/T

(`IM/configs/flowstate_ros_bridge_config.pbtxt:27-51`). Its plugins are
`WorldBridge` and `ExecutiveBridge` (`:27-30`). The planning service needs
it for `/tf` and `/joint_states` (`IM/README.md:69`). The planning node
links the bridge's `World` client library separately, to pull collision
meshes (`IM/moveit_planning_service/CMakeLists.txt:162-164`;
`moveit_planning_node.cpp:613-634`).

### 5.3 The tutorial's driver

`TUT:118-142` runs `//third_party/intrinsic_moveit/tools:moveit_plan_grasp_and_move`
in the `intrinsic-omts` repository, branch `moveit-integration` (`TUT:36-37`).
The architecture diagram is on branch `ac/grasp-planning-demo` (`TUT:160`).
Neither was inspected.

### 5.4 What we would call instead

`robot_service` would be an action client of `/grasp_planning/plan_grasps`
(`PlanGrasps`). That replaces steps 1–3 of §5.1, minus the World frame
writes and the proto mapping. The design's next step, `PlanTrajectory` on
Intrinsic's motion planner, comes after (`robot_platform.md:436`). We would
also have to supply what the bridge and the World synchronizer supply
today:

- `/joint_states` with the URDF joint names, read by `move_group`'s planning
  scene monitor. The project's Franka joints are `panda_joint1..7` and
  `panda_finger_joint1/2` (`project/docs/designs/phases/phase_1_inventory.md:867-875`).
- The target and obstacles as MoveIt collision objects, through
  `/apply_planning_scene` (`IM/README.md:119-148`) or `/collision_object`.
  Their ids are the `target.id` used in the goal.
- No connection to Intrinsic's World. The `use_mock_hardware` path skips it
  (`moveit_planning_node.cpp:585-635`), but that flag also starts mock
  `ros2_control` hardware that publishes joint states
  (`service.launch.py:192-214`). A launch for our setup would leave that
  out (inference, not tested).

---

## 6. Required inputs (Franka)

| Input | Why | Evidence |
|---|---|---|
| `robot_description` (URDF) | MTC robot model via `RobotModelLoader(node, "robot_description")` | `grasp_planning_pipeline.cpp:78-108` |
| `robot_description_semantic` (SRDF) with an arm group = `group_name` | `INVALID_GROUP_NAME` otherwise | `grasp_planning_pipeline.cpp:537-545` |
| SRDF `end_effector` = `end_effector_group`; link = `tool_frame` | `INVALID_ROBOT_STATE` otherwise | `grasp_planning_pipeline.cpp:392-401` |
| SRDF group states **`open`** and **`closed`** on the end-effector group, covering all its active joints | `INVALID_ROBOT_STATE` otherwise | `grasp_planning_pipeline.cpp:559-608` |
| Kinematics solver for the arm group | `ComputeIK` | `IM/third_party/robot_hardware_moveit_config/config/kinematics.yaml:33-37` (UR: KDL) |
| A planning pipeline for `PipelinePlanner(node)` (Humble MTC default pipeline "ompl") | `Connect` stage | `grasp_planning_pipeline.cpp:428`; Humble `pipeline_planner.h:81` (observed, e2) |
| A running `move_group` (`/get_planning_scene`) | object ids, geometry, scene readiness | `moveit_planning_node.cpp:283-291,643-646` |
| `/joint_states` | current state for `CurrentState`/`Connect` | §5.4 |
| Target object as a collision object (BOX primitive or mesh), or `obj_dims_in_meters` + `obj_t_obj_center` | box dimensions | §3.6 |
| Gripper geometry | from the URDF links in the end-effector group | `grasp_planning_pipeline.cpp:477-483` |

**Shipped hardware config** (UR5e + Hand-E):

- groups `ur_manipulator` (chain `base_link`→`flange`) and `hand`
- end effector `hand` on `flange`
- `open` = `hande_left_finger_joint` −0.001, `closed` = 0.025
- virtual joint `world`→`base_link`, fixed

(`IM/third_party/robot_hardware_moveit_config/config/robot_hardware.srdf:47-83`).
No Franka files exist in the repository (file listing of `IM/`).

**Humble apt Panda config** (`ros-humble-moveit-resources-panda-moveit-config`
2.0.7, observed e3, `config/panda.srdf`):

- groups `panda_arm` (chain `panda_link0`→`panda_link8`), `hand`
  (`panda_finger_joint1`, passive `panda_finger_joint2`) and `panda_arm_hand`
- end effector `hand` on `panda_link8`
- states `open` (0.035 per finger) and **`close`** (0)
- virtual joint `world`→`panda_link0`, **floating** (lines 16-17, 49, 75-80, 93-106)

Its `panda.urdf` has `panda_hand` but no `panda_hand_tcp` (observed, e3).
Used as is, the planner would fail with "Named state 'closed' not found"
(`grasp_planning_pipeline.cpp:570-579`). A copy needs `closed` added, a TCP
link whose +Z is the approach direction (§3.6), and probably a fixed
virtual joint. The project's Lula URDF already has `right_gripper` on
`panda_link8` at xyz 0 0 0.1, rpy 0 0 3π/4
(`project/docs/designs/phases/phase_1_inventory.md:909`). Whether its +Z
is the approach axis was not checked.

---

## 7. Licence

| Component | Licence | Evidence |
|---|---|---|
| `intrinsic-moveit` (repository) | Apache-2.0 | `IM/LICENSE:1-3`; `IM/README.md:296-298`; SPDX headers in every source file (e.g. `PlanGrasps.action:1-13`) |
| `moveit_planning_service`, `moveit_planning_interfaces`, `robot_hardware_description` | Apache-2.0 | `IM/moveit_planning_service/package.xml:25`; `IM/moveit_planning_interfaces/package.xml:25`; `IM/robot_hardware_description/package.xml:25` |
| `third_party/robot_hardware_moveit_config` | BSD-3-Clause text | `robot_hardware.srdf:3-34`; `package.xml:5` |
| `sdk-ros` (pinned dependency) | Apache-2.0 | `SR/LICENSE:1-3`; `SR/intrinsic_sdk_cmake/package.xml:12` |
| Humble `moveit_msgs`, Panda config/description | BSD | observed e3/e5 (`package.xml:14`, `:16`, `:12`) |
| Trademark | "Intrinsic" and "Intrinsic Core" are trademarks | `IM/README.md:306-308`; `IM/TRADEMARK.md` |

---

## 8. Can it run on ROS 2 Humble with CycloneDDS in a container?

### 8.1 What Humble apt provides (observed, e1)

| Package | Candidate |
|---|---|
| `ros-humble-moveit` / `-moveit-core` / `-moveit-ros-planning-interface` | 2.5.10 |
| `ros-humble-moveit-task-constructor-core` | 0.1.3 |
| `ros-humble-moveit-configs-utils` | 2.5.10 |
| `ros-humble-geometric-shapes` | 2.3.4 |
| `ros-humble-rmw-cyclonedds-cpp` | 1.3.5 |
| `ros-humble-rmw-zenoh-cpp` / `-zenoh-cpp-vendor` | 0.1.11 |
| `ros-humble-moveit-resources-panda-moveit-config` | 2.0.7 |
| `ros-humble-franka-description` | 1.0.1 |

Humble's MoveIt 2.5.10 has `.h` headers
(`moveit/planning_scene/planning_scene.h`,
`moveit/robot_model/robot_model.h`,
`moveit/robot_model_loader/robot_model_loader.h`). The include tree of
`moveit_core` + `moveit_ros_planning` + MTC + rclcpp holds only 2 `.hpp`
files under `moveit/`. No `events_executor` header exists under
`/opt/ros/humble/include` (observed, e1, e2).

### 8.2 Separable, Intrinsic-free core

The `grasp_planning_pipeline_test` target links only rclcpp, Eigen, MoveIt,
MTC and the interfaces, with no Intrinsic library
(`IM/moveit_planning_service/CMakeLists.txt:279-294`). In the three source
files and their headers, a grep for `intrinsic|flowstate|absl` finds only
copyright lines and one comment URL (observed, `grep -rn`). Their MoveIt
includes are:

- `grasp_planning_pipeline.hpp:8-9`: `moveit/robot_model/robot_model.hpp`,
  `moveit/robot_model_loader/robot_model_loader.hpp`
- `generate_box_grasp_poses.hpp:24`: `moveit/planning_scene/planning_scene.hpp`
- `grasp_planning_pipeline.cpp:26-28`: the same three
- MTC `.h` headers, which Humble also uses

`boost::any` is used (`generate_box_grasp_poses.hpp:21`). Humble MTC's
`PropertyMap` is also `boost::any`-based (observed, e2, `properties.h:58-76`).

### 8.3 Compile test (observed, e4)

Humble MTC 0.1.3 and MoveIt 2.5.10 were installed from apt in a throwaway
container. Results:

- `moveit_planning_interfaces` (unmodified): `colcon build` **OK**.
- The three pipeline files as is: **fail**.
  `grasp_planning_pipeline.hpp:8:10: fatal error: moveit/robot_model/robot_model.hpp: No such file or directory`,
  and the same for `planning_scene.hpp` from `generate_box_grasp_poses.hpp:24`.
- After a `sed` of those three include paths to `.h`: **OK**. Zero
  warnings; `libgp.a` was 7.2 MB.

This covers every MTC call in the pipeline:

- `PipelinePlanner(node)`
- `CartesianPath` setters
- `Connect::GroupPlannerVector`
- `MoveRelative::setMinMaxDistance` / `setDirection`
- `ModifyPlanningScene::allowCollisions(string, vector<string>, bool)`
- `ComputeIK` setters, `setMonitoredStage`
- `SolutionSequence` / `WrappedSolution`
- `Introspection::publishSolution`

It is a compile check only. Linking, MTC plugin loading and planning were
not run.

### 8.4 What does not port as is

| Item | Problem on Humble | Evidence |
|---|---|---|
| `moveit_planning_node.cpp` (736 lines) | `rclcpp::experimental::executors::EventsExecutor` (`:42,719`) is not in Humble rclcpp | observed, e1 |
| Same file | links `flowstate_ros_bridge` + `intrinsic_sdk_cmake`; includes `intrinsic/platform/pubsub/*`, `runtime_context.pb.h`, `moveit_planning_service.pb.h` (`:33-38`). `sdk-ros` is Jazzy-only and Bazel-built | §2 |
| CMake `intrinsic_sdk_generate_service_manifest` | Intrinsic SDK CMake function | `IM/moveit_planning_service/CMakeLists.txt:130-136` |
| `status_monitor` | links `intrinsic_sdk_cmake` | `CMakeLists.txt:182-198` |
| `robot_hardware_moveit_config` | UR5e/Hand-E only; Franka needs its own | §6 |

A Humble build would therefore be the pipeline (§8.2, include fix) plus
`moveit_planning_interfaces` plus a new small `main`. That `main` would
create the node, the `/get_planning_scene` client, the
`GraspPlanningPipeline` and the action server, as
`moveit_planning_node.cpp:637-721` does, with a Humble executor and without
the World synchronizer. It would run next to `move_group` with a Franka
MoveIt config. This is inferred from the code and not built.

### 8.5 CycloneDDS

- **The code does not depend on an RMW.** No source file names an RMW.
  Zenoh appears only in:
  - the Dockerfile `ENV` (`IM/Dockerfile.service:31,149-151`)
  - the launch-test environment (`CMakeLists.txt:305-310`)
  - the Intrinsic pubsub setup in `main` (`moveit_planning_node.cpp:418-429,600-607`),
    which the replacement `main` would not have
- Apt `rmw_cyclonedds_cpp` 1.3.5 is available (e1). Intrinsic's Bazel
  ROS 2 is Humble with CycloneDDS 0.10.5 / rmw_cyclonedds 1.3.4
  (`project/docs/designs/phases/phase_0_5_inventory.md:61-64`). So the
  planner and `robot_service` would both be Humble and both Cyclone, which
  matches the design's same-vendor rule for services and actions
  (`robot_platform.md:289-292,304-305`).
- **`robot_service`'s Bazel side.**
  - Present: `rclcpp_action` (`RR/repositories/rclcpp.BUILD.bazel:99-107`),
    `action_msgs` (`RR/repositories/rcl_interfaces.BUILD.bazel:13-16`) and
    `.action` input to `ros2_interface_library` (`RR/ros2/interfaces.bzl:51`).
    intrinsic-core already uses `ros2_interface_library`
    (`ic/intrinsic_perception/intrinsic/perception/cameras/ros/BUILD:19-38`).
  - Present: `std_msgs`, `geometry_msgs`, `sensor_msgs`, `shape_msgs` and
    `trajectory_msgs` (`RR/repositories/common_interfaces.BUILD.bazel:18-237`).
  - **Missing:** `moveit_msgs`, `object_recognition_msgs` and
    `octomap_msgs`. The `rules_ros2` external repositories are
    `console_bridge`, `cyclonedds`, `ros2_*` core packages and
    `ros2_common_interfaces`, and `grep` for those names under
    `RR/repositories/` finds nothing.
  - `moveit_msgs/CollisionObject` contains `object_recognition_msgs/ObjectType type`,
    and `moveit_msgs` depends on `octomap_msgs`, `object_recognition_msgs`,
    `action_msgs`, `trajectory_msgs` and `shape_msgs` (observed, e5).

  `robot_service` would need those `.msg` files vendored as
  `ros2_interface_library` targets, or a different call path.

---

## 9. Experiments

All ran as
`docker run --rm [--entrypoint bash] intrinsic-core:dev`, default bridge
network, no ports, no GPU, writing only inside the container's `/tmp` or
`/ws`. e4 mounted `IM/` read-only at `/src`.

| # | What | Result |
|---|---|---|
| e1 | `apt-get update`; `apt-cache policy` for the packages in §8.1; `find /opt/ros/humble/include -path '*events_executor*'`; `dpkg -c` of `ros-humble-moveit-core`, MTC | versions in §8.1; no `events_executor`; `.h` headers only |
| e2 | `apt-get download` + `dpkg -x` of MTC, `moveit-core`, `moveit-ros-planning`, `rclcpp`; `grep` of MTC headers | `PipelinePlanner(const rclcpp::Node::SharedPtr&, const std::string& pipeline = "ompl")` (`pipeline_planner.h:81`); `Task::plan(size_t max_solutions = 0)` (`task.h:133`); `allowCollisions` overloads (`modify_planning_scene.h:102-115`); `setMaxVelocityScalingFactor` (`planner_interface.h:90`); 2 `.hpp` files under `include/moveit` |
| e3 | `dpkg -x` of `ros-humble-moveit-resources-panda-moveit-config` and `-panda-description` | SRDF in §6; no `panda_hand_tcp` |
| e4 | install MTC/MoveIt dev packages; `colcon build` `moveit_planning_interfaces` + a test package of the three pipeline files, before and after `.hpp`→`.h` `sed` | §8.3 |
| e5 | `dpkg -x` of `ros-humble-moveit-msgs` | fields of `CollisionObject.msg`, `Grasp.msg`; deps in §8.5; licence BSD |

Also: `git clone` of both repositories into `/tmp/`,
`git log` / `git show <tag>:…`, and `gh release view v0.0.4|v0.0.2 -R intrinsic-ai/intrinsic-moveit`.

---

## 10. Not verified

- **Runtime.** Nothing was linked or run. Not checked:
  - that MTC loads its OMPL pipeline plugin on Humble
  - that `move_group` 2.5.10 serves the planner's `/get_planning_scene`
    requests as Jazzy does
  - that a grasp is actually found for a Franka
- **Humble ↔ Humble action calls over CycloneDDS** between a colcon-built
  planner and the Bazel-built `robot_service`, inside or across containers.
- **`moveit_msgs` in Bazel.** Whether `object_recognition_msgs`,
  `octomap_msgs` and `moveit_msgs` build cleanly as `rules_ros2`
  `ros2_interface_library` targets.
- **The Jazzy/Lyrical build** of the upstream repository (the README's own
  path). It was not attempted, because it needs `sdk-ros`, whose
  `intrinsic_sdk_cmake` runs a Bazel build.
- **The 50 mm gripper-width limit** in `TUT:168`: how it arises. No width
  check was found in the code (§3.6).
- **`CurrentState`'s source on Humble.** That the MTC `CurrentState` stage
  takes the current robot state from `move_group` (via
  `/get_planning_scene`) on Humble, as on Jazzy, was assumed and not read
  from MTC source.
- **Whether `right_gripper`'s +Z** in the project's Franka URDF points along
  the approach direction (§6).
- **Upstream licences.** Those of MoveIt 2, MTC and OMPL themselves (only
  `moveit_msgs` and the Panda resources were read, e3/e5).
- **Other repositories and releases.**
  - The `intrinsic-omts` branches `moveit-integration` and
    `ac/grasp-planning-demo` (`moveit_plan_grasp_and_move`, architecture
    diagram).
  - The contents of the release `.bundle.tar` / `.binarypb` assets.
  - Releases `v0.0.1` and `v0.0.3`.
- **`IM/docs/flowstate_integration.md`, the `moveit_plan_motion_skill`
  sources and `moveit_scene_synchronizer.cpp`** beyond the lines cited.
