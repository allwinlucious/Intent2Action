# Phase 5c: grasp planning with Intrinsic's MoveIt grasp planner

Part of [robot_platform.md](../robot_platform.md) (G9, Q5). Facts:
[phase_5c_inventory.md](phase_5c_inventory.md) ("Inv §n"); upstream file
lines are in `intrinsic-moveit` tag `v0.0.4` ("IM/…"). Requires phases 3
and 4 done (shares Isaac and the container).

## Goal and done criteria

`tests/e2e/phase5c_check.py`:

| Name | Condition |
|---|---|
| `planner_up` | `move_group` and `grasp_planner` run; action server `/grasp_planning/plan_grasps` available [120 s] |
| `cube` | 10 trials, each: `ResetScene`; `PlanGrasp("red_cube")`; `MoveToPose(pregrasp)`; `MoveToPose(grasp, LINEAR)`; `Grasp`; `AttachObject("red_cube")`; `MoveToPose(grasp + 0.10 m world z, LINEAR)`; success = Isaac's true `red_cube` z rose ≥ 0.08 m and `IsGripping`; then `Release`, `DetachObject`. ≥ 9/10 (G9) |
| `peg` | the same for `peg`; ≥ 9/10 |
| `too_wide` | `PlanGrasp("plate")` → `ABORTED`, message contains "0.075" |
| `errors` | `PlanGrasp("nope")` → `NOT_FOUND`; in a container started with `GRASP_PLANNER=off`, `PlanGrasp("red_cube")` → `UNAVAILABLE` "grasp planner not running" |
| `logs` | `robot_service.log` has one line per `PlanGrasp` call with status and duration; `grasp_planner.log` one line per goal |

G9 covers the graspable catalog objects `red_cube` and `peg`; `plate`
(0.18 m across, 0.012 m thick on a table), `hole_block`, `table` and
`obstacle` (static) are not grasp targets.

## Decisions

1.  **What runs:** the planner's Intrinsic-free grasp pipeline (Inv §8.2)
    with a new `main`, plus MoveIt `move_group`, both CycloneDDS (same
    vendor as `robot_service`'s `rclpy`). Intrinsic's own node does not
    build on Humble (Inv §8.4). Not started with `ROBOT_BACKEND=loopback`
    or `GRASP_PLANNER=off` (env, default `on`).
2.  **Vendored sources** (Apache-2.0 headers kept; record the upstream
    commit in `ros_ws/src/VENDORED.md`, Create):
    *   `ros_ws/src/moveit_planning_interfaces/` ← IM `moveit_planning_interfaces/`
        unchanged.
    *   `ros_ws/src/grasp_planner/include/moveit_planning_service/` ←
        `grasp_planning_pipeline.hpp`, `generate_box_grasp_poses.hpp`,
        `object_geometry.hpp`; `ros_ws/src/grasp_planner/src/` ← the three
        `.cpp`. Include fix: `moveit/robot_model/robot_model.hpp`,
        `moveit/robot_model_loader/robot_model_loader.hpp`,
        `moveit/planning_scene/planning_scene.hpp` → `.h` (Inv §8.3).
    *   `ros_ws/src/grasp_planner/src/main.cpp` (Create): port from IM
        `moveit_planning_node.cpp` the goal, cancel, accept and execute
        handlers (`:50-198`) and the node set-up (`:637-721`: separate
        callback group and executor for the `/get_planning_scene` client,
        `GraspPlanningPipeline`, action server `grasp_planning/plan_grasps`),
        with `rclcpp::executors::MultiThreadedExecutor`. Do **not** port
        `orchestrate_scene_synchronization` (`:252-390`) or anything
        Intrinsic; set `scene_ready = true` at start (`robot_service`
        applies the planning scene before every goal). Declare every
        parameter the ported code reads (with upstream defaults). Log one
        line per goal: target, candidates, grasps returned, duration,
        error code.
    *   `ros_ws/src/grasp_planner/CMakeLists.txt`, `package.xml` (Create):
        `ament_cmake`; deps `rclcpp`, `rclcpp_action`, `moveit_core`,
        `moveit_ros_planning`, `moveit_ros_planning_interface`,
        `moveit_task_constructor_core`, `moveit_planning_interfaces`,
        `geometry_msgs`, `sensor_msgs`, `Eigen3`; one executable
        `grasp_planner`; installs to `lib/grasp_planner`.
3.  **Franka MoveIt config** `ros_ws/src/franka_grasp_config/` (Create;
    `ament_cmake`, installs `urdf/`, `srdf/`, `config/`, `launch/`,
    `meshes/`):
    *   `ros_ws/prepare.sh` (Create; run in the Dockerfile before colcon):
        copies `assets/robots/franka/lula_franka_gen.urdf` to
        `franka_grasp_config/urdf/franka.urdf`, removing every link and
        joint whose name starts with `camera_` (Intrinsic's model drops
        them too, phase 1), and rewriting mesh URIs
        `package://franka_description/meshes/{visual,collision}/<n>.{dae,stl}`
        → `package://franka_grasp_config/meshes/<n>.stl`; copies
        `assets/robots/franka/meshes/collision/*.stl` to
        `franka_grasp_config/meshes/`.
    *   `srdf/franka.srdf`: virtual joint `world`→`base_link` fixed; group
        `panda_arm` chain `base_link`→`panda_link8`; group `hand` with
        links `panda_hand`, `panda_leftfinger`, `panda_rightfinger` and
        joints `panda_finger_joint1`, `panda_finger_joint2`; end effector
        `hand`, parent link `panda_link8`, parent group `panda_arm`;
        states on `hand`: `open` (0.04, 0.04), `closed` (0.0, 0.0);
        `disable_collisions` copied from apt
        `moveit_resources_panda_moveit_config/config/panda.srdf`.
    *   Tool frame `right_gripper` (= Intrinsic `tcp`; +z = approach, Inv
        §3.6).
    *   `config/kinematics.yaml` (KDL, `panda_arm`, timeout 0.05 s),
        `config/ompl_planning.yaml` (from the apt Panda config),
        `config/joint_limits.yaml` (velocity from the URDF, acceleration
        from `assets/robots/franka/robot_descriptor.yaml`).
    *   `launch/grasp_planner.launch.py`: `MoveItConfigsBuilder("franka",
        package_name="franka_grasp_config")` with the URDF, SRDF,
        kinematics, joint limits and the OMPL pipeline; `move_group` with
        `moveit_config.to_dict()`, `allow_trajectory_execution: false`,
        `use_sim_time: true`; `grasp_planner` with the same dict and
        `use_sim_time: true`. No `robot_state_publisher` and no static TF
        (`move_group` takes the state from Isaac's `/joint_states`;
        Isaac already publishes `/tf`).
4.  **Image** (`container/Dockerfile`, Modify), in this order: apt
    `ros-humble-moveit ros-humble-moveit-task-constructor-core
    ros-humble-moveit-configs-utils ros-humble-moveit-msgs
    ros-humble-moveit-resources-panda-moveit-config`; `COPY ros_ws
    /opt/ws/ros_ws`; `COPY assets/robots /opt/ws/assets/robots`; run
    `ros_ws/prepare.sh`; one `colcon build --merge-install --install-base
    /opt/ros_ext` over `/opt/ws/src` (the existing `simulation_interfaces`
    clone) and `/opt/ws/ros_ws/src`. Entrypoint (Modify): after
    `scene_sync`, `ros2 launch franka_grasp_config grasp_planner.launch.py`
    with CycloneDDS and ROS sourced under `set +u`; its output goes to
    the `grasp_planner` log (Observability).
5.  **Object dimensions without simulation APIs:** `tools/build_catalog_meshes.py`
    (Modify) writes `dims: [x, y, z]` (bounding box, m) into each
    `object.yaml`. `scene_sync` (Modify) sets `user_data["catalog"]` =
    `StringValue(<catalog>)` on every object it creates; perceived objects
    (phase 5) set it too. `robot_service` reads
    `/opt/app/assets/objects/<catalog>/object.yaml`. No `SimLink` call.
6.  **`robot_service.PlanGrasp`** (`container/services/robot/grasp.py`,
    Create; wired in `robot_service.py`, Modify):
    1.  `GetObject(name)` in `world` (`NOT_FOUND` if absent or without
        `user_data["catalog"]`); dims from the catalog.
    2.  **Graspable check before planning:** graspable if the smaller of
        the object's horizontal extents (`dims.x`, `dims.y`, the object is
        upright) ≤ 0.075 m; else `ABORTED` "object wider than the gripper
        opening (0.075 m) in every horizontal direction".
    3.  `/apply_planning_scene` (timeout 5 s; failure → `UNAVAILABLE`):
        every `world` object with a catalog, except attached ones, as a
        `CollisionObject` with `header.frame_id = "world"`, `id` = name,
        `pose` = the object's world pose (its origin is the bottom centre),
        one BOX primitive of `dims` with `primitive_poses[0]` = (0, 0,
        dims.z/2), operation `ADD`; MoveIt objects no longer in `world`
        with operation `REMOVE`.
    4.  Goal: `group_name panda_arm`, `end_effector_group hand`,
        `tool_frame right_gripper`, `planning_timeout_sec 10`,
        `gripper_motion_duration_sec 0.5`, `retract_dist_m 0.08`,
        `surfaces [0, 1, 2, 3, 4]` (no −Z), `num_rotations 4`,
        `max_num_grasps 20` (all candidates, so filtering is not cut short),
        one candidate: `target.id` = name, `obj_dims_in_meters` = dims,
        `obj_t_obj_center` = (0, 0, dims.z/2). Server not available in
        10 s → `UNAVAILABLE` "grasp planner not running"; goal rejected →
        `FAILED_PRECONDITION`; no result in 30 s → cancel,
        `DEADLINE_EXCEEDED`; planner failure → `ABORTED` "no feasible
        grasp found (see grasp_planner.log)".
    5.  Width filter on all returned grasps: keep those whose object
        extent along the tool y axis (the finger axis) ≤ 0.075 m; none →
        `ABORTED` as in step 2.
    6.  Depth: move the grasp pose along its own +z by
        d = min(0.02, extent along the approach axis − 0.01) so the finger
        pads overlap the object (the planner puts the TCP on the face,
        fingertips are 3.4 mm beyond `right_gripper`, Inv review).
    7.  Return the best (highest `grasp_quality`): `grasp`, `pregrasp`
        = world_T_object ∘ object_T_tool (world = robot base, Franka at
        the origin), `grasp_joints` from `grasp_ik_solution` ordered
        `panda_joint1..7`.
7.  **`ResetScene` cleanup** (phase 3 decision 25 gains, Modify
    `phase_3.md` and `robot_service`): detach every attached object in
    `world` (`DetachObject`) and `Release` the gripper before
    `SimLink.Reset`.
8.  **Design doc** (Modify): G9 as above; Q5 answered (action
    `/grasp_planning/plan_grasps`, vendored pipeline); RobotService row
    `PlanGrasp` per decision 6; architecture diagram "grasp planner (ROS 2
    action)".

## Steps

1.  Decisions 2–5; rebuild the image; `planner_up`. If the pipeline
    fails to link or MTC/OMPL plugins fail to load (only compiled so far,
    Inv §8.3), stop and record it.
2.  Decision 6–7; unit tests `tests/test_plan_grasp.py` (Create; pure
    Python, no ROS imports at module level) with cases: cube at yaw 0 and
    yaw 30°, grasp pose composition; finger-axis extent exactly 0.075
    (kept) and 0.076 (dropped); plate rejected by step 2; depth for the
    cube (0.02) and for a 0.025 m tall box (0.015).
3.  `tests/e2e/phase5c_check.py` (Create) until it passes; write
    `phase_5c_notes.md` (Create): success rates, planning times.

## Constraints

As phase 3 (no `git commit`; Docker via `sg docker -c`; no `pkill -f`;
Bazel only via `container/build_intrinsic.sh`; Observability).

## Addendum A (after the first run, phase_5c_notes.md)

1.  **Frame:** the Franka base is at the world origin, so `robot_service`
    puts every `CollisionObject` in `header.frame_id = "base_link"` (same
    poses as before). No static TF is added; remove any diagnostic one.
2.  **Table vs. robot base:** in the MoveIt scene only, a static object
    whose top is within 0.005 m of z = 0 (the robot base plane) gets its
    box lowered by 0.005 m (dims and pose unchanged otherwise), so the
    robot's start state is not in collision with it. Intrinsic's `world`
    is unchanged. Log it once per object.
3.  `tests/e2e/phase3_cuj_check.py` `cuj4`: stop a `MoveJoints` whose
    planned duration is ≥ 3 s (e.g. joint 1 by 1.5 rad), issued 1 s after
    the start.
Then rerun Step 3; all done criteria must pass.

## Addendum B (blocker 3, phase_5c_notes.md)

1.  Table lowering margin 0.005 → **0.0005 m** (the base link mesh dips
    0.03 mm below z = 0; a bigger margin lets MoveIt accept poses inside
    the table).
2.  Before choosing (decision 6.7), keep only grasps whose `grasp_joints`
    and `pregrasp_ik_solution` are collision-free in Intrinsic's world
    (`MotionPlannerService.CheckCollisions`, as `RobotService.CheckCollisions`
    does), so MoveIt's and Intrinsic's collision models cannot disagree.
    None left → `ABORTED` "no grasp is collision-free in the robot's
    world model". Unit test for the filter with a fake checker.
Rerun `phase5c_check.py` (all pass) and the phase 3/4 checks.
