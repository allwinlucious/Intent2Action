# Phase 5c: grasp planning with Intrinsic's MoveIt grasp planner

Part of [robot_platform.md](../robot_platform.md) (G9, Q5). Facts:
[phase_5c_inventory.md](phase_5c_inventory.md) ("Inv §n"). Requires
phase 3 done. Runs after phase 4 (shares Isaac and the container).

## Goal and done criteria

`tests/e2e/phase5c_check.py`:

| Name | Condition |
|---|---|
| `planner_up` | `move_group` and `grasp_planner` run in the container; `/grasp_planning/plan_grasps` action server is available [120 s] |
| `cube` | 10 trials (`ResetScene` before each): `PlanGrasp("red_cube")` → `MoveToPose(pregrasp)` → `MoveToPose(grasp, LINEAR)` → `Grasp` → `AttachObject` → `MoveToPose(grasp + 0.10 m z, LINEAR)`; success = Isaac's true `red_cube` z rose ≥ 0.08 m and `IsGripping`; ≥ 9/10 (G9) |
| `peg` | the same for `peg`; ≥ 9/10 |
| `too_wide` | `PlanGrasp("plate")` returns `ABORTED` with a message naming the gripper's 0.075 m limit |
| `logs` | `robot_service.log` has one line per `PlanGrasp` with status and duration |

G9 in the design is narrowed to graspable catalog objects (`red_cube`,
`peg`); the plate (0.18 m) is wider than the open gripper on every side.

## Decisions

1.  **What runs:** a Humble build of the planner's Intrinsic-free core
    (the grasp pipeline, Inv §8.2) with a new `main`, next to MoveIt's
    `move_group`, both on CycloneDDS (same vendor as `robot_service`'s
    `rclpy`, design "DDS per process"). Intrinsic's own node does not
    build on Humble (Inv §8.4).
2.  **Sources** (vendored, Apache-2.0 headers kept): from
    `https://github.com/intrinsic-ai/intrinsic-moveit` tag `v0.0.4` copy
    `moveit_planning_interfaces/` (unchanged) and the three pipeline
    sources + headers (`grasp_planning_pipeline.{hpp,cpp}`,
    `generate_box_grasp_poses.{hpp,cpp}`, `object_geometry.{hpp,cpp}`)
    into `ros_ws/src/moveit_planning_interfaces/` and
    `ros_ws/src/grasp_planner/` (Create). Include fix: the three MoveIt
    `.hpp` includes → `.h` (Inv §8.3). `ros_ws/src/grasp_planner/src/main.cpp`
    (Create): node `grasp_planner`, `/get_planning_scene` client,
    `GraspPlanningPipeline`, action server `grasp_planning/plan_grasps`
    as `moveit_planning_node.cpp:637-721` does, with a
    `rclcpp::executors::MultiThreadedExecutor` and without the World
    synchronizer; parameter `expect_collision_objects` true. Logs per
    goal: target, candidates, result count, duration (Observability).
3.  **Franka MoveIt config** `ros_ws/src/franka_grasp_config/` (Create):
    *   URDF: `assets/robots/franka/lula_franka_gen.urdf` (copied at build;
        the same model as everywhere, G3).
    *   SRDF: virtual joint `world`→`base_link` **fixed**; group
        `panda_arm` chain `base_link`→`panda_link8`; group `hand` joints
        `panda_finger_joint1`, `panda_finger_joint2` (passive);
        `end_effector` `hand`, parent link `panda_link8`, parent group
        `panda_arm`; group states on `hand`: `open` (0.04, 0.04), `closed`
        (0.0, 0.0); `disable_collisions` copied from
        `ros-humble-moveit-resources-panda-moveit-config`'s `panda.srdf`
        (BSD) for adjacent/never-colliding link pairs.
    *   Tool frame: `right_gripper` (0.1 m along `panda_link8` z, the
        same frame as Intrinsic's `tcp`); its +z is the approach axis
        (Inv §3.6).
    *   `kinematics.yaml`: KDL for `panda_arm`; `ompl_planning.yaml` from
        the apt Panda config; `joint_limits.yaml` from the URDF.
    *   `launch/grasp_planner.launch.py`: `robot_state_publisher`,
        static TF `world`→`base_link` identity, `move_group`,
        `grasp_planner`. `/joint_states` comes from Isaac (cross-vendor
        topic; joint names match the URDF).
4.  **Image** (`container/Dockerfile`, Modify): apt
    `ros-humble-moveit ros-humble-moveit-task-constructor-core
    ros-humble-moveit-configs-utils ros-humble-moveit-msgs
    ros-humble-moveit-resources-panda-moveit-config`; `colcon build` of
    `ros_ws/src` into `/opt/ros_ext` (with the existing
    `simulation_interfaces` build). Entrypoint (Modify): start the launch
    file after `scene_sync` with CycloneDDS
    (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI`), ROS
    sourced under `set +u`.
5.  **Object dimensions:** `tools/build_catalog_meshes.py` (Modify) writes
    `dims: [x, y, z]` (bounding box, m) into each `object.yaml`; the image
    copies them (already under `assets/objects`).
6.  **`robot_service.PlanGrasp`** (replaces `UNIMPLEMENTED`):
    1.  Look up the object in `world` (`NOT_FOUND` if absent) and its
        catalog `dims` (via `scene_sync`'s catalog map — `SimLink` gains
        `GetCatalog(name) → catalog` in `simlink.proto`, Modify).
    2.  `/apply_planning_scene`: add every `world` object except the robot,
        hand, `d455` and attached objects as a BOX collision object (id =
        name, pose = its world pose with the box centred at
        `dims.z / 2` above the bottom-centre origin); remove MoveIt scene
        objects no longer in `world`.
    3.  `PlanGrasps` goal: `group_name panda_arm`, `end_effector_group
        hand`, `tool_frame right_gripper`, `planning_timeout_sec 10`,
        `gripper_motion_duration_sec 0.5`, `retract_dist_m 0.08`, surfaces
        0–5, `num_rotations 4`, `max_num_grasps 8`, one candidate with
        `target.id` = name, `obj_dims_in_meters` = dims,
        `obj_t_obj_center` = (0, 0, dims.z/2).
    4.  Filter: keep grasps whose object extent along the tool y axis
        (the finger axis; `right_gripper` = Isaac `panda_hand` rotated π
        about z) is ≤ 0.075 m. None left → `ABORTED` "object wider than
        the gripper opening (0.075 m) on every approach". Planner failure
        → `ABORTED` with its `error_code`.
    5.  Return the best remaining grasp (highest `grasp_quality`):
        `grasp` and `pregrasp` = object pose ∘ object_T_tool (base frame,
        TCP = `right_gripper` = Intrinsic `tcp`), `grasp_joints` from
        `grasp_ik_solution` ordered `panda_joint1..7`.
7.  **Design doc** (Modify): G9 narrowed (above); RobotService table row
    `PlanGrasp` describes steps 6.2–6.5.

## Steps

1.  Decisions 2–5; rebuild the image; `planner_up`.
2.  Decision 6; unit tests `tests/test_plan_grasp.py` (Create: box pose
    conversion, the width filter, frame composition).
3.  `tests/e2e/phase5c_check.py` (Create) until it passes; write
    `phase_5c_notes.md` (Create) with success rates and planning times.
4.  If the pipeline fails to link or run on Humble (only compiled so far,
    Inv §8.3), stop and record it.

## Constraints

As phase 3 (no `git commit`; Docker via `sg docker -c`; no `pkill -f`;
Bazel only via `container/build_intrinsic.sh`; Observability).
