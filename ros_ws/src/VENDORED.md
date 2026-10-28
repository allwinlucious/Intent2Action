# Vendored sources

From <https://github.com/intrinsic-ai/intrinsic-moveit>, tag `v0.0.4`, commit
`c117cc5e911c30cea7310cb9ceb854fa6f2af343` (Apache-2.0; licence headers kept).

| Here | Upstream | Change |
|---|---|---|
| `moveit_planning_interfaces/` | `moveit_planning_interfaces/` | none |
| `grasp_planner/include/moveit_planning_service/{grasp_planning_pipeline,generate_box_grasp_poses,object_geometry}.hpp` | `moveit_planning_service/include/moveit_planning_service/` | Humble MoveIt includes: `moveit/robot_model/robot_model.hpp`, `moveit/robot_model_loader/robot_model_loader.hpp`, `moveit/planning_scene/planning_scene.hpp` → `.h` |
| `grasp_planner/src/{grasp_planning_pipeline,generate_box_grasp_poses,object_geometry}.cpp` | `moveit_planning_service/src/` | the same include change |
| `grasp_planner/src/main.cpp` | `moveit_planning_service/src/moveit_planning_node.cpp` | new file: the goal, cancel, accept and execute handlers (`:50-198`) and the node set-up (`:637-721`) ported to Humble (`MultiThreadedExecutor`, parameters declared from overrides), without the Intrinsic World synchronizer, the motion-plan placeholder or any Intrinsic library; one log line per goal |

`franka_grasp_config/srdf/franka.srdf`'s `disable_collisions` and
`franka_grasp_config/config/ompl_planning.yaml` come from apt
`ros-humble-moveit-resources-panda-moveit-config` (BSD-3-Clause).
