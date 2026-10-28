"""move_group and the grasp planner for the Isaac Franka (docs/designs/phases/phase_5c.md decision 3).

No robot_state_publisher and no static TF: move_group takes the robot state from Isaac's /joint_states,
and Isaac already publishes /tf.
"""

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
  moveit_config = (
      MoveItConfigsBuilder("franka", package_name="franka_grasp_config")
      .robot_description(file_path="urdf/franka.urdf")
      .robot_description_semantic(file_path="srdf/franka.srdf")
      .robot_description_kinematics(file_path="config/kinematics.yaml")
      .joint_limits(file_path="config/joint_limits.yaml")
      .planning_pipelines(pipelines=["ompl"])
      .to_moveit_configs()
  )
  config = moveit_config.to_dict()
  # Either node exiting ends the launch, and so the container (entrypoint.sh: any exit stops it).
  return LaunchDescription([
      Node(
          package="moveit_ros_move_group",
          executable="move_group",
          output="screen",
          parameters=[config, {"allow_trajectory_execution": False, "use_sim_time": True}],
          on_exit=Shutdown(),
      ),
      Node(
          package="grasp_planner",
          executable="grasp_planner",
          output="screen",
          parameters=[config, {"use_sim_time": True}],
          on_exit=Shutdown(),
      ),
  ])
