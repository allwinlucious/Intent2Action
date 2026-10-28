cmake_minimum_required(VERSION 3.8)
project(grasp_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_planning_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(grasp_planner
  src/main.cpp
  src/grasp_planning_pipeline.cpp
  src/generate_box_grasp_poses.cpp
  src/object_geometry.cpp
)
target_compile_features(grasp_planner PUBLIC cxx_std_17)
target_include_directories(grasp_planner PRIVATE include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(grasp_planner
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_planning_interfaces
  geometry_msgs
  sensor_msgs
)

install(TARGETS grasp_planner DESTINATION lib/${PROJECT_NAME})

ament_package()
