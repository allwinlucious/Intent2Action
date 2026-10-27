# ros2_hwm: ICON hardware module for the Isaac Franka (docs/designs/phases/phase_3.md, Part 1).
# Built as //ext/ros2_hwm through the ~/intrinsic-core/ext symlink (container/build_intrinsic.sh).

load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("//bazel:cc_macros.bzl", "cc_library", "cc_test")
load("//bazel:protos.bzl", "proto_library")
load("//intrinsic_control/intrinsic/icon/hal/bzl:hardware_module_binary.bzl", "hardware_module_binary")

package(default_visibility = ["//visibility:private"])

proto_library(
    name = "ros2_hwm_config_proto",
    srcs = ["ros2_hwm_config.proto"],
)

cc_proto_library(
    name = "ros2_hwm_config_cc_proto",
    deps = [":ros2_hwm_config_proto"],
)

cc_library(
    name = "ros2_hwm_logic",
    srcs = ["ros2_hwm_logic.cc"],
    hdrs = ["ros2_hwm_logic.h"],
)

cc_test(
    name = "ros2_hwm_logic_test",
    srcs = ["ros2_hwm_logic_test.cc"],
    deps = [
        ":ros2_hwm_logic",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ros2_hwm",
    srcs = ["ros2_hwm.cc"],
    hdrs = ["ros2_hwm.h"],
    copts = ["-fexceptions"],  # ROS can throw exceptions.
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ros2_hwm_config_cc_proto",
        ":ros2_hwm_logic",
        "//intrinsic/util/status:status_macros",
        "//intrinsic_control/intrinsic/icon/control:realtime_clock_interface",
        "//intrinsic_control/intrinsic/icon/control/safety:safety_messages_fbs_cc",
        "//intrinsic_control/intrinsic/icon/control/safety:safety_messages_fbs_utils",
        "//intrinsic_control/intrinsic/icon/control/safety/extern:safety_status_fbs_cc",
        "//intrinsic_control/intrinsic/icon/hal:default_hardware_interfaces",
        "//intrinsic_control/intrinsic/icon/hal:hardware_interface_handle",
        "//intrinsic_control/intrinsic/icon/hal:hardware_interface_registry",
        "//intrinsic_control/intrinsic/icon/hal:hardware_interface_traits",
        "//intrinsic_control/intrinsic/icon/hal:hardware_module_init_context",
        "//intrinsic_control/intrinsic/icon/hal:hardware_module_interface",
        "//intrinsic_control/intrinsic/icon/hal:hardware_module_registry",
        "//intrinsic_control/intrinsic/icon/hal:module_config",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:force_torque_fbs_cc",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:force_torque_fbs_utils",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:gripper_fbs_cc",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:gripper_fbs_utils",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:joint_command_fbs_cc",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:joint_command_fbs_utils",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:joint_state_fbs_cc",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:joint_state_fbs_utils",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:payload_command_fbs_cc",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:payload_command_utils",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:payload_state_fbs_cc",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:payload_state_utils",
        "//intrinsic_control/intrinsic/icon/hal/interfaces:robot_payload_utils",
        "//intrinsic_control/intrinsic/icon/utils:async_buffer",
        "//intrinsic_control/intrinsic/icon/utils:core_time",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status_macro",
        "//intrinsic_sdk/intrinsic/world/robot_payload:robot_payload_base",
        "@abseil-cpp//absl/base:core_headers",
        "@abseil-cpp//absl/log",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/strings",
        "@abseil-cpp//absl/synchronization",
        "@abseil-cpp//absl/time",
        "@ros2_common_interfaces//:cpp_geometry_msgs",
        "@ros2_common_interfaces//:cpp_sensor_msgs",
        "@ros2_common_interfaces//:cpp_std_msgs",
        "@ros2_rcl_interfaces//:cpp_builtin_interfaces",
        "@ros2_rclcpp//:rclcpp",
    ],
    alwayslink = 1,
)

hardware_module_binary(
    name = "ros2_hwm_main",
    hardware_module_lib = ":ros2_hwm",
)
