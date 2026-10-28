# sample_poses: CLI over Intrinsic's pose_sampling library for robot_service's
# calibration flow (docs/designs/phases/phase_4.md, decision 8).
# Built as //ext/calib_poses through the ~/intrinsic-core/ext symlink (container/build_intrinsic.sh).

load("//bazel:cc_macros.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:private"])

cc_library(
    name = "sample_poses_lib",
    hdrs = ["sample_poses.h"],
    deps = [
        "//intrinsic_perception/intrinsic/perception/calibration:pose_sampling",
        "//intrinsic_sdk/intrinsic/eigenmath",
        "//intrinsic_sdk/intrinsic/math:pose3",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
        "@com_github_nlohmann_json//:json",
    ],
)

cc_binary(
    name = "sample_poses",
    srcs = ["sample_poses.cc"],
    deps = [
        ":sample_poses_lib",
        "@abseil-cpp//absl/status:statusor",
        "@com_github_nlohmann_json//:json",
    ],
)

cc_test(
    name = "sample_poses_test",
    srcs = ["sample_poses_test.cc"],
    deps = [
        ":sample_poses_lib",
        "@com_google_googletest//:gtest_main",
    ],
)
