#ifndef EXT_CALIB_POSES_SAMPLE_POSES_H_
#define EXT_CALIB_POSES_SAMPLE_POSES_H_

// Calibration board poses from Intrinsic's pose_sampling library
// (phase_4.md decision 8). JSON in, JSON out, so robot_service (Python) can
// use the C++-only library through the sample_poses CLI.

#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/perception/calibration/pose_sampling.h"
#include "nlohmann/json.hpp"

namespace calib_poses {

struct Request {
  intrinsic::Pose3d base_t_flange;
  intrinsic::Pose3d flange_t_board;
  intrinsic::Pose3d base_t_camera;
  intrinsic::perception::SampleRandomPoseParams params;
  int count = 0;
  uint64_t seed = 0;
};

namespace internal {

inline absl::StatusOr<std::vector<double>> numberArray(const nlohmann::json& value,
                                                   const std::string& name,
                                                   size_t size) {
  if (!value.is_array() || value.size() != size) {
    return absl::InvalidArgumentError(
        absl::StrCat(name, " must be an array of ", size, " numbers"));
  }
  std::vector<double> out;
  for (const nlohmann::json& v : value) {
    if (!v.is_number() || !std::isfinite(v.get<double>())) {
      return absl::InvalidArgumentError(
          absl::StrCat(name, " must hold finite numbers"));
    }
    out.push_back(v.get<double>());
  }
  return out;
}

inline absl::StatusOr<intrinsic::Pose3d> pose(const nlohmann::json& request,
                                              const std::string& name) {
  if (!request.contains(name) || !request[name].is_object()) {
    return absl::InvalidArgumentError(absl::StrCat(name, " is missing"));
  }
  const nlohmann::json& p = request[name];
  absl::StatusOr<std::vector<double>> position =
      numberArray(p.value("position", nlohmann::json()), name + ".position", 3);
  if (!position.ok()) return position.status();
  absl::StatusOr<std::vector<double>> q = numberArray(
      p.value("quaternion_xyzw", nlohmann::json()), name + ".quaternion_xyzw", 4);
  if (!q.ok()) return q.status();
  const double norm =
      std::sqrt((*q)[0] * (*q)[0] + (*q)[1] * (*q)[1] + (*q)[2] * (*q)[2] +
                (*q)[3] * (*q)[3]);
  if (norm < 1e-9) {
    return absl::InvalidArgumentError(
        absl::StrCat(name, ".quaternion_xyzw must not be zero"));
  }
  return intrinsic::Pose3d(
      intrinsic::eigenmath::Quaterniond((*q)[3], (*q)[0], (*q)[1], (*q)[2]),
      intrinsic::eigenmath::Vector3d((*position)[0], (*position)[1],
                                     (*position)[2]));
}

inline absl::StatusOr<double> number(const nlohmann::json& request,
                                     const std::string& name) {
  if (!request.contains(name) || !request[name].is_number() ||
      !std::isfinite(request[name].get<double>())) {
    return absl::InvalidArgumentError(
        absl::StrCat(name, " must be a finite number"));
  }
  return request[name].get<double>();
}

}  // namespace internal

// Parses {"base_t_flange": P, "flange_t_board": P, "base_t_camera": P,
// "halfsize": [x,y,z], "tilt_deg": d, "roll_deg": d, "count": n, "seed": s},
// P = {"position": [x,y,z], "quaternion_xyzw": [x,y,z,w]}.
inline absl::StatusOr<Request> parseRequest(const std::string& text) {
  const nlohmann::json json =
      nlohmann::json::parse(text, nullptr, /*allow_exceptions=*/false);
  if (json.is_discarded() || !json.is_object()) {
    return absl::InvalidArgumentError("input is not a JSON object");
  }
  Request request;
  for (auto [name, target] :
       {std::pair{"base_t_flange", &request.base_t_flange},
        std::pair{"flange_t_board", &request.flange_t_board},
        std::pair{"base_t_camera", &request.base_t_camera}}) {
    absl::StatusOr<intrinsic::Pose3d> p = internal::pose(json, name);
    if (!p.ok()) return p.status();
    *target = *p;
  }
  absl::StatusOr<std::vector<double>> halfsize = internal::numberArray(
      json.value("halfsize", nlohmann::json()), "halfsize", 3);
  if (!halfsize.ok()) return halfsize.status();
  absl::StatusOr<double> tilt = internal::number(json, "tilt_deg");
  if (!tilt.ok()) return tilt.status();
  absl::StatusOr<double> roll = internal::number(json, "roll_deg");
  if (!roll.ok()) return roll.status();
  if (!json.contains("count") || !json["count"].is_number_integer() ||
      json["count"].get<int>() < 1) {
    return absl::InvalidArgumentError("count must be an integer >= 1");
  }
  if (!json.contains("seed") || !json["seed"].is_number_unsigned()) {
    return absl::InvalidArgumentError("seed must be an integer >= 0");
  }
  request.params.sample_box_halfsize = intrinsic::eigenmath::Vector3d(
      (*halfsize)[0], (*halfsize)[1], (*halfsize)[2]);
  request.params.rotation_randomization_angle_radians = *tilt * std::numbers::pi / 180.0;
  request.params.rotation_randomization_roll_angle_radians =
      *roll * std::numbers::pi / 180.0;
  request.count = json["count"].get<int>();
  request.seed = json["seed"].get<uint64_t>();
  return request;
}

// `count` sampled base_t_flange poses; the same seed gives the same poses.
inline std::vector<intrinsic::Pose3d> samplePoses(const Request& request) {
  std::mt19937_64 bitgen(request.seed);
  std::vector<intrinsic::Pose3d> poses;
  poses.reserve(request.count);
  for (int i = 0; i < request.count; ++i) {
    poses.push_back(intrinsic::perception::SampleRandomPose(
        request.base_t_flange, request.flange_t_board, request.base_t_camera,
        request.params, bitgen));
  }
  return poses;
}

inline nlohmann::json poseJson(const intrinsic::Pose3d& pose) {
  const auto& q = pose.quaternion();
  const auto& t = pose.translation();
  return {{"position", {t.x(), t.y(), t.z()}},
          {"quaternion_xyzw", {q.x(), q.y(), q.z(), q.w()}}};
}

}  // namespace calib_poses

#endif  // EXT_CALIB_POSES_SAMPLE_POSES_H_
