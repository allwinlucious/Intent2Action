#include "ext/calib_poses/sample_poses.h"

#include <cmath>
#include <string>

#include "gtest/gtest.h"

namespace calib_poses {
namespace {

// Decision 8 values: start pose (decision 6) with the board facing the
// camera's seed position, board 0.12 m along the hand's z after Rz(-pi/4).
constexpr char REQUEST[] = R"({
  "base_t_flange": {"position": [0.6, 0.0, 0.5],
                    "quaternion_xyzw": [0.0, 0.7071067811865476, 0.0, 0.7071067811865476]},
  "flange_t_board": {"position": [0.0, 0.0, 0.12],
                     "quaternion_xyzw": [0.0, 0.0, -0.3826834323650898, 0.9238795325112867]},
  "base_t_camera": {"position": [1.33, 0.0, 0.9], "quaternion_xyzw": [0, 0, 0, 1]},
  "halfsize": [0.08, 0.12, 0.08], "tilt_deg": 15, "roll_deg": 20,
  "count": 200, "seed": 0})";

TEST(SamplePoses, BoardFacesTheCameraAndPosesDiffer) {
  absl::StatusOr<Request> request = parseRequest(REQUEST);
  ASSERT_TRUE(request.ok()) << request.status();
  const std::vector<intrinsic::Pose3d> poses = samplePoses(*request);
  ASSERT_EQ(poses.size(), 200);
  for (size_t i = 0; i < poses.size(); ++i) {
    const intrinsic::Pose3d board = poses[i] * request->flange_t_board;
    ASSERT_TRUE(board.translation().allFinite()) << i;
    ASSERT_TRUE(board.quaternion().coeffs().allFinite()) << i;
    const intrinsic::eigenmath::Vector3d to_camera =
        request->base_t_camera.translation() - board.translation();
    EXPECT_GT(board.rotationMatrix().col(2).dot(to_camera), 0.0) << i;
    if (i > 0) {
      EXPECT_GT((poses[i].translation() - poses[i - 1].translation()).norm(),
                1e-6)
          << i;
    }
  }
}

TEST(SamplePoses, SameSeedSamePoses) {
  absl::StatusOr<Request> request = parseRequest(REQUEST);
  ASSERT_TRUE(request.ok());
  EXPECT_TRUE(samplePoses(*request)[7].isApprox(samplePoses(*request)[7]));
}

TEST(ParseRequest, RejectsBadInput) {
  EXPECT_FALSE(parseRequest("not json").ok());
  EXPECT_FALSE(parseRequest("{}").ok());
  std::string zero_quaternion = REQUEST;
  zero_quaternion.replace(zero_quaternion.find("[0, 0, 0, 1]"), 12,
                          "[0, 0, 0, 0]");
  EXPECT_FALSE(parseRequest(zero_quaternion).ok());
  std::string no_count = REQUEST;
  no_count.replace(no_count.find("\"count\": 200"), 12, "\"count\": 0  ");
  EXPECT_FALSE(parseRequest(no_count).ok());
}

}  // namespace
}  // namespace calib_poses
