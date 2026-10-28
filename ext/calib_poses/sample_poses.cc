// sample_poses: reads a JSON request on stdin, writes {"poses": [P, ...]}
// (sampled base_t_flange poses) to stdout; exit 1 with a message on stderr on
// bad input. See sample_poses.h for the format (phase_4.md decision 8).

#include <iostream>
#include <iterator>
#include <string>

#include "absl/status/statusor.h"
#include "ext/calib_poses/sample_poses.h"
#include "nlohmann/json.hpp"

int main() {
  const std::string input((std::istreambuf_iterator<char>(std::cin)),
                          std::istreambuf_iterator<char>());
  absl::StatusOr<calib_poses::Request> request =
      calib_poses::parseRequest(input);
  if (!request.ok()) {
    std::cerr << "sample_poses: " << request.status().message() << "\n";
    return 1;
  }
  nlohmann::json poses = nlohmann::json::array();
  for (const intrinsic::Pose3d& pose : calib_poses::samplePoses(*request)) {
    poses.push_back(calib_poses::poseJson(pose));
  }
  std::cout << nlohmann::json{{"poses", poses}}.dump() << "\n";
  return 0;
}
