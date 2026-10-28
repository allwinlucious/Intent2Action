"""Pieces of the camera-to-robot calibration (phase_4.md decisions 6, 8 and 11.3), used by the Step 2
experiments (tests/e2e/exp4/exp4_cd.py). The CalibrateCamera flow itself (Step 3) is not implemented:
experiments c and d failed (phase_4_notes.md).
"""

import json
import math
import subprocess
from typing import Callable

import grpc
from intrinsic.perception.proto.v1 import calibration_service_pb2 as calib_pb
from intrinsic.perception.proto.v1 import camera_config_pb2
from intrinsic.perception.proto.v1 import charuco_pattern_pb2
from intrinsic.perception.proto.v1 import pattern_detection_config_pb2 as pattern_pb
from intrinsic.resources.proto import resource_handle_pb2
from intrinsic.world.proto import object_world_refs_pb2 as refs_pb
import numpy as np
from robot.ops import ApiError
from robot_model.load_camera import quaternion_wxyz
from robot_service import robot_service_pb2 as api

SAMPLE_POSES = "/opt/intrinsic/bin/sample_poses"
CAMERA = "d455"
CAMERA_ADDRESS = "localhost:18005"  # camera_sim, as the CalibrationService reaches it
ROBOT = "franka"
NO_ERRORS = "CAPTURE_DATA_STATUS_NO_ERRORS"
START_FLANGE_M = (0.60, 0.0, 0.50)  # base frame
# The board sits 0.12 m along Isaac's panda_hand z; panda_hand is the flange turned -pi/4 about z.
FLANGE_T_BOARD = api.Pose(z=0.12, qz=-math.sin(math.pi / 8), qw=math.cos(math.pi / 8))
SAMPLING = {"halfsize": [0.08, 0.12, 0.08], "tilt_deg": 15.0, "roll_deg": 20.0, "count": 200, "seed": 0}


def initialize_request() -> calib_pb.InitializeRequest:
  """Decision 11.3: the 7 x 5 ChArUco board (decision 5), camera d455 at camera_sim, arm franka."""
  config = camera_config_pb2.CameraConfig()
  config.identifier.ros.device_id = CAMERA
  handle = resource_handle_pb2.ResourceHandle(name=CAMERA)
  handle.connection_info.grpc.address = CAMERA_ADDRESS
  handle.resource_data["CameraConfig"].contents.Pack(config)
  pattern = charuco_pattern_pb2.CharucoPattern(squares_x=7, squares_y=5, square_length=0.04, marker_length=0.03,
                                               dictionary=charuco_pattern_pb2.DICT_5X5_100)
  return calib_pb.InitializeRequest(
      pattern_detection_config=pattern_pb.PatternDetectionConfig(
          name="charuco", publish_annotated_image=True,
          charuco_pattern_detection_config=pattern_pb.CharucoPatternDetectionConfig(charuco_pattern=pattern)),
      camera_resource_handles=[handle],
      arm_part=refs_pb.ObjectReference(by_name=refs_pb.ObjectReferenceByName(object_name=ROBOT)))


def status_name(response: calib_pb.CaptureDataResponse) -> str:
  if len(response.capture_data_status) != 1:
    raise ApiError(grpc.StatusCode.INTERNAL, f"CaptureData returned {len(response.capture_data_status)} statuses for 1 camera")
  return calib_pb.CaptureDataResponse.CaptureDataStatus.Name(response.capture_data_status[0])


def start_pose(camera_position) -> api.Pose:
  """Decision 6: flange at START_FLANGE_M, z toward the camera, x perpendicular with no world-z part."""
  origin = np.array(START_FLANGE_M)
  z = np.asarray(camera_position, dtype=float) - origin
  z /= np.linalg.norm(z)
  x = np.cross((0.0, 0.0, 1.0), z)
  x /= np.linalg.norm(x)
  w, qx, qy, qz = quaternion_wxyz(np.column_stack([x, np.cross(z, x), z]))
  return api.Pose(x=origin[0], y=origin[1], z=origin[2], qx=qx, qy=qy, qz=qz, qw=w)


def _pose_json(pose: api.Pose) -> dict:
  return {"position": [pose.x, pose.y, pose.z], "quaternion_xyzw": [pose.qx, pose.qy, pose.qz, pose.qw]}


def sample_poses(start: api.Pose, base_t_camera: api.Pose, run: Callable = subprocess.run) -> list[api.Pose]:
  """Decision 8: SAMPLING["count"] flange poses from the sample_poses CLI (Intrinsic's pose_sampling)."""
  request = {"base_t_flange": _pose_json(start), "flange_t_board": _pose_json(FLANGE_T_BOARD),
             "base_t_camera": _pose_json(base_t_camera), **SAMPLING}
  done = run([SAMPLE_POSES], input=json.dumps(request), capture_output=True, text=True, timeout=30)
  if done.returncode != 0:
    raise ApiError(grpc.StatusCode.INTERNAL, f"sample_poses exited {done.returncode}: {done.stderr.strip()}")
  poses = []
  for p in json.loads(done.stdout)["poses"]:
    (x, y, z), (qx, qy, qz, qw) = p["position"], p["quaternion_xyzw"]
    poses.append(api.Pose(x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw))
  return poses
