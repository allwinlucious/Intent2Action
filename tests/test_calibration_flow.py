"""Unit tests of the CalibrateCamera flow (robot/calibration.py run) with a fake arm and CalibrationService."""

from pathlib import Path
import sys

import grpc
from intrinsic.perception.proto.v1 import calibration_service_pb2 as calib_pb
import pytest
from robot_service import robot_service_pb2 as api

# Appended: container/services/robot_service.py must not shadow the generated package robot_service.
sys.path.append(str(Path(__file__).resolve().parents[1] / "container/services"))
from robot import calibration  # noqa: E402
from robot.ops import ApiError  # noqa: E402

STATUS = calib_pb.CaptureDataResponse.CaptureDataStatus
CAMERA = api.Pose(x=1.33, z=0.9, qw=1.0)


def _pose(i: int) -> api.Pose:
  return api.Pose(x=0.5 + i * 1e-3, z=0.5, qw=1.0)


class FakeArm:
  """Records calls; `infeasible` and `failing_moves` hold sample indices (pose x encodes the index)."""

  def __init__(self, attached=True, samples=40, infeasible=(), failing_moves=(), start_fails=False):
    self.attached, self.samples = attached, samples
    self.infeasible, self.failing_moves, self.start_fails = set(infeasible), set(failing_moves), start_fails
    self.moves, self.details, self.calls = [], [], []

  @staticmethod
  def index(pose: api.Pose) -> int:
    return round((pose.x - 0.5) * 1e3)

  def board_attached(self):
    return self.attached

  def camera_pose(self):
    return CAMERA

  def move(self, pose, detail):
    self.moves.append(pose)
    if detail == "moving to the start pose":
      if self.start_fails:
        raise ApiError(grpc.StatusCode.NOT_FOUND, "planning failed: IK solver couldn't find any solutions")
      return
    if self.index(pose) in self.failing_moves:
      raise ApiError(grpc.StatusCode.ABORTED, "planning failed: no path")

  def sample(self, start, camera):
    assert camera == CAMERA
    return [_pose(i) for i in range(self.samples)]

  def feasible(self, pose):
    return self.index(pose) not in self.infeasible

  def wait_synced(self):
    return True

  def report(self, detail):
    self.details.append(detail)

  def mark_calibrated(self):
    self.calls.append("mark_calibrated")

  def home(self):
    self.calls.append("home")


class FakeService:
  """CaptureData returns `statuses` in order; Calibrate returns one stationary result."""

  def __init__(self, statuses):
    self.statuses, self.calls = list(statuses), []

  def Initialize(self, request, timeout):
    self.calls.append("Initialize")
    return calib_pb.InitializeResponse()

  def CaptureData(self, request, timeout):
    self.calls.append("CaptureData")
    return calib_pb.CaptureDataResponse(capture_data_status=[self.statuses.pop(0)])

  def Calibrate(self, request, timeout):
    assert request.calibrate_camera_to_robot
    self.calls.append("Calibrate")
    result = calib_pb.CalibrationResult()
    solved = result.camera_to_robot_calibration_results.add(
        translation_root_mean_square_error=4e-4, rotation_root_mean_square_error_in_degrees=0.05)
    solved.stationary_camera_result_poses.base_t_camera.position.x = 1.3
    solved.stationary_camera_result_poses.base_t_camera.orientation.w = 1.0
    return result

  def Save(self, request, timeout):
    self.calls.append("Save")


def test_board_not_attached_is_failed_precondition():
  arm, service = FakeArm(attached=False), FakeService([])
  with pytest.raises(ApiError) as error:
    calibration.run(arm, service)
  assert error.value.code == grpc.StatusCode.FAILED_PRECONDITION
  assert error.value.message == "calibration board not attached"
  assert arm.moves == [] and service.calls == []


def test_start_pose_failure_is_aborted_with_the_planner_message():
  arm, service = FakeArm(start_fails=True), FakeService([])
  with pytest.raises(ApiError) as error:
    calibration.run(arm, service)
  assert error.value.code == grpc.StatusCode.ABORTED
  assert "IK solver couldn't find any solutions" in error.value.message
  assert service.calls == []


def test_keeps_the_first_20_feasible_poses_and_skips_failed_moves():
  arm = FakeArm(infeasible={1, 4}, failing_moves={2, 7})
  service = FakeService([STATUS.CAPTURE_DATA_STATUS_NO_ERRORS] * 18)
  result = calibration.run(arm, service)
  moved = [FakeArm.index(p) for p in arm.moves[1:]]  # after the start pose
  kept = [i for i in range(40) if i not in {1, 4}][:20]
  assert moved == kept  # every kept pose was tried in order, infeasible ones never
  assert service.calls == ["Initialize"] + ["CaptureData"] * 18 + ["Calibrate", "Save"]
  assert arm.calls == ["mark_calibrated", "home"]
  assert "capture 2/20: skipped (move failed)" in arm.details
  assert "capture 20/20: CAPTURE_DATA_STATUS_NO_ERRORS" in arm.details
  assert result.poses_used == 18
  assert (result.rms_translation_m, result.rms_rotation_deg, result.camera_pose.x) == (4e-4, 0.05, 1.3)


def test_below_12_detections_is_aborted_without_calibrating():
  statuses = [STATUS.CAPTURE_DATA_STATUS_NO_ERRORS] * 11 + [STATUS.CAPTURE_DATA_STATUS_NO_DETECTIONS] * 9
  arm, service = FakeArm(), FakeService(statuses)
  with pytest.raises(ApiError) as error:
    calibration.run(arm, service)
  assert error.value.code == grpc.StatusCode.ABORTED
  assert error.value.message == "only 11 board detections (need 12)"
  assert "Calibrate" not in service.calls and arm.calls == []


def test_cancel_during_a_move_is_not_skipped():
  class CancelledArm(FakeArm):
    def move(self, pose, detail):
      if detail.startswith("capture 3/"):
        raise ApiError(grpc.StatusCode.CANCELLED, "stopped")
      super().move(pose, detail)

  arm = CancelledArm()
  with pytest.raises(ApiError) as error:
    calibration.run(arm, FakeService([STATUS.CAPTURE_DATA_STATUS_NO_ERRORS] * 20))
  assert error.value.code == grpc.StatusCode.CANCELLED
