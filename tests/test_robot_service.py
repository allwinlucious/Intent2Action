"""Unit tests of robot_service's helpers: operation table, LINEAR replan, arm chain, calibration poses, gripper
stall detector, PNG."""

import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import threading
import zlib

from google.rpc import code_pb2
from google.rpc import status_pb2
import grpc
from intrinsic.motion_planning.proto.v1 import motion_planner_service_pb2 as planner_pb
from intrinsic.motion_planning.proto.v1 import motion_planning_error_pb2 as error_pb
import numpy as np
import pytest
from robot_service import robot_service_pb2

# Appended: container/services/robot_service.py must not shadow the generated package robot_service.
sys.path.append(str(Path(__file__).resolve().parents[1] / "container/services"))
from robot import calibration  # noqa: E402
from robot import gripper  # noqa: E402
from robot import motion  # noqa: E402
from robot import ops  # noqa: E402
from robot import png  # noqa: E402


def _info(proto) -> robot_service_pb2.OperationInfo:
  info = robot_service_pb2.OperationInfo()
  proto.metadata.Unpack(info)
  return info


class TestOperationTable:

  def test_lifecycle_done(self):
    table = ops.OperationTable()
    op = table.create("MoveJoints")
    table.update(op, ops.EXECUTING, "executing", planned_duration_s=2.5)
    assert not table.to_proto(op).done
    table.finish(op, robot_service_pb2.MotionResult(duration_s=2.4))
    proto = table.to_proto(op)
    info = _info(proto)
    result = robot_service_pb2.MotionResult()
    proto.response.Unpack(result)
    assert proto.done and proto.name == op.name and not proto.HasField("error")
    assert (info.kind, info.state, info.planned_duration_s) == ("MoveJoints", ops.DONE, 2.5)
    assert result.duration_s == 2.4

  def test_fail_and_cancel_carry_code_and_message(self):
    table = ops.OperationTable()
    failed, cancelled = table.create("MoveToPose"), table.create("MoveJoints")
    table.fail(failed, ops.ApiError(grpc.StatusCode.ABORTED, "planning failed: unreachable"))
    table.fail(cancelled, ops.ApiError(grpc.StatusCode.CANCELLED, "stopped"))
    for op, state, code, message in (
        (failed, ops.FAILED, code_pb2.ABORTED, "planning failed: unreachable"),
        (cancelled, ops.CANCELLED, code_pb2.CANCELLED, "stopped"),
    ):
      proto = table.to_proto(op)
      assert proto.done and (proto.error.code, proto.error.message) == (code, message)
      assert (_info(proto).state, _info(proto).detail) == (state, message)

  def test_finished_operation_does_not_change(self):
    table = ops.OperationTable()
    op = table.create("Grasp")
    table.fail(op, ops.ApiError(grpc.StatusCode.CANCELLED, "stopped"))
    table.finish(op, robot_service_pb2.GripperResult())
    table.update(op, ops.EXECUTING, "late")
    assert (op.state, op.detail, op.response) == (ops.CANCELLED, "stopped", None)

  def test_cancelled_while_planning_is_done_and_rejects_updates(self):
    table = ops.OperationTable()
    op = table.create("MoveToPose")
    assert not op.done
    table.fail(op, ops.ApiError(grpc.StatusCode.CANCELLED, "stopped while planning"))  # what Stop does
    assert op.done and not op.worker_done.is_set()  # the worker may still be planning
    assert table.update(op, ops.EXECUTING, "executing") is False  # so the worker raises CANCELLED
    assert (op.state, op.detail) == (ops.CANCELLED, "stopped while planning")
    running = table.create("MoveJoints")
    assert table.update(running, ops.EXECUTING, "executing") is True and running.state == ops.EXECUTING

  def test_wait(self):
    table = ops.OperationTable()
    op = table.create("Home")
    assert not table.wait(op, 0.05)
    threading.Timer(0.05, lambda: table.finish(op, robot_service_pb2.MotionResult())).start()
    assert table.wait(op, 5)

  def test_get_unknown_and_delete_running(self):
    table = ops.OperationTable()
    op = table.create("Home")
    with pytest.raises(ops.ApiError) as unknown:
      table.get("operations/nope")
    with pytest.raises(ops.ApiError) as running:
      table.delete(op.name)
    assert unknown.value.code == grpc.StatusCode.NOT_FOUND and "operations/nope" in unknown.value.message
    assert running.value.code == grpc.StatusCode.FAILED_PRECONDITION

  def test_keeps_running_and_latest_finished(self):
    table = ops.OperationTable(keep_finished=2)
    running = table.create("MoveJoints")
    for _ in range(4):
      table.finish(table.create("Home"), robot_service_pb2.MotionResult())
    table.create("Home")
    names = [o.name for o in table.list()]
    assert running.name in names and len(names) == 4  # running, 2 finished, the new one


class TestArmChain:

  def test_link_origins_at_zero(self):
    chain = motion.ArmChain(Path(__file__).resolve().parents[1] / "assets/robots/franka/lula_franka_gen.urdf")
    origins = chain.link_origins([0.0] * 7)
    assert len(origins) == 8
    # Franka at zero: link8 (the flange) 0.088 m forward, 0.333 + 0.316 + 0.384 - 0.107 = 0.926 m up.
    assert np.allclose(origins[-1], (0.088, 0.0, 0.926), atol=1e-6)
    turned = chain.link_origins([math.pi / 2] + [0.0] * 6)
    assert np.allclose(turned[-1], (0.0, 0.088, 0.926), atol=1e-6)


class _LinearError(grpc.RpcError):
  """The planner's LINEAR failure: NOT_FOUND with the reached joints in grpc-status-details-bin."""

  def __init__(self, reached):
    error = error_pb.MotionPipelineError()
    found = error.motion_planning_error.add(segment_id=[0])
    found.linear_cartesian_path_planning_error.final_joint_positions.joints.extend(reached)
    status = status_pb2.Status(code=5, message="Could not generate Cartesian linear path")
    status.details.add(type_url="type.googleapis.com/linear_cartesian_path_planning_error",
                       value=error.SerializeToString())
    self._status = status

  def code(self):
    return grpc.StatusCode.NOT_FOUND

  def details(self):
    return self._status.message

  def trailing_metadata(self):
    return (("grpc-status-details-bin", self._status.SerializeToString()),)


class _FakePlannerStub:
  """Fails the first PlanTrajectory with a LINEAR error; ComputeFk returns `reached_pose`."""

  def __init__(self, reached_pose: robot_service_pb2.Pose):
    self.reached_pose = reached_pose
    self.requests = []

  def PlanTrajectory(self, request, timeout):
    self.requests.append(type(request).FromString(request.SerializeToString()))
    if len(self.requests) == 1:
      raise _LinearError([0.1] * 7)
    response = planner_pb.TrajectoryPlanningResponse()
    response.discretized.state.add(position=[0.1] * 7)
    return response

  def ComputeFk(self, request, timeout):
    return planner_pb.FkResponse(reference_t_target=motion.to_pose_proto(self.reached_pose))


class TestLinearReplan:
  TARGET = robot_service_pb2.Pose(x=0.4, y=0.1, z=0.3, qx=1.0)

  def _plan(self, reached_pose):
    planner = motion.Planner("localhost:1")
    planner.stub = _FakePlannerStub(reached_pose)
    waypoint = robot_service_pb2.Waypoint(pose=self.TARGET, motion=robot_service_pb2.LINEAR)
    return planner, planner.plan([0.0] * 7, [motion.segment(waypoint, robot_service_pb2.TCP)])

  def test_replans_when_reached_joints_are_at_the_pose(self):
    near = robot_service_pb2.Pose(x=0.4003, y=0.1, z=0.3, qx=1.0)  # 0.3 mm off
    planner, response = self._plan(near)
    replanned = planner.stub.requests[1].motion_specification.motion_segments[0]
    assert list(replanned.target.joint_position.joints) == [0.1] * 7
    assert replanned.motion_type == motion.spec_pb.MotionSegment.LINEAR
    assert len(response.discretized.state) == 1

  def test_aborts_when_the_path_stopped_short(self):
    short = robot_service_pb2.Pose(x=0.35, y=0.1, z=0.3, qx=1.0)  # blocked 5 cm before the target
    with pytest.raises(ops.ApiError) as error:
      self._plan(short)
    assert error.value.code == grpc.StatusCode.ABORTED
    assert "Could not generate Cartesian linear path" in error.value.message


class TestCalibrationPoses:

  CAMERA = (1.33, 0.0, 0.9)

  def test_start_pose_points_flange_z_at_the_camera(self):
    m = motion.pose_matrix(calibration.start_pose(self.CAMERA))
    toward = np.subtract(self.CAMERA, calibration.START_FLANGE_M)
    np.testing.assert_allclose(m[:3, 3], calibration.START_FLANGE_M)
    np.testing.assert_allclose(m[:3, 2], toward / np.linalg.norm(toward), atol=1e-12)
    assert abs(m[2, 0]) < 1e-12  # x has no world-z component

  def test_sample_poses_round_trips_the_cli_json(self):
    start = calibration.start_pose(self.CAMERA)
    sent = {}

    def fake_cli(args, input, **kwargs):  # pylint: disable=redefined-builtin
      sent.update(json.loads(input))
      out = {"poses": [{"position": [0.1, 0.2, 0.3], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}]}
      return subprocess.CompletedProcess(args, 0, json.dumps(out), "")

    poses = calibration.sample_poses(start, robot_service_pb2.Pose(x=1.33, z=0.9, qw=1.0), run=fake_cli)
    assert sent["count"] == 200 and sent["seed"] == 0 and sent["halfsize"] == [0.08, 0.12, 0.08]
    assert sent["base_t_flange"]["position"] == [start.x, start.y, start.z]
    assert sent["flange_t_board"]["position"] == [0.0, 0.0, 0.12]
    assert [(p.x, p.y, p.z, p.qw) for p in poses] == [(0.1, 0.2, 0.3, 1.0)]

  def test_sample_poses_cli_failure_is_internal_with_its_message(self):
    def failing_cli(args, **kwargs):
      return subprocess.CompletedProcess(args, 1, "", "sample_poses: count must be an integer >= 1\n")

    with pytest.raises(ops.ApiError) as error:
      calibration.sample_poses(robot_service_pb2.Pose(qw=1.0), robot_service_pb2.Pose(qw=1.0), run=failing_cli)
    assert error.value.code == grpc.StatusCode.INTERNAL
    assert "count must be an integer >= 1" in error.value.message


class TestStallDetector:

  def test_completes_when_width_stops_changing(self):
    detector = gripper.StallDetector(start_s=10.0)
    t, width, done_at = 10.0, 0.08, None
    while done_at is None:
      width = max(0.02, width - 0.1 * 0.01)  # closes at 0.1 m/s onto a 2 cm object
      if detector.update(t, width):
        done_at = t
      t += 0.01
    # The fingers reach 0.02 m at t = 10.59; a full 0.2 s window of no change is needed.
    assert 10.78 <= done_at <= 10.81

  def test_small_motion_counts_as_stopped(self):
    detector = gripper.StallDetector(start_s=0.0)
    results = [detector.update(i * 0.05, 0.03 + (i % 2) * 5e-5) for i in range(6)]
    assert results == [False, False, False, False, True, True]

  def test_times_out_after_3_s(self):
    detector = gripper.StallDetector(start_s=1.0)
    assert not detector.update(3.99, 0.05)
    assert not detector.update(3.995, 0.04)
    assert detector.update(4.0, 0.03)

  def test_gripping_and_release(self):
    assert gripper.gripping(True, 0.02) and not gripper.gripping(True, 0.004) and not gripper.gripping(False, 0.02)
    assert gripper.release_complete(0.0, 0.5, 0.0755) and gripper.release_complete(0.0, 3.0, 0.01)
    assert not gripper.release_complete(0.0, 2.9, 0.07)


def _decode_png(data: bytes) -> tuple[dict, bytes]:
  """IHDR fields and the unfiltered pixel bytes; checks signature and CRCs."""
  assert data[:8] == b"\x89PNG\r\n\x1a\n"
  pos, chunks = 8, []
  while pos < len(data):
    (length,) = struct.unpack(">I", data[pos:pos + 4])
    kind, body = data[pos + 4:pos + 8], data[pos + 8:pos + 8 + length]
    (crc,) = struct.unpack(">I", data[pos + 8 + length:pos + 12 + length])
    assert crc == zlib.crc32(kind + body)
    chunks.append((kind, body))
    pos += 12 + length
  assert [k for k, _ in chunks] == [b"IHDR", b"IDAT", b"IEND"]
  width, height, depth, color, *_ = struct.unpack(">IIBBBBB", chunks[0][1])
  raw = zlib.decompress(chunks[1][1])
  row = len(raw) // height
  assert all(raw[y * row] == 0 for y in range(height))
  pixels = b"".join(raw[y * row + 1:(y + 1) * row] for y in range(height))
  return {"width": width, "height": height, "depth": depth, "color": color}, pixels


class TestPng:

  def test_rgb8(self):
    image = np.random.default_rng(1).integers(0, 256, (5, 7, 3), dtype=np.uint8)
    header, pixels = _decode_png(png.encode(image.tobytes(), 7, 5, 3, 8))
    assert header == {"width": 7, "height": 5, "depth": 8, "color": 2}
    assert pixels == image.tobytes()

  def test_gray16_big_endian(self):
    depth_mm = np.array([[0, 1, 65535], [1000, 2048, 300]], dtype=">u2")
    header, pixels = _decode_png(png.encode(depth_mm.tobytes(), 3, 2, 1, 16))
    assert header == {"width": 3, "height": 2, "depth": 16, "color": 0}
    assert np.array_equal(np.frombuffer(pixels, dtype=">u2").reshape(2, 3), depth_mm)

  def test_wrong_size_raises(self):
    with pytest.raises(ValueError):
      png.encode(b"\x00" * 10, 2, 2, 3, 8)
