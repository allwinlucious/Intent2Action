"""Unit tests of PlanGrasp's pure steps (robot/grasp.py, phase_5c.md decision 6); no ROS needed."""

import math
from pathlib import Path
import sys

import grpc
import numpy as np
import pytest
from robot_service import robot_service_pb2 as api

# Appended: container/services/robot_service.py must not shadow the generated package robot_service.
sys.path.append(str(Path(__file__).resolve().parents[1] / "container/services"))
from robot import grasp  # noqa: E402
from robot.motion import pose_matrix  # noqa: E402
from robot.ops import ApiError  # noqa: E402

CATALOG = Path(__file__).resolve().parents[1] / "assets/objects"
CUBE = np.array([0.05, 0.05, 0.05])
JOINTS = {f"panda_joint{i}": i / 10 for i in range(7, 0, -1)}  # deliberately not in panda_joint1..7 order


def _transform(x_axis, y_axis, z_axis, position) -> np.ndarray:
  m = np.eye(4)
  m[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])
  m[:3, 3] = position
  return m


def _top_down(dims: np.ndarray, fingers=(1, 0, 0)) -> np.ndarray:
  """Tool on the top face, approaching down (-z), fingers closing along `fingers`."""
  y, z = np.array(fingers, float), np.array([0.0, 0.0, -1.0])
  return _transform(np.cross(y, z), y, z, [0, 0, dims[2]])


def _side(dims: np.ndarray) -> np.ndarray:
  """Tool on the -x face at half height, approaching along +x, fingers along y."""
  y, z = np.array([0.0, 1.0, 0.0]), np.array([1.0, 0.0, 0.0])
  return _transform(np.cross(y, z), y, z, [-dims[0] / 2, 0, dims[2] / 2])


def _grasp(object_t_tool: np.ndarray, quality=0.5) -> grasp.Grasp:
  pregrasp = object_t_tool.copy()
  pregrasp[:3, 3] -= 0.08 * object_t_tool[:3, 2]
  return grasp.Grasp(quality=quality, object_t_tool=object_t_tool, object_t_pregrasp=pregrasp, joints=dict(JOINTS))


def _yaw(degrees: float, x=0.5, y=0.1, z=0.75) -> api.Pose:
  half = math.radians(degrees) / 2
  return api.Pose(x=x, y=y, z=z, qz=math.sin(half), qw=math.cos(half))


@pytest.mark.parametrize("degrees", [0.0, 30.0])
def test_side_grasp_composed_into_world(degrees):
  world_t_object = _yaw(degrees)
  out = grasp.response(world_t_object, _grasp(_side(CUBE)), CUBE)
  c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
  approach = np.array([c, s, 0.0])
  # Tool on the -x face (-0.025) moved 0.02 deeper along its approach: -0.005 from the centre line.
  centre = np.array([0.5, 0.1, 0.775])
  grasp_m, pregrasp_m = pose_matrix(out.grasp), pose_matrix(out.pregrasp)
  np.testing.assert_allclose(grasp_m[:3, 3], centre - 0.005 * approach, atol=1e-9)
  np.testing.assert_allclose(grasp_m[:3, 2], approach, atol=1e-9)
  np.testing.assert_allclose(grasp_m[:3, 1], [-s, c, 0.0], atol=1e-9)
  # The pregrasp is not deepened: 0.08 back from the face.
  np.testing.assert_allclose(pregrasp_m[:3, 3], centre - 0.105 * approach, atol=1e-9)
  np.testing.assert_allclose(pregrasp_m[:3, :3], grasp_m[:3, :3], atol=1e-9)
  assert list(out.grasp_joints.positions) == pytest.approx([i / 10 for i in range(1, 8)])


def test_missing_ik_joint_is_internal():
  g = _grasp(_side(CUBE))
  del g.joints["panda_joint4"]
  with pytest.raises(ApiError) as e:
    grasp.response(_yaw(0), g, CUBE)
  assert e.value.code == grpc.StatusCode.INTERNAL


def test_finger_axis_extent_of_the_opening_is_kept():
  dims = np.array([0.075, 0.03, 0.05])
  assert grasp.choose([_grasp(_top_down(dims, fingers=(1, 0, 0)))], dims).quality == 0.5
  # Rotated 90° about the approach, the 0.075 side lies along the tool y axis expressed as -x.
  assert grasp.choose([_grasp(_top_down(dims, fingers=(-1, 0, 0)))], dims).quality == 0.5


def test_finger_axis_extent_beyond_the_opening_is_dropped():
  dims = np.array([0.076, 0.03, 0.05])
  with pytest.raises(ApiError) as e:
    grasp.choose([_grasp(_top_down(dims, fingers=(1, 0, 0)))], dims)
  assert e.value.code == grpc.StatusCode.ABORTED and e.value.message == grasp.TOO_WIDE
  # The other horizontal direction fits; the best fitting grasp wins over a better one that does not fit.
  kept = grasp.choose([_grasp(_top_down(dims, fingers=(1, 0, 0)), quality=0.9),
                       _grasp(_top_down(dims, fingers=(0, 1, 0)), quality=0.4),
                       _grasp(_top_down(dims, fingers=(0, -1, 0)), quality=0.3)], dims)
  assert kept.quality == 0.4


def test_plate_rejected_before_planning():
  dims = grasp.catalog_dims("plate", CATALOG)
  np.testing.assert_allclose(dims, [0.18, 0.18, 0.012])
  with pytest.raises(ApiError) as e:
    grasp.require_graspable(dims)
  assert e.value.code == grpc.StatusCode.ABORTED and "0.075" in e.value.message


def test_graspable_catalog_objects():
  for name in ("red_cube", "peg"):
    grasp.require_graspable(grasp.catalog_dims(name, CATALOG))
  np.testing.assert_allclose(grasp.catalog_dims("red_cube", CATALOG), CUBE)


def test_unknown_catalog_is_not_found(tmp_path):
  with pytest.raises(ApiError) as e:
    grasp.catalog_dims("nope", tmp_path)
  assert e.value.code == grpc.StatusCode.NOT_FOUND


@pytest.mark.parametrize("dims, depth", [(CUBE, 0.02), (np.array([0.05, 0.05, 0.025]), 0.015)])
def test_depth(dims, depth):
  tool = _top_down(dims)
  np.testing.assert_allclose(grasp.deepened(tool, dims)[:3, 3], [0, 0, dims[2] - depth], atol=1e-12)
  np.testing.assert_array_equal(grasp.deepened(tool, dims)[:3, :3], tool[:3, :3])


@pytest.mark.parametrize("axis, degrees", [((0, 0, 1), 0), ((0, 0, 1), 30), ((1, 0, 0), 180), ((0, 1, 0), 180),
                                           ((0, 0, 1), 180), ((1, 1, 1), 120), ((1, -2, 0.5), 250)])
def test_matrix_pose_round_trip(axis, degrees):
  a = np.array(axis, float) / np.linalg.norm(axis)
  half = math.radians(degrees) / 2
  pose = api.Pose(x=0.1, y=-0.2, z=0.3, qx=a[0] * math.sin(half), qy=a[1] * math.sin(half), qz=a[2] * math.sin(half),
                  qw=math.cos(half))
  back = grasp.matrix_pose(pose_matrix(pose))
  np.testing.assert_allclose(pose_matrix(back), pose_matrix(pose), atol=1e-12)
  assert back.qw >= 0
