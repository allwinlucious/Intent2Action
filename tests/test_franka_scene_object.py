"""Unit tests for robot_model.franka_scene_object (no services needed)."""

import math
from pathlib import Path

from intrinsic.geometry.proto.v1 import geometry_storage_refs_pb2
from intrinsic.world.proto import kinematics_component_pb2
import pytest

from robot_model import franka_scene_object as fso

URDF_PATH = (
    Path(__file__).resolve().parents[1]
    / "assets/robots/franka/lula_franka_gen.urdf"
)
HALF_SQRT2 = math.sqrt(0.5)
REVOLUTE = kinematics_component_pb2.KinematicsComponent.MOTION_TYPE_REVOLUTE

# (lower, upper, effort, velocity) from phase_1_inventory.md §5.1.
EXPECTED_LIMITS = {
    "panda_joint1": (-2.8973, 2.8973, 87, 2.1750),
    "panda_joint2": (-1.7628, 1.7628, 87, 2.1750),
    "panda_joint3": (-2.8973, 2.8973, 87, 2.1750),
    "panda_joint4": (-3.0718, -0.0698, 87, 2.1750),
    "panda_joint5": (-2.8973, 2.8973, 12, 2.6100),
    "panda_joint6": (-0.0175, 3.7525, 12, 2.6100),
    "panda_joint7": (-2.8973, 2.8973, 12, 2.6100),
    "panda_finger_joint1": (0.0, 0.04, 20, 0.2),
    "panda_finger_joint2": (0.0, 0.04, 20, 0.2),
}


@pytest.fixture(name="urdf", scope="module")
def _urdf():
  return fso.parse_urdf(URDF_PATH)


@pytest.mark.parametrize(
    "rpy, expected",
    [
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ((0.0, 0.0, math.pi / 2), (0.0, 0.0, HALF_SQRT2, HALF_SQRT2)),
        # Ry(pi/2) * Rx(pi/2): q = qy * qx = (0.5, 0.5, -0.5, 0.5).
        ((math.pi / 2, math.pi / 2, 0.0), (0.5, 0.5, -0.5, 0.5)),
    ],
)
def test_rpy_to_quaternion(rpy, expected):
  assert fso.rpy_to_quaternion(*rpy) == pytest.approx(expected, abs=1e-12)


def test_parse_urdf_joints(urdf):
  by_type = {}
  for joint in urdf.joints.values():
    by_type.setdefault(joint.type, []).append(joint.name)
  assert len(by_type["revolute"]) == 7
  assert len(by_type["prismatic"]) == 2
  for name, (lower, upper, effort, velocity) in EXPECTED_LIMITS.items():
    limit = urdf.joints[name].limit
    assert (limit.lower, limit.upper, limit.effort, limit.velocity) == (
        pytest.approx((lower, upper, effort, velocity))
    ), name


def test_build_arm(urdf):
  meshes = [
      mesh
      for link in urdf.links.values()
      for mesh in (link.visual, link.collision)
      if mesh is not None
  ]
  refs = {
      fso.mesh_key(
          mesh.filename
      ): geometry_storage_refs_pb2.GeometryStorageRefs(
          exact_geometry_ref=f"test://{mesh.filename}"
      )
      for mesh in meshes
  }
  lula = fso.LulaLimits(
      acceleration=(15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0),
      jerk=(7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0),
  )
  arm = fso.build_arm(urdf, refs, lula)
  joints = [e for e in arm.entities if e.WhichOneof("entity_type") == "joint"]
  revolute = [
      e for e in joints if e.joint.kinematics_component.motion_type == REVOLUTE
  ]
  assert len(revolute) == 7
  flanges = [
      e
      for e in arm.entities
      if e.WhichOneof("entity_type") == "frame" and e.frame.is_attachment_frame
  ]
  assert [e.name for e in flanges] == ["flange"]
  (home,) = [
      c
      for c in arm.properties.kinematics.named_configurations
      if c.name == "home"
  ]
  assert len(home.joint_positions) == 7
