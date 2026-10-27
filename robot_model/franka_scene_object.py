"""Builds the Franka SceneObjects (`franka` arm, `franka_hand`) from the URDF.

The SceneObject protos are built directly from the URDF Isaac uses, so Isaac
and Intrinsic share one model (docs/designs/phases/phase_1.md).
Python 3.10, standard library + grpcio + protobuf + gen/ stubs only.
"""

import dataclasses
import math
from pathlib import Path
from typing import NamedTuple, Sequence
import xml.etree.ElementTree as ET

from intrinsic.geometry.proto import geometry_service_pb2
from intrinsic.geometry.proto import geometry_service_types_pb2
from intrinsic.geometry.proto.v1 import geometric_transform_pb2
from intrinsic.geometry.proto.v1 import geometry_pb2
from intrinsic.geometry.proto.v1 import geometry_storage_refs_pb2
from intrinsic.geometry.proto.v1 import transformed_geometry_pb2
from intrinsic.math.proto import point_pb2
from intrinsic.math.proto import pose_pb2
from intrinsic.math.proto import quaternion_pb2
from intrinsic.math.proto import vector3_pb2
from intrinsic.scene.proto.v1 import entity_pb2
from intrinsic.scene.proto.v1 import object_properties_pb2
from intrinsic.scene.proto.v1 import scene_object_pb2
from intrinsic.world.proto import kinematics_component_pb2

ARM_NAME = "franka"
HAND_NAME = "franka_hand"
FLANGE_FRAME = "flange"
TCP_FRAME = "tcp"
ARM_ROOT_LINK = "panda_link0"
ARM_LAST_LINK = "panda_link8"
ARM_REVOLUTE_JOINTS = tuple(f"panda_joint{i}" for i in range(1, 8))
HAND_LINK = "panda_hand"
HAND_JOINT = "panda_hand_joint"
TCP_JOINT = "right_gripper"
# (geometry name, URDF finger link, URDF finger joint)
FINGERS = (
    ("left_finger", "panda_leftfinger", "panda_finger_joint1"),
    ("right_finger", "panda_rightfinger", "panda_finger_joint2"),
)
FINGER_OPENING_M = 0.04  # fully open; finger motion is phase 3
COLLISION_SET = "Intrinsic_Collision"
VISUAL_SET = "Intrinsic_Visual"
IK_SOLVER = "kinematic_chain"
HOME = (0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785)  # "ready"
CARTESIAN_LIMITS = object_properties_pb2.CartesianLimits(
    min_translational_position=[-2.0] * 3,
    max_translational_position=[2.0] * 3,
    min_translational_velocity=[-1.7] * 3,
    max_translational_velocity=[1.7] * 3,
    min_translational_acceleration=[-13.0] * 3,
    max_translational_acceleration=[13.0] * 3,
    min_translational_jerk=[-6500.0] * 3,
    max_translational_jerk=[6500.0] * 3,
    max_rotational_velocity=2.5,
    max_rotational_acceleration=25.0,
    max_rotational_jerk=12500.0,
)

_KC = kinematics_component_pb2.KinematicsComponent
_MOTION_TYPES = {
    "revolute": _KC.MOTION_TYPE_REVOLUTE,
    "fixed": _KC.MOTION_TYPE_FIXED,
}

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]


@dataclasses.dataclass(frozen=True)
class Origin:
  xyz: Vec3 = (0.0, 0.0, 0.0)
  rpy: Vec3 = (0.0, 0.0, 0.0)


@dataclasses.dataclass(frozen=True)
class Mesh:
  filename: str
  origin: Origin


@dataclasses.dataclass(frozen=True)
class Link:
  name: str
  visual: Mesh | None
  collision: Mesh | None


@dataclasses.dataclass(frozen=True)
class Limit:
  lower: float
  upper: float
  effort: float
  velocity: float


@dataclasses.dataclass(frozen=True)
class Joint:
  name: str
  type: str
  parent: str
  child: str
  origin: Origin
  axis: Vec3
  limit: Limit | None


@dataclasses.dataclass(frozen=True)
class Urdf:
  links: dict[str, Link]
  joints: dict[str, Joint]


class LulaLimits(NamedTuple):
  """Per-joint limits from Lula's robot_descriptor.yaml, joint1..joint7."""

  acceleration: Sequence[float]
  jerk: Sequence[float]


def _floats(text: str | None, default: Vec3) -> Vec3:
  return tuple(float(v) for v in text.split()) if text else default


def _origin(element: ET.Element | None) -> Origin:
  if element is None:
    return Origin()
  return Origin(
      xyz=_floats(element.get("xyz"), (0.0, 0.0, 0.0)),
      rpy=_floats(element.get("rpy"), (0.0, 0.0, 0.0)),
  )


def _mesh(link: ET.Element, kind: str) -> Mesh | None:
  for element in link.findall(kind):
    mesh = element.find("geometry/mesh")
    if mesh is not None:
      return Mesh(mesh.get("filename"), _origin(element.find("origin")))
  return None


def parse_urdf(path: str | Path) -> Urdf:
  root = ET.parse(path).getroot()
  links = {}
  for link in root.findall("link"):
    name = link.get("name")
    links[name] = Link(name, _mesh(link, "visual"), _mesh(link, "collision"))
  joints = {}
  for joint in root.findall("joint"):
    limit = joint.find("limit")
    axis = joint.find("axis")
    joints[joint.get("name")] = Joint(
        name=joint.get("name"),
        type=joint.get("type"),
        parent=joint.find("parent").get("link"),
        child=joint.find("child").get("link"),
        origin=_origin(joint.find("origin")),
        axis=_floats(
            axis.get("xyz") if axis is not None else None, (1.0, 0.0, 0.0)
        ),
        limit=None
        if limit is None
        else Limit(
            lower=float(limit.get("lower", 0.0)),
            upper=float(limit.get("upper", 0.0)),
            effort=float(limit.get("effort")),
            velocity=float(limit.get("velocity")),
        ),
    )
  return Urdf(links, joints)


def rpy_to_quaternion(
    r: float, p: float, y: float
) -> tuple[float, float, float, float]:
  """(x, y, z, w) of R = Rz(y)·Ry(p)·Rx(r) (URDF fixed-axis XYZ)."""
  cr, sr = math.cos(r / 2), math.sin(r / 2)
  cp, sp = math.cos(p / 2), math.sin(p / 2)
  cy, sy = math.cos(y / 2), math.sin(y / 2)
  return (
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
      cr * cp * cy + sr * sp * sy,
  )


def _rpy_to_matrix(r: float, p: float, y: float) -> Mat3:
  cr, sr = math.cos(r), math.sin(r)
  cp, sp = math.cos(p), math.sin(p)
  cy, sy = math.cos(y), math.sin(y)
  return (
      (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
      (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
      (-sp, cp * sr, cp * cr),
  )


def _matrix_to_rpy(m: Mat3) -> Vec3:
  return (
      math.atan2(m[2][1], m[2][2]),
      math.asin(max(-1.0, min(1.0, -m[2][0]))),
      math.atan2(m[1][0], m[0][0]),
  )


def _compose(a: Origin, b: Origin) -> Origin:
  """a · b as rigid transforms."""
  ra, rb = _rpy_to_matrix(*a.rpy), _rpy_to_matrix(*b.rpy)
  rotation = tuple(
      tuple(sum(ra[i][k] * rb[k][j] for k in range(3)) for j in range(3))
      for i in range(3)
  )
  xyz = tuple(
      a.xyz[i] + sum(ra[i][k] * b.xyz[k] for k in range(3)) for i in range(3)
  )
  return Origin(xyz, _matrix_to_rpy(rotation))


def pose_proto(origin: Origin = Origin()) -> pose_pb2.Pose:
  x, y, z, w = rpy_to_quaternion(*origin.rpy)
  return pose_pb2.Pose(
      position=point_pb2.Point(
          x=origin.xyz[0], y=origin.xyz[1], z=origin.xyz[2]
      ),
      orientation=quaternion_pb2.Quaternion(x=x, y=y, z=z, w=w),
  )


def mesh_key(filename: str) -> str:
  """`package://…/meshes/visual/link1.dae` -> `visual/link1.glb`."""
  key = filename.split("/meshes/", 1)[1]
  return key[: -len(".dae")] + ".glb" if key.endswith(".dae") else key


def _transformed_geometry(
    refs: geometry_storage_refs_pb2.GeometryStorageRefs, ref_t_shape: Origin
) -> transformed_geometry_pb2.TransformedGeometry:
  trs = geometric_transform_pb2.GeometricTransform.TRS(
      translation=vector3_pb2.Vector3(
          x=ref_t_shape.xyz[0], y=ref_t_shape.xyz[1], z=ref_t_shape.xyz[2]
      ),
      rotation_rpy=geometric_transform_pb2.GeometricTransform.TRS.RotationRPY(
          r=ref_t_shape.rpy[0], p=ref_t_shape.rpy[1], y=ref_t_shape.rpy[2]
      ),
  )
  return transformed_geometry_pb2.TransformedGeometry(
      geometry=geometry_pb2.Geometry(geo_ref=refs),
      ref_t_shape=geometric_transform_pb2.GeometricTransform(trs=trs),
  )


def _add_shapes(
    entity: entity_pb2.Entity,
    shape_name: str,
    link: Link,
    geometry_refs: dict[str, geometry_storage_refs_pb2.GeometryStorageRefs],
    link_t_ref: Origin = Origin(),
) -> None:
  """Adds the link's visual and collision meshes, each placed at link_t_ref · mesh origin."""
  geometry = entity.link.geometry_component.named_geometries
  for set_name, mesh in (
      (COLLISION_SET, link.collision),
      (VISUAL_SET, link.visual),
  ):
    geometry[set_name].named_geometries[shape_name].CopyFrom(
        _transformed_geometry(
            geometry_refs[mesh_key(mesh.filename)],
            _compose(link_t_ref, mesh.origin),
        )
    )


def _joint_limits(joint: Joint, acceleration: float, jerk: float) -> _KC.Limits:
  return _KC.Limits(
      velocity=joint.limit.velocity,
      acceleration=acceleration,
      jerk=jerk,
      effort=joint.limit.effort,
      fixed_limits=_KC.Limits.LimitPair(
          lower=joint.limit.lower, upper=joint.limit.upper
      ),
  )


APPLICATION_LIMIT_FRACTION = 0.95


def _joint_entity(joint: Joint, limits: _KC.Limits | None) -> entity_pb2.Entity:
  kinematics = _KC(
      parent_t_inboard=pose_proto(joint.origin),
      outboard_t_child=pose_proto(),
      motion_type=_MOTION_TYPES[joint.type],
  )
  if limits is not None:
    kinematics.axis.CopyFrom(
        vector3_pb2.Vector3(x=joint.axis[0], y=joint.axis[1], z=joint.axis[2])
    )
    kinematics.system_limits.CopyFrom(limits)
    # ICON requires application velocity/acceleration/jerk >= 5% below system
    # (joint_limits.cc:109-143); 95% as in Intrinsic's UR definitions.
    application = _KC.Limits()
    application.CopyFrom(limits)
    application.velocity *= APPLICATION_LIMIT_FRACTION
    application.acceleration *= APPLICATION_LIMIT_FRACTION
    application.jerk *= APPLICATION_LIMIT_FRACTION
    kinematics.application_limits.CopyFrom(application)
  # parent_t_this = parent_t_inboard at joint value 0 (the world refreshes it).
  return entity_pb2.Entity(
      name=joint.name,
      parent_name=joint.parent,
      parent_t_this=pose_proto(joint.origin),
      joint=entity_pb2.Joint(kinematics_component=kinematics),
  )


def build_arm(
    urdf: Urdf,
    geometry_refs: dict[str, geometry_storage_refs_pb2.GeometryStorageRefs],
    lula_limits: LulaLimits,
) -> scene_object_pb2.SceneObject:
  """`franka`: panda_link0 … panda_link8 with a `flange` attachment frame."""
  root = entity_pb2.Entity(name=ARM_ROOT_LINK)
  _add_shapes(root, ARM_ROOT_LINK, urdf.links[ARM_ROOT_LINK], geometry_refs)
  entities = [root]
  link_name = ARM_ROOT_LINK
  while link_name != ARM_LAST_LINK:
    (joint,) = [
        j
        for j in urdf.joints.values()
        if j.parent == link_name
        and (j.name in ARM_REVOLUTE_JOINTS or j.child == ARM_LAST_LINK)
    ]
    if joint.type == "revolute":
      i = ARM_REVOLUTE_JOINTS.index(joint.name)
      limits = _joint_limits(
          joint, lula_limits.acceleration[i], lula_limits.jerk[i]
      )
    else:
      limits = None
    entities.append(_joint_entity(joint, limits))
    link = entity_pb2.Entity(
        name=joint.child, parent_name=joint.name, parent_t_this=pose_proto()
    )
    link.link.SetInParent()
    if urdf.links[joint.child].visual is not None:
      _add_shapes(link, joint.child, urdf.links[joint.child], geometry_refs)
    entities.append(link)
    link_name = joint.child
  entities.append(
      entity_pb2.Entity(
          name=FLANGE_FRAME,
          parent_name=ARM_LAST_LINK,
          parent_t_this=pose_proto(),
          frame=entity_pb2.Frame(is_attachment_frame=True),
      )
  )
  kinematics = object_properties_pb2.Kinematics(
      limits=CARTESIAN_LIMITS,
      ik_solvers=[object_properties_pb2.IkSolver(ik_solver=IK_SOLVER)],
      named_configurations=[
          object_properties_pb2.NamedConfiguration(
              name="home",
              joint_positions=dict(zip(ARM_REVOLUTE_JOINTS, HOME)),
          )
      ],
  )
  return scene_object_pb2.SceneObject(
      name=ARM_NAME,
      properties=object_properties_pb2.ObjectProperties(kinematics=kinematics),
      entities=entities,
  )


def build_hand(
    urdf: Urdf,
    geometry_refs: dict[str, geometry_storage_refs_pb2.GeometryStorageRefs],
) -> scene_object_pb2.SceneObject:
  """`franka_hand`: one link at the flange with hand and open fingers, plus `tcp`."""
  # The world forces an object on an attachment frame to identity, so the
  # flange-to-hand rotation (panda_hand_joint) goes into every shape instead.
  flange_t_hand = urdf.joints[HAND_JOINT].origin
  hand = entity_pb2.Entity(name=HAND_LINK)
  _add_shapes(hand, "hand", urdf.links[HAND_LINK], geometry_refs, flange_t_hand)
  for shape_name, link_name, joint_name in FINGERS:
    joint = urdf.joints[joint_name]
    hand_t_finger = _compose(
        joint.origin,
        Origin(xyz=tuple(FINGER_OPENING_M * a for a in joint.axis)),
    )
    _add_shapes(
        hand,
        shape_name,
        urdf.links[link_name],
        geometry_refs,
        _compose(flange_t_hand, hand_t_finger),
    )
  tcp = entity_pb2.Entity(
      name=TCP_FRAME,
      parent_name=HAND_LINK,
      parent_t_this=pose_proto(urdf.joints[TCP_JOINT].origin),
      frame=entity_pb2.Frame(),
  )
  return scene_object_pb2.SceneObject(name=HAND_NAME, entities=[hand, tcp])


def upload_meshes(
    geometry_stub, asset_dir: str | Path, timeout_s: float = 30.0
) -> dict[str, geometry_storage_refs_pb2.GeometryStorageRefs]:
  """Uploads meshes/collision/*.stl and meshes/visual/*.glb with CreateGeometry."""
  meshes_dir = Path(asset_dir) / "meshes"
  refs = {}
  for path in sorted(meshes_dir.glob("collision/*.stl")) + sorted(
      meshes_dir.glob("visual/*.glb")
  ):
    data = path.read_bytes()
    geometry_data = (
        geometry_service_types_pb2.GeometryData(stl_bytes=data)
        if path.suffix == ".stl"
        else geometry_service_types_pb2.GeometryData(gltf_bytes=data)
    )
    response = geometry_stub.CreateGeometry(
        geometry_service_pb2.CreateGeometryRequest(data=geometry_data),
        timeout=timeout_s,
    )
    refs[path.relative_to(meshes_dir).as_posix()] = (
        response.geometry_storage_refs
    )
  return refs
