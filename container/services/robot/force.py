"""Pure helpers of MoveToContact and MoveCompliant (phase_5b.md decision 2): argument checks, ICON action
parameters (plain protos; Intrinsic's Python helpers need unbuilt C++ extensions) and the force along a push.

No rclpy and no ICON session here; tests/test_force.py covers it.
"""

import math

import grpc
from intrinsic.icon.actions import cartesian_admittance_action_pb2 as admittance_pb
from intrinsic.icon.actions import cartesian_impedance_pb2 as impedance_pb
from intrinsic.icon.actions import cartesian_impedance_reference_generators_pb2 as generators_pb
from intrinsic.icon.actions import force_primitive_pb2
from intrinsic.icon.control.primitives.force_control.proto import force_primitives_pb2
from intrinsic.icon.equipment import force_control_settings_pb2
from intrinsic.icon.proto import cart_space_pb2
from intrinsic.icon.proto import matrix_pb2
from intrinsic.manipulation.skills.force import contact_stiffness_pb2
from intrinsic.math.proto import vector3_pb2
import numpy as np
from robot_service import robot_service_pb2 as api

from robot.motion import to_pose_proto
from robot.ops import ApiError

# MakeContact has no speed field: the free-space speed is F / D with D = 2 sqrt(m (1234 + k_env)) per axis
# (overdamping.cc kMockStiffness 1234; MEDIUM contact stiffness 60000 N/m), so the virtual inertia m sets it.
MOCK_STIFFNESS = 1234.0
ENVIRONMENT_STIFFNESS = 60000.0
INERTIA_RANGE_KG = (1.0, 200.0)
MAX_FORCE_N = 85.0  # also the excessive-force threshold given to ICON
MAX_TORQUE_NM = 8.5
STIFFNESS_RANGE = (50.0, 5000.0)  # N/m
ROTATIONAL_INERTIA = 1.0  # kg m², MakeContact
DEADBAND = (1.5, 0.2)  # N, N·m
ADMITTANCE_ROTATIONAL_STIFFNESS = 30.0  # N·m/rad
ADMITTANCE_INERTIA = (5.0, 0.05)  # kg, kg m²


def contact_inertia(force: float, speed: float) -> float:
  """Virtual translational inertia (kg) for which MakeContact approaches at `speed` (m/s) with `force` (N)."""
  return (force / (2 * speed)) ** 2 / (MOCK_STIFFNESS + ENVIRONMENT_STIFFNESS)


def _speed(force: float, inertia: float) -> float:
  return force / (2 * math.sqrt(inertia * (MOCK_STIFFNESS + ENVIRONMENT_STIFFNESS)))


def _check_force(max_force: float) -> None:
  if not 0 < max_force <= MAX_FORCE_N:
    raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, f"max_force must be in (0, {MAX_FORCE_N:.0f}] N, got {max_force}")


def validate_contact(direction, force: float, speed: float) -> np.ndarray:
  """INVALID_ARGUMENT unless the MoveToContact arguments are usable; returns the unit direction."""
  direction = np.asarray(direction, dtype=float)
  norm = float(np.linalg.norm(direction))
  if not (math.isfinite(norm) and norm > 0):
    raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, f"direction must be a non-zero finite vector, got {direction.tolist()}")
  _check_force(force)
  if not speed > 0:
    raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, f"speed must be > 0 m/s, got {speed}")
  if not INERTIA_RANGE_KG[0] <= contact_inertia(force, speed) <= INERTIA_RANGE_KG[1]:
    v_min, v_max = _speed(force, INERTIA_RANGE_KG[1]), _speed(force, INERTIA_RANGE_KG[0])
    raise ApiError(grpc.StatusCode.INVALID_ARGUMENT,
                   f"speed must be between {v_min:.4f} and {v_max:.4f} m/s for {force:g} N, got {speed}")
  return direction / norm


def validate_compliant(stiffness: float, max_force: float) -> None:
  if not STIFFNESS_RANGE[0] <= stiffness <= STIFFNESS_RANGE[1]:
    raise ApiError(grpc.StatusCode.INVALID_ARGUMENT,
                   f"stiffness must be in [{STIFFNESS_RANGE[0]:.0f}, {STIFFNESS_RANGE[1]:.0f}] N/m, got {stiffness}")
  _check_force(max_force)


def _deadband() -> cart_space_pb2.Wrench:
  f, t = DEADBAND
  return cart_space_pb2.Wrench(x=f, y=f, z=f, rx=t, ry=t, rz=t)


def _diagonal(translational: float, rotational: float) -> matrix_pb2.Matrix6d:
  return matrix_pb2.Matrix6d(data=np.diag([translational] * 3 + [rotational] * 3).ravel().tolist())


def _transform(pose: api.Pose) -> cart_space_pb2.Transform:
  p = to_pose_proto(pose)
  return cart_space_pb2.Transform(
      pos=cart_space_pb2.Point(x=p.position.x, y=p.position.y, z=p.position.z),
      rot=cart_space_pb2.Rotation(qx=p.orientation.x, qy=p.orientation.y, qz=p.orientation.z, qw=p.orientation.w))


def make_contact_params(direction: np.ndarray, force: float, inertia: float, flange_t_tcp: api.Pose,
                        stabilize: bool = False) -> force_primitive_pb2.ForcePrimitiveFixedParams:
  """intrinsic.force_primitive MakeContact along `direction` (base frame) up to `force`, controlling the TCP.

  The stabilize variant (started in contact) disables the reference lowpass filter, as Intrinsic's
  move_to_contact skill does.
  """
  return force_primitive_pb2.ForcePrimitiveFixedParams(
      force_primitive=force_primitives_pb2.ForcePrimitive(make_contact=force_primitives_pb2.MakeContact(
          motion_direction=force_primitives_pb2.Direction(
              vector=vector3_pb2.Vector3(x=direction[0], y=direction[1], z=direction[2])),
          max_contact_force=force)),
      force_control_settings=force_control_settings_pb2.ForceControlSettings(
          excessive_force_threshold=MAX_FORCE_N, excessive_torque_threshold=MAX_TORQUE_NM,
          virtual_translational_inertia=inertia, virtual_rotational_inertia=ROTATIONAL_INERTIA,
          sensed_wrench_deadband=_deadband()),
      task_params=force_primitive_pb2.TaskParams(
          robot_tip_t_robot_tool=to_pose_proto(flange_t_tcp),
          robot_base_t_task=to_pose_proto(api.Pose(qw=1.0)),
          environment_stiffness=contact_stiffness_pb2.ContactStiffnessParams(
              default_contact_stiffness=contact_stiffness_pb2.CONTACT_STIFFNESS_MEDIUM)),
      disable_reference_lowpass_filter=stabilize,
  )


def admittance_params(target: api.Pose, stiffness: float,
                      flange_t_tcp: api.Pose) -> admittance_pb.CartesianAdmittanceActionFixedParams:
  """intrinsic.cartesian_admittance_action: a pure spring of `stiffness` pulling the TCP to `target` (base frame).

  Damping is left unset (ICON makes it critical); the reference wrench is zero.
  """
  params = impedance_pb.CartesianImpedanceParameters(
      cartesian_target=impedance_pb.CartesianTarget(
          robot_tip_t_robot_tool=_transform(flange_t_tcp),
          task_t_tool_reference_pose=_transform(target),
          tool_reference_wrench=cart_space_pb2.Wrench(),
          virtual_cartesian_inertia=_diagonal(*ADMITTANCE_INERTIA),
          cartesian_stiffness=_diagonal(stiffness, ADMITTANCE_ROTATIONAL_STIFFNESS)),
      algorithm_configuration=impedance_pb.AlgorithmConfiguration(sensed_wrench_deadband=_deadband()),
      # Required by ICON; the default lowpass (alpha 0.005) as in Intrinsic's create_cartesian_admittance_action.
      reference_generators=generators_pb.ReferenceGenerators(
          cartesian=generators_pb.ReferenceGenerators.Cartesian(
              simple_lowpass=generators_pb.SimpleImpedanceReferenceGenerator()),
          nullspace=generators_pb.ReferenceGenerators.Nullspace(
              simple_lowpass=generators_pb.SimpleNullspaceReferenceGenerator())),
  )
  return admittance_pb.CartesianAdmittanceActionFixedParams(cartesian_impedance_parameters=params)


def push_force(wrench_flange, direction_base: np.ndarray, base_r_flange: np.ndarray) -> float:
  """|force of `wrench_flange` (flange frame) along the push direction (base frame)|, in N."""
  direction_flange = np.asarray(base_r_flange).T @ np.asarray(direction_base, dtype=float)
  return abs(float(np.dot(np.asarray(wrench_flange, dtype=float)[:3], direction_flange)))
