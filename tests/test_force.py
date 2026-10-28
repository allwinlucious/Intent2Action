"""Unit tests of robot/force.py: contact inertia, argument checks, ICON parameters, force along a push."""

import math
from pathlib import Path
import sys

import grpc
import numpy as np
import pytest
from robot_service import robot_service_pb2 as api

# Appended: container/services/robot_service.py must not shadow the generated package robot_service.
sys.path.append(str(Path(__file__).resolve().parents[1] / "container/services"))
from robot import force  # noqa: E402
from robot.ops import ApiError  # noqa: E402

# Flange -> tcp of the URDF (xyz 0 0 0.1, rpy 0 0 2.356).
FLANGE_T_TCP = api.Pose(z=0.1, qz=math.sin(2.356 / 2), qw=math.cos(2.356 / 2))


def _code(error_info) -> grpc.StatusCode:
  return error_info.value.code


class TestContactInertia:

  def test_cuj6_values(self):
    assert force.contact_inertia(15, 0.01) == pytest.approx(9.19, abs=0.005)

  def test_inverse_of_the_speed_formula(self):
    m = force.contact_inertia(10, 0.02)
    assert 10 / (2 * math.sqrt(m * (1234 + 60000))) == pytest.approx(0.02)


class TestValidateContact:

  def test_returns_the_unit_direction(self):
    np.testing.assert_allclose(force.validate_contact((0, 0, -2), 15, 0.01), [0, 0, -1])

  def test_zero_direction(self):
    with pytest.raises(ApiError) as error:
      force.validate_contact((0, 0, 0), 15, 0.01)
    assert _code(error) == grpc.StatusCode.INVALID_ARGUMENT
    assert "direction" in error.value.message

  @pytest.mark.parametrize("value", [0, 90, -1, math.nan])
  def test_force_out_of_range(self, value):
    with pytest.raises(ApiError) as error:
      force.validate_contact((0, 0, -1), value, 0.01)
    assert _code(error) == grpc.StatusCode.INVALID_ARGUMENT
    assert "max_force" in error.value.message

  def test_force_at_the_limit(self):
    force.validate_contact((0, 0, -1), 85, 0.1)

  @pytest.mark.parametrize("speed", [0.2, 0.001])
  def test_speed_outside_the_range_names_both_bounds(self, speed):
    with pytest.raises(ApiError) as error:
      force.validate_contact((0, 0, -1), 15, speed)
    assert _code(error) == grpc.StatusCode.INVALID_ARGUMENT
    v_min = 15 / (2 * math.sqrt(200 * 61234))
    v_max = 15 / (2 * math.sqrt(1 * 61234))
    assert f"{v_min:.4f}" in error.value.message and f"{v_max:.4f}" in error.value.message
    assert "15" in error.value.message

  def test_zero_speed(self):
    with pytest.raises(ApiError) as error:
      force.validate_contact((0, 0, -1), 15, 0)
    assert _code(error) == grpc.StatusCode.INVALID_ARGUMENT
    assert "speed" in error.value.message


class TestValidateCompliant:

  @pytest.mark.parametrize("stiffness,max_force", [(50, 20), (5000, 85), (200, 0.1)])
  def test_in_range(self, stiffness, max_force):
    force.validate_compliant(stiffness, max_force)

  @pytest.mark.parametrize("stiffness,max_force,word", [
      (49, 20, "stiffness"), (5001, 20, "stiffness"), (200, 0, "max_force"), (200, 86, "max_force"),
      (math.nan, 20, "stiffness"),
  ])
  def test_out_of_range(self, stiffness, max_force, word):
    with pytest.raises(ApiError) as error:
      force.validate_compliant(stiffness, max_force)
    assert _code(error) == grpc.StatusCode.INVALID_ARGUMENT
    assert word in error.value.message


class TestParams:

  @pytest.mark.parametrize("stabilize", [False, True])
  def test_make_contact(self, stabilize):
    params = force.make_contact_params(np.array([0.0, 0.0, -1.0]), 15, 9.19, FLANGE_T_TCP, stabilize=stabilize)
    contact = params.force_primitive.make_contact
    assert (contact.motion_direction.vector.x, contact.motion_direction.vector.y,
            contact.motion_direction.vector.z) == (0, 0, -1)
    assert contact.max_contact_force == 15
    assert params.disable_reference_lowpass_filter == stabilize
    task = params.task_params
    assert task.robot_base_t_task.orientation.w == 1 and task.robot_base_t_task.position.z == 0
    assert task.robot_tip_t_robot_tool.position.z == pytest.approx(0.1)
    assert task.robot_tip_t_robot_tool.orientation.z == pytest.approx(FLANGE_T_TCP.qz)
    assert task.environment_stiffness.default_contact_stiffness == 2  # MEDIUM
    settings = params.force_control_settings
    assert (settings.virtual_translational_inertia, settings.virtual_rotational_inertia) == (9.19, 1.0)
    assert settings.sensed_wrench_deadband.z == 1.5 and settings.sensed_wrench_deadband.rz == 0.2

  def test_admittance(self):
    target = api.Pose(x=0.6, y=0.3, z=0.1, qx=1.0)
    params = force.admittance_params(target, 200, FLANGE_T_TCP).cartesian_impedance_parameters
    goal = params.cartesian_target
    assert (goal.task_t_tool_reference_pose.pos.x, goal.task_t_tool_reference_pose.rot.qx) == (0.6, 1.0)
    assert goal.robot_tip_t_robot_tool.pos.z == pytest.approx(0.1)
    stiffness = np.array(goal.cartesian_stiffness.data).reshape(6, 6)
    np.testing.assert_allclose(stiffness, np.diag([200, 200, 200, 30, 30, 30]))
    inertia = np.array(goal.virtual_cartesian_inertia.data).reshape(6, 6)
    np.testing.assert_allclose(inertia, np.diag([5, 5, 5, 0.05, 0.05, 0.05]))
    assert not goal.HasField("cartesian_damping")
    assert goal.tool_reference_wrench.z == 0
    assert params.algorithm_configuration.sensed_wrench_deadband.x == 1.5
    # ICON rejects the action without reference generators (REFERENCE_GENERATOR_NOT_SET).
    assert params.reference_generators.cartesian.HasField("simple_lowpass")
    assert params.reference_generators.nullspace.HasField("simple_lowpass")


class TestPushForce:

  def test_downward_push_tool_down(self):
    base_r_flange = np.diag([1.0, -1.0, -1.0])  # flange z points down
    assert force.push_force((3.0, 0.0, 15.0, 0.1, 0.0, 0.0), np.array([0, 0, -1.0]), base_r_flange) == pytest.approx(15)

  def test_sign_is_dropped(self):
    base_r_flange = np.diag([1.0, -1.0, -1.0])
    assert force.push_force((0.0, 0.0, -12.0), np.array([0, 0, -1.0]), base_r_flange) == pytest.approx(12)


class TestJamDetector:

  def test_moving_is_no_jam(self):
    detector = force.JamDetector(target=(0.0, 0.0, 0.0))
    # 1 mm/s toward the target, 5 cm away: 0.1 mm per sample, 1 mm per window.
    assert not any(detector.update(i * 0.1, (0.05 - i * 1e-4, 0.0, 0.0)) for i in range(50))

  def test_still_for_1_s_is_jam(self):
    detector = force.JamDetector(target=(0.0, 0.0, -0.03))
    results = [detector.update(i * 0.1, (0.0, 0.0, (i % 2) * 1e-4)) for i in range(12)]
    # The samples span the 1 s window from t = 1.0 on.
    assert results == [False] * 10 + [True, True]

  def test_still_at_the_target_is_no_jam(self):
    detector = force.JamDetector(target=(0.0, 0.0, 0.0))
    assert not any(detector.update(i * 0.1, (0.0005, 0.0, 0.0)) for i in range(20))

  def test_due_every_sample_period(self):
    detector = force.JamDetector(target=(0.0, 0.0, 0.0))
    assert detector.due(0.0)
    detector.update(0.0, (0.1, 0.0, 0.0))
    assert not detector.due(0.05) and detector.due(0.1)
