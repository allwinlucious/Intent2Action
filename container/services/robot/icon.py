"""ICON access for robot_service: status, one-action sessions, recovery, speed, payload, jogging.

Uses only the vendored `_session.Session` with raw stubs (phase_3.md decision 1). Parts: `arm`,
`gripper`, `ft_sensor` (container/config/icon.txtpb).
"""

import logging
import math
import threading
import time
from typing import Callable

import grpc
from intrinsic.icon.proto.v1 import jogging_service_pb2 as jog_pb
from intrinsic.icon.proto.v1 import jogging_service_pb2_grpc as jog_grpc
from intrinsic.icon.proto.v1 import service_pb2 as icon_pb
from intrinsic.icon.proto.v1 import service_pb2_grpc as icon_grpc
from intrinsic.icon.proto.v1 import types_pb2
from intrinsic.icon.python import _session
from intrinsic.icon.python import errors as icon_errors
from intrinsic.icon.python import reactions
from intrinsic.math.proto import matrix_pb2
from intrinsic.math.proto import point_pb2
from intrinsic.math.proto import pose_pb2
from intrinsic.math.proto import quaternion_pb2
from intrinsic.world.proto import robot_payload_pb2

from robot.ops import ApiError

ARM, GRIPPER, FT_SENSOR = "arm", "gripper", "ft_sensor"
RPC_TIMEOUT_S = 10.0
POLL_S = 0.05
# JoggingService clamps each joint's max velocity to 30 deg/s before scaling normalized_velocity
# (intrinsic_control/intrinsic/icon/server/jogging_service.cc, kMaxJointJoggingSpeedRadPerSecond).
JOG_MAX_VELOCITY_RAD_S = math.radians(30.0)

log = logging.getLogger("robot_service")


def _describe(error: Exception) -> str:
  if isinstance(error, grpc.Call):
    return f"{error.code().name}: {error.details()}"
  return str(error) or type(error).__name__


class Icon:
  """One channel to ICON on localhost:18003 (thread-safe; each action gets its own session)."""

  def __init__(self, address: str):
    self.channel = grpc.insecure_channel(address)
    self.stub = icon_grpc.IconApiStub(self.channel)
    self.jogging = jog_grpc.JoggingServiceStub(self.channel)
    self._max_velocities: list[float] | None = None
    self._lock = threading.Lock()

  def _call(self, method: str, request, timeout_s: float = RPC_TIMEOUT_S):
    try:
      return getattr(self.stub, method)(request, timeout=timeout_s)
    except grpc.RpcError as error:
      code = error.code() if error.code() == grpc.StatusCode.UNAVAILABLE else grpc.StatusCode.INTERNAL
      raise ApiError(code, f"ICON {method}: {_describe(error)}") from error

  def status(self) -> icon_pb.GetStatusResponse:
    return self._call("GetStatus", icon_pb.GetStatusRequest())

  def operational(self) -> tuple[str, str]:
    """(ENABLED | DISABLED | FAULTED | UNKNOWN, fault reason)."""
    status = self._call("GetOperationalStatus", icon_pb.GetOperationalStatusRequest()).operational_status
    return types_pb2.OperationalState.Name(status.state), status.fault_reason

  def require_enabled(self) -> None:
    """FAILED_PRECONDITION with ICON's fault text unless ICON is ENABLED."""
    state, reason = self.operational()
    if state == "FAULTED":
      raise ApiError(grpc.StatusCode.FAILED_PRECONDITION, f"robot faulted: {reason}; call ClearFaults")
    if state != "ENABLED":
      raise ApiError(grpc.StatusCode.FAILED_PRECONDITION, f"robot not enabled (ICON state {state})")

  def wait_state(self, want: str, timeout_s: float) -> tuple[str, str]:
    """Polls until ICON is in state `want`; returns the last (state, reason)."""
    deadline = time.monotonic() + timeout_s
    while True:
      try:
        state = self.operational()
      except ApiError as error:
        state = ("UNREACHABLE", error.message)
      if state[0] == want or time.monotonic() > deadline:
        return state
      time.sleep(0.1)

  def clear_faults(self, enabled_timeout_s: float = 10.0) -> None:
    """ClearFaults, then waits for ENABLED (ICON's ready state)."""
    self._call("ClearFaults", icon_pb.ClearFaultsRequest(), timeout_s=30.0)
    state, reason = self.wait_state("ENABLED", enabled_timeout_s)
    if state != "ENABLED":
      raise ApiError(
          grpc.StatusCode.UNAVAILABLE,
          f"ICON is {state} {enabled_timeout_s:.0f} s after ClearFaults" + (f": {reason}" if reason else ""),
      )

  def set_speed(self, factor: float) -> None:
    self._call("SetSpeedOverride", icon_pb.SetSpeedOverrideRequest(override_factor=factor))

  def set_payload(self, mass_kg: float, com: tuple[float, float, float]) -> None:
    payload = robot_payload_pb2.RobotPayload(
        mass_kg=mass_kg,
        tip_t_cog=pose_pb2.Pose(
            position=point_pb2.Point(x=com[0], y=com[1], z=com[2]),
            orientation=quaternion_pb2.Quaternion(w=1.0),
        ),
        inertia=matrix_pb2.Matrixd(rows=3, cols=3, values=[0.0] * 9),
    )
    self._call(
        "SetPayload", icon_pb.SetPayloadRequest(part_name=ARM, payload_name="full_payload", payload=payload)
    )

  def arm_joints(self, status: icon_pb.GetStatusResponse | None = None) -> list[float]:
    status = status or self.status()
    return [j.position_sensed for j in status.part_status[ARM].joint_states]

  def joint_max_velocities(self) -> list[float]:
    """The arm's application max velocities as JoggingService uses them (clamped to 30 deg/s)."""
    with self._lock:
      if self._max_velocities is None:
        configs = self._call("GetConfig", icon_pb.GetConfigRequest()).part_configs
        arm = next(c for c in configs if c.name == ARM)
        limits = arm.generic_config.joint_limits_config.application_limits.max_velocity.values
        self._max_velocities = [min(v, JOG_MAX_VELOCITY_RAD_S) for v in limits]
      return self._max_velocities

  def run_action(
      self,
      parts: list[str],
      action,
      stop_when: Callable[[bool], bool],
      cancel: threading.Event | None = None,
      condition: reactions.Condition | None = None,
      on_started: Callable[[], None] | None = None,
  ) -> None:
    """Runs one action in its own session on `parts`, then ends the session.

    on_started() runs right after the action started. Then every 50 ms: stop_when(condition met)
    returning True ends the run (it may raise ApiError, e.g. on a timeout). Raises ApiError CANCELLED
    if `cancel` is set, FAILED_PRECONDITION if the session cannot be opened, ABORTED if ICON ends it
    (with ICON's fault text).
    """
    try:
      session = _session.Session(self.stub, parts)
    except grpc.RpcError as error:
      raise ApiError(
          grpc.StatusCode.FAILED_PRECONDITION, f"ICON session on {parts} not opened: {_describe(error)}"
      ) from error
    try:
      with session:
        session.add_action(action)
        flag = session.add_reaction(action, condition or reactions.Condition.is_done())
        session.start_action(action.id)
        if on_started is not None:
          on_started()
        while True:
          if stop_when(flag.wait(POLL_S)):
            return
          if cancel is not None and cancel.is_set():
            raise ApiError(grpc.StatusCode.CANCELLED, "cancelled (Stop, CancelOperation or ResetScene)")
    except (grpc.RpcError, icon_errors.Session.SessionEndedError, icon_errors.Session.ActionError) as error:
      state, reason = self.wait_state("FAULTED", 0.0)
      fault = f"; ICON {state}: {reason}" if reason else ""
      raise ApiError(
          grpc.StatusCode.ABORTED, f"ICON ended the {action.proto.action_type_name} action: {_describe(error)}{fault}"
      ) from error

  def jog(self, joint_index: int, normalized_velocity: float, keep_going: Callable[[], bool],
          period_s: float = 0.1) -> None:
    """JoggingService.JogRobot on `arm`: one command every period_s (wall) while keep_going(), then 0.

    Commands start once the service answered the initial request (its jogging action is running).
    """
    started = threading.Event()

    def requests():
      yield jog_pb.JoggingRequest(
          initial_jogging_data=jog_pb.InitialJoggingData(
              part_name=ARM,
              initial_joint_jogging_spec=jog_pb.InitialJoggingData.InitialJointJogging(),
              client_name="robot_service",
          )
      )
      if not started.wait(RPC_TIMEOUT_S):
        return
      while keep_going():
        yield self._jog_command(joint_index, normalized_velocity)
        time.sleep(period_s)
      yield self._jog_command(joint_index, 0.0)

    try:
      for _ in self.jogging.JogRobot(requests()):
        started.set()
    except grpc.RpcError as error:
      raise ApiError(error.code(), f"ICON JogRobot: {error.details()}") from error
    if not started.is_set():
      raise ApiError(grpc.StatusCode.UNAVAILABLE, f"ICON JogRobot did not answer within {RPC_TIMEOUT_S:.0f} s")

  @staticmethod
  def _jog_command(joint_index: int, normalized_velocity: float) -> jog_pb.JoggingRequest:
    return jog_pb.JoggingRequest(
        jogging_command=jog_pb.JoggingCommand(
            joint_jogging_command=jog_pb.JointJoggingCommand(
                joint_index=joint_index, normalized_velocity=normalized_velocity
            )
        )
    )
