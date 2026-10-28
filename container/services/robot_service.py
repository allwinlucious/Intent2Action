"""RobotService, SimService and google.longrunning.Operations on 127.0.0.1:18100 (phase_3.md decisions 18-28).

Plain Python in the image, started by container/entrypoint.sh with CycloneDDS rclpy (for /clock). Uses
only Intrinsic APIs: ICON (localhost:18003), the motion planner (:18002), the world (:18000),
camera_sim's CameraService (:18005), the CalibrationService (:18006), the grasp planner and MoveIt (ROS 2,
robot/grasp.py); simulation-only calls go through
scene_sync's SimLink (:18101). Every RPC is logged with its status; errors carry a specific code and
a message naming the cause.
"""

import sys

# This file would shadow the generated (namespace) package `robot_service`: import the package with
# this directory off sys.path, then restore the directory for the `robot` helpers.
_HERE = sys.path.pop(0)
import robot_service  # noqa: E402,F401  pylint: disable=g-import-not-at-top
sys.path.insert(0, _HERE)

import argparse  # noqa: E402
from concurrent import futures  # noqa: E402
import logging  # noqa: E402
import math  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402

from google.longrunning import operations_pb2  # noqa: E402
from google.longrunning import operations_pb2_grpc  # noqa: E402
from google.protobuf import empty_pb2  # noqa: E402
from google.protobuf import text_format  # noqa: E402
from google.protobuf import wrappers_pb2  # noqa: E402
import grpc  # noqa: E402
from intrinsic.icon.actions import simple_gripper_pb2  # noqa: E402
from intrinsic.icon.actions import stop_utils  # noqa: E402
from intrinsic.icon.actions import tare_force_torque_sensor_utils  # noqa: E402
from intrinsic.icon.actions import trajectory_tracking_action_utils  # noqa: E402
from intrinsic.icon.python import actions as icon_actions  # noqa: E402
from intrinsic.icon.python import reactions  # noqa: E402
from intrinsic.perception.proto.v1 import camera_config_pb2  # noqa: E402
from intrinsic.perception.proto.v1 import camera_identifier_pb2  # noqa: E402
from intrinsic.perception.proto.v1 import camera_service_pb2 as camera_pb  # noqa: E402
from intrinsic.perception.proto.v1 import camera_service_pb2_grpc as camera_grpc  # noqa: E402
from intrinsic.perception.proto.v1 import calibration_service_pb2_grpc as calib_grpc  # noqa: E402
from intrinsic.world.proto import object_world_refs_pb2 as refs_pb  # noqa: E402
from intrinsic.world.proto import object_world_service_pb2 as world_pb  # noqa: E402
from intrinsic.world.proto import object_world_service_pb2_grpc as world_grpc  # noqa: E402
from intrinsic.world.proto import object_world_updates_pb2 as updates_pb  # noqa: E402
from intrinsic.world.proto import user_data_pb2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from robot import calibration  # noqa: E402
from robot import grasp  # noqa: E402
from robot import gripper  # noqa: E402
from robot import png  # noqa: E402
from robot.icon import ARM, FT_SENSOR, GRIPPER, Icon  # noqa: E402
from robot.motion import ROBOT, ArmChain, Planner, blending, from_pose_proto, joints, segment, duration_s  # noqa: E402
from robot.ops import CANCELLED, EXECUTING, PLANNING, ApiError, Operation, OperationTable  # noqa: E402
from robot_model.franka_scene_object import HOME  # noqa: E402
from robot_model.load_camera import CALIBRATED_KEY, INIT_WORLD, SENSOR_FRAME  # noqa: E402
from robot_service import robot_service_pb2 as api  # noqa: E402
from robot_service import robot_service_pb2_grpc as api_grpc  # noqa: E402
from robot_service import simlink_pb2  # noqa: E402
from robot_service import simlink_pb2_grpc  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
import yaml  # noqa: E402

SETTINGS_PATH = "/opt/app/config/robot_service.yaml"
WAIT_S = 60.0  # wait=true blocks at most this long (wall)
RPC_TIMEOUT_S = 10.0
SUMMARY_MAX_CHARS = 200
WORLD = "world"
ROBOT_OBJECTS = {"root", "franka", "franka_hand"}
CAMERA_OBJECT = "d455"
CALIB_BOARD = "calib_board"
OBJECT_WAIT_S = 5.0  # AddObject / RemoveObject / AttachCalibrationBoard / DetachObject wait this long for `world`
SYNC_TOLERANCE_M = 0.005  # a detached object counts as back in sync within this of its true pose
STOP_WAIT_S = 5.0
STOP_SETTLE_TIMEOUT_S = 5.0  # sim
SETTLE_TOLERANCE_RAD = 0.01  # a motion completes when every sensed joint is this close to the trajectory end
SETTLE_TIMEOUT_S = 2.0  # sim
TARE_TIMEOUT_S = 5.0  # sim
FAULT_WATCH_PERIOD_S = 0.5
AUTO_CLEAR_AFTER_S = 2.0
CLOCK_ADVANCING_S = 1.0  # the watcher clears faults only if /clock moved this recently (Isaac running)
RESET_FAULT_WAIT_S = 5.0
RESET_ENABLED_WAIT_S = 10.0
POINT_CLOUD_STRIDE = 4  # GetPointCloud default: <= 230 400 points (< 2.8 MB), under gRPC's 4 MB limit
# Before each calibration capture (decision 9): this much sim time, then world's joints within the
# tolerance of ICON's, polled every SYNC_POLL_S for at most SYNC_TIMEOUT_S (wall).
CAPTURE_SETTLE_S = 0.2
SYNC_TOLERANCE_RAD = 1e-4
SYNC_POLL_S = 0.02
SYNC_TIMEOUT_S = 2.0
# Planner codes that reject a calibration pose (unreachable, no plan); anything else fails the calibration.
POSE_REJECTED_CODES = (grpc.StatusCode.ABORTED, grpc.StatusCode.NOT_FOUND, grpc.StatusCode.INVALID_ARGUMENT)
CLOCK_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

log = logging.getLogger("robot_service")


class SimClock:
  """Latest Isaac /clock (decision 20)."""

  def __init__(self):
    self._now: float | None = None
    self._advanced_at: float | None = None  # time.monotonic() of the last /clock that moved forward
    node = rclpy.create_node("robot_service")
    node.create_subscription(Clock, "/clock", self._on_clock, CLOCK_QOS)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

  def _on_clock(self, msg: Clock) -> None:
    now = msg.clock.sec + msg.clock.nanosec * 1e-9
    if self._now is None or now != self._now:
      self._advanced_at = time.monotonic()
    self._now = now

  def advancing(self, within_s: float) -> bool:
    """True if /clock moved within the last within_s (wall), i.e. Isaac is running."""
    return self._advanced_at is not None and time.monotonic() - self._advanced_at <= within_s

  def now(self) -> float:
    if self._now is None:
      raise ApiError(grpc.StatusCode.UNAVAILABLE, "no /clock received from Isaac yet (is Isaac running?)")
    return self._now


def _by_name(name: str) -> refs_pb.ObjectReference:
  return refs_pb.ObjectReference(by_name=refs_pb.ObjectReferenceByName(object_name=name))


def _dependency_error(what: str, error: grpc.RpcError) -> ApiError:
  """A downstream failure keeps its code (NOT_FOUND, UNAVAILABLE, ...) and names the dependency."""
  return ApiError(error.code(), f"{what}: {error.details()}")


class World:
  """`world` in the world service: objects, poses, attach/detach (planning)."""

  def __init__(self, address: str):
    self.stub = world_grpc.ObjectWorldServiceStub(grpc.insecure_channel(address))

  def _call(self, method: str, request):
    try:
      return getattr(self.stub, method)(request, timeout=RPC_TIMEOUT_S)
    except grpc.RpcError as error:
      raise _dependency_error(f"world service {method}", error) from error

  def objects(self) -> list:
    return [o for o in self._call("ListObjects", world_pb.ListObjectsRequest(world_id=WORLD, view=updates_pb.FULL)).objects
            if o.name != "root"]

  def get(self, name: str):
    """The object (FULL view); NOT_FOUND if `world` has none of that name."""
    found = [o for o in self.objects() if o.name == name]
    if not found:
      raise ApiError(grpc.StatusCode.NOT_FOUND, f"no object {name!r} in world")
    return found[0]

  def exists(self, name: str) -> bool:
    return any(o.name == name for o in self.objects())

  def pose(self, name: str) -> api.Pose:
    """Pose of the object in the base frame (= world root)."""
    node = lambda n: refs_pb.TransformNodeReference(  # noqa: E731
        by_name=refs_pb.TransformNodeReferenceByName(object=refs_pb.ObjectReferenceByName(object_name=n))
    )
    response = self._call(
        "GetTransform", world_pb.GetTransformRequest(world_id=WORLD, node_a=refs_pb.TransformNodeReference(id="root"),
                                                     node_b=node(name))
    )
    return from_pose_proto(response.a_t_b)

  def frame_pose(self, name: str, frame: str) -> api.Pose:
    """Pose of the object's frame in the base frame."""
    response = self._call("GetTransform", world_pb.GetTransformRequest(
        world_id=WORLD, node_a=refs_pb.TransformNodeReference(id="root"),
        node_b=refs_pb.TransformNodeReference(by_name=refs_pb.TransformNodeReferenceByName(
            frame=refs_pb.FrameReferenceByName(object_name=name, frame_name=frame)))))
    return from_pose_proto(response.a_t_b)

  def flag(self, name: str, key: str) -> bool:
    """A BoolValue in the object's user data (False if absent)."""
    value = wrappers_pb2.BoolValue()
    user_data = self.get(name).object_component.user_data
    if key in user_data:
      user_data[key].Unpack(value)
    return value.value

  def set_flag(self, name: str, key: str, value: bool, world_id: str = WORLD) -> None:
    """Sets a BoolValue in the object's user data."""
    request = updates_pb.UpdateObjectPropertiesRequest(
        world_id=world_id, object=_by_name(name),
        user_data=user_data_pb2.UpdateUserData(policy=user_data_pb2.UpdateUserData.POLICY_INSERT_OR_UPDATE))
    request.user_data.user_data[key].Pack(wrappers_pb2.BoolValue(value=value))
    self._call("UpdateObjectProperties", request)

  def joints(self, name: str) -> list[float]:
    """Joint positions of a kinematic object (rad)."""
    obj = self._call("GetObject", world_pb.GetObjectRequest(world_id=WORLD, object=_by_name(name), view=updates_pb.FULL))
    return list(obj.kinematic_object_component.joint_positions)

  def attach(self, name: str) -> None:
    """Parents the object to the flange of `franka`, keeping its pose."""
    self._call(
        "ReparentObject",
        updates_pb.ReparentObjectRequest(
            world_id=WORLD, object=_by_name(name),
            parent_frame=refs_pb.FrameReference(by_name=refs_pb.FrameReferenceByName(object_name="franka", frame_name="flange")),
        ),
    )

  def detach(self, name: str) -> None:
    """Parents the object back to the root, keeping its pose."""
    self._call(
        "ReparentObject",
        updates_pb.ReparentObjectRequest(
            world_id=WORLD, object=_by_name(name),
            parent_object=refs_pb.ObjectReferenceWithEntityFilter(
                reference=refs_pb.ObjectReference(id="root"),
                entity_filter=refs_pb.ObjectEntityFilter(include_final_entity=True),
            ),
        ),
    )

  def attached(self) -> list[str]:
    """Objects (other than the robot's own) whose parent is not the root."""
    return [o.name for o in self.objects() if o.name not in ROBOT_OBJECTS and o.parent.name != "root"]

  @staticmethod
  def source(o) -> str:
    if "source" not in o.object_component.user_data:
      return ""
    value = wrappers_pb2.StringValue()
    o.object_component.user_data["source"].Unpack(value)
    return value.value


class Robot:
  """State shared by the three services: ICON, planner, world, operations, payload, speed."""

  def __init__(self, args, mirror_world: bool):
    self.icon = Icon(args.icon)
    self.planner = Planner(args.planner)
    self.chain = ArmChain()
    self.world = World(args.world)
    self.camera = camera_grpc.CameraServiceStub(grpc.insecure_channel(args.camera))
    self.calibration = calib_grpc.CalibrationServiceStub(grpc.insecure_channel(args.calibration))
    self.simlink = simlink_pb2_grpc.SimLinkStub(grpc.insecure_channel(args.simlink))
    self.clock = SimClock()
    self.grasps = grasp.GraspPlanner()
    self.ops = OperationTable()
    self.mirror_world = mirror_world
    self.lock = threading.Lock()
    self.recover_lock = threading.Lock()
    self.reset_lock = threading.Lock()  # one ResetScene at a time
    self.motion: Operation | None = None  # the arm operation (motion or jog), one at a time
    self.gripper_op: Operation | None = None
    self.last_grasp = False
    self.payload = (0.0, (0.0, 0.0, 0.0))
    self.speed = 1.0
    self.resetting = threading.Event()
    threading.Thread(target=self._watch_faults, daemon=True).start()

  # ---- dependencies ----

  def sim(self, method: str, request):
    try:
      return getattr(self.simlink, method)(request, timeout=30.0)
    except grpc.RpcError as error:
      raise _dependency_error(f"scene_sync SimLink.{method}", error) from error

  def capture(self, sensor_id: int):
    """One CameraService.Capture of sensor 1 (color) or 4 (depth); returns the SensorImage."""
    identifier = camera_identifier_pb2.CameraIdentifier()
    identifier.ros.device_id = CAMERA_OBJECT
    request = camera_pb.CaptureRequest(camera_config=camera_config_pb2.CameraConfig(identifier=identifier),
                                       sensor_ids=[sensor_id])
    try:
      result = self.camera.Capture(request, timeout=30.0).capture_result
    except grpc.RpcError as error:
      raise _dependency_error("camera_sim CameraService.Capture", error) from error
    image = next((i for i in result.sensor_images if i.sensor_config.id == sensor_id), None)
    if image is None:
      raise ApiError(grpc.StatusCode.INTERNAL, f"camera_sim returned no image of sensor {sensor_id}")
    return image

  # ---- operations ----

  def reply(self, op: Operation, wait: bool) -> operations_pb2.Operation:
    """wait=true: blocks up to 60 s; an operation that failed in that time fails the RPC with its status."""
    if wait and self.ops.wait(op, WAIT_S) and op.error is not None:
      raise ApiError(op.error.code, f"{op.name}: {op.error.message}")
    return self.ops.to_proto(op)

  def _worker(self, op: Operation, work) -> None:
    def run():
      try:
        work(op)
      except ApiError as error:
        self.ops.fail(op, error)
      except Exception as error:  # pylint: disable=broad-except
        log.exception("operation %s crashed", op.name)
        self.ops.fail(op, ApiError(grpc.StatusCode.INTERNAL, f"{type(error).__name__}: {error}"))
      finally:
        op.worker_done.set()
        level = logging.INFO if op.error is None else logging.WARNING
        log.log(level, "operation %s %s state=%s detail=%s", op.name, op.kind, op.state, op.detail)

    threading.Thread(target=run, daemon=True).start()

  def start_arm(self, kind: str, work, detail: str = "planning") -> Operation:
    """Claims the arm for one operation (FAILED_PRECONDITION if faulted or another one runs)."""
    self.icon.require_enabled()
    with self.lock:
      if self.resetting.is_set():
        raise ApiError(grpc.StatusCode.FAILED_PRECONDITION, "ResetScene is in progress")
      # op.done, not worker_done: a motion cancelled while planning ends at once (its worker sees it later).
      if self.motion is not None and not self.motion.done:
        raise ApiError(grpc.StatusCode.FAILED_PRECONDITION,
                       f"another motion is running ({self.motion.name}); wait for it or call Stop")
      op = self.motion = self.ops.create(kind, PLANNING if detail == "planning" else EXECUTING, detail)
    self._worker(op, work)
    return op

  def start_motion(self, kind: str, segments, curve=None) -> Operation:
    return self.start_arm(kind, lambda op: self._run_motion(op, segments, curve))

  def _run_motion(self, op: Operation, segments, curve) -> None:
    took, planned = self.execute(op, segments, curve)
    self.ops.finish(op, api.MotionResult(final_joints=api.Joints(positions=self.icon.arm_joints()), duration_s=took),
                    f"done in {took:.2f} s of sim time (planned {planned:.2f} s)")

  def execute(self, op: Operation, segments, curve=None, detail: str | None = None) -> tuple[float, float]:
    """Plans and runs `segments` as part of `op`; returns (sim seconds taken, planned seconds).

    The operation becomes EXECUTING with `detail` (default: the planned duration).
    """
    start = self.icon.arm_joints()
    trajectory = self.planner.plan(start, segments, curve).discretized
    planned = duration_s(trajectory) / self.speed  # the speed override stretches execution
    if op.cancel.is_set() or not self.ops.update(op, EXECUTING, detail or f"executing a {planned:.2f} s trajectory",
                                                 planned_duration_s=planned):
      raise ApiError(grpc.StatusCode.CANCELLED, "stopped while planning")
    timeout = 2 * planned + 2.0
    times = {}

    def stop_when(done: bool) -> bool:
      if done:
        return True
      if self.clock.now() - times["start"] > timeout:
        raise ApiError(grpc.StatusCode.DEADLINE_EXCEEDED,
                       f"trajectory not finished after {timeout:.1f} s of sim time (planned {planned:.2f} s)")
      return False

    action = trajectory_tracking_action_utils.create_trajectory_tracking_action(1, ARM, trajectory)
    self.icon.run_action([ARM], action, stop_when, op.cancel,
                         on_started=lambda: times.setdefault("start", self.clock.now()))
    took = self.clock.now() - times["start"]
    self._wait_arm_at(trajectory.state[-1].position)
    return took, planned

  def _wait_arm_at(self, target) -> None:
    """Waits until the sensed joints reach `target`.

    Isaac's joint drives lag the setpoints; ICON rejects the next trajectory if its start (the sensed
    joints) is more than 0.1 rad from the last setpoint.
    """
    start = self.clock.now()
    while True:
      error = max(abs(a - b) for a, b in zip(self.icon.arm_joints(), target))
      if error <= SETTLE_TOLERANCE_RAD:
        return
      if self.clock.now() - start > SETTLE_TIMEOUT_S:
        raise ApiError(grpc.StatusCode.DEADLINE_EXCEEDED,
                       f"arm {error:.3f} rad from the trajectory end {SETTLE_TIMEOUT_S:.0f} s of sim time after it")
      time.sleep(0.02)

  def stop(self) -> None:
    """Cancels the running arm operation, then intrinsic.stop in a new session (decision 19)."""
    op = self.motion
    if op is not None and not op.done:
      op.cancel.set()
      if op.state == PLANNING:
        self.ops.fail(op, ApiError(grpc.StatusCode.CANCELLED, "stopped while planning"))
      elif not op.worker_done.wait(STOP_WAIT_S):
        raise ApiError(grpc.StatusCode.DEADLINE_EXCEEDED, f"{op.name} did not end within {STOP_WAIT_S:.0f} s of Stop")
    state, reason = self.icon.operational()
    if state != "ENABLED":
      log.info("stop: ICON is %s (%s); no intrinsic.stop needed", state, reason)
      return
    start = self.clock.now()

    def settled(done: bool) -> bool:
      if not done and self.clock.now() - start > STOP_SETTLE_TIMEOUT_S:
        raise ApiError(grpc.StatusCode.DEADLINE_EXCEEDED, f"arm not settled {STOP_SETTLE_TIMEOUT_S:.0f} s after intrinsic.stop")
      return done

    self.icon.run_action([ARM], stop_utils.create_stop_action(1, ARM), settled,
                         condition=reactions.Condition.is_true(stop_utils.StateVariables.IS_SETTLED))

  def start_gripper(self, kind: str, command: int, resetting_ok: bool = False) -> Operation:
    self.icon.require_enabled()
    with self.lock:
      if self.resetting.is_set() and not resetting_ok:
        raise ApiError(grpc.StatusCode.FAILED_PRECONDITION, "ResetScene is in progress")
      if self.gripper_op is not None and not self.gripper_op.worker_done.is_set():
        raise ApiError(grpc.StatusCode.FAILED_PRECONDITION, f"another gripper call is running ({self.gripper_op.name})")
      op = self.gripper_op = self.ops.create(kind, EXECUTING, f"{kind} started")
      self.last_grasp = command == simple_gripper_pb2.SimpleGripperFixedParams.GRASP
    self._worker(op, lambda op: self._run_gripper(op, command))
    return op

  def gripper_width(self) -> float:
    return self.icon.status().part_status[GRIPPER].linear_gripper_state.sensed_width

  def _run_gripper(self, op: Operation, command: int) -> None:
    grasp = command == simple_gripper_pb2.SimpleGripperFixedParams.GRASP
    state = {}

    def started() -> None:
      state["start"] = self.clock.now()
      state["detector"] = gripper.StallDetector(state["start"])

    def complete(done: bool) -> bool:
      t, width = self.clock.now(), self.gripper_width()
      if grasp:
        return state["detector"].update(t, width) or done
      return gripper.release_complete(state["start"], t, width) or done

    action = icon_actions.Action(1, "intrinsic.simple_gripper", GRIPPER,
                                 simple_gripper_pb2.SimpleGripperFixedParams(command=command))
    self.icon.run_action([GRIPPER], action, complete, op.cancel, on_started=started)
    width = self.gripper_width()
    holding = gripper.gripping(grasp, width)
    self.ops.finish(op, api.GripperResult(width=width, gripping=holding),
                    f"width {width * 1e3:.1f} mm after {self.clock.now() - state['start']:.2f} s, gripping {holding}")

  # ---- faults ----

  def recover(self) -> None:
    """ClearFaults, wait for ENABLED, re-apply payload and speed (decision 24; tare is not re-applied)."""
    with self.recover_lock:
      self.icon.clear_faults()
      mass, com = self.payload
      if mass > 0:
        self.icon.set_payload(mass, com)
      if self.speed != 1.0:
        self.icon.set_speed(self.speed)
      log.info("faults cleared; re-applied payload=%.3f kg speed=%.2f", mass, self.speed)

  def _watch_faults(self) -> None:
    faulted_since, waiting_for_isaac = None, False
    while True:
      time.sleep(FAULT_WATCH_PERIOD_S)
      try:
        state, reason = self.icon.operational()
      except ApiError:
        faulted_since = None
        continue
      if state != "FAULTED" or self.resetting.is_set():
        faulted_since, waiting_for_isaac = None, False
        continue
      if not self.clock.advancing(CLOCK_ADVANCING_S):
        if not waiting_for_isaac:
          log.warning("ICON faulted (%s) and Isaac's /clock is not advancing; leaving the fault until it does", reason)
        faulted_since, waiting_for_isaac = None, True
        continue
      faulted_since = faulted_since or time.monotonic()
      if time.monotonic() - faulted_since > AUTO_CLEAR_AFTER_S:
        log.warning("ICON faulted for > %.0f s (%s); clearing faults", AUTO_CLEAR_AFTER_S, reason)
        try:
          self.recover()
        except ApiError as error:
          log.error("automatic ClearFaults failed: %s", error)
        faulted_since = None


def _summary(message) -> str:
  return text_format.MessageToString(message, as_one_line=True)[:SUMMARY_MAX_CHARS]


class LoggingInterceptor(grpc.ServerInterceptor):
  """One log line per RPC (method, request, ms, status, message); ApiError and dependency errors become
  the RPC status, anything else INTERNAL with the exception text (decision 28)."""

  def intercept_service(self, continuation, handler_call_details):
    handler = continuation(handler_call_details)
    if handler is None or handler.unary_unary is None:
      return handler
    method, inner = handler_call_details.method.rsplit("/", 1)[-1], handler.unary_unary

    def wrapper(request, context):
      start = time.monotonic()
      code, message = grpc.StatusCode.OK, ""
      try:
        return inner(request, context)
      except ApiError as error:
        code, message = error.code, error.message
      except grpc.RpcError as error:
        code, message = (error.code(), error.details()) if isinstance(error, grpc.Call) else (grpc.StatusCode.INTERNAL, str(error))
      except Exception as error:  # pylint: disable=broad-except
        if context.code() is not None:  # context.abort() was called
          code, message = context.code(), context.details()
          raise
        log.exception("%s crashed", method)
        code, message = grpc.StatusCode.INTERNAL, f"{type(error).__name__}: {error}"
      finally:
        level = logging.INFO if code == grpc.StatusCode.OK else logging.WARNING
        log.log(level, "rpc %s request={%s} ms=%.0f status=%s message=%s",
                method, _summary(request), (time.monotonic() - start) * 1e3, code.name, message)
      context.abort(code, message)

    return grpc.unary_unary_rpc_method_handler(
        wrapper, request_deserializer=handler.request_deserializer, response_serializer=handler.response_serializer
    )


class CalibrationArm:
  """What calibration.run needs from the robot, inside one CalibrateCamera operation."""

  def __init__(self, robot: Robot, op: Operation):
    self.r, self.op = robot, op
    self._plan_from: list[float] | None = None  # end joints of the last kept pose's plan

  def board_attached(self) -> bool:
    return CALIB_BOARD in self.r.world.attached()

  def camera_pose(self) -> api.Pose:
    """The belief (seed or last calibration), never Isaac's true pose."""
    return self.r.world.frame_pose(CAMERA_OBJECT, SENSOR_FRAME)

  def move(self, pose: api.Pose, detail: str) -> None:
    self.r.execute(self.op, [segment(api.Waypoint(pose=pose), api.FLANGE)], detail=detail)

  def home(self) -> None:
    self.r.execute(self.op, [segment(api.Waypoint(joints=api.Joints(positions=HOME)), api.TCP)], detail="homing")

  def sample(self, start: api.Pose, camera: api.Pose) -> list[api.Pose]:
    return calibration.sample_poses(start, camera)

  def feasible(self, pose: api.Pose) -> bool:
    """Decision 11.4: a collision-free IK solution and a planned trajectory from the last kept pose."""
    start = self._plan_from or self.r.icon.arm_joints()
    try:
      q = self.r.planner.ik(pose, api.FLANGE, start)
      in_collision, debug = self.r.planner.check_collisions(q)
      if in_collision:
        log.info("calibration: pose rejected: IK solution in collision: %s", debug)
        return False
      trajectory = self.r.planner.plan(start, [segment(api.Waypoint(pose=pose), api.FLANGE)]).discretized
    except ApiError as error:
      if error.code not in POSE_REJECTED_CODES:
        raise
      log.info("calibration: pose rejected: %s %s", error.code.name, error.message)
      return False
    self._plan_from = list(trajectory.state[-1].position)
    return True

  def wait_synced(self) -> bool:
    """Decision 9: CAPTURE_SETTLE_S of sim time, then world's franka joints equal ICON's sensed joints."""
    start = self.r.clock.now()
    while self.r.clock.now() - start < CAPTURE_SETTLE_S:
      time.sleep(SYNC_POLL_S)
    deadline = time.monotonic() + SYNC_TIMEOUT_S
    while True:
      error = max(abs(a - b) for a, b in zip(self.r.world.joints(ROBOT), self.r.icon.arm_joints(), strict=True))
      if error <= SYNC_TOLERANCE_RAD:
        return True
      if time.monotonic() > deadline:
        log.info("calibration: world's %s joints %.5f rad from ICON's after %.0f s", ROBOT, error, SYNC_TIMEOUT_S)
        return False
      time.sleep(SYNC_POLL_S)

  def report(self, detail: str) -> None:
    if self.op.cancel.is_set() or not self.r.ops.update(self.op, detail=detail):
      raise ApiError(grpc.StatusCode.CANCELLED, "calibration cancelled")

  def mark_calibrated(self) -> None:
    for world_id in (WORLD, INIT_WORLD):
      self.r.world.set_flag(CAMERA_OBJECT, CALIBRATED_KEY, True, world_id)


def _unimplemented(what: str, phase: str):
  def rpc(self, request, context):
    raise ApiError(grpc.StatusCode.UNIMPLEMENTED, f"{what} is not implemented yet (phase {phase})")
  return rpc


class RobotService(api_grpc.RobotServiceServicer):

  def __init__(self, robot: Robot):
    self.r = robot

  def GetState(self, request, context):
    state, reason = self.r.icon.operational()
    response = api.GetStateResponse(enabled=state == "ENABLED", faulted=state == "FAULTED", fault_reason=reason,
                                    speed_factor=self.r.speed)
    try:
      status = self.r.icon.status()
    except ApiError:
      if state == "FAULTED":
        return response  # ICON's status is unavailable while faulted; the fault fields say why
      raise
    q = self.r.icon.arm_joints(status)
    w = status.part_status[FT_SENSOR].wrench_at_ft
    width = status.part_status[GRIPPER].linear_gripper_state.sensed_width
    response.joints.positions.extend(q)
    response.tool_pose.CopyFrom(self.r.planner.fk(q, api.TCP))
    response.wrench.CopyFrom(api.Wrench(fx=w.x, fy=w.y, fz=w.z, tx=w.rx, ty=w.ry, tz=w.rz))
    response.gripper_width = width
    response.gripping = gripper.gripping(self.r.last_grasp, width)
    return response

  def MoveToPose(self, request, context):
    waypoint = api.Waypoint(pose=request.pose, motion=request.motion)
    return self.r.reply(self.r.start_motion("MoveToPose", [segment(waypoint, request.tool)]), request.wait)

  def MoveJoints(self, request, context):
    waypoint = api.Waypoint(joints=request.joints)
    return self.r.reply(self.r.start_motion("MoveJoints", [segment(waypoint, api.TCP)]), request.wait)

  def Home(self, request, context):
    waypoint = api.Waypoint(joints=api.Joints(positions=HOME))
    return self.r.reply(self.r.start_motion("Home", [segment(waypoint, api.TCP)]), request.wait)

  @staticmethod
  def _segments(waypoints, tool):
    if not waypoints:
      raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, "no waypoints")
    return [segment(w, tool) for w in waypoints], blending(waypoints)

  def MoveWaypoints(self, request, context):
    segments, curve = self._segments(request.waypoints, request.tool)
    return self.r.reply(self.r.start_motion("MoveWaypoints", segments, curve), request.wait)

  def PlanOnly(self, request, context):
    segments, curve = self._segments(request.waypoints, request.tool)
    trajectory = self.r.planner.plan(self.r.icon.arm_joints(), segments, curve).discretized
    swept_min, swept_max = self.r.planner.swept_bounds(trajectory, self.r.chain)
    return api.PlanOnlyResponse(
        duration_s=duration_s(trajectory) / self.r.speed,  # as the operation's planned_duration_s
        final_joints=api.Joints(positions=trajectory.state[-1].position),
        swept_min=swept_min,
        swept_max=swept_max,
    )

  def _seed(self, seed) -> list[float]:
    return joints(seed.positions, "seed") if seed.positions else self.r.icon.arm_joints()

  def ComputeIk(self, request, context):
    return api.ComputeIkResponse(joints=api.Joints(positions=self.r.planner.ik(request.pose, request.tool, self._seed(request.seed))))

  def ComputeFk(self, request, context):
    return api.ComputeFkResponse(pose=self.r.planner.fk(request.joints.positions, request.tool))

  def CheckCollisions(self, request, context):
    if request.WhichOneof("target") == "pose":
      q = self.r.planner.ik(request.pose, request.tool, self.r.icon.arm_joints())
    elif request.WhichOneof("target") == "joints":
      q = request.joints.positions
    else:
      raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, "set joints or pose")
    in_collision, debug = self.r.planner.check_collisions(q)
    # The planner reports pairs only in its debug text; it is passed on as one entry.
    return api.CheckCollisionsResponse(in_collision=in_collision, colliding_pairs=[debug] if in_collision and debug else [])

  def GetWrench(self, request, context):
    w = self.r.icon.status().part_status[FT_SENSOR].wrench_at_ft
    return api.Wrench(fx=w.x, fy=w.y, fz=w.z, tx=w.rx, ty=w.ry, tz=w.rz)

  def TareWrench(self, request, context):
    self.r.icon.require_enabled()
    start = self.r.clock.now()

    def done(finished: bool) -> bool:
      if not finished and self.r.clock.now() - start > TARE_TIMEOUT_S:
        raise ApiError(grpc.StatusCode.DEADLINE_EXCEEDED, f"taring not finished after {TARE_TIMEOUT_S:.0f} s of sim time")
      return finished

    self.r.icon.run_action([FT_SENSOR], tare_force_torque_sensor_utils.create_tare_force_torque_sensor_action(1, FT_SENSOR), done)
    return empty_pb2.Empty()

  MoveToContact = _unimplemented("MoveToContact", "5b")
  MoveCompliant = _unimplemented("MoveCompliant", "5b")

  def PlanGrasp(self, request, context):
    return self.r.grasps.plan_grasp(self.r.world, request.object)

  def CalibrateCamera(self, request, context):
    def work(op: Operation) -> None:
      result = calibration.run(CalibrationArm(self.r, op), self.r.calibration)
      self.r.ops.finish(op, result, f"calibrated from {result.poses_used} captures: rms {result.rms_translation_m * 1e3:.2f} mm"
                                    f" / {result.rms_rotation_deg:.3f} deg")

    return self.r.reply(self.r.start_arm("CalibrateCamera", work), request.wait)

  DetectObjects = _unimplemented("DetectObjects", "5")

  def GetOperation(self, request, context):
    return get_operation(self.r, request)

  def WaitOperation(self, request, context):
    return wait_operation(self.r, request)

  def Stop(self, request, context):
    self.r.stop()
    return empty_pb2.Empty()

  def SetSpeed(self, request, context):
    if not 0 < request.factor <= 1:
      raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, f"factor must be in (0, 1], got {request.factor}")
    if self.r.motion is not None and not self.r.motion.done:
      raise ApiError(grpc.StatusCode.FAILED_PRECONDITION,
                     f"the speed applies to following motions; {self.r.motion.name} is running")
    self.r.icon.set_speed(request.factor)
    self.r.speed = request.factor
    return empty_pb2.Empty()

  def ClearFaults(self, request, context):
    self.r.recover()
    return empty_pb2.Empty()

  def Jog(self, request, context):
    if not 1 <= request.joint <= 7:
      raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, f"joint must be 1..7, got {request.joint}")
    if not (math.isfinite(request.duration_s) and request.duration_s > 0):
      raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, f"duration_s must be > 0, got {request.duration_s}")
    index = request.joint - 1
    max_velocity = self.r.icon.joint_max_velocities()[index]
    normalized = request.velocity / max_velocity
    if not abs(normalized) <= 1:
      raise ApiError(grpc.StatusCode.INVALID_ARGUMENT,
                     f"|velocity| must be <= {max_velocity:.3f} rad/s for joint {request.joint}, got {request.velocity}")

    def run(op: Operation) -> None:
      times = {}

      def keep_going() -> bool:
        now = self.r.clock.now()
        times.setdefault("start", now)
        return not op.cancel.is_set() and now - times["start"] < request.duration_s

      self.r.icon.jog(index, normalized, keep_going, request.duration_s)
      if op.cancel.is_set():
        raise ApiError(grpc.StatusCode.CANCELLED, "jog stopped")
      self.r.ops.finish(op, empty_pb2.Empty(), f"jogged joint {request.joint} for {request.duration_s} s")

    op = self.r.start_arm("Jog", run, detail=f"jogging joint {request.joint} at {request.velocity} rad/s")
    wait_s = request.duration_s * 10 + 10
    if not self.r.ops.wait(op, wait_s):
      raise ApiError(grpc.StatusCode.DEADLINE_EXCEEDED, f"{op.name} still running after {wait_s:.0f} s")
    if op.error is not None:
      raise op.error
    return empty_pb2.Empty()

  def Grasp(self, request, context):
    return self.r.reply(self.r.start_gripper("Grasp", simple_gripper_pb2.SimpleGripperFixedParams.GRASP), request.wait)

  def Release(self, request, context):
    return self.r.reply(self.r.start_gripper("Release", simple_gripper_pb2.SimpleGripperFixedParams.RELEASE), request.wait)

  def IsGripping(self, request, context):
    width = self.r.gripper_width()
    return api.IsGrippingResponse(gripping=gripper.gripping(self.r.last_grasp, width), width=width)

  @staticmethod
  def _require_scene_object(name: str) -> None:
    if name in ROBOT_OBJECTS:
      raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, f"{name!r} is part of the robot")
    if name == CAMERA_OBJECT:
      raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, f"{name!r} is the camera, not a scene object")

  def AttachObject(self, request, context):
    self._require_scene_object(request.name)
    self.r.world.get(request.name)
    self.r.world.attach(request.name)
    self.r.sim("SetAttached", simlink_pb2.SetAttachedRequest(name=request.name, attached=True))
    return empty_pb2.Empty()

  def DetachObject(self, request, context):
    self._require_scene_object(request.name)
    if self.r.world.get(request.name).parent.name != "root":
      self.r.world.detach(request.name)
    self.r.sim("SetAttached", simlink_pb2.SetAttachedRequest(name=request.name, attached=False))
    if self.r.mirror_world:
      self._wait_synced(request.name)
    return empty_pb2.Empty()

  def _wait_synced(self, name: str) -> None:
    """Waits until scene_sync (which skips attached objects) has moved `name` to its true pose."""
    deadline = time.monotonic() + OBJECT_WAIT_S
    while True:
      true = self.r.sim("GetTruePose", simlink_pb2.TruePoseRequest(frame=name))
      belief = self.r.world.pose(name)
      error = math.dist((true.x, true.y, true.z), (belief.x, belief.y, belief.z))
      if error <= SYNC_TOLERANCE_M:
        return
      if time.monotonic() > deadline:
        raise ApiError(grpc.StatusCode.DEADLINE_EXCEEDED,
                       f"{name!r} in world is {error * 1e3:.0f} mm from its true pose {OBJECT_WAIT_S:.0f} s after detaching")
      time.sleep(0.05)

  def SetPayload(self, request, context):
    values = (request.mass, request.com_x, request.com_y, request.com_z)
    if not all(math.isfinite(v) for v in values) or request.mass < 0:
      raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, f"mass must be >= 0 and all values finite, got {values}")
    self.r.icon.require_enabled()
    if self.r.motion is not None and not self.r.motion.done:
      raise ApiError(grpc.StatusCode.FAILED_PRECONDITION, f"ICON sets the payload only between motions ({self.r.motion.name} runs)")
    com = (request.com_x, request.com_y, request.com_z)
    self.r.icon.set_payload(request.mass, com)
    self.r.payload = (request.mass, com)
    return empty_pb2.Empty()

  def ListObjects(self, request, context):
    response = api.ListObjectsResponse()
    for o in self.r.world.objects():
      if o.name in ROBOT_OBJECTS or o.name == CAMERA_OBJECT:
        continue
      response.objects.add(name=o.name, source=World.source(o), pose=self.r.world.pose(o.name),
                           attached_to="" if o.parent.name == "root" else o.parent.name)
    return response

  def GetImage(self, request, context):
    depth = request.kind == api.DEPTH
    image = self.r.capture(4 if depth else 1)
    buffer = image.buffer
    width, height = buffer.dimensions.cols, buffer.dimensions.rows
    if depth:
      metres = np.frombuffer(buffer.data, dtype="<f4").reshape(height, width)
      valid = np.isfinite(metres) & (metres > 0)
      mm = np.where(valid, np.clip(np.round(metres * 1000.0), 0, 65535), 0).astype(">u2")
      data = png.encode(mm.tobytes(), width, height, 1, 16)
    else:
      data = png.encode(buffer.data, width, height, 3, 8)
    stamp = image.acquisition_time
    return api.GetImageResponse(kind=api.DEPTH if depth else api.RGB, width=width, height=height, png=data,
                                sim_time=stamp.seconds + stamp.nanos * 1e-9)

  def GetPointCloud(self, request, context):
    if request.stride < 0:
      raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, f"stride must be >= 0 (0 = 4), got {request.stride}")
    stride = request.stride or POINT_CLOUD_STRIDE
    image = self.r.capture(4)
    buffer, k = image.buffer, image.sensor_config.camera_params.intrinsic_params
    z = np.frombuffer(buffer.data, dtype="<f4")
    v, u = np.divmod(np.arange(z.size), buffer.dimensions.cols)
    z, u, v = z[::stride], u[::stride], v[::stride]  # every stride-th pixel in row-major order
    valid = np.isfinite(z) & (z > 0)
    z, u, v = z[valid], u[valid], v[valid]
    xyz = np.stack([(u - k.principal_point_x) * z / k.focal_length_x,
                    (v - k.principal_point_y) * z / k.focal_length_y, z], axis=1).astype(np.float32)
    response = api.GetPointCloudResponse(camera_pose=self.r.world.frame_pose(CAMERA_OBJECT, SENSOR_FRAME),
                                         camera_calibrated=self.r.world.flag(CAMERA_OBJECT, CALIBRATED_KEY))
    response.xyz.extend(xyz.ravel().tolist())
    return response


class SimService(api_grpc.SimServiceServicer):

  def __init__(self, robot: Robot):
    self.r = robot

  def _wait_in_world(self, name: str, present: bool) -> None:
    if not self.r.mirror_world:
      return  # with belief_source PERCEPTION scene_sync does not write `world`
    deadline = time.monotonic() + OBJECT_WAIT_S
    while self.r.world.exists(name) != present:
      if time.monotonic() > deadline:
        raise ApiError(grpc.StatusCode.DEADLINE_EXCEEDED,
                       f"{name!r} {'not in' if present else 'still in'} world {OBJECT_WAIT_S:.0f} s after the Isaac call")
      time.sleep(0.05)

  def AddObject(self, request, context):
    self.r.sim("Spawn", simlink_pb2.SpawnRequest(catalog=request.catalog, name=request.name, pose=request.pose))
    self._wait_in_world(request.name, True)
    return empty_pb2.Empty()

  def RemoveObject(self, request, context):
    self.r.sim("Delete", simlink_pb2.NameRequest(name=request.name))
    self._wait_in_world(request.name, False)
    return empty_pb2.Empty()

  def ResetScene(self, request, context):
    """Decision 25: cancel, detach + release, SimLink.Reset, wait FAULTED, ClearFaults, payload and speed reset."""
    r = self.r
    if not r.reset_lock.acquire(blocking=False):
      raise ApiError(grpc.StatusCode.FAILED_PRECONDITION, "another ResetScene is in progress")
    try:
      r.resetting.set()
      # recover_lock: a watcher recover() that started before `resetting` must not re-apply the old
      # payload and speed after the reset.
      with r.recover_lock:
        self._reset()
    finally:
      r.resetting.clear()
      r.reset_lock.release()
    return empty_pb2.Empty()

  def _reset(self) -> None:
    r = self.r
    for op in (r.motion, r.gripper_op):
      if op is not None and not op.done:
        op.cancel.set()
        if op.state == PLANNING:
          r.ops.fail(op, ApiError(grpc.StatusCode.CANCELLED, "cancelled by ResetScene"))
        elif not op.worker_done.wait(STOP_WAIT_S):
          raise ApiError(grpc.StatusCode.DEADLINE_EXCEEDED, f"{op.name} did not end within {STOP_WAIT_S:.0f} s")
    for name in r.world.attached():
      r.world.detach(name)
      r.sim("SetAttached", simlink_pb2.SetAttachedRequest(name=name, attached=False))
    if r.icon.operational()[0] == "ENABLED":
      release = r.start_gripper("Release", simple_gripper_pb2.SimpleGripperFixedParams.RELEASE, resetting_ok=True)
      r.ops.wait(release, WAIT_S)
    epoch = r.sim("Reset", empty_pb2.Empty()).epoch
    state, reason = r.icon.wait_state("FAULTED", RESET_FAULT_WAIT_S)
    log.info("reset: epoch %d, ICON %s (%s)", epoch, state, reason)
    r.icon.clear_faults(RESET_ENABLED_WAIT_S)
    r.icon.set_payload(0.0, (0.0, 0.0, 0.0))
    r.icon.set_speed(1.0)
    r.payload, r.speed, r.last_grasp = (0.0, (0.0, 0.0, 0.0)), 1.0, False

  def GetTruePose(self, request, context):
    return self.r.sim("GetTruePose", simlink_pb2.TruePoseRequest(frame=request.frame))

  def AttachCalibrationBoard(self, request, context):
    self.r.sim("AttachBoard", empty_pb2.Empty())
    if self.r.mirror_world:
      self._wait_in_world(CALIB_BOARD, True)
      self.r.world.attach(CALIB_BOARD)
      self.r.sim("SetAttached", simlink_pb2.SetAttachedRequest(name=CALIB_BOARD, attached=True))
    return empty_pb2.Empty()

  def DetachCalibrationBoard(self, request, context):
    """Decision 10: scene_sync deletes calib_board from the worlds once it has left Isaac's /tf."""
    self.r.sim("SetAttached", simlink_pb2.SetAttachedRequest(name=CALIB_BOARD, attached=False))
    self.r.sim("DetachBoard", empty_pb2.Empty())
    return empty_pb2.Empty()


def get_operation(r: Robot, request) -> operations_pb2.Operation:
  return r.ops.to_proto(r.ops.get(request.name))


def wait_operation(r: Robot, request) -> operations_pb2.Operation:
  op = r.ops.get(request.name)
  timeout = request.timeout.ToTimedelta().total_seconds() if request.HasField("timeout") else WAIT_S
  r.ops.wait(op, min(timeout, WAIT_S))
  return r.ops.to_proto(op)


class Operations(operations_pb2_grpc.OperationsServicer):

  def __init__(self, robot: Robot):
    self.r = robot

  def GetOperation(self, request, context):
    return get_operation(self.r, request)

  def ListOperations(self, request, context):
    return operations_pb2.ListOperationsResponse(operations=[self.r.ops.to_proto(o) for o in self.r.ops.list()])

  def DeleteOperation(self, request, context):
    self.r.ops.delete(request.name)
    return empty_pb2.Empty()

  def WaitOperation(self, request, context):
    return wait_operation(self.r, request)

  def CancelOperation(self, request, context):
    """The running motion: same as Stop. Others: cancelled at their next check."""
    op = self.r.ops.get(request.name)
    if op is self.r.motion:
      self.r.stop()
    else:
      op.cancel.set()
    return empty_pb2.Empty()


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--port", type=int, default=18100)
  parser.add_argument("--icon", default="localhost:18003")
  parser.add_argument("--planner", default="localhost:18002")
  parser.add_argument("--world", default="localhost:18000")
  parser.add_argument("--camera", default="127.0.0.1:18005")
  parser.add_argument("--simlink", default="127.0.0.1:18101")
  parser.add_argument("--calibration", default="localhost:18006")
  args = parser.parse_args()
  logging.basicConfig(stream=sys.stdout, level=logging.INFO,
                      format="%(asctime)s %(levelname)s robot_service %(message)s")
  logging.getLogger("absl").setLevel(logging.WARNING)  # the ICON client logs every session start/end
  with open(SETTINGS_PATH) as f:
    belief_source = yaml.safe_load(f)["belief_source"]
  rclpy.init()
  robot = Robot(args, mirror_world=belief_source == "GROUND_TRUTH")
  server = grpc.server(futures.ThreadPoolExecutor(max_workers=16), interceptors=[LoggingInterceptor()])
  api_grpc.add_RobotServiceServicer_to_server(RobotService(robot), server)
  api_grpc.add_SimServiceServicer_to_server(SimService(robot), server)
  operations_pb2_grpc.add_OperationsServicer_to_server(Operations(robot), server)
  server.add_insecure_port(f"127.0.0.1:{args.port}")
  server.start()
  log.info("started port=127.0.0.1:%d belief_source=%s icon=%s planner=%s world=%s camera=%s simlink=%s calibration=%s",
           args.port, belief_source, args.icon, args.planner, args.world, args.camera, args.simlink, args.calibration)
  server.wait_for_termination()
  return 0


if __name__ == "__main__":
  sys.exit(main())
