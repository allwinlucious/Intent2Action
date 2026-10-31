"""Object model, backends, and the API exposed to generated programs."""

from dataclasses import asdict, dataclass, replace
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "gen"))

import grpc
from google.longrunning import operations_pb2
from google.protobuf import duration_pb2, empty_pb2, json_format

from robot_service import robot_service_pb2 as service_pb2  # noqa: E402
from robot_service import robot_service_pb2_grpc as service_pb2_grpc  # noqa: E402


IMMOVABLE = {"table", "box", "pillar", "block_with_hole"}
FILTER_FIELDS = {"category", "color", "size"}


def size_class(sx: float, sy: float, sz: float) -> str:
  """Returns the small, medium, or large class for object dimensions."""
  maximum = max(sx, sy, sz)
  if maximum < 0.04:
    return "small"
  if maximum < 0.06:
    return "medium"
  return "large"


def _field(value, name, default=None):
  if isinstance(value, dict):
    return value.get(name, default)
  return getattr(value, name, default)


@dataclass(frozen=True)
class Obj:
  """A stable snapshot of one perceived object."""

  id: str
  category: str
  color: str
  size: str
  x: float
  y: float
  z: float
  yaw: float
  sx: float
  sy: float
  sz: float
  attached: bool

  @classmethod
  def from_world_object(cls, value) -> "Obj":
    """Builds an object snapshot from a WorldObject message or equivalent dict."""
    pose = _field(value, "pose", {})
    qx = float(_field(pose, "qx", 0.0))
    qy = float(_field(pose, "qy", 0.0))
    qz = float(_field(pose, "qz", 0.0))
    qw = float(_field(pose, "qw", 0.0))
    sx = float(_field(value, "size_x", _field(value, "sx", 0.0)))
    sy = float(_field(value, "size_y", _field(value, "sy", 0.0)))
    sz = float(_field(value, "size_z", _field(value, "sz", 0.0)))
    return cls(
      id=str(_field(value, "id", "")),
      category=str(_field(value, "category", "")),
      color=str(_field(value, "color", "")),
      size=str(_field(value, "size", size_class(sx, sy, sz))),
      x=float(_field(pose, "x", _field(value, "x", 0.0))),
      y=float(_field(pose, "y", _field(value, "y", 0.0))),
      z=float(_field(pose, "z", _field(value, "z", 0.0))),
      yaw=math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)),
      sx=sx,
      sy=sy,
      sz=sz,
      attached=bool(_field(value, "attached", _field(value, "attached_to", "") != "")),
    )


class RobotFailure(Exception):
  """A classified failure from the robot-facing API."""

  def __init__(self, kind: str, message: str, call: str = "", line: int | None = None):
    super().__init__(message)
    self.kind = kind
    self.message = message
    self.call = call
    self.line = line


def classify(code_name: str, message: str) -> str:
  """Classifies an RPC or operation error using the ordered runtime rules."""
  code = (code_name or "").upper()
  text = (message or "").lower()
  if code == "DEADLINE_EXCEEDED":
    return "timeout"
  if "fault" in text or "not enabled" in text:
    return "fault"
  if code == "NOT_FOUND" and "no object with id" in text:
    return "perception"
  if "not holding" in text or "already holding" in text:
    return "precondition"
  if "grasp" in text or "closed on nothing" in text or "wider than the gripper" in text:
    return "grasp"
  if "free collision-free spot" in text or "box is full" in text or "place pose" in text:
    return "place"
  if "ik" in text or "reach" in text or "unreachable" in text:
    return "reachability"
  if "collision" in text:
    return "collision"
  return "other"


def _injection(backend, method: str) -> None:
  queue = backend.inject.get(method, [])
  if queue:
    failure = queue.pop(0)
    if failure is not None:
      raise failure


class GrpcBackend:
  """Robot backend using RobotService over gRPC."""

  def __init__(self, address: str = "127.0.0.1:18100", stub=None):
    self.address = address
    self.channel = None
    if stub is None:
      self.channel = grpc.insecure_channel(address)
      stub = service_pb2_grpc.RobotServiceStub(self.channel)
    self.stub = stub
    self.inject: dict[str, list[RobotFailure | None]] = {}

  def _rpc(self, name: str, request):
    try:
      return getattr(self.stub, name)(request, timeout=90)
    except grpc.RpcError as error:
      code = error.code().name
      message = error.details() or str(error)
      raise RobotFailure(classify(code, message), message) from error

  def list_objects(self) -> list[Obj]:
    _injection(self, "list_objects")
    response = self._rpc("ListObjects", empty_pb2.Empty())
    return [Obj.from_world_object(value) for value in response.objects]

  def _operation(self, method: str, request) -> None:
    operation = self._rpc(method, request)
    for _ in range(2):
      if operation.done:
        break
      wait = operations_pb2.WaitOperationRequest(
        name=operation.name, timeout=duration_pb2.Duration(seconds=60)
      )
      operation = self._rpc("WaitOperation", wait)
    if not operation.done:
      raise RobotFailure("timeout", f"operation {operation.name} is not done")
    if operation.HasField("error"):
      code = next(
        (item.name for item in grpc.StatusCode if item.value[0] == operation.error.code),
        str(operation.error.code),
      )
      raise RobotFailure(classify(code, operation.error.message), operation.error.message)

  def pick(self, object_id: str) -> None:
    _injection(self, "pick")
    self._operation("Pick", service_pb2.PickRequest(object_id=object_id, wait=True))

  def place(self, object_id: str, target_id: str) -> None:
    _injection(self, "place")
    request = service_pb2.PlaceRequest(object_id=object_id, target_id=target_id, wait=True)
    self._operation("Place", request)

  def state(self) -> dict:
    _injection(self, "state")
    response = self._rpc("GetState", empty_pb2.Empty())
    return json_format.MessageToDict(response, preserving_proto_field_name=True)


class FakeBackend:
  """In-memory backend for dry runs and tests."""

  def __init__(self, objects: list[Obj]):
    self.objects = {obj.id: replace(obj) for obj in objects}
    self.inject: dict[str, list[RobotFailure | None]] = {}

  def list_objects(self) -> list[Obj]:
    _injection(self, "list_objects")
    return [replace(obj) for obj in self.objects.values()]

  def pick(self, object_id: str) -> None:
    _injection(self, "pick")
    held = next((obj for obj in self.objects.values() if obj.attached and obj.id != object_id), None)
    if held:
      raise RobotFailure("precondition", f"already holding {held.id}")
    if object_id not in self.objects:
      raise RobotFailure("perception", f"object {object_id} no longer exists")
    self.objects[object_id] = replace(self.objects[object_id], attached=True)

  def place(self, object_id: str, target_id: str) -> None:
    _injection(self, "place")
    if object_id not in self.objects or target_id not in self.objects:
      missing = object_id if object_id not in self.objects else target_id
      raise RobotFailure("perception", f"object {missing} no longer exists")
    obj, target = self.objects[object_id], self.objects[target_id]
    if not obj.attached:
      raise RobotFailure("precondition", f"not holding {object_id}")
    x, y = target.x, target.y
    if target.category == "box":
      nx = math.floor((target.sx - 0.02) / 0.06)
      ny = math.floor((target.sy - 0.02) / 0.06)
      slots = []
      for row in range(ny):
        for column in range(nx):
          local_x = (column - (nx - 1) / 2) * 0.06
          local_y = (row - (ny - 1) / 2) * 0.06
          slots.append((
            target.x + math.cos(target.yaw) * local_x - math.sin(target.yaw) * local_y,
            target.y + math.sin(target.yaw) * local_x + math.cos(target.yaw) * local_y,
          ))
      occupied = [other for other in self.objects.values() if other.id not in {object_id, target_id}
                  and _inside_values(other, target)]
      slot = next((candidate for candidate in slots if all(
        math.hypot(candidate[0] - other.x, candidate[1] - other.y) >= 0.03 for other in occupied
      )), None)
      if slot is None:
        raise RobotFailure("place", "box is full")
      x, y = slot
      z = target.z + 0.01
    else:
      z = target.z + target.sz
    self.objects[object_id] = replace(obj, x=x, y=y, z=z, attached=False)

  def state(self) -> dict:
    return {}


def _inside_values(obj: Obj, box: Obj) -> bool:
  dx, dy = obj.x - box.x, obj.y - box.y
  local_x = math.cos(box.yaw) * dx + math.sin(box.yaw) * dy
  local_y = -math.sin(box.yaw) * dx + math.cos(box.yaw) * dy
  return (
    abs(local_x) <= box.sx / 2 and abs(local_y) <= box.sy / 2
    and box.z <= obj.z <= box.z + box.sz
  )


def _on_top_values(obj: Obj, target: Obj) -> bool:
  return (
    math.hypot(obj.x - target.x, obj.y - target.y) <= max(target.sx, target.sy) / 2
    and abs(obj.z - (target.z + target.sz)) <= 0.01
  )


def _call_text(name: str, *objects: Obj) -> str:
  return f"{name}({', '.join(obj.id for obj in objects)})"


class RobotAPI:
  """The small object-level API available to programs."""

  def __init__(self, backend):
    self.backend = backend
    self.report_lines: list[str] = []

  def find(self, **filters) -> Obj:
    """Find exactly one fresh object matching category, color, or size filters."""
    matches = self.find_all(**filters)
    if not matches:
      raise RobotFailure("perception", f"no object matches {filters}", f"find({filters})")
    if len(matches) != 1:
      raise RobotFailure(
        "perception", f"{len(matches)} objects match {filters}; add a filter", f"find({filters})"
      )
    return matches[0]

  def find_all(self, **filters) -> list[Obj]:
    """Find all fresh objects matching category, color, or size filters, sorted by id."""
    objects = self.backend.list_objects()
    if not filters:
      objects = [obj for obj in objects if obj.category not in IMMOVABLE]
    else:
      objects = [obj for obj in objects if all(getattr(obj, key, None) == value
                                               for key, value in filters.items())]
    return sorted(objects, key=lambda obj: obj.id)

  def pick(self, obj: Obj) -> None:
    """Pick an object and require it to be held afterward."""
    call = _call_text("pick", obj)
    try:
      self.backend.pick(obj.id)
    except RobotFailure as failure:
      failure.call = call
      raise
    if not self.held(obj):
      raise RobotFailure("postcondition", f"after {call}: expected held({obj.id})", call)

  def place(self, obj: Obj, target: Obj) -> None:
    """Place an object inside a box or on top of another object and check the result."""
    call = _call_text("place", obj, target)
    try:
      self.backend.place(obj.id, target.id)
    except RobotFailure as failure:
      failure.call = call
      raise
    if not self._postcondition("place", (obj, target)):
      expected = "inside" if target.category == "box" else "on_top"
      raise RobotFailure("postcondition", f"after {call}: expected {expected}", call)

  def _postcondition(self, name: str, args: tuple) -> bool:
    if name == "pick":
      return self.held(args[0])
    if name == "place":
      obj, target = args
      return self.inside(obj, target) if target.category == "box" else self.on_top(obj, target)
    return True

  def held(self, obj: Obj) -> bool:
    """Return whether the fresh object state says the object is attached."""
    try:
      fresh = {item.id: item for item in self.backend.list_objects()}
    except RobotFailure:
      return False
    return obj.id in fresh and fresh[obj.id].attached

  def inside(self, obj: Obj, box: Obj) -> bool:
    """Return whether the object's fresh pose is inside the box frame and height."""
    try:
      fresh = {item.id: item for item in self.backend.list_objects()}
    except RobotFailure:
      return False
    if obj.id not in fresh or box.id not in fresh:
      return False
    return _inside_values(fresh[obj.id], fresh[box.id])

  def on_top(self, obj: Obj, target: Obj) -> bool:
    """Return whether the object's fresh pose is on top of the target."""
    try:
      fresh = {item.id: item for item in self.backend.list_objects()}
    except RobotFailure:
      return False
    if obj.id not in fresh or target.id not in fresh:
      return False
    return _on_top_values(fresh[obj.id], fresh[target.id])

  def verify(self, check, message: str = "verify failed") -> None:
    """Evaluate a zero-argument lambda and fail when it is false."""
    if not check():
      raise RobotFailure("verify", message, "verify")

  def report(self, text: str) -> None:
    """Append a human-readable line to this run's report."""
    self.report_lines.append(str(text))

  def object_dicts(self) -> list[dict]:
    return [asdict(obj) for obj in self.backend.list_objects()]
