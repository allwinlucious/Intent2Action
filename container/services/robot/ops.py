"""Operation table of the long-running calls (google.longrunning; phase_3.md decisions 18 and 28).

An operation moves PLANNING -> EXECUTING -> DONE, or ends FAILED / CANCELLED with a status code and a
message. Every error in robot_service is raised as ApiError, which the server turns into the RPC status.
"""

import collections
import dataclasses
import itertools
import threading
import time

from google.longrunning import operations_pb2
from google.protobuf import message as _message
from google.rpc import status_pb2
import grpc
from robot_service import robot_service_pb2

PLANNING, EXECUTING, DONE, FAILED, CANCELLED = "PLANNING", "EXECUTING", "DONE", "FAILED", "CANCELLED"
FINISHED = (DONE, FAILED, CANCELLED)
KEEP_FINISHED = 100  # finished operations kept for GetOperation


class ApiError(Exception):
  """An RPC failure: a gRPC status code and a message that says what failed."""

  def __init__(self, code: grpc.StatusCode, message: str):
    super().__init__(f"{code.name}: {message}")
    self.code = code
    self.message = message


@dataclasses.dataclass
class Operation:
  name: str
  kind: str
  state: str = PLANNING
  detail: str = ""
  planned_duration_s: float = 0.0
  response: _message.Message | None = None
  error: ApiError | None = None
  cancel: threading.Event = dataclasses.field(default_factory=threading.Event)
  worker_done: threading.Event = dataclasses.field(default_factory=threading.Event)  # set by its worker on exit

  @property
  def done(self) -> bool:
    return self.state in FINISHED


class OperationTable:
  """Thread-safe; workers update operations, RPCs read and wait on them."""

  def __init__(self, keep_finished: int = KEEP_FINISHED):
    self._ops: collections.OrderedDict[str, Operation] = collections.OrderedDict()
    self._changed = threading.Condition()
    self._ids = itertools.count(1)
    self._keep_finished = keep_finished

  def create(self, kind: str, state: str = PLANNING, detail: str = "") -> Operation:
    with self._changed:
      op = Operation(name=f"operations/{kind}-{next(self._ids)}", kind=kind, state=state, detail=detail)
      self._ops[op.name] = op
      finished = [o.name for o in self._ops.values() if o.done]
      for name in finished[: max(0, len(finished) - self._keep_finished)]:
        del self._ops[name]
      return op

  def get(self, name: str) -> Operation:
    with self._changed:
      if name not in self._ops:
        raise ApiError(grpc.StatusCode.NOT_FOUND, f"no operation {name!r} (unknown or expired)")
      return self._ops[name]

  def list(self) -> list[Operation]:
    with self._changed:
      return list(self._ops.values())

  def delete(self, name: str) -> None:
    with self._changed:
      if not self.get(name).done:
        raise ApiError(grpc.StatusCode.FAILED_PRECONDITION, f"operation {name} is still running; cancel it first")
      del self._ops[name]

  def update(self, op: Operation, state: str | None = None, detail: str | None = None,
             planned_duration_s: float | None = None) -> bool:
    """False (and no change) if the operation already ended, e.g. cancelled by Stop while planning."""
    with self._changed:
      if op.done:
        return False
      op.state = state or op.state
      op.detail = op.detail if detail is None else detail
      op.planned_duration_s = op.planned_duration_s if planned_duration_s is None else planned_duration_s
      self._changed.notify_all()
      return True

  def finish(self, op: Operation, response: _message.Message, detail: str = "done") -> None:
    with self._changed:
      if op.done:
        return
      op.state, op.detail, op.response = DONE, detail, response
      self._changed.notify_all()

  def fail(self, op: Operation, error: ApiError) -> None:
    """Ends the operation FAILED, or CANCELLED if the code is CANCELLED."""
    with self._changed:
      if op.done:
        return
      op.state = CANCELLED if error.code == grpc.StatusCode.CANCELLED else FAILED
      op.detail, op.error = error.message, error
      self._changed.notify_all()

  def wait(self, op: Operation, timeout_s: float) -> bool:
    """True if the operation finished within timeout_s."""
    deadline = time.monotonic() + timeout_s
    with self._changed:
      while not op.done:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
          return False
        self._changed.wait(remaining)
      return True

  def to_proto(self, op: Operation) -> operations_pb2.Operation:
    with self._changed:
      proto = operations_pb2.Operation(name=op.name, done=op.done)
      proto.metadata.Pack(
          robot_service_pb2.OperationInfo(
              kind=op.kind, planned_duration_s=op.planned_duration_s, state=op.state, detail=op.detail
          )
      )
      if op.error is not None:
        proto.error.CopyFrom(status_pb2.Status(code=op.error.code.value[0], message=op.error.message))
      elif op.response is not None:
        proto.response.Pack(op.response)
      return proto
