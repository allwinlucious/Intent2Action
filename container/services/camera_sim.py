"""Serves Intrinsic's CameraService from Isaac's D455 topics (phase_3.md decision 17).

A Fast DDS ROS 2 node in the image, started by container/entrypoint.sh. One camera,
ros { device_id: "d455" }, with sensor 1 "color" (RGB) and sensor 4 "depth" (metres), on
127.0.0.1:18005. Capture asks Isaac for a frame (/camera/request) and returns the first RGB and
depth images stamped at or after the request.
"""

import argparse
import collections
from concurrent import futures
import logging
import sys
import threading
import time
from typing import NamedTuple

from google.protobuf import duration_pb2
from google.protobuf import timestamp_pb2
import grpc
from intrinsic.perception.proto.v1 import camera_identifier_pb2
from intrinsic.perception.proto.v1 import camera_params_pb2
from intrinsic.perception.proto.v1 import camera_service_pb2 as camera_pb
from intrinsic.perception.proto.v1 import camera_service_pb2_grpc as camera_grpc
from intrinsic.perception.proto.v1 import capture_result_pb2
from intrinsic.perception.proto.v1 import dimensions_pb2
from intrinsic.perception.proto.v1 import image_buffer_pb2 as ib
from intrinsic.perception.proto.v1 import intrinsic_params_pb2
from intrinsic.perception.proto.v1 import sensor_config_pb2
from intrinsic.perception.proto.v1 import sensor_image_pb2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Header

DEVICE_ID = "d455"
WIDTH, HEIGHT = 848, 480
REQUEST_AHEAD_S = 0.05  # the frame must show sim time >= latest /clock + this
DEFAULT_TIMEOUT_S = 5.0
INFO_REQUEST_PERIOD_S = 1.0
STAMP_TOLERANCE_S = 1e-6
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)
RGB, DEPTH, INFO = "/camera/rgb", "/camera/depth", "/camera/camera_info"


class Sensor(NamedTuple):
  id: int
  name: str
  topic: str
  encoding: str  # ROS image encoding
  pixel_type: int
  channels: int
  data_type: int
  bytes_per_pixel: int


SENSORS = (
    Sensor(1, "color", RGB, "rgb8", ib.PIXEL_INTENSITY, 3, ib.TYPE_UINT8, 3),
    Sensor(4, "depth", DEPTH, "32FC1", ib.PIXEL_DEPTH, 1, ib.TYPE_FLOAT32, 4),
)

log = logging.getLogger("camera_sim")


def _identifier() -> camera_identifier_pb2.CameraIdentifier:
  identifier = camera_identifier_pb2.CameraIdentifier()
  identifier.ros.device_id = DEVICE_ID
  return identifier


def _seconds(stamp) -> float:
  return stamp.sec + stamp.nanosec * 1e-9


def _camera_params(info: CameraInfo) -> camera_params_pb2.CameraParams:
  return camera_params_pb2.CameraParams(
      intrinsic_params=intrinsic_params_pb2.IntrinsicParams(
          focal_length_x=info.k[0],
          focal_length_y=info.k[4],
          principal_point_x=info.k[2],
          principal_point_y=info.k[5],
          dimensions=dimensions_pb2.Dimensions(cols=info.width, rows=info.height),
      )
  )


class Ros:
  """Latest /clock and the last few frames per topic; requests frames from Isaac."""

  def __init__(self):
    self.node = n = rclpy.create_node("camera_sim")
    self.changed = threading.Condition()
    self.frames = {topic: collections.deque(maxlen=4) for topic in (RGB, DEPTH, INFO)}
    self.clock: float | None = None
    self.request = n.create_publisher(Header, "/camera/request", QOS)
    n.create_subscription(Clock, "/clock", self._on_clock, QOS)
    for topic in (RGB, DEPTH):
      n.create_subscription(Image, topic, lambda m, t=topic: self._on_frame(t, m), QOS)
    n.create_subscription(CameraInfo, INFO, lambda m: self._on_frame(INFO, m), QOS)
    self.executor = SingleThreadedExecutor()
    self.executor.add_node(n)
    threading.Thread(target=self.executor.spin, daemon=True).start()
    threading.Thread(target=self._request_info, daemon=True).start()

  def _request_info(self) -> None:
    """Isaac sends camera_info only with a requested frame; DescribeCamera needs one before any Capture."""
    while self.latest_info() is None:
      if self.clock is not None:
        header = Header()
        header.stamp.sec, header.stamp.nanosec = divmod(round(self.clock * 1e9), 1_000_000_000)
        self.request.publish(header)
      time.sleep(INFO_REQUEST_PERIOD_S)
    info = self.latest_info()
    log.info("camera_info received width=%d height=%d fx=%.2f", info.width, info.height, info.k[0])

  def _on_clock(self, msg: Clock) -> None:
    self.clock = _seconds(msg.clock)

  def _on_frame(self, topic: str, msg) -> None:
    with self.changed:
      self.frames[topic].append(msg)
      self.changed.notify_all()

  def latest_info(self) -> CameraInfo | None:
    with self.changed:
      return self.frames[INFO][-1] if self.frames[INFO] else None

  def capture(self, stamp: float, timeout_s: float) -> dict | None:
    """Requests a frame showing sim time >= stamp; the first one per topic, or None on timeout."""
    header = Header()
    header.stamp.sec, header.stamp.nanosec = divmod(round(stamp * 1e9), 1_000_000_000)
    self.request.publish(header)
    deadline = time.monotonic() + timeout_s
    with self.changed:
      while True:
        found = {}
        for topic, queue in self.frames.items():
          fresh = [m for m in queue if _seconds(m.header.stamp) >= stamp - STAMP_TOLERANCE_S]
          if fresh:
            found[topic] = fresh[0]
        remaining = deadline - time.monotonic()
        if len(found) == len(self.frames) or remaining <= 0:
          return found if len(found) == len(self.frames) else None
        self.changed.wait(remaining)


class CameraService(camera_grpc.CameraServiceServicer):

  def __init__(self, ros: Ros):
    self.ros = ros

  def _check_camera(self, identifier, context) -> None:
    if identifier.ByteSize() and identifier != _identifier():
      context.abort(grpc.StatusCode.NOT_FOUND, f"unknown camera {identifier}; this service has ros {{ device_id: {DEVICE_ID!r} }}")

  def ListAvailableCameras(self, request, context):
    return camera_pb.ListAvailableCamerasResponse(camera_identifiers=[_identifier()])

  def DescribeCamera(self, request, context):
    self._check_camera(request.camera_identifier, context)
    info = self.ros.latest_info()
    if info is None:
      context.abort(grpc.StatusCode.UNAVAILABLE, f"no {INFO} received from Isaac yet (it comes with the first frame)")
    response = camera_pb.DescribeCameraResponse()
    for sensor in SENSORS:
      described = response.sensors.add(
          id=sensor.id,
          display_name=sensor.name,
          factory_camera_params=_camera_params(info),
          supported_pixel_types=[sensor.pixel_type],
          dimensions=dimensions_pb2.Dimensions(cols=WIDTH, rows=HEIGHT),
      )
      described.camera_t_sensor.orientation.w = 1.0
    return response

  def Capture(self, request, context):
    self._check_camera(request.camera_config.identifier, context)
    if request.HasField("capture_result_location"):
      context.abort(grpc.StatusCode.UNIMPLEMENTED, "capture_result_location (KV store) is not supported; results are inline")
    wanted = set(request.sensor_ids) or {s.id for s in SENSORS}
    if unknown := wanted - {s.id for s in SENSORS}:
      context.abort(grpc.StatusCode.INVALID_ARGUMENT, f"unknown sensor ids {sorted(unknown)}; sensors are 1 (color) and 4 (depth)")
    if self.ros.clock is None:
      context.abort(grpc.StatusCode.UNAVAILABLE, "no /clock received from Isaac yet")
    timeout_s = request.timeout.ToTimedelta().total_seconds() if request.HasField("timeout") else DEFAULT_TIMEOUT_S
    stamp = self.ros.clock + REQUEST_AHEAD_S
    start = time.monotonic()
    found = self.ros.capture(stamp, timeout_s)
    wait_ms = (time.monotonic() - start) * 1e3
    if found is None:
      message = f"no RGB, depth and camera_info stamped >= {stamp:.3f} s (sim) within {timeout_s} s (wall)"
      log.warning("capture failed stamp=%.3f wait_ms=%.0f status=DEADLINE_EXCEEDED message=%s", stamp, wait_ms, message)
      context.abort(grpc.StatusCode.DEADLINE_EXCEEDED, message)
    params = _camera_params(found[INFO])
    result = capture_result_pb2.CaptureResult(
        capture_at=timestamp_pb2.Timestamp(seconds=found[RGB].header.stamp.sec, nanos=found[RGB].header.stamp.nanosec),
        capture_duration=duration_pb2.Duration(nanos=round(wait_ms * 1e6)),
    )
    for sensor in SENSORS:
      if sensor.id not in wanted:
        continue
      image = found[sensor.topic]
      if image.encoding != sensor.encoding or image.step != image.width * sensor.bytes_per_pixel or image.is_bigendian:
        context.abort(
            grpc.StatusCode.INTERNAL,
            f"{sensor.topic}: unexpected encoding {image.encoding} step {image.step} width {image.width}"
            f" big-endian {image.is_bigendian} (want {sensor.encoding}, packed, little-endian)",
        )
      sensor_image = sensor_image_pb2.SensorImage(
          sensor_config=sensor_config_pb2.SensorConfig(id=sensor.id, camera_params=params),
          acquisition_time=timestamp_pb2.Timestamp(seconds=image.header.stamp.sec, nanos=image.header.stamp.nanosec),
          buffer=ib.ImageBuffer(
              encoding=ib.ENCODING_UNSPECIFIED,
              pixel_type=sensor.pixel_type,
              num_channels=sensor.channels,
              type=sensor.data_type,
              dimensions=dimensions_pb2.Dimensions(cols=image.width, rows=image.height),
              data=bytes(image.data),
              packing_type=ib.PACKING_TYPE_INTERLEAVED,
          ),
      )
      sensor_image.sensor_config.camera_t_sensor.orientation.w = 1.0
      result.sensor_images.append(sensor_image)
    log.info(
        "capture stamp=%.3f image_stamp=%.3f wait_ms=%.0f sensors=%s",
        stamp, _seconds(found[RGB].header.stamp), wait_ms, sorted(wanted),
    )
    return camera_pb.CaptureResponse(capture_result=result)


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--port", type=int, default=18005)
  args = parser.parse_args()
  logging.basicConfig(
      stream=sys.stdout, level=logging.INFO, format="%(asctime)s %(levelname)s camera_sim %(message)s"
  )
  rclpy.init()
  ros = Ros()
  server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
  camera_grpc.add_CameraServiceServicer_to_server(CameraService(ros), server)
  server.add_insecure_port(f"127.0.0.1:{args.port}")
  server.start()
  log.info("started camera_service=127.0.0.1:%d camera=ros/%s", args.port, DEVICE_ID)
  server.wait_for_termination()
  return 0


if __name__ == "__main__":
  sys.exit(main())
