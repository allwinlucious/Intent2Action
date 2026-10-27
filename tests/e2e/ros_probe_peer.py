"""Phase 0.5 container peer: echoes /probe/ping to /probe/pong, calls /probe/trigger.

Runs in the container's Python with RMW_IMPLEMENTATION set by the caller
(tests/e2e/phase0_5_check.py). Prints FIRST_PING on the first ping and
SERVICE <ok>/10 after ten Trigger calls.
"""

import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

FIRST_PING_TIMEOUT_S = 180
IDLE_EXIT_S = 10
CALL_COUNT = 10
CALL_TIMEOUT_S = 5
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


def main() -> int:
  rclpy.init()
  node = rclpy.create_node("probe_peer")
  executor = SingleThreadedExecutor()
  executor.add_node(node)
  pong = node.create_publisher(JointState, "/probe/pong", QOS)
  client = node.create_client(Trigger, "/probe/trigger")
  last_ping = None

  def on_ping(msg):
    nonlocal last_ping
    pong.publish(msg)
    if last_ping is None:
      print("FIRST_PING", flush=True)
    last_ping = time.monotonic()

  node.create_subscription(JointState, "/probe/ping", on_ping, QOS)

  start = time.monotonic()
  calls_started = 0
  ok = 0
  future = None
  call_start = 0.0
  # Service calls are sequenced here, outside callbacks, so nothing blocks the executor.
  while True:
    executor.spin_once(timeout_sec=0.01)
    now = time.monotonic()
    if last_ping is None:
      if now - start > FIRST_PING_TIMEOUT_S:
        print("NO_PING", flush=True)
        return 1
      continue
    calls_done = calls_started == CALL_COUNT and future is None
    if now - last_ping > IDLE_EXIT_S and calls_done:
      return 0
    if future is not None and (
        future.done() or now - call_start > CALL_TIMEOUT_S
    ):
      if future.done():
        result = future.result()
        ok += bool(result.success and result.message == "isaac")
      future = None
      if calls_started == CALL_COUNT:
        print(f"SERVICE {ok}/{CALL_COUNT}", flush=True)
    if future is None and calls_started < CALL_COUNT:
      future = client.call_async(Trigger.Request())
      call_start = now
      calls_started += 1


if __name__ == "__main__":
  sys.exit(main())
