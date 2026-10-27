"""Phase 0.5 ROS 2 probe, run in Isaac Sim's Python (Fast DDS, Windows).

Serves /probe/trigger, publishes /probe/ping at 250 Hz for 20 s and records
the round trip of each ping echoed back on /probe/pong. Writes
{"sent": n, "received": m, "rtt_ms": [...]} to C:\\isaac-sim-work\\probe_result.json.
Run from WSL as:
  cmd.exe /c "cd /d C:\\isaac-sim && python.bat -" < sim/ros_probe.py
"""

import json
import os
import time

# The bridge needs these before SimulationApp starts (phase_0_5_inventory.md §1.5).
os.environ["ROS_DISTRO"] = "humble"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["PATH"] += r";C:\isaac-sim\exts\isaacsim.ros2.bridge\humble\lib"
os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = r"C:\isaac-sim-work\fastdds.xml"
os.environ["ROS_DOMAIN_ID"] = "0"

from isaacsim import SimulationApp  # noqa: E402  pylint: disable=g-import-not-at-top

simulation_app = SimulationApp(
    {"headless": True, "extra_args": ["--/app/vulkan=false"]}
)

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402  pylint: disable=g-import-not-at-top

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy  # noqa: E402  pylint: disable=g-import-not-at-top
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402  pylint: disable=g-import-not-at-top
from sensor_msgs.msg import JointState  # noqa: E402  pylint: disable=g-import-not-at-top
from std_srvs.srv import Trigger  # noqa: E402  pylint: disable=g-import-not-at-top

RESULT_PATH = r"C:\isaac-sim-work\probe_result.json"
PING_COUNT = 5000  # 20 s at 250 Hz
PERIOD_NS = 4_000_000
MATCH_TIMEOUT_S = 120
DRAIN_S = 30
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


def _write_result(sent: int, rtt_ns: dict) -> None:
  with open(RESULT_PATH, "w") as f:
    json.dump(
        {
            "sent": sent,
            "received": len(rtt_ns),
            "rtt_ms": [rtt_ns[k] / 1e6 for k in sorted(rtt_ns)],
        },
        f,
    )


def main() -> None:
  rclpy.init()
  node = rclpy.create_node("isaac_probe")
  rtt_ns = {}

  def on_trigger(request, response):
    del request
    response.success = True
    response.message = "isaac"
    return response

  def on_pong(msg):
    stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    rtt_ns[int(msg.position[0])] = time.perf_counter_ns() - stamp_ns

  node.create_service(Trigger, "/probe/trigger", on_trigger)
  ping = node.create_publisher(JointState, "/probe/ping", QOS)
  node.create_subscription(JointState, "/probe/pong", on_pong, QOS)

  deadline = time.monotonic() + MATCH_TIMEOUT_S
  while (
      node.count_subscribers("/probe/ping") < 1
      or node.count_publishers("/probe/pong") < 1
  ):
    if time.monotonic() > deadline:
      print("NO_MATCH", flush=True)
      _write_result(0, {})
      simulation_app.close()
      return
    rclpy.spin_once(node, timeout_sec=0.05)

  print("PING_START", flush=True)
  # Absolute schedule: ping k goes out at t0 + k * 4 ms, so spin time never drifts the rate.
  t0 = time.perf_counter_ns()
  for k in range(PING_COUNT):
    msg = JointState(position=[float(k)])
    now = time.perf_counter_ns()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(now, 1_000_000_000)
    ping.publish(msg)
    next_ns = t0 + (k + 1) * PERIOD_NS
    while (remaining := next_ns - time.perf_counter_ns()) > 0:
      rclpy.spin_once(node, timeout_sec=remaining / 1e9)

  drain_end = time.monotonic() + DRAIN_S
  while (remaining := drain_end - time.monotonic()) > 0:
    rclpy.spin_once(node, timeout_sec=min(remaining, 0.1))

  _write_result(PING_COUNT, rtt_ns)
  print(f"PROBE_DONE sent {PING_COUNT} received {len(rtt_ns)}", flush=True)
  simulation_app.close()


main()
