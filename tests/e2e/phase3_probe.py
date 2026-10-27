"""Phase 3 ROS probe, run in the intrinsic-core:dev image by phase3_hwm_check.py.

  phase3_probe.py joint_states  prints {"names", "position", "stamp_s"} of the
                                first /joint_states received from Isaac
  phase3_probe.py reset         calls /sim/reset; prints {"success", "message"}
Run with Fast DDS (Isaac's services are not reachable from CycloneDDS).
"""

import json
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

TIMEOUT_S = 30
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


def _spin_until(node, done, timeout_s: float) -> bool:
  deadline = time.monotonic() + timeout_s
  while not done() and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
  return done()


def joint_states(node) -> dict:
  received = []
  node.create_subscription(JointState, "/joint_states", received.append, QOS)
  if not _spin_until(node, lambda: received, TIMEOUT_S):
    return {"error": f"no /joint_states within {TIMEOUT_S} s"}
  msg = received[0]
  return {
      "names": list(msg.name),
      "position": list(msg.position),
      "stamp_s": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
  }


def reset(node) -> dict:
  client = node.create_client(Trigger, "/sim/reset")
  if not client.wait_for_service(timeout_sec=TIMEOUT_S):
    return {"error": f"/sim/reset not available within {TIMEOUT_S} s"}
  future = client.call_async(Trigger.Request())
  if not _spin_until(node, future.done, TIMEOUT_S):
    return {"error": f"/sim/reset did not answer within {TIMEOUT_S} s"}
  return {"success": future.result().success, "message": future.result().message}


def main() -> int:
  rclpy.init()
  node = rclpy.create_node("phase3_probe")
  result = {"joint_states": joint_states, "reset": reset}[sys.argv[1]](node)
  print(json.dumps(result), flush=True)
  return 0


if __name__ == "__main__":
  sys.exit(main())
