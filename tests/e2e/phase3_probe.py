"""Phase 3 ROS probe, run in the intrinsic-core:dev image by phase3_hwm_check.py.

  phase3_probe.py joint_states  prints {"names", "position", "stamp_s"} of the
                                first /joint_states received from Isaac
  phase3_probe.py reset         calls /sim/reset; prints {"success", "message"}
  phase3_probe.py tf            prints {"frames": {name: [x, y, z, qx, qy, qz, qw]}}
                                of the /tf frames with parent world seen in 2 s
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
from tf2_msgs.msg import TFMessage

TIMEOUT_S = 30
TF_COLLECT_S = 2.0
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


def tf(node) -> dict:
  frames = {}

  def on_tf(msg: TFMessage) -> None:
    for tr in msg.transforms:
      if tr.header.frame_id == "world":
        p, q = tr.transform.translation, tr.transform.rotation
        frames[tr.child_frame_id] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]

  # Two bridge nodes publish /tf back to back; depth 1 would drop one.
  node.create_subscription(
      TFMessage, "/tf", on_tf, QoSProfile(depth=10, reliability=QOS.reliability)
  )
  if not _spin_until(node, lambda: frames, TIMEOUT_S):
    return {"error": f"no /tf within {TIMEOUT_S} s"}
  _spin_until(node, lambda: False, TF_COLLECT_S)
  return {"frames": frames}


def main() -> int:
  rclpy.init()
  node = rclpy.create_node("phase3_probe")
  result = {"joint_states": joint_states, "reset": reset, "tf": tf}[
      sys.argv[1]
  ](node)
  print(json.dumps(result), flush=True)
  return 0


if __name__ == "__main__":
  sys.exit(main())
