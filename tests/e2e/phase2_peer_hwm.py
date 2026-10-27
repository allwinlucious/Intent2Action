"""Phase 2 fake ros2_hwm (CycloneDDS): lockstep and free-run against run_scene.py.

For each /joint_states + /wrist_wrench pair with the same stamp t (within
1 µs) publishes /joint_command stamped t: panda_joint1 follows home +
0.3 sin(2π 0.5 τ) with velocity feed-forward, the other joints home. After
--pause-at pairs it stops reading and replying for --pause-s seconds. With
--request-period-s it also publishes /camera/request at that wall period.
Run in the intrinsic-core:dev image by tests/e2e/phase2_check.py; prints one
JSON line at the end:
  {"cycles", "missed", "max_track_err_rad", "rtf", "pause_sim_advance_s",
   "resume_cycles", "start_pairs", "epoch_seen"}
epoch_seen is the latched /sim/epoch it received (None if none);
start_pairs the replies it sent before Isaac followed them.

τ is the trajectory time: 0 until joint 1 follows the commands (Isaac may
hold the arm while discovery is still delivering the first replies), then
advanced by the stamp step, and by one control cycle across the pause, so the
command never jumps. The cycles, RTF, misses and tracking error count from that
start: before it, /joint_states may match a cycle before /wrist_wrench does.
The first SETTLE_CYCLES after the start and after the resume are left out of the
tracking error, as in the measurement it is based on (phase_2_inventory.md
§2.3). Lockstep has resumed when a state shows joint 1 reacting to a command
again (its velocity steps by more than RESUMED_VELOCITY_STEP).
"""

import argparse
import json
import math
import sys
import time

from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import UInt32
import yaml

TIMING_PATH = "/opt/app/config/timing.yaml"
HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
NAMES = [f"panda_joint{i}" for i in range(1, 8)]
AMPLITUDE_RAD = 0.3
FREQUENCY_HZ = 0.5
MATCH_NS = 1000
MISS_TIMEOUT_S = 1.0
FIRST_STATE_TIMEOUT_S = 120
RUN_TIMEOUT_S = 300
SETTLE_CYCLES = 50
RESUMED_VELOCITY_STEP = (
    0.05  # rad/s per cycle; a held, settled joint 1 changes far less
)
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)
LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _ns(header) -> int:
  return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--cycles", type=int, default=2000)
  parser.add_argument("--pause-at", type=int, default=0)
  parser.add_argument("--pause-s", type=float, default=0.0)
  parser.add_argument("--request-period-s", type=float, default=0.0)
  args = parser.parse_args()
  with open(TIMING_PATH) as f:
    cycle_s = 1.0 / yaml.safe_load(f)["control_hz"]

  rclpy.init()
  node = rclpy.create_node("phase2_peer_hwm")
  command = node.create_publisher(JointState, "/joint_command", QOS)
  request = node.create_publisher(Header, "/camera/request", QOS)
  states = {}  # stamp ns -> (arrival, message, arrived after the start)
  wrenches = {}  # stamp ns -> arrival
  run = {
      "started": False,
      "start_pairs": 0,
      "epoch": None,
      "pairs": 0,
      "missed": 0,
      "tau": 0.0,
      "segment_pairs": 0,
      "prev": None,  # (stamp ns, joint-1 command, joint-1 velocity)
      "resumed": False,
      "replies_since_resume": 0,
      "max_err": 0.0,
      "first": None,  # (stamp ns, wall)
      "pause_stamp": None,
      "pause_wall": None,
      "pause_advance_s": None,
      "resume_cycles": None,
  }

  def publish(state: JointState, tau: float) -> float:
    """Commands the trajectory at τ, stamped as state; returns joint 1's target."""
    phase = 2 * math.pi * FREQUENCY_HZ * tau
    position = list(HOME)
    velocity = [0.0] * 7
    position[0] += AMPLITUDE_RAD * math.sin(phase)
    velocity[0] = AMPLITUDE_RAD * 2 * math.pi * FREQUENCY_HZ * math.cos(phase)
    out = JointState(name=NAMES, position=position, velocity=velocity)
    out.header.stamp = state.header.stamp
    command.publish(out)
    return position[0]

  def reply(stamp_ns: int, state: JointState) -> None:
    q1 = state.position[state.name.index("panda_joint1")]
    v1 = state.velocity[state.name.index("panda_joint1")]
    prev = run["prev"]
    if not run["started"]:
      # The first replies may not reach Isaac (discovery), and Isaac holding
      # the arm at home looks like lockstep; so τ stays 0 until joint 1
      # follows the velocity feed-forward of the τ = 0 command.
      if prev is None or abs(v1) <= RESUMED_VELOCITY_STEP:
        run["start_pairs"] += 1
        run["prev"] = (stamp_ns, publish(state, 0.0), v1)
        return
      run["started"] = True
      run["first"] = (stamp_ns, time.monotonic())
    elif run["resumed"]:
      run["tau"] += cycle_s
      run["resumed"] = False
      run["pause_advance_s"] = (stamp_ns - run["pause_stamp"]) * 1e-9
      run["segment_pairs"] = 0
    else:
      run["tau"] += (stamp_ns - prev[0]) * 1e-9
      if run["segment_pairs"] >= SETTLE_CYCLES:
        run["max_err"] = max(run["max_err"], abs(prev[1] - q1))
    if (
        run["pause_stamp"] is not None
        and run["resume_cycles"] is None
        and run["replies_since_resume"] > 0
        and abs(v1 - prev[2]) > RESUMED_VELOCITY_STEP
    ):
      run["resume_cycles"] = run["replies_since_resume"]

    run["prev"] = (stamp_ns, publish(state, run["tau"]), v1)
    run["pairs"] += 1
    run["segment_pairs"] += 1
    if run["pause_stamp"] is not None:
      run["replies_since_resume"] += 1

  def try_pair(stamp_ns: int) -> None:
    for state_ns in list(states):
      if abs(state_ns - stamp_ns) > MATCH_NS:
        continue
      wrench_ns = next(
          (w for w in wrenches if abs(w - state_ns) <= MATCH_NS), None
      )
      if wrench_ns is None:
        continue
      _, state, _ = states.pop(state_ns)
      del wrenches[wrench_ns]
      reply(state_ns, state)

  def on_state(msg: JointState) -> None:
    stamp_ns = _ns(msg.header)
    states[stamp_ns] = (time.monotonic(), msg, run["started"])
    try_pair(stamp_ns)

  def on_wrench(msg: WrenchStamped) -> None:
    stamp_ns = _ns(msg.header)
    wrenches[stamp_ns] = time.monotonic()
    try_pair(stamp_ns)

  node.create_subscription(JointState, "/joint_states", on_state, QOS)
  node.create_subscription(WrenchStamped, "/wrist_wrench", on_wrench, QOS)
  # Started after Isaac, so this is the latched value (decision 6).
  node.create_subscription(
      UInt32, "/sim/epoch", lambda m: run.update(epoch=m.data), LATCHED
  )

  def prune(now: float) -> None:
    for state_ns, (arrival, _, started) in list(states.items()):
      if now - arrival > MISS_TIMEOUT_S:
        del states[state_ns]
        run["missed"] += started
    for wrench_ns, arrival in list(wrenches.items()):
      if now - arrival > MISS_TIMEOUT_S:
        del wrenches[wrench_ns]

  start = time.monotonic()
  next_request = start
  rtf = None
  while run["pairs"] < args.cycles:
    rclpy.spin_once(node, timeout_sec=0.001)
    now = time.monotonic()
    prune(now)
    if run["first"] is None and now - start > FIRST_STATE_TIMEOUT_S:
      break
    if now - start > RUN_TIMEOUT_S:
      break
    if args.request_period_s and run["prev"] and now >= next_request:
      header = Header()
      header.stamp.sec, header.stamp.nanosec = divmod(
          run["prev"][0], 1_000_000_000
      )
      request.publish(header)
      next_request = now + args.request_period_s
    if (
        args.pause_at
        and run["pairs"] == args.pause_at
        and run["pause_stamp"] is None
    ):
      run["pause_stamp"], run["pause_wall"] = run["prev"][0], now
      rtf = (run["prev"][0] - run["first"][0]) * 1e-9 / (now - run["first"][1])
      time.sleep(args.pause_s)
      run["resumed"] = True

  end = time.monotonic()
  if rtf is None and run["first"] and run["prev"]:
    rtf = (run["prev"][0] - run["first"][0]) * 1e-9 / (end - run["first"][1])
  print(
      json.dumps(
          {
              "cycles": run["pairs"],
              "missed": run["missed"],
              "max_track_err_rad": run["max_err"],
              "rtf": rtf,
              "pause_sim_advance_s": run["pause_advance_s"],
              "resume_cycles": run["resume_cycles"],
              "start_pairs": run["start_pairs"],
              "epoch_seen": run["epoch"],
          }
      ),
      flush=True,
  )
  return 0


if __name__ == "__main__":
  sys.exit(main())
