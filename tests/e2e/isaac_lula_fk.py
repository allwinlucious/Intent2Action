"""Lula FK of panda_link8 and right_gripper, run in Isaac Sim's Python.

Reads a JSON list of joint vectors from C:\\isaac-sim-work\\fk_in.json and
writes, per vector, {frame: {"translation": [3], "rotation": [[3]x3]}} to
C:\\isaac-sim-work\\fk_out.json (rotation row-major). Run from WSL as:
  cat tests/e2e/isaac_lula_fk.py | cmd.exe /c "cd /d C:\\isaac-sim && python.bat -"
Lula needs no SimulationApp.
"""

import json
import sys

sys.path.insert(
    0, r"C:\isaac-sim\exts\isaacsim.robot_motion.lula\pip_prebundle"
)
import lula  # noqa: E402  pylint: disable=g-import-not-at-top
import numpy as np  # noqa: E402  pylint: disable=g-import-not-at-top

CONFIG_DIR = (
    r"C:\isaac-sim\exts\isaacsim.robot_motion.motion_generation"
    r"\motion_policy_configs\franka"
)
WORK_DIR = r"C:\isaac-sim-work"
FRAMES = ("panda_link8", "right_gripper")


def main() -> None:
  robot = lula.load_robot(
      CONFIG_DIR + r"\rmpflow\robot_descriptor.yaml",
      CONFIG_DIR + r"\lula_franka_gen.urdf",
  )
  kinematics = robot.kinematics()
  with open(WORK_DIR + r"\fk_in.json") as f:
    joint_vectors = json.load(f)
  results = []
  for q in joint_vectors:
    column = np.array(q, dtype=np.float64).reshape(-1, 1)
    poses = {}
    for frame in FRAMES:
      pose = kinematics.pose(column, frame)
      poses[frame] = {
          "translation": np.asarray(pose.translation).reshape(3).tolist(),
          "rotation": np.asarray(pose.rotation.matrix()).tolist(),
      }
    results.append(poses)
  with open(WORK_DIR + r"\fk_out.json", "w") as f:
    json.dump(results, f)
  print(f"wrote {len(results)} FK results")


main()
