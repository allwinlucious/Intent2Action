"""Isaac side of the robot platform: the default scene, stepped in lockstep.

Builds sim/scenes/default.yaml, then runs the control cycle of
config/timing.yaml forever, talking only ROS 2 (phase_2.md, target file 10).
Run from WSL after tools/sync_isaac.sh:
  cmd.exe /c "cd /d C:\\isaac-sim && python.bat C:\\isaac-sim-work\\repo\\sim\\run_scene.py"
Prints READY pid=<pid> once the scene is up; stop it with taskkill.exe /PID <pid> /T /F.
"""

import os
import sys
import time

REPO = r"C:\isaac-sim-work\repo"
# --gui: open the viewer window, render ~30 fps and save viewer frames to
# GUI_FRAMES (for demo videos). Slower than headless; not used by the checks.
GUI = "--gui" in sys.argv
# --physics-only: no camera, no image graphs, a tiny render target and no AA: Isaac leaves the GPU to a
# model (the images and the vision tools are unavailable).
PHYSICS_ONLY = "--physics-only" in sys.argv
# --pretty: presentation lighting and materials for demo videos (sim/pretty.py); physics unchanged.
PRETTY = "--pretty" in sys.argv
GUI_FRAMES = r"C:\isaac-sim-work\viewer_frames"
GUI_RENDER_PERIOD_S = 1 / 30
# ISAAC_FRAME_PERIOD_S lets the demo stream run at video frame rate.
GUI_FRAME_PERIOD_S = float(os.environ.get("ISAAC_FRAME_PERIOD_S", "0.2"))
# --pretty renders only when a video frame is due, every PRETTY_FRAME_PERIOD_S of sim time: the heavy scene would
# otherwise slow physics far below real time, and the video plays at real speed whatever a render costs.
PRETTY_FRAME_PERIOD_S = 1 / 15
sys.path += [REPO, r"C:\isaac-sim\exts\omni.pip.compute\pip_prebundle"]

# The bridge needs these before SimulationApp starts (phase_0_5_inventory.md §1.5).
os.environ["ROS_DISTRO"] = "humble"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["PATH"] += r";C:\isaac-sim\exts\isaacsim.ros2.bridge\humble\lib"
os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = r"C:\isaac-sim-work\fastdds.xml"
os.environ["ROS_DOMAIN_ID"] = "0"

from isaacsim import SimulationApp  # noqa: E402  pylint: disable=g-import-not-at-top

# Synchronous rendering: a render shows the current physics state (Inv §4.4).
simulation_app = SimulationApp(
    {
        "headless": not GUI,
        # /rtx/post/aa/op = 4 (DLAA): anti-aliasing at native resolution; DLSS
        # upscaling smears the ChArUco markers (phase_4.md, Addendum A).
        # SimulationApp sets this setting from its config after start-up, so
        # it cannot go in extra_args.
        "anti_aliasing": 0 if PHYSICS_ONLY else 4,
        **({"width": 64, "height": 64} if PHYSICS_ONLY else {}),
        "extra_args": [
            "--/app/vulkan=false",
            "--/app/asyncRendering=0",
            "--/app/asyncRenderingLowLatency=0",
            "--/omni/replicator/asyncRendering=0",
            "--/app/hydraEngine/waitIdle=1",
        ],
    }
)

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402  pylint: disable=g-import-not-at-top

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.ros2.sim_control")
simulation_app.update()

from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
# pylint: disable=g-import-not-at-top,g-bad-import-order
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import UsdPhysics  # noqa: E402
import rclpy  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
import yaml  # noqa: E402

from sim import graphs as graphs_lib  # noqa: E402
from sim import scene as scene_lib  # noqa: E402

# pylint: enable=g-import-not-at-top,g-bad-import-order

TIMING_PATH = os.path.join(REPO, "config", "timing.yaml")
SCENE_PATH = os.path.join(REPO, "sim", "scenes", "default.yaml")
ARM_HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
FINGER_OPEN_M = 0.04
ARM_JOINTS = np.arange(7)
# panda_hand_joint is joint 7; its measured-force row is joint index + 1 (Inv §2.4).
HAND_JOINT_ROW = 8
HAND_JOINT_PRIM = "/World/franka/panda_link7/panda_hand_joint"
HAND_PRIM = "/World/franka/panda_hand"  # on /tf: the true hand pose
FINGER_JOINT = "panda_finger_joint1"
# Franka Hand continuous grasp force (Franka datasheet).
FINGER_MAX_FORCE_N = 70.0
LOG_PERIOD_S = 10.0


def quat_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
  return np.array(
      [
          [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
      ]
  )


def stamp(t: float) -> tuple[int, int]:
  """(sec, nanosec) of t, nanosec rounded (decision 9)."""
  return divmod(round(t * 1e9), 1_000_000_000)


class Scene:
  """Owns the simulation, the robot and the loop state the services change."""

  def __init__(self):
    with open(TIMING_PATH) as f:
      self.timing = yaml.safe_load(f)
    with open(SCENE_PATH) as f:
      self.scene = yaml.safe_load(f)
    self.sim = SimulationContext(
        physics_dt=1.0 / (self.timing["control_hz"] * self.timing["substeps"]),
        stage_units_in_meters=1.0,
    )
    self.stage = omni.usd.get_context().get_stage()
    self.camera = scene_lib.build(
        self.stage, self.scene, get_assets_root_path(), camera=not PHYSICS_ONLY
    )
    if PRETTY:
      from sim import pretty  # pylint: disable=g-import-not-at-top

      pretty.apply(self.stage, get_assets_root_path())
    while is_stage_loading():
      simulation_app.update()
    self.sim.reset()
    self.robot = SingleArticulation(scene_lib.ROBOT_PRIM)
    self.robot.initialize()
    self.finger = self.robot.get_dof_index(FINGER_JOINT)
    self.home = np.concatenate([ARM_HOME, [FINGER_OPEN_M, FINGER_OPEN_M]])
    self.held = ARM_HOME.copy()
    self.go_home()

    joint = UsdPhysics.Joint(self.stage.GetPrimAtPath(HAND_JOINT_PRIM))
    rot1 = joint.GetLocalRot1Attr().Get()
    self.hand_rot = quat_to_matrix(rot1.GetReal(), *rot1.GetImaginary())
    self.hand_pos = np.array(joint.GetLocalPos1Attr().Get(), dtype=float)
    print(f"panda_hand_joint localPos1 {self.hand_pos.tolist()}", flush=True)
    if self.camera is not None:
      print(
          f"camera intrinsics {self.camera.get_intrinsics_matrix().tolist()}",
          flush=True,
      )

    self.graphs = graphs_lib.Graphs(
        scene_lib.ROBOT_PRIM,
        None if self.camera is None else self.camera.get_render_product_path(),
    )
    # The first render activates the Camera Helper writers and their image
    # gates; from then on images are published only on request (the second
    # render is the one the closed gate still lets through, see render()).
    self.sim.render()
    self.graphs.set_image_publishing(False)
    self.sim.render()
    self.refresh_tf_targets()
    self.epoch = 0
    self.graphs.publish_epoch(self.epoch)

  def go_home(self) -> None:
    """Robot at home, at rest, drive targets at home, gripper open."""
    self.robot.set_joint_positions(self.home)
    self.robot.set_joint_velocities(np.zeros_like(self.home))
    self.robot.apply_action(
        ArticulationAction(
            joint_positions=self.home, joint_velocities=np.zeros_like(self.home)
        )
    )
    self.held = ARM_HOME.copy()

  def refresh_tf_targets(self, prims_replaced: bool = False) -> None:
    self.graphs.set_tf_targets(
        scene_lib.object_paths(self.stage)
        + [scene_lib.CAMERA_PRIM, HAND_PRIM],
        prims_replaced,
    )

  def wrench(self) -> tuple[np.ndarray, np.ndarray]:
    """Load on the wrist sensor in panda_hand axes (decision 8)."""
    row = np.asarray(self.robot.get_measured_joint_forces())[HAND_JOINT_ROW]
    force = -(self.hand_rot @ row[:3])
    torque = -(self.hand_rot @ row[3:])
    return force, torque - np.cross(self.hand_pos, force)

  # Trigger services (decision 1); they run between physics steps.
  def on_reset(self, request, response):
    del request
    scene_lib.reset_objects(self.stage, self.scene)
    self.go_home()
    self.refresh_tf_targets(prims_replaced=True)
    self.epoch += 1
    self.graphs.publish_epoch(self.epoch)
    response.success, response.message = True, f"epoch {self.epoch}"
    return response

  def on_attach(self, request, response):
    del request
    response.success, response.message = scene_lib.attach_board(
        self.stage, self.robot
    )
    self.refresh_tf_targets()
    return response

  def on_detach(self, request, response):
    del request
    response.success, response.message = scene_lib.detach_board(self.stage)
    self.refresh_tf_targets()
    return response


def wait_for_command(
    graphs, t: float, timeout_s: float, tolerance_s: float
) -> bool:
  """Busy-polls the command graph until /joint_command is stamped t."""
  deadline = time.perf_counter() + timeout_s
  while abs(graphs.poll_command() - t) >= tolerance_s:
    if time.perf_counter() >= deadline:
      return False
  return True


_frame_state = {"next": 0.0, "index": 0, "sim_next": 0.0}


def _setup_viewer() -> None:
  """Points the viewer at the work area and clears old frames."""
  import glob  # pylint: disable=g-import-not-at-top

  from isaacsim.core.utils.viewports import set_camera_view  # pylint: disable=g-import-not-at-top

  if PRETTY:
    from sim import pretty  # pylint: disable=g-import-not-at-top

    set_camera_view(eye=pretty.EYE, target=pretty.TARGET)
    import carb.settings  # pylint: disable=g-import-not-at-top

    carb.settings.get_settings().set("/app/window/hideUi", True)  # viewport only, as F7 does
  else:
    set_camera_view(eye=[1.35, 1.05, 0.85], target=[0.45, 0.05, 0.05])
  os.makedirs(GUI_FRAMES, exist_ok=True)
  for path in glob.glob(os.path.join(GUI_FRAMES, "*.png")):
    os.remove(path)


def _save_viewer_frame(force: bool = False) -> None:
  """Saves the viewer image every GUI_FRAME_PERIOD_S of wall time (or now, with force)."""
  now = time.perf_counter()
  if now < _frame_state["next"] and not force:
    return
  _frame_state["next"] = now + GUI_FRAME_PERIOD_S
  from omni.kit.viewport.utility import capture_viewport_to_file  # pylint: disable=g-import-not-at-top
  from omni.kit.viewport.utility import get_active_viewport  # pylint: disable=g-import-not-at-top

  path = os.path.join(GUI_FRAMES, f"frame_{_frame_state['index']:05d}.png")
  capture_viewport_to_file(get_active_viewport(), path)
  _frame_state["index"] += 1


def main() -> None:
  s = Scene()
  timing, sim, robot, graphs = s.timing, s.sim, s.robot, s.graphs
  tolerance_s = timing["stamp_match_tolerance_s"]

  node = rclpy.create_node("run_scene")
  node.create_service(Trigger, "/sim/reset", s.on_reset)
  node.create_service(Trigger, "/sim/attach_board", s.on_attach)
  node.create_service(Trigger, "/sim/detach_board", s.on_detach)
  if GUI:
    _setup_viewer()
  print(f"READY pid={os.getpid()}", flush=True)

  free_running = False
  gripper_stamp = graphs.gripper_command()[0]
  request_count = graphs.poll_camera_request()[0]
  pending_request = None
  cycles = free_run_cycles = renders = 0
  render_s = 0.0
  last_render = time.perf_counter()
  log_wall, log_sim, log_cycles = time.perf_counter(), sim.current_time, 0
  log_renders, log_render_s = 0, 0.0

  def render(count: int, publish_images: bool) -> None:
    """count renders; with publish_images, the last one publishes the images.

    The first render after physics steps can still show the previous state,
    so only the last of a request's renders is published. A closed image
    gate takes effect one render late, so one more render follows at the
    same sim time; otherwise the next render would publish an old frame with
    a new stamp (phase_2_notes.md, Addendum A).
    """
    nonlocal renders, render_s, last_render
    start = time.perf_counter()
    for i in range(count):
      graphs.set_image_publishing(publish_images and i == count - 1)
      sim.render()
    if publish_images:
      graphs.set_image_publishing(False)
      sim.render()
      count += 1
    last_render = time.perf_counter()
    renders += count
    render_s += last_render - start
    # sim_control spawns and deletes only inside renders (Inv §5.2).
    s.refresh_tf_targets()
    if PRETTY:
      from sim import pretty  # pylint: disable=g-import-not-at-top

      pretty.bind_objects(s.stage)

  while simulation_app.is_running():
    for _ in range(timing["substeps"]):
      sim.step(render=False)
    t = sim.current_time
    cycles += 1
    graphs.publish_state(*stamp(t), *s.wrench())

    timeout_s = (
        timing["free_run_wait_s"]
        if free_running
        else timing["free_run_timeout_s"]
    )
    if wait_for_command(graphs, t, timeout_s, tolerance_s):
      free_running = False
      position, velocity = graphs.arm_command()
      robot.apply_action(
          ArticulationAction(
              joint_positions=position,
              joint_velocities=velocity if velocity.size else None,
              joint_indices=ARM_JOINTS,
          )
      )
      s.held = position
    else:
      free_running = True
      free_run_cycles += 1
      robot.apply_action(
          ArticulationAction(
              joint_positions=s.held,
              joint_velocities=np.zeros_like(s.held),
              joint_indices=ARM_JOINTS,
          )
      )

    new_stamp, width_half, effort = graphs.gripper_command()
    if new_stamp != gripper_stamp:
      gripper_stamp = new_stamp
      robot.get_articulation_controller().set_max_efforts(
          np.array([min(max(effort, 0.0), FINGER_MAX_FORCE_N)]),
          joint_indices=np.array([s.finger]),
      )
      robot.apply_action(
          ArticulationAction(
              joint_positions=np.array([width_half]),
              joint_indices=np.array([s.finger]),
          )
      )

    if cycles % timing["tf_every_n_cycles"] == 0:
      graphs.publish_tf()

    count, request_stamp = graphs.poll_camera_request()
    if count != request_count:
      request_count, pending_request = count, request_stamp
    if pending_request is not None and t >= pending_request - tolerance_s:
      pending_request = None
      render(timing["camera_request_renders"], publish_images=True)
    elif PRETTY and t >= _frame_state["sim_next"]:
      _frame_state["sim_next"] = t + PRETTY_FRAME_PERIOD_S
      render(1, publish_images=False)
      if GUI:
        _save_viewer_frame(force=True)
    elif time.perf_counter() - last_render >= (
        GUI_RENDER_PERIOD_S if GUI and not PRETTY else timing["render_period_wall_s"]
    ):
      render(1, publish_images=False)
      if GUI:
        _save_viewer_frame()

    rclpy.spin_once(node, timeout_sec=0)

    now = time.perf_counter()
    if now - log_wall >= LOG_PERIOD_S:
      rtf = (sim.current_time - log_sim) / (now - log_wall)
      period_renders = renders - log_renders
      render_ms = (
          (render_s - log_render_s) / period_renders * 1e3
          if period_renders
          else 0.0
      )
      print(
          f"cycles {cycles} (+{cycles - log_cycles}) rtf {rtf:.2f}"
          f" free_run_cycles {free_run_cycles} renders {renders}"
          f" (+{period_renders}, {render_ms:.0f} ms each) epoch {s.epoch}",
          flush=True,
      )
      log_wall, log_sim, log_cycles = now, sim.current_time, cycles
      log_renders, log_render_s = renders, render_s

  node.destroy_node()
  simulation_app.close()


main()
