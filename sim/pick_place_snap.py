# Runs Isaac Sim's built-in Franka pick-and-place demo headless and saves camera snapshots.
# Run with: C:\isaac-sim\python.bat pick_place_snap.py
import os

from isaacsim import SimulationApp

# --/app/vulkan=false: Vulkan crashes in rtx.scenedb on NVIDIA drivers newer than 580 (isaac-sim/IsaacSim#912).
simulation_app = SimulationApp({"headless": True, "extra_args": ["--/app/vulkan=false"]})

import numpy as np
import omni.timeline
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.examples.franka import FrankaPickPlace
from isaacsim.sensors.camera import Camera
from PIL import Image

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "shots")
os.makedirs(OUT, exist_ok=True)

SimulationManager.set_physics_sim_device("cpu")
simulation_app.update()

pick_place = FrankaPickPlace()
pick_place.setup_scene()

camera = Camera(prim_path="/World/SnapCam", resolution=(1280, 720))
camera.initialize()
set_camera_view(eye=[1.6, 1.4, 1.1], target=[0.3, 0.0, 0.2], camera_prim_path="/World/SnapCam")

omni.timeline.get_timeline_interface().play()
simulation_app.update()
pick_place.reset()


def snap(name):
    rgba = camera.get_rgba()
    if rgba is None or rgba.size == 0:
        print(f"[snap] {name}: no frame yet", flush=True)
        return
    path = os.path.join(OUT, f"{name}.png")
    Image.fromarray(rgba[:, :, :3].astype(np.uint8)).save(path)
    print(f"[snap] saved {path}", flush=True)


# Let the renderer warm up so the first frame isn't black.
for _ in range(30):
    simulation_app.update()
snap("00_start")

step = 0
while simulation_app.is_running() and not pick_place.is_done() and step < 3000:
    pick_place.forward()
    simulation_app.update()
    step += 1
    if step % 150 == 0:
        snap(f"step_{step:04d}")

print(f"done={pick_place.is_done()} after {step} steps", flush=True)
for _ in range(30):
    simulation_app.update()
snap("99_end")
simulation_app.close()
