"""--pretty: a clean studio look for demo videos (run_scene.py). Changes only light and materials, never physics.

A studio HDRI dome (image-based light and reflections), a soft key light and softer sun shadows; OmniPBR
materials: a matte light-grey table, a dark floor, glossy plastic cubes, satin boxes. Objects spawned later get
their material on the next render (bind_objects).
"""

from pathlib import Path

import numpy as np

from isaacsim.core.api.materials import OmniPBR
from pxr import Gf
from pxr import Sdf
from pxr import UsdGeom
from pxr import UsdLux
from pxr import UsdShade
import yaml

HDRI = "NVIDIA/Assets/Skies/Studio/photo_studio_01_4k.hdr"
CATALOG_ROOT = Path("C:/isaac-sim-work/repo/assets/objects")
LOOKS = "/World/Looks"
# category -> (colour or None for the catalog's, reflection roughness)
FINISH = {"cube": (None, 0.28), "box": (None, 0.55), "table": ((0.78, 0.78, 0.76), 0.6)}
FLOOR_COLOR = (0.2, 0.2, 0.21)
EYE, TARGET = [2.0, 1.4, 1.2], [0.4, -0.05, -0.05]  # the whole table, both boxes, the robot and some room around

_bound: set[str] = set()


def _material(name: str, color, roughness: float) -> OmniPBR:
  mat = OmniPBR(f"{LOOKS}/{name}", name=name, color=np.array(color, dtype=float))
  mat.set_reflection_roughness(roughness)
  mat.set_metallic_constant(0.0)
  return mat


def _bind(stage, path: str, mat: OmniPBR) -> None:
  UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(
      mat.material, UsdShade.Tokens.strongerThanDescendants)


def apply(stage, asset_root: str) -> None:
  """Lights and the floor; call once after scene.build."""
  dome = UsdLux.DomeLight(stage.GetPrimAtPath("/World/lights/dome"))
  dome.CreateTextureFileAttr().Set(Sdf.AssetPath(f"{asset_root}/{HDRI}"))
  dome.CreateTextureFormatAttr().Set(UsdLux.Tokens.latlong)
  dome.GetIntensityAttr().Set(550.0)
  sun = UsdLux.DistantLight(stage.GetPrimAtPath("/World/lights/sun"))
  sun.GetIntensityAttr().Set(1100.0)
  sun.CreateAngleAttr().Set(2.5)  # soft shadow edges
  key = UsdLux.RectLight.Define(stage, "/World/lights/key")  # a softbox above the table, facing down
  key.CreateWidthAttr(1.2)
  key.CreateHeightAttr(0.9)
  key.CreateIntensityAttr(2200.0)
  key.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))
  UsdGeom.XformCommonAPI(key).SetTranslate((0.5, 0.0, 1.5))
  _bind(stage, "/World/ground", _material("floor", FLOOR_COLOR, 0.85))
  bind_objects(stage)


def bind_objects(stage) -> None:
  """Gives every object under /World/objects without a look yet its colour and finish."""
  root = stage.GetPrimAtPath("/World/objects")
  if not root.IsValid():
    return
  for prim in root.GetChildren():
    path = str(prim.GetPath())
    if path in _bound:
      continue
    catalog = prim.GetAttribute("catalog:name").Get() if prim.HasAttribute("catalog:name") else None
    entry = CATALOG_ROOT / str(catalog) / "object.yaml"
    if not catalog or not entry.is_file():
      continue
    description = yaml.safe_load(entry.read_text())
    color, roughness = FINISH.get(description.get("category", ""), (None, 0.5))
    _bind(stage, path, _material(f"look_{catalog}", color or description["color"], roughness))
    _bound.add(path)
