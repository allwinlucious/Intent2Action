"""--pretty: a clean studio look for demo videos (run_scene.py). Changes only light and materials, never physics.

A dark stage: the studio HDRI dome only faintly (reflections, no visible backdrop), a key light shaped into a
cone over the robot and table, and a dim rim light from behind; OmniPBR materials: a brushed-steel table, a
near-black floor, glossy plastic cubes, satin boxes. Objects spawned later get
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
# category -> (colour or None for the catalog's, reflection roughness, metallic)
FINISH = {"cube": (None, 0.28, 0.0), "box": (None, 0.55, 0.0), "carton": (None, 0.7, 0.0),
          "table": ((0.78, 0.79, 0.81), 0.3, 1.0)}
FLOOR_COLOR = (0.03, 0.03, 0.035)
RESOLUTION = (1920, 1080)
EYE, TARGET = [1.6, 1.05, 1.05], [0.32, 0.0, 0.3]  # closer and lower: the whole arm, the work area large in frame



def _material(name: str, color, roughness: float, metallic: float = 0.0) -> OmniPBR:
  mat = OmniPBR(f"{LOOKS}/{name}", name=name, color=np.array(color, dtype=float))
  mat.set_reflection_roughness(roughness)
  mat.set_metallic_constant(metallic)
  return mat


def _bind(stage, path: str, mat: OmniPBR, strength=UsdShade.Tokens.strongerThanDescendants) -> None:
  UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(mat.material, strength)


def _print_top(stage, path: str, dims, image: Path) -> None:
  """A printed top (assets/objects/<name>/top.png) as a textured quad just above the object's top face: visual only
  (no collision API), a child of the object so it moves with it. Image left-right runs along x, top along +y."""
  hx, hy, z = dims[0] / 2, dims[1] / 2, dims[2] + 0.001  # clear of the top face: no depth fighting
  quad = UsdGeom.Mesh.Define(stage, f"{path}/top_print")
  quad.CreatePointsAttr([Gf.Vec3f(-hx, -hy, z), Gf.Vec3f(hx, -hy, z), Gf.Vec3f(hx, hy, z), Gf.Vec3f(-hx, hy, z)])
  quad.CreateFaceVertexCountsAttr([4])
  quad.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
  quad.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
  quad.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
  UsdGeom.PrimvarsAPI(quad).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
      [Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
  mat = OmniPBR(f"{LOOKS}/print_{image.parent.name}", name=f"print_{image.parent.name}", texture_path=str(image))
  mat.set_reflection_roughness(0.6)
  _bind(stage, f"{path}/top_print", mat)


def apply(stage, asset_root: str) -> None:
  """Lights and the floor; call once after scene.build."""
  dome = UsdLux.DomeLight(stage.GetPrimAtPath("/World/lights/dome"))
  dome.CreateTextureFileAttr().Set(Sdf.AssetPath(f"{asset_root}/{HDRI}"))
  dome.CreateTextureFormatAttr().Set(UsdLux.Tokens.latlong)
  dome.GetIntensityAttr().Set(140.0)  # reflections for the steel, no visible backdrop
  # RTX: the dome still lights and reflects, but the camera sees black behind the scene.
  dome.GetPrim().CreateAttribute("visibleInPrimaryRay", Sdf.ValueTypeNames.Bool).Set(False)
  sun = UsdLux.DistantLight(stage.GetPrimAtPath("/World/lights/sun"))
  sun.GetIntensityAttr().Set(0.0)
  key = UsdLux.RectLight.Define(stage, "/World/lights/key")  # a softbox above the robot and table, facing down
  key.CreateWidthAttr(0.9)
  key.CreateHeightAttr(0.7)
  key.CreateIntensityAttr(38000.0)
  key.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))
  shaping = UsdLux.ShapingAPI.Apply(key.GetPrim())  # a cone: light on the work area, falling off into the dark
  shaping.CreateShapingConeAngleAttr(55.0)
  shaping.CreateShapingConeSoftnessAttr(0.5)
  UsdGeom.XformCommonAPI(key).SetTranslate((0.35, 0.0, 1.7))
  rim = UsdLux.RectLight.Define(stage, "/World/lights/rim")  # behind the robot, low: separates it from the dark
  rim.CreateWidthAttr(1.0)
  rim.CreateHeightAttr(0.5)
  rim.CreateIntensityAttr(5000.0)
  rim.CreateColorAttr(Gf.Vec3f(0.85, 0.9, 1.0))
  UsdGeom.XformCommonAPI(rim).SetTranslate((-0.9, 0.0, 0.9))
  UsdGeom.XformCommonAPI(rim).SetRotate((0.0, -60.0, 0.0))
  _bind(stage, "/World/ground", _material("floor", FLOOR_COLOR, 0.85))
  camera = stage.GetPrimAtPath("/World/d455")  # the sensor's model would float in the shot; it is not filmed
  if camera.IsValid():
    UsdGeom.Imageable(camera).MakeInvisible()
  bind_objects(stage)


def bind_objects(stage) -> None:
  """Gives every object under /World/objects without a look yet its colour and finish."""
  root = stage.GetPrimAtPath("/World/objects")
  if not root.IsValid():
    return
  for prim in root.GetChildren():
    path = str(prim.GetPath())
    # A reset re-creates objects at the same paths: check the prim's own binding, not a remembered path.
    if UsdShade.MaterialBindingAPI(prim).GetDirectBindingRel().GetTargets():
      continue
    catalog = prim.GetAttribute("catalog:name").Get() if prim.HasAttribute("catalog:name") else None
    entry = CATALOG_ROOT / str(catalog) / "object.yaml"
    if not catalog or not entry.is_file():
      continue
    description = yaml.safe_load(entry.read_text())
    color, roughness, metallic = FINISH.get(description.get("category", ""), (None, 0.5, 0.0))
    top = CATALOG_ROOT / str(catalog) / "top.png"
    # With a printed top the body's colour must not override the print's own material below it.
    strength = UsdShade.Tokens.weakerThanDescendants if top.is_file() else UsdShade.Tokens.strongerThanDescendants
    _bind(stage, path, _material(f"look_{catalog}", color or description["color"], roughness, metallic), strength)
    if top.is_file():
      _print_top(stage, path, description["dims"], top)
