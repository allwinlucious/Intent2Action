"""Writes model.usd next to each catalog mesh, in Isaac Sim's Python.

Reads C:\\isaac-sim-work\\repo\\assets\\objects\\<name>\\{mesh.glb,object.yaml}
(tools/build_catalog_meshes.py) and writes model.usd: Z up, metres, default
prim /Object (attribute catalog:name) with a child Mesh and physics. Run from
WSL (phase_2.md, target file 6):
  tools/sync_isaac.sh && cmd.exe /c "cd /d C:\\isaac-sim && python.bat C:\\isaac-sim-work\\repo\\sim\\tools\\build_catalog.py" && tools/sync_isaac.sh --pull-catalog
"""

from pathlib import Path
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"headless": True, "extra_args": ["--/app/vulkan=false"]}
)

# trimesh and PyYAML ship in this prebundle (phase_2_inventory.md §7.2).
sys.path.append(r"C:\isaac-sim\exts\omni.pip.compute\pip_prebundle")

from pxr import Gf  # noqa: E402  pylint: disable=g-import-not-at-top
from pxr import PhysxSchema
from pxr import Sdf
from pxr import Usd
from pxr import UsdGeom
from pxr import UsdPhysics
from pxr import UsdShade
from pxr import Vt
import trimesh  # noqa: E402  pylint: disable=g-import-not-at-top
import yaml  # noqa: E402  pylint: disable=g-import-not-at-top

OBJECTS_DIR = Path(r"C:\isaac-sim-work\repo\assets\objects")
COMPLIANT_DAMPING = 2000.0  # N·s/m


def write_model(directory: Path) -> None:
  description = yaml.safe_load((directory / "object.yaml").read_text())
  mesh = trimesh.load(directory / "mesh.glb", force="mesh")
  stage = Usd.Stage.CreateNew(str(directory / "model.usd"))
  UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
  UsdGeom.SetStageMetersPerUnit(stage, 1.0)

  root = UsdGeom.Xform.Define(stage, "/Object").GetPrim()
  stage.SetDefaultPrim(root)
  root.CreateAttribute("catalog:name", Sdf.ValueTypeNames.String).Set(
      directory.name
  )

  geom = UsdGeom.Mesh.Define(stage, "/Object/mesh")
  geom.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(mesh.vertices.astype("f4")))
  geom.CreateFaceVertexCountsAttr([3] * len(mesh.faces))
  geom.CreateFaceVertexIndicesAttr(mesh.faces.reshape(-1).tolist())
  geom.CreateExtentAttr([Gf.Vec3f(*mesh.bounds[0]), Gf.Vec3f(*mesh.bounds[1])])
  # Rendered as the flat triangles PhysX collides with, not a subdivision surface.
  geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
  geom.CreateDisplayColorAttr([Gf.Vec3f(*description["color"])])
  if not description.get("collides", True):  # paint only (a floor marking): no physics at all
    stage.GetRootLayer().Save()
    print(f"CATALOG {directory.name}: {len(mesh.faces)} faces, visual only", flush=True)
    return
  UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
  UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr(
      description["collision"]
  )
  # Contacts start 1 mm out and rest at the surface: the peg-in-hole clearance is 1 mm per side.
  physx = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
  physx.CreateContactOffsetAttr(0.001)
  physx.CreateRestOffsetAttr(0.0)
  # Compliant contact at MakeContact's MEDIUM environment stiffness (60 kN/m): rigid PhysX contact spikes past 85 N.
  material = UsdShade.Material.Define(stage, "/Object/physics_material")
  UsdPhysics.MaterialAPI.Apply(material.GetPrim()).CreateStaticFrictionAttr(
      description.get("friction_static", 0.5)
  )
  UsdPhysics.MaterialAPI(material.GetPrim()).CreateDynamicFrictionAttr(
      description.get("friction_dynamic", 0.4)
  )
  physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
  physx_material.CreateCompliantContactStiffnessAttr(60000.0)
  physx_material.CreateCompliantContactDampingAttr(COMPLIANT_DAMPING)
  UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(
      material, UsdShade.Tokens.weakerThanDescendants, "physics"
  )

  if not description["static"]:
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(description["mass"])
  stage.GetRootLayer().Save()
  print(
      f"CATALOG {directory.name}: {len(mesh.faces)} faces,"
      f" static {description['static']}, collision {description['collision']},"
      f" compliant damping {COMPLIANT_DAMPING}",
      flush=True,
  )


def main() -> None:
  for directory in sorted(p.parent for p in OBJECTS_DIR.glob("*/mesh.glb")):
    write_model(directory)
  simulation_app.close()


main()
