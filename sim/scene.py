"""Builds and edits run_scene.py's stage (phase_2.md, target file 8).

Objects are direct children of /World/objects (decision 11), each a reference
to a catalog model.usd whose origin is the object's bottom centre.
"""

from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.sensors.camera import Camera
import numpy as np
from pxr import Gf
from pxr import PhysxSchema
from pxr import Sdf
from pxr import Usd
from pxr import UsdGeom
from pxr import UsdLux
from pxr import UsdPhysics
from pxr import UsdShade

OBJECTS_ROOT = "/World/objects"
CATALOG_ROOT = "C:/isaac-sim-work/repo/assets/objects"
CHARUCO_PNG = "C:/isaac-sim-work/repo/assets/calib/charuco_7x5.png"
ROBOT_PRIM = "/World/franka"
FINGER_PRIMS = (
    f"{ROBOT_PRIM}/panda_leftfinger",
    f"{ROBOT_PRIM}/panda_rightfinger",
)
FINGER_MATERIAL = "/World/physics_materials/finger"
FINGER_FRICTION = (1.2, 1.0)  # static, dynamic
D455_PRIM = "/World/d455"
CAMERA_PRIM = "/World/d455/RSD455/Camera_Pseudo_Depth"
CAMERA_RESOLUTION = (1280, 720)
CAMERA_CLIPPING_M = (0.05, 20.0)

BOARD_PRIM = f"{OBJECTS_ROOT}/calib_board"
BOARD_SIZE_M = (0.28, 0.20, 0.005)
BOARD_MASS_KG = 0.2
# Beyond the fingertips along the hand's +z (Inv §6.2).
BOARD_OFFSET_M = (0.0, 0.0, 0.12)
# The texture sits just above the top face so it is not z-fighting the box.
TEXTURE_LIFT_M = 1e-4


def build(stage: Usd.Stage, scene: dict, asset_root: str) -> Camera:
  """Builds the default scene; returns the D455 depth camera."""
  GroundPlane(prim_path="/World/ground", z_position=scene["ground_z"])
  # Without lights the RGB image is black.
  UsdLux.DomeLight.Define(stage, "/World/lights/dome").CreateIntensityAttr(800.0)
  sun = UsdLux.DistantLight.Define(stage, "/World/lights/sun")
  sun.CreateIntensityAttr(2500.0)
  UsdGeom.XformCommonAPI(sun).SetRotate((-35.0, 25.0, 0.0))
  add_reference_to_stage(
      f"{asset_root}/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
      ROBOT_PRIM,
  )
  SingleXFormPrim(ROBOT_PRIM, position=np.array(scene["robot"]["position"]))
  _add_finger_material(stage)
  UsdGeom.Xform.Define(stage, OBJECTS_ROOT)
  _add_objects(stage, scene)

  add_reference_to_stage(
      f"{asset_root}/Isaac/Sensors/Intel/RealSense/rsd455.usd", D455_PRIM
  )
  SingleXFormPrim(
      D455_PRIM,
      position=np.array(scene["camera"]["position"]),
      orientation=euler_angles_to_quat(
          np.array(scene["camera"]["euler_deg"]), degrees=True
      ),
  )
  # The RSD455 root is a dynamic rigid body and falls otherwise (Inv §4.1).
  UsdPhysics.RigidBodyAPI(
      stage.GetPrimAtPath(f"{D455_PRIM}/RSD455")
  ).GetKinematicEnabledAttr().Set(True)
  camera = Camera(prim_path=CAMERA_PRIM, resolution=CAMERA_RESOLUTION)
  camera.initialize()
  camera.set_clipping_range(*CAMERA_CLIPPING_M)
  return camera


def reset_objects(stage: Usd.Stage, scene: dict) -> None:
  """Deletes every object under /World/objects and re-creates the defaults."""
  for child in stage.GetPrimAtPath(OBJECTS_ROOT).GetChildren():
    stage.RemovePrim(child.GetPath())
  _add_objects(stage, scene)


def object_paths(stage: Usd.Stage) -> list[str]:
  return [
      str(child.GetPath())
      for child in stage.GetPrimAtPath(OBJECTS_ROOT).GetChildren()
  ]


def attach_board(stage: Usd.Stage, robot) -> tuple[bool, str]:
  """Fixes the ChArUco board to the hand, BOARD_OFFSET_M along its +z."""
  if stage.GetPrimAtPath(BOARD_PRIM).IsValid():
    return False, "already attached"
  hand_path = f"{robot.prim_path}/panda_hand"
  hand_to_world = UsdGeom.Xformable(
      stage.GetPrimAtPath(hand_path)
  ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
  # Created where the joint wants it, so the joint starts satisfied.
  board_to_world = (
      Gf.Matrix4d().SetTranslate(Gf.Vec3d(*BOARD_OFFSET_M)) * hand_to_world
  )

  board = UsdGeom.Xform.Define(stage, BOARD_PRIM)
  board.AddTransformOp().Set(board_to_world)
  UsdPhysics.RigidBodyAPI.Apply(board.GetPrim())
  UsdPhysics.MassAPI.Apply(board.GetPrim()).CreateMassAttr(BOARD_MASS_KG)

  box = UsdGeom.Cube.Define(stage, f"{BOARD_PRIM}/box")
  box.CreateSizeAttr(1.0)
  box.AddScaleOp().Set(Gf.Vec3f(*BOARD_SIZE_M))
  UsdPhysics.CollisionAPI.Apply(box.GetPrim())
  _add_charuco_face(stage)

  joint = UsdPhysics.FixedJoint.Define(stage, f"{BOARD_PRIM}/attach_joint")
  joint.CreateBody0Rel().SetTargets([Sdf.Path(hand_path)])
  joint.CreateBody1Rel().SetTargets([Sdf.Path(BOARD_PRIM)])
  joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*BOARD_OFFSET_M))
  joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
  joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
  joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
  return True, "attached"


def detach_board(stage: Usd.Stage) -> tuple[bool, str]:
  if not stage.GetPrimAtPath(BOARD_PRIM).IsValid():
    return False, "not attached"
  stage.RemovePrim(BOARD_PRIM)
  return True, "detached"


def _add_finger_material(stage: Usd.Stage) -> None:
  """Grip friction on the finger colliders (phase_5b.md, Addendum C.1).

  PhysX gives the "max" combine mode priority over the objects' "average", so
  finger contacts use these values while object-object contacts are unchanged.
  """
  material = UsdShade.Material.Define(stage, FINGER_MATERIAL)
  friction = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
  friction.CreateStaticFrictionAttr(FINGER_FRICTION[0])
  friction.CreateDynamicFrictionAttr(FINGER_FRICTION[1])
  PhysxSchema.PhysxMaterialAPI.Apply(
      material.GetPrim()
  ).CreateFrictionCombineModeAttr("max")
  for path in FINGER_PRIMS:
    finger = stage.GetPrimAtPath(path)
    UsdShade.MaterialBindingAPI.Apply(finger).Bind(
        material, UsdShade.Tokens.strongerThanDescendants, "physics"
    )
    for prim in Usd.PrimRange(finger, Usd.TraverseInstanceProxies()):
      if prim.HasAPI(UsdPhysics.CollisionAPI):
        bound, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(
            "physics"
        )
        print(
            f"finger collider {prim.GetPath()} physics material"
            f" {bound.GetPath()}",
            flush=True,
        )


def _add_objects(stage: Usd.Stage, scene: dict) -> None:
  for name, spec in scene["objects"].items():
    path = f"{OBJECTS_ROOT}/{name}"
    add_reference_to_stage(f"{CATALOG_ROOT}/{spec['catalog']}/model.usd", path)
    pose = UsdGeom.XformCommonAPI(stage.GetPrimAtPath(path))
    pose.SetTranslate(Gf.Vec3d(*spec["position"]))
    pose.SetRotate(Gf.Vec3f(0.0, 0.0, spec["yaw"]))


def _add_charuco_face(stage: Usd.Stage) -> None:
  """A quad on the board's +z face textured with charuco_7x5.png (Inv §7.3)."""
  w, h = BOARD_SIZE_M[0] / 2, BOARD_SIZE_M[1] / 2
  z = BOARD_SIZE_M[2] / 2 + TEXTURE_LIFT_M
  face = UsdGeom.Mesh.Define(stage, f"{BOARD_PRIM}/charuco")
  face.CreatePointsAttr([(-w, -h, z), (w, -h, z), (w, h, z), (-w, h, z)])
  face.CreateFaceVertexCountsAttr([4])
  face.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
  face.CreateNormalsAttr([(0, 0, 1)] * 4)
  face.CreateExtentAttr([(-w, -h, z), (w, h, z)])
  face.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
  UsdGeom.PrimvarsAPI(face).CreatePrimvar(
      "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
  ).Set([(0, 0), (1, 0), (1, 1), (0, 1)])

  looks = f"{BOARD_PRIM}/Looks/charuco"
  material = UsdShade.Material.Define(stage, looks)
  surface = UsdShade.Shader.Define(stage, f"{looks}/surface")
  surface.CreateIdAttr("UsdPreviewSurface")
  surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
  reader = UsdShade.Shader.Define(stage, f"{looks}/st")
  reader.CreateIdAttr("UsdPrimvarReader_float2")
  reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
  texture = UsdShade.Shader.Define(stage, f"{looks}/texture")
  texture.CreateIdAttr("UsdUVTexture")
  texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(CHARUCO_PNG)
  texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
      reader.ConnectableAPI(), "result"
  )
  texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
  surface.CreateInput(
      "diffuseColor", Sdf.ValueTypeNames.Color3f
  ).ConnectToSource(texture.ConnectableAPI(), "rgb")
  material.CreateSurfaceOutput().ConnectToSource(
      surface.ConnectableAPI(), "surface"
  )
  UsdShade.MaterialBindingAPI.Apply(face.GetPrim()).Bind(material)
