"""Writes the catalog objects' meshes and physics descriptions.

For each object: assets/objects/<name>/mesh.glb (metres, Z up, origin at the
bottom centre, except calib_board: box centre, as Isaac's board prim) and
object.yaml (static, mass, color, collision, dims, category, color_name; origin: center if centred;
friction_static/friction_dynamic where not the default; container: wall and floor of the open boxes).
sim/tools/build_catalog.py turns them into model.usd. Dev-time only:
  .venv/bin/python tools/build_catalog_meshes.py
"""

from pathlib import Path
import sys

import numpy as np
import trimesh
import yaml

OUT_DIR = Path(__file__).resolve().parents[1] / "assets/objects"
# Cuts through the top face so the boolean leaves no coplanar sliver.
HOLE_OVERSHOOT_M = 0.002
TOLERANCE_M = 1e-6


def _box(x: float, y: float, z: float) -> trimesh.Trimesh:
  mesh = trimesh.creation.box(extents=(x, y, z))
  mesh.apply_translation((0.0, 0.0, z / 2))
  return mesh


def _cylinder(radius: float, height: float) -> trimesh.Trimesh:
  mesh = trimesh.creation.cylinder(radius=radius, height=height)
  mesh.apply_translation((0.0, 0.0, height / 2))
  return mesh


def _hole_block() -> trimesh.Trimesh:
  block = _box(0.08, 0.08, 0.05)
  hole = trimesh.creation.cylinder(radius=0.011, height=0.04 + HOLE_OVERSHOOT_M)
  hole.apply_translation((0.0, 0.0, 0.05 - 0.04 / 2 + HOLE_OVERSHOOT_M / 2))
  return trimesh.boolean.difference([block, hole], engine="manifold")


def _open_box(x: float, y: float, z: float, wall: float, floor: float) -> trimesh.Trimesh:
  # The inner box reaches past the top, so the box is open and the mesh stays watertight.
  inner = _box(x - 2 * wall, y - 2 * wall, z)
  inner.apply_translation((0.0, 0.0, floor))
  return trimesh.boolean.difference([_box(x, y, z), inner], engine="manifold")


# name: (mesh, static, mass kg, colour RGB)
OBJECTS = {
    "table": (_box(1.2, 0.8, 0.75), True, None, (0.55, 0.45, 0.35)),
    "red_cube": (_box(0.05, 0.05, 0.05), False, 0.10, (0.8, 0.1, 0.1)),
    "blue_cube": (_box(0.05, 0.05, 0.05), False, 0.10, (0.1, 0.2, 0.8)),
    "green_cube": (_box(0.05, 0.05, 0.05), False, 0.10, (0.1, 0.7, 0.2)),
    "small_yellow_cube": (_box(0.035, 0.035, 0.035), False, 0.05, (0.9, 0.8, 0.1)),
    "plate": (_cylinder(0.09, 0.012), False, 0.30, (0.9, 0.9, 0.9)),
    "peg": (_cylinder(0.010, 0.08), False, 0.05, (0.2, 0.4, 0.8)),
    "hole_block": (_hole_block(), True, None, (0.3, 0.3, 0.3)),
    # The ChArUco board sim/scene.py attaches to the hand (same size and mass;
    # origin at the box centre like its prim, so /tf poses match).
    "calib_board": (trimesh.creation.box(extents=(0.28, 0.20, 0.005)), False, 0.2, (1.0, 1.0, 1.0)),
    "obstacle": (_box(0.1, 0.1, 0.5), True, None, (0.5, 0.5, 0.5)),
    # Static: Isaac uses the exact triangle mesh, so the walls collide and the inside is free.
    "red_box": (_open_box(0.25, 0.21, 0.08, 0.01, 0.01), True, None, (0.8, 0.15, 0.15)),
    "blue_box": (_open_box(0.25, 0.21, 0.08, 0.01, 0.01), True, None, (0.15, 0.25, 0.8)),
}
# name: (category, colour word), the attributes list_objects reports (phase_ids.md decision 3).
ATTRIBUTES = {
    "table": ("table", "brown"),
    "red_cube": ("cube", "red"),
    "blue_cube": ("cube", "blue"),
    "green_cube": ("cube", "green"),
    "small_yellow_cube": ("cube", "yellow"),
    "plate": ("plate", "white"),
    "peg": ("peg", "blue"),
    "hole_block": ("block_with_hole", "grey"),
    "obstacle": ("pillar", "grey"),
    "calib_board": ("calibration_board", "white"),
    "red_box": ("box", "red"),
    "blue_box": ("box", "blue"),
}
# Static/dynamic friction where not build_catalog.py's default 0.5/0.4: the peg
# must slide on the block's face to the hole (phase_5b.md, Addendum B2).
FRICTION = {"peg": (0.3, 0.25), "hole_block": (0.3, 0.25)}
# name: (wall, floor) thickness (m) of the open boxes.
CONTAINERS = {"red_box": (0.01, 0.01), "blue_box": (0.01, 0.01)}


def main() -> int:
  failures = 0
  for name, (mesh, static, mass, color) in OBJECTS.items():
    assert mesh.is_watertight, f"{name} is not watertight"
    out = OUT_DIR / name
    out.mkdir(parents=True, exist_ok=True)
    glb = out / "mesh.glb"
    glb.write_bytes(mesh.export(file_type="glb"))
    # PhysX allows concave triangle meshes only on static bodies.
    description = {
        "static": static,
        "mass": mass,
        "color": list(color),
        "collision": "none" if static else "convexHull",
        # Bounding box (m); robot_service's PlanGrasp reads it (phase_5c.md decision 5).
        "dims": [round(float(v), 6) for v in mesh.extents],
        "category": ATTRIBUTES[name][0],
        "color_name": ATTRIBUTES[name][1],
    }
    if name == "calib_board":
      description["origin"] = "center"
    if name in FRICTION:
      description["friction_static"], description["friction_dynamic"] = FRICTION[name]
    if name in CONTAINERS:
      # robot_service's place planner reads it.
      wall, floor = CONTAINERS[name]
      description["container"] = {"wall": wall, "floor": floor}
    (out / "object.yaml").write_text(yaml.safe_dump(description))
    bounds = trimesh.load(glb, force="mesh").bounds
    ok = np.allclose(bounds, mesh.bounds, atol=TOLERANCE_M, rtol=0)
    failures += not ok
    print(
        f"{'OK  ' if ok else 'FAIL'} {name}: bounds {bounds.round(4).tolist()},"
        f" {len(mesh.faces)} faces, watertight {mesh.is_watertight}"
    )
  return 1 if failures else 0


if __name__ == "__main__":
  sys.exit(main())
