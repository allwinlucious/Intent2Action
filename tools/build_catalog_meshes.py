"""Writes the catalog objects' meshes and physics descriptions.

For each object: assets/objects/<name>/mesh.glb (metres, Z up, origin at the
bottom centre) and object.yaml (static, mass, color, collision).
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


# name: (mesh, static, mass kg, colour RGB)
OBJECTS = {
    "table": (_box(1.2, 0.8, 0.75), True, None, (0.55, 0.45, 0.35)),
    "red_cube": (_box(0.05, 0.05, 0.05), False, 0.10, (0.8, 0.1, 0.1)),
    "plate": (_cylinder(0.09, 0.012), False, 0.30, (0.9, 0.9, 0.9)),
    "peg": (_cylinder(0.010, 0.08), False, 0.05, (0.2, 0.4, 0.8)),
    "hole_block": (_hole_block(), True, None, (0.3, 0.3, 0.3)),
    # The ChArUco board sim/scene.py attaches to the hand (same size and mass).
    "calib_board": (_box(0.21, 0.15, 0.005), False, 0.2, (1.0, 1.0, 1.0)),
    "obstacle": (_box(0.1, 0.1, 0.5), True, None, (0.5, 0.5, 0.5)),
}


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
    }
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
