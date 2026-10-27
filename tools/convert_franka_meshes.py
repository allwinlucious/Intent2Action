"""Converts Isaac's Franka visual meshes (COLLADA) to GLB for GeometryService.

CreateGeometry has no DAE input, so the visual meshes are converted once.
Dev-time only (needs trimesh and pycollada in the venv):
  .venv/bin/python tools/convert_franka_meshes.py
"""

from pathlib import Path
import sys

import numpy as np
import trimesh

SOURCE_DIR = Path(
    "/mnt/c/isaac-sim/exts/isaacsim.asset.importer.urdf/data/urdf/robots/"
    "franka_description/meshes/visual"
)
OUT_DIR = (
    Path(__file__).resolve().parents[1] / "assets/robots/franka/meshes/visual"
)
TOLERANCE_M = 1e-6


def main() -> int:
  OUT_DIR.mkdir(parents=True, exist_ok=True)
  failures = 0
  for dae in sorted(SOURCE_DIR.glob("*.dae")):
    scene = trimesh.load(dae, force="scene")
    dae_bounds = scene.bounds
    # GeometryService's assimp import turns zero-area triangles into lines and
    # then rejects the mesh ("Unsupported number of vertices in a face").
    dropped = 0
    for mesh in scene.geometry.values():
      keep = mesh.nondegenerate_faces()
      if keep.all():
        continue
      dropped += int((~keep).sum())
      normals = (
          mesh.vertex_normals
      )  # keep the DAE's normals (vertices are unchanged)
      mesh.update_faces(keep)
      mesh.vertex_normals = normals
    glb = OUT_DIR / f"{dae.stem}.glb"
    glb.write_bytes(scene.export(file_type="glb"))
    glb_bounds = trimesh.load(glb, force="scene").bounds
    ok = np.allclose(dae_bounds, glb_bounds, atol=TOLERANCE_M, rtol=0)
    failures += not ok
    print(
        f"{'OK  ' if ok else 'FAIL'} {dae.name}: dae {dae_bounds.tolist()}"
        f" glb {glb_bounds.tolist()}, dropped {dropped} zero-area faces"
    )
  return 1 if failures else 0


if __name__ == "__main__":
  sys.exit(main())
