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


def _textured_box(x: float, y: float, z: float, faces_dir: Path, top_name: str = "top.png",
                  front_name: str = "front.png", back_name: str | None = None) -> trimesh.Trimesh:
  """A box (bottom-centre origin) with one UV region per face on a texture sheet composed from photos of the real
  object: faces_dir/top.png (seen from above, image top = +y) and front.png (the -y face). The back mirrors the
  front, the ends reuse a crop of it, the bottom is the face's plain colour."""
  from PIL import Image
  top = Image.open(faces_dir / top_name).convert("RGB")
  front = Image.open(faces_dir / front_name).convert("RGB")
  ppm = top.width / x  # pixels per metre on the sheet
  size = lambda a, b: (max(1, round(a * ppm)), max(1, round(b * ppm)))  # noqa: E731
  tw, th = size(x, y)
  fw, fh = size(x, z)
  ew, _ = size(y, z)
  # The side's red, matched to the top's (the photo's side is in shadow).
  scale = np.median(np.asarray(top.resize((50, 40))).reshape(-1, 3), 0) / np.median(np.asarray(front.resize((50, 20))).reshape(-1, 3), 0)
  front = Image.fromarray(np.clip(np.asarray(front.resize((fw, fh))) * scale, 0, 255).astype(np.uint8))
  plain = tuple(int(c) for c in np.median(np.asarray(front).reshape(-1, 3), 0))
  rects = {"top": (0, 0, tw, th), "bottom": (tw, 0, tw, th), "front": (0, th, fw, fh), "back": (fw, th, fw, fh),
           "left": (2 * fw, th, ew, fh), "right": (2 * fw + ew, th, ew, fh)}
  sheet = Image.new("RGB", (max(2 * tw, 2 * fw + 2 * ew), th + fh), plain)
  sheet.paste(top.resize((tw, th)), (0, 0))
  sheet.paste(front, (0, th))
  back = Image.open(faces_dir / back_name).convert("RGB").resize((fw, fh)) if back_name else \
      front.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
  sheet.paste(back, (fw, th))
  end = front.crop((0, 0, ew, fh))
  sheet.paste(end, (2 * fw, th))
  sheet.paste(end.transpose(Image.Transpose.FLIP_LEFT_RIGHT), (2 * fw + ew, th))
  hx, hy = x / 2, y / 2
  # Each face: corners counter-clockwise seen from outside, as the image's bottom-left, bottom-right, top-right, top-left.
  corners = {
      "top": [(-hx, -hy, z), (hx, -hy, z), (hx, hy, z), (-hx, hy, z)],
      "bottom": [(-hx, hy, 0), (hx, hy, 0), (hx, -hy, 0), (-hx, -hy, 0)],
      "front": [(-hx, -hy, 0), (hx, -hy, 0), (hx, -hy, z), (-hx, -hy, z)],
      "back": [(hx, hy, 0), (-hx, hy, 0), (-hx, hy, z), (hx, hy, z)],
      "left": [(-hx, hy, 0), (-hx, -hy, 0), (-hx, -hy, z), (-hx, hy, z)],
      "right": [(hx, -hy, 0), (hx, hy, 0), (hx, hy, z), (hx, -hy, z)],
  }
  W, H = sheet.size
  vertices, uv, triangles = [], [], []
  for face, points in corners.items():
    left, upper, width, height = rects[face]
    inset = 2  # pixels: no bleeding from the neighbouring face
    u0, u1 = (left + inset) / W, (left + width - inset) / W
    v0, v1 = 1 - (upper + height - inset) / H, 1 - (upper + inset) / H  # UV v runs up, image rows run down
    base = len(vertices)
    vertices += points
    uv += [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
    triangles += [(base, base + 1, base + 2), (base, base + 2, base + 3)]
  mesh = trimesh.Trimesh(vertices, triangles, process=False)
  mesh.visual = trimesh.visual.TextureVisuals(uv=np.array(uv), image=sheet)
  return mesh


def _lidded_box(base, lid, faces_dir: Path) -> trimesh.Trimesh:
  """A closed box with a slip-on lid as one solid: the base (x, y, height below the lid) and the lid (x, y,
  height) stacked, each textured from its own photo faces (base_front.png; top.png and lid_side.png)."""
  lower = _textured_box(*base, faces_dir, top_name="top.png", front_name="base_front.png")
  upper = _textured_box(*lid, faces_dir, top_name="top.png", front_name="lid_side.png")
  upper.apply_translation((0.0, 0.0, base[2]))
  return trimesh.util.concatenate([lower, upper])


def _textured_open_box(x: float, y: float, z: float, wall: float, floor: float, faces_dir: Path) -> trimesh.Trimesh:
  """An open box (bottom-centre origin) from five textured slabs: walls with chevron.png outside and white.png inside
  and on their top edge, a white floor. Overlapping closed slabs: a static body collides with the exact triangles."""
  def slab(length, height, angle, dx, dy):
    part = _textured_box(length, wall, height, faces_dir, top_name="white.png", front_name="chevron.png",
                         back_name="white.png")  # front (-y) outward before the turn
    part.apply_transform(trimesh.transformations.rotation_matrix(angle, (0, 0, 1)))
    part.apply_translation((dx, dy, 0.0))
    return part
  parts = [_textured_box(x, y, floor, faces_dir, top_name="white.png", front_name="chevron.png"),
           slab(x, z, 0.0, 0.0, -(y - wall) / 2), slab(x, z, np.pi, 0.0, (y - wall) / 2),
           slab(y - 2 * wall, z, -np.pi / 2, -(x - wall) / 2, 0.0), slab(y - 2 * wall, z, np.pi / 2, (x - wall) / 2, 0.0)]
  return trimesh.util.concatenate(parts)


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
    "red_box": (_open_box(0.25, 0.21, 0.05, 0.01, 0.01), True, None, (0.8, 0.15, 0.15)),
    "blue_box": (_open_box(0.25, 0.21, 0.05, 0.01, 0.01), True, None, (0.15, 0.25, 0.8)),
    # Push demo: a carton wider than the gripper opens (0.075 m) in every direction, so it can only be pushed.
    "big_carton": (_box(0.12, 0.12, 0.08), False, 0.40, (0.66, 0.50, 0.33)),
    # The owner's red gift box (11 x 9 x 4 cm) at its proportions, scaled so its longest side (7 cm) fits the
    # gripper; it is grasped across the 5.7 cm side.
    "red_gift": (_textured_box(0.07, 0.0573, 0.0255, OUT_DIR / "red_gift"), False, 0.08, (0.72, 0.12, 0.12)),
    # The owner's green box at the same scale: base 12.5 x 8.5, lid 13 x 9 x 2, closed height 8 cm; the lid's skirt
    # covers the base's top 2 cm, so the base shows 381.82 cm below the lid.
    "green_gift": (_lidded_box((0.0795, 0.0541, 0.0382),
                               (0.0827, 0.0573, 0.0127), OUT_DIR / "green_gift"),
                   False, 0.15, (0.12, 0.2, 0.17)),
    # The owner's heart box at the same scale: an open base 8.8 x 8.6 x 8.6 cm (2 mm walls; a container that can be
    # picked up and stacked),
    # its removable lid 9 x 9 x 2.5 cm, and the small heart box (the same design, 6 cm closed) that goes inside.
    "heart_box": (_textured_open_box(0.056, 0.0547, 0.0547, 0.0013, 0.0013,
                                     OUT_DIR / "heart_box"), False, 0.05, (0.93, 0.93, 0.94)),
    "heart_lid": (_textured_box(0.0573, 0.0573, 0.0159, OUT_DIR / "heart_lid"),
                  False, 0.03, (0.68, 0.62, 0.59)),
    "small_heart_gift": (_lidded_box((0.0374, 0.0365, 0.0276),
                                     (0.0382, 0.0382, 0.0106),
                                     OUT_DIR / "small_heart_gift"), False, 0.04, (0.68, 0.62, 0.59)),
    # The owner's second gift box (white, reindeer and sleigh), the same size.
    "white_gift": (_textured_box(0.07, 0.0573, 0.0255, OUT_DIR / "white_gift"), False, 0.08, (0.92, 0.92, 0.92)),
    # A painted area: 2 mm thick, spawned 1.9 mm into the table so it is flush and a pushed carton slides over it.
    "target_zone": (_box(0.16, 0.16, 0.002), True, None, (0.20, 0.65, 0.30)),
    # Objects wider than the gripper opens, and flat goal markers, for the push experience (learn/push_experience.py).
    "wide_crate": (_box(0.14, 0.10, 0.07), False, 0.35, (0.15, 0.30, 0.70)),
    "flat_tray": (_box(0.16, 0.12, 0.04), False, 0.25, (0.55, 0.55, 0.58)),
    "tote": (_box(0.13, 0.13, 0.09), False, 0.40, (0.90, 0.75, 0.15)),
    "pallet_mat": (_box(0.18, 0.15, 0.002), True, None, (0.20, 0.35, 0.75)),
    "marker_pad": (_box(0.15, 0.15, 0.002), True, None, (0.85, 0.85, 0.85)),
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
    "big_carton": ("carton", "brown"),
    "red_gift": ("gift", "red"),
    "white_gift": ("gift", "white"),
    "green_gift": ("gift", "green"),
    "heart_box": ("box", "grey"),
    "heart_lid": ("lid", "mauve"),
    "small_heart_gift": ("gift", "mauve"),
    "target_zone": ("zone", "green"),
    "wide_crate": ("crate", "blue"),
    "flat_tray": ("tray", "grey"),
    "tote": ("tote", "yellow"),
    "pallet_mat": ("mat", "blue"),
    "marker_pad": ("pad", "white"),
}
# Static/dynamic friction where not build_catalog.py's default 0.5/0.4: the peg
# must slide on the block's face to the hole (phase_5b.md, Addendum B2).
FRICTION = {"peg": (0.3, 0.25), "hole_block": (0.3, 0.25), "big_carton": (0.35, 0.3), "wide_crate": (0.35, 0.3),
            "flat_tray": (0.35, 0.3), "tote": (0.35, 0.3)}
# Paint only: no collision in Isaac (a pushed object must slide over the zone's edge, not snag on it).
NO_COLLISION = {"target_zone", "pallet_mat", "marker_pad"}
# Built from overlapping closed slabs (static: Isaac collides with the exact triangles, the planner with
# collision_parts), so the merged surface is not one closed shell.
SLAB_BUILT = {"heart_box"}
# A moving open box keeps its cavity: Isaac collides with its slabs (collision_parts), one convex hull each.
COLLISION = {"heart_box": "convexHull"}  # Isaac uses one hull per collision part (sim/tools/build_catalog.py)
# name: (wall, floor) thickness (m) of the open boxes.
CONTAINERS = {"red_box": (0.01, 0.01), "blue_box": (0.01, 0.01), "heart_box": (0.0013, 0.0013)}


def _container_parts(x: float, y: float, z: float, wall: float, floor: float) -> list[trimesh.Trimesh]:
  """Floor and four walls of an open box as separate convex slabs (bottom-centre origin): Intrinsic's collision
  checker treats each geometry as convex, so a single open-box mesh would fill the cavity."""
  def slab(sx, sy, sz, cx, cy, cz):
    mesh = trimesh.creation.box(extents=(sx, sy, sz))
    mesh.apply_translation((cx, cy, cz + sz / 2))
    return mesh
  return [
      slab(x, y, floor, 0.0, 0.0, 0.0),
      slab(wall, y, z, -(x - wall) / 2, 0.0, 0.0),
      slab(wall, y, z, (x - wall) / 2, 0.0, 0.0),
      slab(x - 2 * wall, wall, z, 0.0, -(y - wall) / 2, 0.0),
      slab(x - 2 * wall, wall, z, 0.0, (y - wall) / 2, 0.0),
  ]


def main() -> int:
  failures = 0
  for name, (mesh, static, mass, color) in OBJECTS.items():
    closed = mesh.copy()
    closed.merge_vertices(merge_tex=True, merge_norm=True)  # a textured mesh has one vertex per face corner
    assert closed.is_watertight or name in SLAB_BUILT, f"{name} is not watertight"
    out = OUT_DIR / name
    out.mkdir(parents=True, exist_ok=True)
    glb = out / "mesh.glb"
    glb.write_bytes(mesh.export(file_type="glb"))
    if isinstance(mesh.visual, trimesh.visual.TextureVisuals):  # sim/tools/build_catalog.py binds it as the look
      mesh.visual.material.image.save(out / "texture.png")
    # PhysX allows concave triangle meshes only on static bodies.
    description = {
        "static": static,
        "mass": mass,
        "color": list(color),
        "collision": "none" if static else COLLISION.get(name, "convexHull"),
        # Bounding box (m); robot_service's PlanGrasp reads it (phase_5c.md decision 5).
        "dims": [round(float(v), 6) for v in mesh.extents],
        "category": ATTRIBUTES[name][0],
        "color_name": ATTRIBUTES[name][1],
    }
    if name == "calib_board":
      description["origin"] = "center"
    if name in NO_COLLISION:
      description["collides"] = False
    if name in FRICTION:
      description["friction_static"], description["friction_dynamic"] = FRICTION[name]
    if name in CONTAINERS:
      # robot_service's place planner reads it.
      wall, floor = CONTAINERS[name]
      description["container"] = {"wall": wall, "floor": floor}
    (out / "object.yaml").write_text(yaml.safe_dump(description))
    if name in CONTAINERS:  # robot_service's planner world gets these as the collision set (scene_sync.py)
      parts = out / "collision_parts"
      parts.mkdir(exist_ok=True)
      wall, floor = CONTAINERS[name]
      for k, part in enumerate(_container_parts(*mesh.extents, wall, floor)):
        (parts / f"part_{k}.glb").write_bytes(part.export(file_type="glb"))
    bounds = trimesh.load(glb, force="mesh").bounds
    ok = np.allclose(bounds, mesh.bounds, atol=TOLERANCE_M, rtol=0)
    failures += not ok
    print(
        f"{'OK  ' if ok else 'FAIL'} {name}: bounds {bounds.round(4).tolist()},"
        f" {len(mesh.faces)} faces, watertight {closed.is_watertight}"
    )
  return 1 if failures else 0


if __name__ == "__main__":
  sys.exit(main())
