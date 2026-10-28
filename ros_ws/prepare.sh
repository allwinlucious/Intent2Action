#!/usr/bin/env bash
# Generates franka_grasp_config's urdf/ and meshes/ from assets/robots/franka (phase_5c.md decision 3).
# Run before colcon (container/Dockerfile does); paths are relative to this script, so it also works
# from the repository root.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FRANKA="$ROOT/assets/robots/franka"
OUT="$ROOT/ros_ws/src/franka_grasp_config"

mkdir -p "$OUT/urdf" "$OUT/meshes"
cp "$FRANKA"/meshes/collision/*.stl "$OUT/meshes/"
python3 - "$FRANKA/lula_franka_gen.urdf" "$OUT/urdf/franka.urdf" <<'EOF'
import re
import sys
import xml.etree.ElementTree as ET

source, target = sys.argv[1:]
tree = ET.parse(source)
robot = tree.getroot()
# Intrinsic's model drops the wrist camera too (phase 1).
for element in [e for e in robot if e.tag in ("link", "joint") and e.get("name", "").startswith("camera_")]:
  robot.remove(element)
for mesh in robot.iter("mesh"):
  mesh.set("filename", re.sub(r"^package://franka_description/meshes/(?:visual|collision)/(\w+)\.(?:dae|stl)$",
                              r"package://franka_grasp_config/meshes/\1.stl", mesh.get("filename")))
  if not mesh.get("filename").startswith("package://franka_grasp_config/meshes/"):
    sys.exit(f"unexpected mesh URI {mesh.get('filename')}")
tree.write(target, encoding="utf-8", xml_declaration=True)
EOF
echo "prepare.sh: wrote $OUT/urdf/franka.urdf and $(ls "$OUT/meshes" | wc -l) meshes"
