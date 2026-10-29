"""Place planner (phase_v1.md decision 5, Addendum A.3): where to release a held object, in a box or on top.

Pure functions, no gRPC calls, no ROS. Footprints are axis-aligned rectangles (half extents) in the box frame:
the box position, rotated by the box yaw. Poses are bottom centres in the world (= base) frame.
"""

import math

from robot_service import robot_service_pb2 as api

GRID_M = 0.01  # spacing of candidate spots inside a box
EDGE_MARGIN_M = 0.005  # held footprint to inner wall
GAP_M = 0.005  # held footprint to objects already in the box
FINGER_HALF_SPAN_M = 0.0475  # open fingers: 0.075 / 2 + pad thickness 0.01
FINGER_HALF_WIDTH_M = 0.012  # finger pad half width across the finger axis
RELEASE_CLEARANCE_M = 0.005  # object bottom above the surface at release
ABOVE_M = 0.10  # approach / retreat height above the release pose
BELOW_BOX_M = 0.01  # occupied(): objects whose bottom is this far below the box bottom still count

Spot = tuple[float, float, float]  # x, y, surface z in the world frame


def yaw(pose: api.Pose) -> float:
  return math.atan2(2 * (pose.qw * pose.qz + pose.qx * pose.qy), 1 - 2 * (pose.qy ** 2 + pose.qz ** 2))


def half_extents(dims, angle: float) -> tuple[float, float]:
  """Half extents of the dims[0] x dims[1] rectangle rotated by `angle`."""
  c, s = abs(math.cos(angle)), abs(math.sin(angle))
  return c * dims[0] / 2 + s * dims[1] / 2, s * dims[0] / 2 + c * dims[1] / 2


def held_footprint(held_dims, held_yaw: float, finger_yaw: float, box_yaw: float) -> tuple[float, float]:
  """The rectangle covering the held object and the opened fingers (along the TCP's y axis) at release."""
  ox, oy = half_extents(held_dims, held_yaw - box_yaw)
  fx, fy = half_extents((2 * FINGER_HALF_WIDTH_M, 2 * FINGER_HALF_SPAN_M), finger_yaw - box_yaw)
  return max(ox, fx), max(oy, fy)


def _to_box(x: float, y: float, box_pose: api.Pose) -> tuple[float, float]:
  dx, dy, a = x - box_pose.x, y - box_pose.y, yaw(box_pose)
  return math.cos(a) * dx + math.sin(a) * dy, -math.sin(a) * dx + math.cos(a) * dy


def occupied(box_pose: api.Pose, box_dims, others) -> list[tuple[float, float, float, float]]:
  """(cx, cy, ex, ey) in the box frame of each (pose, dims) in `others` that stands in the box.

  The height test excludes the table and anything under the box.
  """
  taken = []
  for pose, dims in others:
    cx, cy = _to_box(pose.x, pose.y, box_pose)
    inside = abs(cx) <= box_dims[0] / 2 and abs(cy) <= box_dims[1] / 2
    if inside and box_pose.z - BELOW_BOX_M <= pose.z <= box_pose.z + box_dims[2]:
      taken.append((cx, cy, *half_extents(dims, yaw(pose) - yaw(box_pose))))
  return taken


def candidates(target_pose: api.Pose, target_dims, container: dict | None, footprint: tuple[float, float],
               taken) -> list[Spot]:
  """Release spots, best first: free grid points inside a box, filled from its -x/-y corner, or the top centre
  of any other target (`container` None)."""
  if container is None:
    return [(target_pose.x, target_pose.y, target_pose.z + target_dims[2])]
  fx, fy = footprint
  lx = target_dims[0] / 2 - container["wall"] - EDGE_MARGIN_M - fx
  ly = target_dims[1] / 2 - container["wall"] - EDGE_MARGIN_M - fy
  if lx < 0 or ly < 0:
    return []
  nx, ny = int(lx / GRID_M), int(ly / GRID_M)
  free = [(i * GRID_M, j * GRID_M) for i in range(-nx, nx + 1) for j in range(-ny, ny + 1)
          if not any(abs(i * GRID_M - cx) < fx + GAP_M + ex and abs(j * GRID_M - cy) < fy + GAP_M + ey
                     for cx, cy, ex, ey in taken)]
  a, z = yaw(target_pose), target_pose.z + container["floor"]
  c, s = math.cos(a), math.sin(a)
  return [(target_pose.x + c * x - s * y, target_pose.y + s * x + c * y, z) for x, y in free]


def release_pose(tcp: api.Pose, held_pose: api.Pose, spot: Spot) -> api.Pose:
  """`tcp` moved so the held object's bottom centre is RELEASE_CLEARANCE_M above the spot."""
  pose = api.Pose()
  pose.CopyFrom(tcp)
  pose.x, pose.y = spot[0] + (tcp.x - held_pose.x), spot[1] + (tcp.y - held_pose.y)
  pose.z = spot[2] + (tcp.z - held_pose.z) + RELEASE_CLEARANCE_M
  return pose


def above(pose: api.Pose) -> api.Pose:
  moved = api.Pose()
  moved.CopyFrom(pose)
  moved.z += ABOVE_M
  return moved
