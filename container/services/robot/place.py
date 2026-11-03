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
# In a box the hand may have to stay above the walls: release from a little higher (a short drop) if needed.
BOX_RELEASE_CLEARANCES_M = (0.005, 0.02, 0.035)
ABOVE_M = 0.10  # approach / retreat height above the release pose
SQUARE_TOLERANCE_M = 0.001  # footprint sides this close count as square (finger_turns)
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


def covers(held_half: tuple[float, float], target_dims, container: dict) -> bool:
  """Whether a held object (half extents in the box frame) is wider than the box opening on both axes: a lid."""
  return (held_half[0] >= target_dims[0] / 2 - container["wall"]
          and held_half[1] >= target_dims[1] / 2 - container["wall"])


def rim_spots(target_pose: api.Pose, target_dims, container: dict, held_half: tuple[float, float],
              taken) -> list[Spot]:
  """The box's top centre, for a held object that the open fingers cannot lower into the box (candidates() is
  empty): one that covers the opening (a lid) rests on the walls; one that fits the opening is released just
  above the walls and drops in, if the box is empty."""
  # shortcut: a lid ignores what is in the box (assumes the contents are lower than the walls); check heights
  # when tall contents become possible.
  if covers(held_half, target_dims, container):
    return [(target_pose.x, target_pose.y, target_pose.z + target_dims[2])]
  fits = all(h <= d / 2 - container["wall"] - EDGE_MARGIN_M for h, d in zip(held_half, target_dims))
  return [(target_pose.x, target_pose.y, target_pose.z + target_dims[2])] if fits and not taken else []


def release_pose(tcp: api.Pose, held_pose: api.Pose, spot: Spot, clearance: float = RELEASE_CLEARANCE_M) -> api.Pose:
  """`tcp` moved so the held object's bottom centre is `clearance` above the spot."""
  pose = api.Pose()
  pose.CopyFrom(tcp)
  pose.x, pose.y = spot[0] + (tcp.x - held_pose.x), spot[1] + (tcp.y - held_pose.y)
  pose.z = spot[2] + (tcp.z - held_pose.z) + clearance
  return pose


def finger_turns(held_dims, held_yaw: float, finger_yaw: float, box_yaw: float) -> list[float]:
  """Wrist turns (rad, about the vertical) to try at release. An object with a square footprint looks the same
  turned by 90 deg, so the fingers are aligned with the box's x axis, else its y axis, always in that order:
  placements then share one finger direction and pack the box (mixed directions fragment it: 3-4 cubes instead
  of 6). Other objects keep their orientation."""
  if abs(float(held_dims[0]) - float(held_dims[1])) > SQUARE_TOLERANCE_M:
    return [0.0]
  turns = []
  for axis in (box_yaw, box_yaw + math.pi / 2):
    turn = math.remainder(axis - finger_yaw, math.pi)  # the gripper is symmetric under a half turn
    turns.append(turn)
  return turns


def turned(tcp: api.Pose, held_pose: api.Pose, turn: float) -> tuple[api.Pose, api.Pose]:
  """`tcp` and the held object turned together by `turn` about the vertical through the object's centre."""
  c, s = math.cos(turn), math.sin(turn)
  dx, dy = tcp.x - held_pose.x, tcp.y - held_pose.y
  rz = (0.0, 0.0, math.sin(turn / 2), math.cos(turn / 2))  # (x, y, z, w)

  def rotate(q: api.Pose, out: api.Pose) -> None:
    x, y, z, w = q.qx, q.qy, q.qz, q.qw
    ax, ay, az, aw = rz
    out.qw = aw * w - ax * x - ay * y - az * z
    out.qx = aw * x + ax * w + ay * z - az * y
    out.qy = aw * y - ax * z + ay * w + az * x
    out.qz = aw * z + ax * y - ay * x + az * w

  new_tcp, new_held = api.Pose(), api.Pose()
  new_tcp.CopyFrom(tcp)
  new_held.CopyFrom(held_pose)
  new_tcp.x, new_tcp.y = held_pose.x + c * dx - s * dy, held_pose.y + s * dx + c * dy
  rotate(tcp, new_tcp)
  rotate(held_pose, new_held)
  return new_tcp, new_held


def above(pose: api.Pose) -> api.Pose:
  moved = api.Pose()
  moved.CopyFrom(pose)
  moved.z += ABOVE_M
  return moved
