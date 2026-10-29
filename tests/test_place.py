"""Unit tests of the place planner (robot/place.py, phase_v1.md decision 5 and Addendum A.3); no sim needed."""

import math
from pathlib import Path
import sys

import pytest
from robot_service import robot_service_pb2 as api

# Appended: container/services/robot_service.py must not shadow the generated package robot_service.
sys.path.append(str(Path(__file__).resolve().parents[1] / "container/services"))
from robot import place  # noqa: E402

BOX = (0.25, 0.21, 0.08)
CONTAINER = {"wall": 0.01, "floor": 0.01}
CUBE = (0.05, 0.05, 0.05)
CUBE_HALF = 0.025


def _pose(x=0.0, y=0.0, z=0.0, yaw=0.0) -> api.Pose:
  return api.Pose(x=x, y=y, z=z, qz=math.sin(yaw / 2), qw=math.cos(yaw / 2))


def _to_box(spot, box: api.Pose) -> tuple[float, float]:
  dx, dy, a = spot[0] - box.x, spot[1] - box.y, place.yaw(box)
  return math.cos(a) * dx + math.sin(a) * dy, -math.sin(a) * dx + math.cos(a) * dy


def _limits(footprint) -> tuple[float, float]:
  fx, fy = footprint
  return (BOX[0] / 2 - CONTAINER["wall"] - place.EDGE_MARGIN_M - fx,
          BOX[1] / 2 - CONTAINER["wall"] - place.EDGE_MARGIN_M - fy)


def test_empty_box():
  box = _pose(0.5, 0.0, 0.0)
  footprint = place.held_footprint(CUBE, 0.0, 0.0, 0.0)
  spots = place.candidates(box, BOX, CONTAINER, footprint, [])
  lx, ly = _limits(footprint)
  assert (lx, ly) == pytest.approx((0.085, 0.0425))  # footprint (0.025, 0.0475): the cube and the open fingers
  assert _to_box(spots[0], box) == pytest.approx((-0.08, -0.04))
  for spot in spots:
    x, y = _to_box(spot, box)
    assert abs(x) <= lx + 1e-9 and abs(y) <= ly + 1e-9
    assert spot[2] == pytest.approx(0.01)


def test_capacity():
  box = _pose(0.5, 0.0, 0.0)
  footprint = place.held_footprint(CUBE, 0.0, 0.0, 0.0)
  taken, chosen = [], []
  while spots := place.candidates(box, BOX, CONTAINER, footprint, taken):
    x, y = _to_box(spots[0], box)
    chosen.append((x, y))
    taken.append((x, y, CUBE_HALF, CUBE_HALF))
    assert len(chosen) < 100
  assert len(chosen) >= 6
  for i, a in enumerate(chosen):
    for b in chosen[i + 1:]:
      assert abs(a[0] - b[0]) >= 2 * CUBE_HALF or abs(a[1] - b[1]) >= 2 * CUBE_HALF


def test_too_small():
  footprint = place.held_footprint((0.3, 0.3, 0.05), 0.0, 0.0, 0.0)
  assert place.candidates(_pose(0.5), BOX, CONTAINER, footprint, []) == []


def test_box_at_yaw_90():
  box = _pose(0.5, 0.1, 0.0, math.pi / 2)
  footprint = place.held_footprint(CUBE, 0.0, 0.0, math.pi / 2)
  spots = place.candidates(box, BOX, CONTAINER, footprint, [])
  lx, ly = _limits(footprint)
  assert spots
  for spot in spots:
    x, y = _to_box(spot, box)
    assert abs(x) <= lx + 1e-9 and abs(y) <= ly + 1e-9


def test_not_a_box():
  spots = place.candidates(_pose(0.4, 0.1, 0.0), CUBE, None, place.held_footprint(CUBE, 0.0, 0.0, 0.0), [])
  assert spots == [pytest.approx((0.4, 0.1, 0.05))]


def test_release_pose():
  tcp = api.Pose(x=0.5, y=0.0, z=0.2, qx=1.0, qw=0.0)
  release = place.release_pose(tcp, api.Pose(x=0.51, y=0.0, z=0.17, qw=1.0), (0.3, 0.1, 0.01))
  assert (release.x, release.y, release.z) == pytest.approx((0.29, 0.1, 0.045))
  assert (release.qx, release.qy, release.qz, release.qw) == (1.0, 0.0, 0.0, 0.0)
  assert place.above(release).z == pytest.approx(0.045 + place.ABOVE_M)


def test_occupied():
  box = _pose(0.5, 0.0, 0.0)
  inside = (_pose(0.55, 0.03, 0.01, 0.3), CUBE)
  outside = (_pose(0.8, 0.0, 0.0), CUBE)
  table = (_pose(0.5, 0.0, -0.75), (1.2, 0.8, 0.75))
  found = place.occupied(box, BOX, [inside, outside, table])
  assert len(found) == 1
  cx, cy, ex, ey = found[0]
  assert (cx, cy) == pytest.approx((0.05, 0.03))
  assert ex == pytest.approx(ey) and ex > CUBE_HALF


def test_held_footprint():
  assert place.held_footprint(CUBE, 0.0, 0.0, 0.0)[1] >= place.FINGER_HALF_SPAN_M
  assert place.held_footprint(CUBE, 0.0, math.pi / 2, 0.0)[0] >= place.FINGER_HALF_SPAN_M
