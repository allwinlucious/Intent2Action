"""Unit tests of scene_sync's IdAllocator (container/services/scene_ids.py, phase_ids.md Addendum A.1)."""

import random
import re

from container.services.scene_ids import IdAllocator

PRIMS = ["table", "red_cube", "plate", "peg", "hole_block"]
ID = re.compile(r"obj_\d{2}")


def test_ids_are_unique_and_well_formed():
  ids = IdAllocator(random.Random(1)).draw(PRIMS)
  assert sorted(ids) == sorted(PRIMS)
  assert len(set(ids.values())) == len(PRIMS)
  assert all(ID.fullmatch(i) and int(i[4:]) >= 10 for i in ids.values())


def test_each_map_is_disjoint_from_the_previous_one():
  allocator = IdAllocator(random.Random(2))
  previous = allocator.draw(PRIMS)
  for _ in range(50):
    ids = allocator.draw(PRIMS)
    assert not set(ids.values()) & set(previous.values())
    assert len(set(ids.values())) == len(PRIMS)
    previous = ids


def test_add_and_adopt_keep_the_map_unique():
  allocator = IdAllocator(random.Random(3))
  allocator.adopt({"table": "obj_42"})
  ids = {p: allocator.add(p) for p in PRIMS[1:]}
  assert allocator.ids == {"table": "obj_42", **ids}
  assert len(set(allocator.ids.values())) == len(PRIMS)
  assert allocator.prim("obj_42") == "table" and allocator.prim("obj_00") is None
  allocator.forget("table")
  assert "table" not in allocator.ids and allocator.prim("obj_42") is None
