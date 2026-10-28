"""Perception-style object IDs for scene_sync (phase_ids.md decision 4, Addendum A.1).

IDs are obj_<k>, k from 10-99, unique within a map. A new map (after a scene reset) shares no k with the
previous one, so no object keeps its ID across a reset. Pure Python: no ROS, no gRPC.
"""

import random
import re

K_RANGE = range(10, 100)
ID_PATTERN = re.compile(r"obj_\d{2}")


def _k(object_id: str) -> int:
  return int(object_id.removeprefix("obj_"))


class IdAllocator:
  """The current map Isaac prim name -> ID."""

  def __init__(self, rng: random.Random | None = None):
    self._rng = rng or random.SystemRandom()
    self.ids: dict[str, str] = {}
    self._previous: set[int] = set()  # k of the map before the last draw()

  def draw(self, prims) -> dict[str, str]:
    """Replaces the map with fresh IDs for `prims`, none of them a k of the current map."""
    excluded = {_k(i) for i in self.ids.values()}
    ks = self._rng.sample([k for k in K_RANGE if k not in excluded], len(prims))
    self._previous = excluded
    self.ids = {prim: f"obj_{k}" for prim, k in zip(prims, ks)}
    return dict(self.ids)

  def adopt(self, ids: dict[str, str]) -> None:
    """Keeps existing IDs (start-up with persisted worlds, Addendum A.5)."""
    self.ids.update(ids)

  def add(self, prim: str) -> str:
    """A fresh ID for one more prim (Spawn, or a prim without an adopted ID)."""
    used = self._previous | {_k(i) for i in self.ids.values()}
    self.ids[prim] = f"obj_{self._rng.choice([k for k in K_RANGE if k not in used])}"
    return self.ids[prim]

  def forget(self, prim: str) -> None:
    self.ids.pop(prim, None)

  def prim(self, object_id: str) -> str | None:
    return next((p for p, i in self.ids.items() if i == object_id), None)
