"""When Grasp and Release complete, and whether the gripper holds something (phase_3.md decision 22).

Times are simulation seconds (/clock); widths are metres.
"""

import collections

STALL_WIDTH_M = 1e-4  # Grasp: the width changed less than this ...
STALL_WINDOW_S = 0.2  # ... over this much sim time
TIMEOUT_S = 3.0  # Grasp and Release complete (OK) after this much sim time at the latest
GRIPPING_MIN_WIDTH_M = 0.005  # an empty close measures about 0.004 m
RELEASED_WIDTH_M = 0.075  # ICON's default_release_width (container/config/icon.txtpb)


def gripping(last_command_grasp: bool, width_m: float) -> bool:
  return last_command_grasp and width_m > GRIPPING_MIN_WIDTH_M


class StallDetector:
  """Grasp completion: fed (sim time, width) samples, true once the fingers stopped or time ran out."""

  def __init__(self, start_s: float):
    self.start_s = start_s
    self.samples: collections.deque[tuple[float, float]] = collections.deque()

  def update(self, t_s: float, width_m: float) -> bool:
    if t_s - self.start_s >= TIMEOUT_S:
      return True
    self.samples.append((t_s, width_m))
    # Keep the newest sample at or before the window start, so the samples span the whole window.
    while len(self.samples) >= 2 and self.samples[1][0] <= t_s - STALL_WINDOW_S:
      self.samples.popleft()
    if self.samples[0][0] > t_s - STALL_WINDOW_S:
      return False
    widths = [w for _, w in self.samples]
    return max(widths) - min(widths) < STALL_WIDTH_M


def release_complete(start_s: float, t_s: float, width_m: float) -> bool:
  return width_m >= RELEASED_WIDTH_M or t_s - start_s >= TIMEOUT_S
