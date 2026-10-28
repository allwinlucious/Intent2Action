"""Helpers of container/services/robot_service.py (phase_3.md decision 18)."""

import grpc

from robot.ops import ApiError


def unknown_object(object_id: str) -> ApiError:
  """phase_ids.md Addendum A.6: the reply to an object argument that is not an id from list_objects."""
  return ApiError(grpc.StatusCode.NOT_FOUND, f"no object with id {object_id}; list_objects shows the current ids")
