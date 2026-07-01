"""Compatibility helpers for MoveIt commander API differences."""

import inspect


def compute_cartesian_path_compat(group, waypoints, eef_step, jump_threshold, avoid_collisions):
    """Call compute_cartesian_path across MoveIt commander signature variants."""
    parameters = inspect.signature(group.compute_cartesian_path).parameters
    if "jump_threshold" in parameters:
        return group.compute_cartesian_path(waypoints, eef_step, jump_threshold, avoid_collisions)
    return group.compute_cartesian_path(waypoints, eef_step, avoid_collisions)
