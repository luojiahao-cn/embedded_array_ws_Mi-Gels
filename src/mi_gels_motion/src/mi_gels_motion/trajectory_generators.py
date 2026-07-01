"""Pure geometry trajectory generators for end-effector experiments."""

import copy
import math


AXES = ("x", "y", "z")


def generate_waypoints(trajectory_config, start_pose):
    """Generate pose dictionaries from a trajectory config and start pose."""
    config = dict(trajectory_config or {})
    traj_type = str(config.get("type", "line")).strip().lower()
    if traj_type == "line":
        return _line(config, start_pose)
    if traj_type == "s_curve":
        return _s_curve(config, start_pose)
    if traj_type == "figure8":
        return _figure8(config, start_pose)
    if traj_type == "spiral":
        return _spiral(config, start_pose)
    raise ValueError("Unsupported trajectory type: {}".format(traj_type))


def path_length(waypoints, start_pose=None):
    """Return Cartesian path length through waypoint positions."""
    if not waypoints:
        return 0.0
    total = 0.0
    prev = _position(start_pose or waypoints[0])
    for waypoint in waypoints:
        cur = _position(waypoint)
        total += math.sqrt(sum((cur[axis] - prev[axis]) ** 2 for axis in AXES))
        prev = cur
    return total


def _line(config, start_pose):
    points = _points(config)
    vector = config.get("vector")
    if vector is None:
        axis = _axis(config.get("axis", "x"), "axis")
        distance = float(config.get("distance_m", 0.0))
        vector = {axis: distance}
    dx = _vector(vector)
    return [_offset_pose(start_pose, _scale(dx, i / float(points))) for i in range(1, points + 1)]


def _s_curve(config, start_pose):
    points = _points(config)
    axis = _axis(config.get("axis", "x"), "axis")
    lateral_axis = _axis(config.get("lateral_axis", "y"), "lateral_axis")
    if lateral_axis == axis:
        raise ValueError("lateral_axis must differ from axis")
    length = float(config.get("length_m", config.get("distance_m", 0.0)))
    amplitude = float(config.get("amplitude_m", 0.0))
    cycles = float(config.get("cycles", 1.0))
    waypoints = []
    for i in range(1, points + 1):
        t = i / float(points)
        offset = {axis: length * t, lateral_axis: amplitude * math.sin(2.0 * math.pi * cycles * t)}
        waypoints.append(_offset_pose(start_pose, offset))
    return waypoints


def _figure8(config, start_pose):
    points = _points(config)
    axis_u = _axis(config.get("axis_u", "x"), "axis_u")
    axis_v = _axis(config.get("axis_v", "y"), "axis_v")
    if axis_u == axis_v:
        raise ValueError("axis_u must differ from axis_v")
    radius = float(config.get("radius_m", 0.0))
    waypoints = []
    for i in range(1, points + 1):
        theta = 2.0 * math.pi * i / float(points)
        offset = {
            axis_u: radius * math.sin(theta),
            axis_v: 0.5 * radius * math.sin(2.0 * theta),
        }
        waypoints.append(_offset_pose(start_pose, offset))
    return waypoints


def _spiral(config, start_pose):
    points = _points(config)
    axis_u = _axis(config.get("axis_u", "x"), "axis_u")
    axis_v = _axis(config.get("axis_v", "y"), "axis_v")
    if axis_u == axis_v:
        raise ValueError("axis_u must differ from axis_v")
    radius = float(config.get("radius_m", 0.0))
    turns = float(config.get("turns", 1.0))
    waypoints = []
    for i in range(1, points + 1):
        t = i / float(points)
        theta = 2.0 * math.pi * turns * t
        r = radius * t
        offset = {axis_u: r * math.cos(theta), axis_v: r * math.sin(theta)}
        waypoints.append(_offset_pose(start_pose, offset))
    return waypoints


def _points(config):
    points = int(config.get("points", config.get("steps", 1)))
    if points <= 0:
        raise ValueError("points must be positive")
    return points


def _axis(value, name):
    axis = str(value).strip().lower()
    if axis not in AXES:
        raise ValueError("{} must be one of {}".format(name, ", ".join(AXES)))
    return axis


def _position(pose):
    return {axis: float(pose.get("position", {}).get(axis, 0.0)) for axis in AXES}


def _orientation(pose):
    ori = pose.get("orientation", {})
    return {
        "x": float(ori.get("x", 0.0)),
        "y": float(ori.get("y", 0.0)),
        "z": float(ori.get("z", 0.0)),
        "w": float(ori.get("w", 1.0)),
    }


def _vector(value):
    return {axis: float((value or {}).get(axis, 0.0)) for axis in AXES}


def _scale(vector, factor):
    return {axis: float(vector.get(axis, 0.0)) * factor for axis in AXES}


def _offset_pose(start_pose, offset):
    pose = {
        "position": copy.deepcopy(_position(start_pose)),
        "orientation": copy.deepcopy(_orientation(start_pose)),
    }
    for axis in AXES:
        pose["position"][axis] += float(offset.get(axis, 0.0))
    return pose

