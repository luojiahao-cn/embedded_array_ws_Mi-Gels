"""Start-pose policy helpers for MI-GELS motion execution."""


def requires_move_to_start(start_config):
    """Return true when the configured start pose must be reached before the path."""
    mode = str((start_config or {}).get("mode", "current")).strip().lower()
    return mode == "absolute"


def active_joint_target_from_solution(solution, active_joints):
    """Extract a MoveIt joint target dict for the group's active joints."""
    names = list(solution.joint_state.name)
    positions = list(solution.joint_state.position)
    by_name = dict(zip(names, positions))
    return {joint: by_name[joint] for joint in active_joints if joint in by_name}
