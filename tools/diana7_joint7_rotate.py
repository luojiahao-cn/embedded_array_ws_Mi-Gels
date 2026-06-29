#!/usr/bin/env python3
"""Rotate only Diana7 joint 7 by a relative angle using MoveIt."""

import argparse
import math
import signal
import sys

ACTIVE_GROUP = None


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Rotate Diana7 joint 7 by a relative angle.")
    parser.add_argument("--group", default="diana7", help="MoveIt group name. Default: diana7")
    parser.add_argument("--joint-index", type=int, default=7, help="1-based joint index. Default: 7")
    parser.add_argument("--angle-deg", type=float, default=None, help="Relative rotation angle in degrees.")
    parser.add_argument("--move-to-deg", type=float, default=None, help="Absolute joint target in degrees.")
    parser.add_argument(
        "--sweep-to-limit",
        action="store_true",
        help="Move joint 7 toward the opposite safe limit instead of applying a fixed relative angle.",
    )
    parser.add_argument(
        "--direction",
        choices=("auto", "positive", "negative"),
        default="auto",
        help="Rotation direction. auto chooses the side with enough joint-limit margin.",
    )
    parser.add_argument("--fallback-lower-deg", type=float, default=-178.0, help="Fallback lower joint limit.")
    parser.add_argument("--fallback-upper-deg", type=float, default=178.0, help="Fallback upper joint limit.")
    parser.add_argument("--limit-margin-deg", type=float, default=5.0, help="Safety margin from joint limits.")
    parser.add_argument("--speed-scaling", type=float, default=0.05, help="MoveIt velocity scaling factor.")
    parser.add_argument("--acc-scaling", type=float, default=0.05, help="MoveIt acceleration scaling factor.")
    parser.add_argument(
        "--accept-final-error-deg",
        type=float,
        default=6.0,
        help="Treat an execution abort as acceptable if the final joint error is within this many degrees.",
    )
    parser.add_argument(
        "--max-segment-deg",
        type=float,
        default=90.0,
        help="Split large single-joint moves into segments no larger than this many degrees.",
    )
    parser.add_argument("--connect-wait-s", type=float, default=2.0, help="Delay before connecting to move_group.")
    parser.add_argument("--dry-run", action="store_true", help="Print target joints without moving.")
    parser.add_argument("--print-current-deg", action="store_true", help="Print current joint angle in degrees and exit.")
    return parser


def main(argv=None):
    args = build_arg_parser().parse_args(argv)

    import rospy
    import moveit_commander

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("diana7_joint7_rotate", anonymous=True)
    rospy.sleep(max(0.0, args.connect_wait_s))

    group = moveit_commander.MoveGroupCommander(args.group)
    global ACTIVE_GROUP
    ACTIVE_GROUP = group
    signal.signal(signal.SIGINT, _stop_active_group)
    signal.signal(signal.SIGTERM, _stop_active_group)
    group.set_max_velocity_scaling_factor(float(args.speed_scaling))
    group.set_max_acceleration_scaling_factor(float(args.acc_scaling))

    joint_names = group.get_active_joints()
    joints = list(group.get_current_joint_values())
    latest_joints = _latest_joint_positions(joint_names)
    if latest_joints is not None:
        joints = latest_joints
    joint_idx = int(args.joint_index) - 1
    if joint_idx < 0 or joint_idx >= len(joints):
        raise RuntimeError(
            "Group {} has {} active joints, cannot use joint index {}".format(
                args.group,
                len(joints),
                args.joint_index,
            )
        )

    if args.print_current_deg:
        print("CURRENT_JOINT7_DEG={:.9f}".format(math.degrees(joints[joint_idx])))
        return 0

    lower, upper = _joint_bounds(group, joint_names[joint_idx], joint_idx, args)
    magnitude = None if args.angle_deg is None else abs(math.radians(float(args.angle_deg)))
    margin = abs(math.radians(float(args.limit_margin_deg)))
    if args.move_to_deg is not None:
        target_value = math.radians(float(args.move_to_deg))
        safe_lower = lower + margin
        safe_upper = upper - margin
        if target_value < safe_lower or target_value > safe_upper:
            raise RuntimeError(
                "Absolute target {:.3f} deg is outside safe limits [{:.3f}, {:.3f}] deg".format(
                    float(args.move_to_deg),
                    math.degrees(safe_lower),
                    math.degrees(safe_upper),
                )
            )
        delta = target_value - joints[joint_idx]
    elif args.sweep_to_limit or magnitude is None:
        delta = _choose_sweep_delta(joints[joint_idx], lower, upper, margin, args.direction)
    else:
        delta = _choose_delta(joints[joint_idx], magnitude, lower, upper, margin, args.direction)
    target = list(joints)
    target[joint_idx] = target[joint_idx] + delta

    print("group={}".format(args.group))
    print("joint_names={}".format(",".join(joint_names)))
    print("joint_limit_rad=[{:.6f},{:.6f}], margin_rad={:.6f}".format(lower, upper, margin))
    print("moving {} by {:.3f} deg: {:.6f} -> {:.6f} rad".format(
        joint_names[joint_idx],
        math.degrees(delta),
        joints[joint_idx],
        target[joint_idx],
    ))

    if args.dry_run:
        print("dry_run=true")
        return 0

    _execute_segmented_move(
        group=group,
        joint_names=joint_names,
        joint_idx=joint_idx,
        start_joints=joints,
        target_joints=target,
        accept_final_error_deg=float(args.accept_final_error_deg),
        max_segment_deg=float(args.max_segment_deg),
        group_name=args.group,
        joint_index=args.joint_index,
    )
    print("success=true")
    return 0


def _stop_active_group(signum, _frame):
    if ACTIVE_GROUP is not None:
        try:
            ACTIVE_GROUP.stop()
            ACTIVE_GROUP.clear_pose_targets()
        except Exception:
            pass
    raise SystemExit(130 if signum == signal.SIGINT else 143)


def _joint_bounds(group, joint_name, joint_idx, args):
    try:
        bounds = group.get_joint_bounds(joint_name)
        if bounds and len(bounds) >= 2:
            return float(bounds[0]), float(bounds[1])
    except Exception:
        pass

    try:
        robot = group.get_robot_model()
        joint = robot.get_joint(joint_name)
        bounds = joint.bounds()
        if bounds and len(bounds) >= 2:
            return float(bounds[0]), float(bounds[1])
    except Exception:
        pass

    return math.radians(float(args.fallback_lower_deg)), math.radians(float(args.fallback_upper_deg))


def _latest_joint_position(joint_name, timeout_s=1.0):
    try:
        import rospy
        from sensor_msgs.msg import JointState

        msg = rospy.wait_for_message("/joint_states", JointState, timeout=timeout_s)
        if joint_name in msg.name:
            return float(msg.position[msg.name.index(joint_name)])
    except Exception:
        pass
    return None


def _latest_joint_positions(joint_names, timeout_s=1.0):
    try:
        import rospy
        from sensor_msgs.msg import JointState

        msg = rospy.wait_for_message("/joint_states", JointState, timeout=timeout_s)
        positions = []
        for joint_name in joint_names:
            if joint_name not in msg.name:
                return None
            positions.append(float(msg.position[msg.name.index(joint_name)]))
        return positions
    except Exception:
        return None


def _execute_segmented_move(
    group,
    joint_names,
    joint_idx,
    start_joints,
    target_joints,
    accept_final_error_deg,
    max_segment_deg,
    group_name,
    joint_index,
):
    start_value = float(start_joints[joint_idx])
    target_value = float(target_joints[joint_idx])
    total_delta = target_value - start_value
    max_segment = max(math.radians(1.0), abs(math.radians(max_segment_deg)))
    segment_count = max(1, int(math.ceil(abs(total_delta) / max_segment)))

    for segment_idx in range(1, segment_count + 1):
        fraction = float(segment_idx) / float(segment_count)
        segment_target_value = start_value + total_delta * fraction
        segment_target = list(target_joints)
        segment_target[joint_idx] = segment_target_value

        latest = _latest_joint_positions(joint_names)
        if latest is not None:
            for idx, value in enumerate(latest):
                if idx != joint_idx:
                    segment_target[idx] = value

        print(
            "segment={}/{} target_deg={:.3f}".format(
                segment_idx,
                segment_count,
                math.degrees(segment_target_value),
            )
        )
        group.set_start_state_to_current_state()
        group.set_joint_value_target(segment_target)
        success = group.go(wait=True)
        group.stop()
        if success:
            continue

        final_value = _latest_joint_position(joint_names[joint_idx])
        if final_value is None:
            final_value = list(group.get_current_joint_values())[joint_idx]
        final_error = segment_target_value - final_value
        final_error_deg = abs(math.degrees(final_error))
        if segment_idx == segment_count and final_error_deg <= accept_final_error_deg:
            print(
                "warning=moveit_abort_accepted final_error_deg={:.3f} threshold_deg={:.3f}".format(
                    final_error_deg,
                    accept_final_error_deg,
                )
            )
            return
        print(
            "error=moveit_abort segment={}/{} final_error_deg={:.3f} threshold_deg={:.3f}".format(
                segment_idx,
                segment_count,
                final_error_deg,
                accept_final_error_deg,
            ),
            file=sys.stderr,
        )
        raise RuntimeError("MoveIt execution failed for {} joint {}".format(group_name, joint_index))


def _choose_delta(current, magnitude, lower, upper, margin, direction):
    pos_ok = current + magnitude <= upper - margin
    neg_ok = current - magnitude >= lower + margin
    if direction == "positive":
        if not pos_ok:
            raise RuntimeError("Positive one-turn move would exceed joint limit")
        return magnitude
    if direction == "negative":
        if not neg_ok:
            raise RuntimeError("Negative one-turn move would exceed joint limit")
        return -magnitude
    if pos_ok and neg_ok:
        pos_margin = (upper - margin) - (current + magnitude)
        neg_margin = (current - magnitude) - (lower + margin)
        return magnitude if pos_margin >= neg_margin else -magnitude
    if pos_ok:
        return magnitude
    if neg_ok:
        return -magnitude
    raise RuntimeError(
        "Neither positive nor negative one-turn move fits joint limits: "
        "current={:.6f}, lower={:.6f}, upper={:.6f}, magnitude={:.6f}, margin={:.6f}".format(
            current,
            lower,
            upper,
            magnitude,
            margin,
        )
    )


def _choose_sweep_delta(current, lower, upper, margin, direction):
    safe_lower = lower + margin
    safe_upper = upper - margin
    if safe_lower >= safe_upper:
        raise RuntimeError("Joint limit margin leaves no valid sweep range")

    positive_delta = safe_upper - current
    negative_delta = safe_lower - current
    if direction == "positive":
        if positive_delta <= 0:
            raise RuntimeError("Current joint position is already at/above positive safe limit")
        return positive_delta
    if direction == "negative":
        if negative_delta >= 0:
            raise RuntimeError("Current joint position is already at/below negative safe limit")
        return negative_delta

    if abs(positive_delta) >= abs(negative_delta):
        return positive_delta
    return negative_delta


if __name__ == "__main__":
    raise SystemExit(main())
