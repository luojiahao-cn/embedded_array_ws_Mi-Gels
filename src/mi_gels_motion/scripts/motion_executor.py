#!/usr/bin/env python3
"""Execute configurable MI-GELS Cartesian experiment trajectories with MoveIt."""

import copy
import math
import sys

import actionlib
import moveit_commander
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.msg import DisplayTrajectory, MoveGroupAction, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

from mi_gels_motion.moveit_compat import compute_cartesian_path_compat
from mi_gels_motion.start_policy import active_joint_target_from_solution, requires_move_to_start
from mi_gels_motion.trajectory_generators import generate_waypoints, path_length


def _param_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _pose_from_dict(data):
    pose = Pose()
    position = data.get("position", {})
    orientation = data.get("orientation", {})
    pose.position.x = float(position.get("x", 0.0))
    pose.position.y = float(position.get("y", 0.0))
    pose.position.z = float(position.get("z", 0.0))
    pose.orientation.x = float(orientation.get("x", 0.0))
    pose.orientation.y = float(orientation.get("y", 0.0))
    pose.orientation.z = float(orientation.get("z", 0.0))
    pose.orientation.w = float(orientation.get("w", 1.0))
    return pose


def _pose_to_dict(pose):
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }


def _dict_to_pose_array(frame_id, waypoints):
    msg = PoseArray()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.poses = [_pose_from_dict(waypoint) for waypoint in waypoints]
    return msg


def _trajectory_duration(plan):
    points = getattr(plan.joint_trajectory, "points", [])
    if not points:
        return 0.0
    return points[-1].time_from_start.to_sec()


def _scale_duration(duration, factor):
    secs = duration.to_sec() * factor
    return rospy.Duration.from_sec(secs)


def _stretch_plan_duration(plan, target_duration_s):
    current = _trajectory_duration(plan)
    if current <= 0.0 or target_duration_s <= current:
        return False
    factor = target_duration_s / current
    for point in plan.joint_trajectory.points:
        point.time_from_start = _scale_duration(point.time_from_start, factor)
        point.velocities = [v / factor for v in point.velocities]
        point.accelerations = [a / (factor * factor) for a in point.accelerations]
    return True


class MotionExecutor:
    def __init__(self):
        rospy.init_node("mi_gels_motion_executor", anonymous=True)
        config_file = rospy.get_param("~config_file")
        self.config = self._load_config(config_file)
        self.plan_only = _param_bool(rospy.get_param("~plan_only", self.config.get("motion", {}).get("plan_only", True)))

        self.arm = str(self.config.get("arm", "diana7"))
        self.frame_id = str(self.config.get("frame_id", "world"))
        self.ee_link = str(self.config.get("ee_link", "diana7_ee_link"))
        self.move_group_name = str(self.config.get("move_group", self.arm))
        self.waypoint_topic = str(self.config.get("waypoint_topic", "/mi_gels_motion/waypoints"))
        self.tf_timeout_s = float(self.config.get("tf_timeout_s", 2.0))
        self.move_group_timeout_s = float(rospy.get_param("~move_group_timeout_s", self.config.get("move_group_timeout_s", 120.0)))
        self.ik_timeout_s = float(self.config.get("ik_timeout_s", 2.0))
        self.eef_step = float(self.config.get("motion", {}).get("eef_step_m", 0.005))
        self.jump_threshold = float(self.config.get("motion", {}).get("jump_threshold", 0.0))
        self.avoid_collisions = _param_bool(self.config.get("motion", {}).get("avoid_collisions", True))
        motion_config = self.config.get("motion", {})
        legacy_velocity_scaling = float(motion_config.get("velocity_scaling", 0.03))
        legacy_acceleration_scaling = float(motion_config.get("acceleration_scaling", 0.03))
        legacy_tcp_speed_mps = float(motion_config.get("tcp_speed_mps", 0.0))
        self.positioning_velocity_scaling = float(motion_config.get("positioning_velocity_scaling", legacy_velocity_scaling))
        self.positioning_acceleration_scaling = float(motion_config.get("positioning_acceleration_scaling", legacy_acceleration_scaling))
        self.trajectory_velocity_scaling = float(motion_config.get("trajectory_velocity_scaling", legacy_velocity_scaling))
        self.trajectory_acceleration_scaling = float(motion_config.get("trajectory_acceleration_scaling", legacy_acceleration_scaling))
        self.trajectory_tcp_speed_mps = float(motion_config.get("trajectory_tcp_speed_mps", legacy_tcp_speed_mps))

        self.waypoint_pub = rospy.Publisher(self.waypoint_topic, PoseArray, queue_size=1, latch=True)
        self.display_pub = rospy.Publisher(
            "/mi_gels_motion/display_trajectory",
            DisplayTrajectory,
            queue_size=1,
            latch=True,
        )

        moveit_commander.roscpp_initialize(sys.argv)
        self.wait_for_move_group()
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(self.move_group_name)
        self.group.set_end_effector_link(self.ee_link)
        self.group.set_pose_reference_frame(self.frame_id)
        self.apply_trajectory_speed()
        rospy.wait_for_service("/compute_ik", timeout=self.move_group_timeout_s)
        self.compute_ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _load_config(self, config_file):
        with open(config_file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        if not isinstance(data, dict):
            raise ValueError("Motion config must be a YAML mapping")
        return data

    def wait_for_move_group(self):
        timeout = rospy.Duration(self.move_group_timeout_s)
        rospy.loginfo("Waiting up to %.1fs for /move_group action server...", self.move_group_timeout_s)
        client = actionlib.SimpleActionClient("/move_group", MoveGroupAction)
        if not client.wait_for_server(timeout):
            raise RuntimeError("/move_group action server did not become ready within {:.1f}s".format(self.move_group_timeout_s))
        rospy.loginfo("/move_group action server is ready.")

    def apply_positioning_speed(self):
        self.group.set_max_velocity_scaling_factor(self.positioning_velocity_scaling)
        self.group.set_max_acceleration_scaling_factor(self.positioning_acceleration_scaling)

    def apply_trajectory_speed(self):
        self.group.set_max_velocity_scaling_factor(self.trajectory_velocity_scaling)
        self.group.set_max_acceleration_scaling_factor(self.trajectory_acceleration_scaling)

    def start_pose(self):
        start = self.config.get("start", {})
        mode = str(start.get("mode", "current")).strip().lower()
        if mode == "absolute":
            return copy.deepcopy(start.get("pose", {}))
        if mode == "current":
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id,
                    start.get("tf_frame", self.ee_link),
                    rospy.Time(0),
                    rospy.Duration(self.tf_timeout_s),
                )
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation
                return _pose_to_dict(pose)
            except Exception as exc:
                rospy.logwarn("TF start pose lookup failed, using MoveIt current pose: %s", exc)
                stamped = self.group.get_current_pose(self.ee_link)
                return _pose_to_dict(stamped.pose)
        raise ValueError("Unsupported start.mode: {}".format(mode))

    def plan(self, poses):
        plan, fraction = compute_cartesian_path_compat(
            self.group,
            poses,
            self.eef_step,
            self.jump_threshold,
            self.avoid_collisions,
        )
        plan = self.group.retime_trajectory(
            self.robot.get_current_state(),
            plan,
            velocity_scaling_factor=self.trajectory_velocity_scaling,
            acceleration_scaling_factor=self.trajectory_acceleration_scaling,
        )
        return plan, fraction

    def publish_display_trajectory(self, plan):
        display = DisplayTrajectory()
        display.trajectory_start = self.robot.get_current_state()
        display.trajectory.append(plan)
        self.display_pub.publish(display)

    def move_to_start(self, start):
        pose = _pose_from_dict(start)
        rospy.loginfo(
            "Planning move to absolute start pose: position=(%.4f, %.4f, %.4f)",
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = self.move_group_name
        ik_request.ik_request.ik_link_name = self.ee_link
        ik_request.ik_request.robot_state = self.robot.get_current_state()
        ik_request.ik_request.pose_stamped.header.frame_id = self.frame_id
        ik_request.ik_request.pose_stamped.header.stamp = rospy.Time.now()
        ik_request.ik_request.pose_stamped.pose = pose
        ik_request.ik_request.timeout = rospy.Duration(self.ik_timeout_s)
        ik_response = self.compute_ik(ik_request)
        if ik_response.error_code.val != MoveItErrorCodes.SUCCESS:
            rospy.logerr("Failed to solve IK for absolute start pose; refusing to run Cartesian path.")
            return False
        joint_target = active_joint_target_from_solution(ik_response.solution, self.group.get_active_joints())
        if not joint_target:
            rospy.logerr("IK solution did not include active joints for %s; refusing to run Cartesian path.", self.move_group_name)
            return False
        self.apply_positioning_speed()
        self.group.set_joint_value_target(joint_target)
        result = self.group.plan()
        if isinstance(result, tuple):
            success = bool(result[0])
            plan = result[1]
        else:
            plan = result
            success = bool(getattr(plan, "joint_trajectory", None) and plan.joint_trajectory.points)
        self.publish_display_trajectory(plan)
        self.group.clear_pose_targets()
        if not success:
            rospy.logerr("Failed to plan move to absolute start pose; refusing to run Cartesian path.")
            return False
        if self.plan_only:
            rospy.loginfo("Plan-only mode enabled; absolute start pose plan was generated but not executed.")
            return False
        self.group.execute(plan, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        rospy.loginfo("Reached absolute start pose.")
        self.apply_trajectory_speed()
        return True

    def current_pose_dict(self):
        rospy.sleep(0.5)
        stamped = self.group.get_current_pose(self.ee_link)
        return _pose_to_dict(stamped.pose)

    def run(self):
        start = self.start_pose()
        if requires_move_to_start(self.config.get("start", {})):
            if not self.move_to_start(start):
                return
            if not self.plan_only:
                start = self.current_pose_dict()
                rospy.loginfo(
                    "Using reached start pose for Cartesian path: position=(%.4f, %.4f, %.4f)",
                    start["position"]["x"],
                    start["position"]["y"],
                    start["position"]["z"],
                )

        waypoints = generate_waypoints(self.config.get("trajectory", {}), start)
        msg = _dict_to_pose_array(self.frame_id, waypoints)
        self.waypoint_pub.publish(msg)

        length_m = path_length(waypoints, start)
        rospy.loginfo(
            "Generated %d %s waypoints for %s, path_length=%.4fm, plan_only=%s",
            len(waypoints),
            self.config.get("trajectory", {}).get("type", "line"),
            self.arm,
            length_m,
            self.plan_only,
        )

        plan, fraction = self.plan(msg.poses)
        rospy.loginfo("Cartesian planning fraction: %.3f", fraction)
        self.publish_display_trajectory(plan)
        min_fraction = float(self.config.get("motion", {}).get("min_fraction", 0.95))
        if fraction < min_fraction:
            rospy.logerr("Planning fraction %.3f below min_fraction %.3f; refusing to execute.", fraction, min_fraction)
            return

        if self.trajectory_tcp_speed_mps > 0.0 and length_m > 0.0:
            desired_duration = length_m / self.trajectory_tcp_speed_mps
            if _stretch_plan_duration(plan, desired_duration):
                rospy.loginfo(
                    "Stretched plan to target duration %.3fs for trajectory_tcp_speed_mps=%.4f",
                    desired_duration,
                    self.trajectory_tcp_speed_mps,
                )
            else:
                rospy.loginfo("MoveIt-retimed plan is already slower than trajectory_tcp_speed_mps target; no stretch applied.")

        if self.plan_only:
            rospy.loginfo("Plan-only mode enabled; plan was generated but not executed.")
            return
        self.group.execute(plan, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        rospy.loginfo("Motion execution complete.")


if __name__ == "__main__":
    try:
        MotionExecutor().run()
    except rospy.ROSInterruptException:
        pass
