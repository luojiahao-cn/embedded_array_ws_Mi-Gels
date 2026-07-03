import importlib.util
import sys
import threading
import types
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "motion_executor.py"


def load_motion_executor():
    package_src = str(SCRIPT.parents[1] / "src")
    if package_src not in sys.path:
        sys.path.insert(0, package_src)

    rospy = types.ModuleType("rospy")
    rospy.Duration = types.SimpleNamespace(from_sec=lambda seconds: seconds)
    rospy.Time = types.SimpleNamespace(now=lambda: 0)
    rospy.loginfo = lambda *_args, **_kwargs: None
    rospy.logerr = lambda *_args, **_kwargs: None
    rospy.logwarn = lambda *_args, **_kwargs: None
    rospy.Service = lambda *_args, **_kwargs: None
    rospy.ServiceProxy = lambda *_args, **_kwargs: None
    rospy.get_param = lambda _name, default=None: default
    rospy.init_node = lambda *_args, **_kwargs: None
    rospy.sleep = lambda *_args, **_kwargs: None

    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")

    class TriggerResponse:
        def __init__(self, success=False, message=""):
            self.success = success
            self.message = message

    std_srvs_srv.Trigger = type("Trigger", (), {})
    std_srvs_srv.TriggerResponse = TriggerResponse

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Pose = type("Pose", (), {})
    geometry_msgs_msg.PoseArray = type("PoseArray", (), {})

    moveit_msgs = types.ModuleType("moveit_msgs")
    moveit_msgs_msg = types.ModuleType("moveit_msgs.msg")
    moveit_msgs_msg.DisplayTrajectory = type("DisplayTrajectory", (), {})
    moveit_msgs_msg.MoveGroupAction = type("MoveGroupAction", (), {})
    moveit_msgs_msg.MoveItErrorCodes = types.SimpleNamespace(SUCCESS=1)
    moveit_msgs_srv = types.ModuleType("moveit_msgs.srv")
    moveit_msgs_srv.GetPositionIK = type("GetPositionIK", (), {})
    moveit_msgs_srv.GetPositionIKRequest = type("GetPositionIKRequest", (), {})

    for name, module in {
        "actionlib": types.ModuleType("actionlib"),
        "moveit_commander": types.ModuleType("moveit_commander"),
        "rospy": rospy,
        "tf2_ros": types.ModuleType("tf2_ros"),
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "moveit_msgs": moveit_msgs,
        "moveit_msgs.msg": moveit_msgs_msg,
        "moveit_msgs.srv": moveit_msgs_srv,
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
    }.items():
        sys.modules[name] = module

    spec = importlib.util.spec_from_file_location("motion_executor", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MotionExecutorPhaseTest(unittest.TestCase):
    def make_executor(self, module):
        executor = module.MotionExecutor.__new__(module.MotionExecutor)
        executor.calls = []
        executor.config = {"start": {"mode": "absolute"}}
        executor.plan_only = False
        executor._run_lock = threading.Lock()
        executor.start_pose = lambda: {"position": {"x": 1.0, "y": 2.0, "z": 3.0}}
        executor.current_pose_dict = lambda: {"position": {"x": 4.0, "y": 5.0, "z": 6.0}}
        executor.move_to_start = lambda start: executor.calls.append(("move_to_start", start)) or "executed"
        executor.run_trajectory_from_current = lambda: executor.calls.append(("run_trajectory",)) or True
        executor._run_trajectory = lambda start: executor.calls.append(("run_trajectory",)) or True
        return executor

    def test_prepare_start_moves_to_configured_initial_pose_only(self):
        module = load_motion_executor()
        executor = self.make_executor(module)

        self.assertTrue(executor.prepare_start())

        self.assertEqual(executor.calls, [("move_to_start", {"position": {"x": 1.0, "y": 2.0, "z": 3.0}})])

    def test_run_trajectory_service_does_not_move_to_start_again(self):
        module = load_motion_executor()
        executor = self.make_executor(module)

        response = executor.handle_run_trajectory(None)

        self.assertTrue(response.success)
        self.assertEqual(executor.calls, [("run_trajectory",)])

    def test_legacy_run_experiment_prepares_then_runs_trajectory(self):
        module = load_motion_executor()
        executor = self.make_executor(module)

        self.assertTrue(executor.run())

        self.assertEqual(
            executor.calls,
            [
                ("move_to_start", {"position": {"x": 1.0, "y": 2.0, "z": 3.0}}),
                ("run_trajectory",),
            ],
        )


if __name__ == "__main__":
    unittest.main()
