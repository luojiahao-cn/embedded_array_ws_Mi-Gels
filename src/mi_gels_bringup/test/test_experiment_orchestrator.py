import importlib.util
import sys
import types
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "experiment_orchestrator.py"


class FakeROSException(Exception):
    pass


class FakeRospy(types.ModuleType):
    ROSException = FakeROSException

    def __init__(self):
        super().__init__("rospy")
        self.messages = []

    def wait_for_message(self, *_args, **_kwargs):
        if not self.messages:
            raise FakeROSException("timeout exceeded while waiting for message on topic /fy8300/ch1/status")
        item = self.messages.pop(0)
        if isinstance(item, Exception):
            raise item
        return item

    def is_shutdown(self):
        return False

    def loginfo(self, *_args, **_kwargs):
        pass

    def logwarn(self, *_args, **_kwargs):
        pass

    def logdebug(self, *_args, **_kwargs):
        pass


class FakeStatus:
    valid = False
    waveform = 2
    frequency = 1.0
    amplitude = 5.0
    offset = 2.5
    phase = 0.0
    duty_cycle = 25.0
    output_enabled = True


def load_orchestrator(rospy_module):
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.CameraInfo = type("CameraInfo", (), {})
    sensor_msgs_msg.Image = type("Image", (), {})
    signal_generator = types.ModuleType("signal_generator")
    signal_generator_msg = types.ModuleType("signal_generator.msg")
    signal_generator_msg.ChannelStatus = type("ChannelStatus", (), {})
    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = type("Trigger", (), {})

    sys.modules["rospy"] = rospy_module
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg
    sys.modules["signal_generator"] = signal_generator
    sys.modules["signal_generator.msg"] = signal_generator_msg
    sys.modules["std_srvs"] = std_srvs
    sys.modules["std_srvs.srv"] = std_srvs_srv

    spec = importlib.util.spec_from_file_location("experiment_orchestrator", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ExperimentOrchestratorTest(unittest.TestCase):
    def test_signal_generator_ros_timeout_reports_last_status_mismatch(self):
        rospy = FakeRospy()
        rospy.messages = [FakeStatus()]
        module = load_orchestrator(rospy)

        orchestrator = module.ExperimentOrchestrator.__new__(module.ExperimentOrchestrator)
        orchestrator.signal_status_prefix = "/fy8300"
        orchestrator.signal_expected = {
            1: {
                "waveform": 2,
                "frequency": 1.0,
                "amplitude": 5.0,
                "offset": 2.5,
                "phase": 0.0,
                "duty_cycle": 25.0,
                "output_enabled": True,
            }
        }
        orchestrator.tolerances = {
            "frequency": 0.01,
            "amplitude": 0.05,
            "offset": 0.05,
            "phase": 0.5,
            "duty_cycle": 0.5,
        }
        orchestrator._remaining_timeout = lambda _deadline: 0.0

        with self.assertRaisesRegex(RuntimeError, "status.valid is false"):
            orchestrator.wait_for_signal_generator(0.0)

    def test_signal_generator_deadline_timeout_reports_last_status_mismatch(self):
        rospy = FakeRospy()
        rospy.messages = [FakeStatus()]
        module = load_orchestrator(rospy)

        orchestrator = module.ExperimentOrchestrator.__new__(module.ExperimentOrchestrator)
        orchestrator.signal_status_prefix = "/fy8300"
        orchestrator.signal_expected = {
            1: {
                "waveform": 2,
                "frequency": 1.0,
                "amplitude": 5.0,
                "offset": 2.5,
                "phase": 0.0,
                "duty_cycle": 25.0,
                "output_enabled": True,
            }
        }
        orchestrator.tolerances = {
            "frequency": 0.01,
            "amplitude": 0.05,
            "offset": 0.05,
            "phase": 0.5,
            "duty_cycle": 0.5,
        }

        def timeout_after_first_status(_deadline):
            if not rospy.messages:
                raise RuntimeError("Timed out waiting for experiment prerequisites")
            return 0.1

        orchestrator._remaining_timeout = timeout_after_first_status

        with self.assertRaisesRegex(RuntimeError, "status.valid is false"):
            orchestrator.wait_for_signal_generator(0.0)


if __name__ == "__main__":
    unittest.main()
