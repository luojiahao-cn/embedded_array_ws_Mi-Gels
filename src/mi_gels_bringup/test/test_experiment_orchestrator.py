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
        self.events = []
        self.service_responses = []

    def wait_for_message(self, *_args, **_kwargs):
        if _args:
            self.events.append(("wait_for_message", _args[0]))
        if not self.messages:
            raise FakeROSException("timeout exceeded while waiting for message on topic /fy8300/ch1/status")
        item = self.messages.pop(0)
        if isinstance(item, Exception):
            raise item
        return item

    def is_shutdown(self):
        return False

    def wait_for_service(self, name, **_kwargs):
        self.events.append(("wait_for_service", name))

    def ServiceProxy(self, name, _service_type):
        def call():
            self.events.append(("service", name))
            if self.service_responses:
                return self.service_responses.pop(0)
            return types.SimpleNamespace(success=True, message="ok")

        return call

    def Publisher(self, topic, _msg_type, queue_size=1, latch=False):
        class Publisher:
            def publish(inner_self, msg):
                self.events.append(("publish", topic, msg.data))

        return Publisher()

    def sleep(self, seconds):
        self.events.append(("sleep", round(seconds, 1)))

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


class ReadyStatus:
    valid = True
    waveform = 2
    frequency = 1.0
    amplitude = 5.0
    offset = 2.5
    duty_cycle = 25.0
    output_enabled = True

    def __init__(self, phase):
        self.phase = phase


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
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class Bool:
        def __init__(self, data=False):
            self.data = data

    class String:
        def __init__(self, data=""):
            self.data = data

    std_msgs_msg.Bool = Bool
    std_msgs_msg.String = String

    sys.modules["rospy"] = rospy_module
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg
    sys.modules["signal_generator"] = signal_generator
    sys.modules["signal_generator.msg"] = signal_generator_msg
    sys.modules["std_srvs"] = std_srvs
    sys.modules["std_srvs.srv"] = std_srvs_srv
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

    spec = importlib.util.spec_from_file_location("experiment_orchestrator", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ExperimentOrchestratorTest(unittest.TestCase):
    def make_orchestrator(self, module):
        orchestrator = module.ExperimentOrchestrator.__new__(module.ExperimentOrchestrator)
        orchestrator.prepare_start_service = "/mi_gels_motion/prepare_start"
        orchestrator.run_trajectory_service = "/mi_gels_motion/run_trajectory"
        orchestrator.record_trigger_topic = "/maggrad_continuous_collection/record_trigger"
        orchestrator.signal_status_prefix = "/fy8300"
        orchestrator.signal_expected = {
            1: {
                "waveform": 2,
                "frequency": 1.0,
                "amplitude": 5.0,
                "offset": 2.5,
                "phase": 0.0,
                "duty_cycle": 25.0,
                "output_enabled": False,
            },
            2: {
                "waveform": 2,
                "frequency": 1.0,
                "amplitude": 5.0,
                "offset": 2.5,
                "phase": 90.0,
                "duty_cycle": 25.0,
                "output_enabled": False,
            },
            3: {
                "waveform": 2,
                "frequency": 1.0,
                "amplitude": 5.0,
                "offset": 2.5,
                "phase": 180.0,
                "duty_cycle": 25.0,
                "output_enabled": False,
            },
        }
        orchestrator.tolerances = {
            "frequency": 0.01,
            "amplitude": 0.05,
            "offset": 0.05,
            "phase": 0.5,
            "duty_cycle": 0.5,
        }
        orchestrator.initial_settle_time_s = 2.0
        orchestrator.pre_motion_cycles = 5
        orchestrator.coil_frequency_hz = 1.0
        orchestrator.stage_prefix = "[MI-GELS]"
        orchestrator._remaining_timeout = lambda _deadline: 10.0
        orchestrator.record_trigger_pub = module.rospy.Publisher(
            orchestrator.record_trigger_topic,
            module.Bool,
            queue_size=1,
            latch=True,
        )
        orchestrator.signal_output_pubs = {
            channel: module.rospy.Publisher(
                "/fy8300/ch{}/output_en".format(channel),
                module.Bool,
                queue_size=1,
                latch=True,
            )
            for channel in orchestrator.signal_expected
        }
        return orchestrator

    def test_run_sequence_prepares_enables_signal_records_waits_runs_then_stops_everything(self):
        rospy = FakeRospy()
        rospy.messages = [ReadyStatus(0.0), ReadyStatus(90.0), ReadyStatus(180.0)]
        module = load_orchestrator(rospy)
        orchestrator = self.make_orchestrator(module)

        orchestrator.run_experiment_sequence(9999999999.0)

        self.assertEqual(
            rospy.events,
            [
                ("wait_for_service", "/mi_gels_motion/prepare_start"),
                ("service", "/mi_gels_motion/prepare_start"),
                ("publish", "/fy8300/ch1/output_en", True),
                ("publish", "/fy8300/ch2/output_en", True),
                ("publish", "/fy8300/ch3/output_en", True),
                ("wait_for_message", "/fy8300/ch1/status"),
                ("wait_for_message", "/fy8300/ch2/status"),
                ("wait_for_message", "/fy8300/ch3/status"),
                ("sleep", 2.0),
                ("publish", "/maggrad_continuous_collection/record_trigger", True),
                ("sleep", 5.0),
                ("wait_for_service", "/mi_gels_motion/run_trajectory"),
                ("service", "/mi_gels_motion/run_trajectory"),
                ("publish", "/maggrad_continuous_collection/record_trigger", False),
                ("publish", "/fy8300/ch1/output_en", False),
                ("publish", "/fy8300/ch2/output_en", False),
                ("publish", "/fy8300/ch3/output_en", False),
            ],
        )

    def test_run_sequence_stops_recording_and_signal_when_trajectory_fails(self):
        rospy = FakeRospy()
        rospy.messages = [ReadyStatus(0.0), ReadyStatus(90.0), ReadyStatus(180.0)]
        rospy.service_responses = [
            types.SimpleNamespace(success=True, message="prepared"),
            types.SimpleNamespace(success=False, message="motion failed"),
        ]
        module = load_orchestrator(rospy)
        orchestrator = self.make_orchestrator(module)

        with self.assertRaisesRegex(RuntimeError, "motion failed"):
            orchestrator.run_experiment_sequence(9999999999.0)

        self.assertEqual(
            rospy.events[-4:],
            [
                ("publish", "/maggrad_continuous_collection/record_trigger", False),
                ("publish", "/fy8300/ch1/output_en", False),
                ("publish", "/fy8300/ch2/output_en", False),
                ("publish", "/fy8300/ch3/output_en", False),
            ],
        )

    def test_run_sequence_does_not_record_when_signal_never_becomes_ready(self):
        rospy = FakeRospy()
        rospy.service_responses = [types.SimpleNamespace(success=True, message="prepared")]
        module = load_orchestrator(rospy)
        orchestrator = self.make_orchestrator(module)

        with self.assertRaisesRegex(RuntimeError, "Timed out waiting for FY8300"):
            orchestrator.run_experiment_sequence(0.0)

        self.assertNotIn(("publish", "/maggrad_continuous_collection/record_trigger", True), rospy.events)
        self.assertEqual(
            rospy.events[-3:],
            [
                ("publish", "/fy8300/ch1/output_en", False),
                ("publish", "/fy8300/ch2/output_en", False),
                ("publish", "/fy8300/ch3/output_en", False),
            ],
        )

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
