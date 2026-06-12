#!/usr/bin/env python3
import importlib.util
import tempfile
import sys
import types
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "serial_node_maggrad.py"


def install_ros_stubs():
    rospy = types.ModuleType("rospy")
    rospy.get_param = lambda *args, **kwargs: args[1] if len(args) > 1 else None
    rospy.init_node = lambda *args, **kwargs: None
    rospy.Publisher = lambda *args, **kwargs: None
    rospy.Subscriber = lambda *args, **kwargs: None
    rospy.on_shutdown = lambda *args, **kwargs: None
    rospy.loginfo = lambda *args, **kwargs: None
    rospy.logwarn = lambda *args, **kwargs: None
    rospy.logerr = lambda *args, **kwargs: None
    rospy.logdebug = lambda *args, **kwargs: None
    rospy.signal_shutdown = lambda *args, **kwargs: None
    rospy.sleep = lambda *args, **kwargs: None
    rospy.is_shutdown = lambda: True
    rospy.Time = types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: 0.0))
    sys.modules["rospy"] = rospy

    serial = types.ModuleType("serial")
    serial.SerialException = Exception
    serial.Serial = object
    tools = types.ModuleType("serial.tools")
    list_ports = types.ModuleType("serial.tools.list_ports")
    list_ports.comports = lambda: []
    tools.list_ports = list_ports
    serial.tools = tools
    sys.modules["serial"] = serial
    sys.modules["serial.tools"] = tools
    sys.modules["serial.tools.list_ports"] = list_ports

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = type("Bool", (), {})
    std_msgs_msg.Float32MultiArray = type("Float32MultiArray", (), {})
    std_msgs_msg.Header = type("Header", (), {})
    std_msgs_msg.String = type("String", (), {})
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Imu = type("Imu", (), {})
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    serial_processor = types.ModuleType("serial_processor")
    serial_processor_msg = types.ModuleType("serial_processor.msg")
    for name in ("MagGradImuRaw", "SensorData", "StmUplink"):
        serial_processor_msg.__dict__[name] = type(name, (), {})
    sys.modules["serial_processor"] = serial_processor
    sys.modules["serial_processor.msg"] = serial_processor_msg

    sensor_array_config = types.ModuleType("sensor_array_config")
    sensor_array_config.HardwareConfig = object
    sensor_array_config.get_hardware_config = lambda name: None
    sensor_array_config.get_runtime_config = lambda: {}
    sensor_array_config.resolve_runtime_value = lambda value, key, fallback: fallback
    sys.modules["sensor_array_config"] = sensor_array_config


def load_module():
    install_ros_stubs()
    spec = importlib.util.spec_from_file_location("serial_node_maggrad", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MagGradRuntimeProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = load_module()

    def test_parse_info_reply(self):
        info = self.mod.parse_kv_reply(
            "OK INFO project=MagGrad board=MagGrad_AK_TMAG_V1 "
            "protocol=maggrad_binary_v1 sensors=AK,TMAG,ICM"
        )
        self.assertEqual(info["project"], "MagGrad")
        self.assertEqual(info["board"], "MagGrad_AK_TMAG_V1")
        self.assertEqual(info["sensors"], "AK,TMAG,ICM")

    def test_expected_hardware_config_for_board(self):
        self.assertTrue(self.mod.board_supports_hardware_config("MagGrad_QMC_V1", "qmc6309"))
        self.assertTrue(self.mod.board_supports_hardware_config("MagGrad_AK_TMAG_V1", "ak09973d"))
        self.assertTrue(self.mod.board_supports_hardware_config("MagGrad_AK_TMAG_V1", "tmag3001"))
        self.assertFalse(self.mod.board_supports_hardware_config("MagGrad_QMC_V1", "ak09973d"))

    def test_frame_type_must_match_hardware_config(self):
        proto = self.mod.MagGradProtocol
        self.assertTrue(self.mod.frame_type_matches_hardware_config(proto.TYPE_AK_ARRAY, "ak09973d"))
        self.assertTrue(self.mod.frame_type_matches_hardware_config(proto.TYPE_TMAG_ARRAY, "tmag3001"))
        self.assertTrue(self.mod.frame_type_matches_hardware_config(proto.TYPE_QMC_ARRAY, "qmc6309"))
        self.assertFalse(self.mod.frame_type_matches_hardware_config(proto.TYPE_QMC_ARRAY, "ak09973d"))

    def test_startup_commands_include_profile_rate_before_mode(self):
        commands = self.mod.build_startup_commands(
            startup_strategy="cont",
            sensors="AK_ICM",
            rate_hz=200,
            profile="LOW_NOISE",
            icm_rate_hz=480,
        )
        self.assertEqual(commands, ["PROFILE LOW_NOISE", "RATE 200", "ICM_RATE 480", "MODE CONT AK_ICM 200"])

    def test_auto_sensors_follow_hardware_config(self):
        self.assertEqual(self.mod.default_sensors_for_hardware_config("ak09973d"), "AK_ICM")
        self.assertEqual(self.mod.default_sensors_for_hardware_config("tmag3001"), "TMAG_ICM")
        self.assertEqual(self.mod.default_sensors_for_hardware_config("qmc6309"), "QMC_ICM")

    def test_status_reply_reports_profile_warning_and_rates(self):
        status = self.mod.parse_kv_reply(
            "OK STATUS strategy=CONT sensors=AK_ICM target_hz=200 actual_hz=199 "
            "profile=STABLE profile_status=fallback fallback_from=LOW_NOISE "
            "warning=LOW_NOISE_INIT_FAILED errors=0"
        )
        self.assertEqual(status["strategy"], "CONT")
        self.assertEqual(status["profile"], "STABLE")
        self.assertEqual(status["warning"], "LOW_NOISE_INIT_FAILED")
        self.assertEqual(status["target_hz"], "200")

    def test_status_warning_detects_partial_ak_without_shutdown(self):
        status = self.mod.parse_kv_reply(
            "OK STATUS strategy=CONT sensors=AK_ICM target_hz=200 actual_hz=200 "
            "warning=AK_INIT_PARTIAL init=AK:1,TMAG:0,ICM:1 ak_count=11 ak_bitmap=0xEFF errors=0"
        )
        self.assertEqual(
            self.mod.status_warning_message(status),
            "Firmware warning=AK_INIT_PARTIAL ak_count=11 ak_bitmap=0xEFF",
        )

    def test_recording_metadata_includes_latest_status(self):
        node = object.__new__(self.mod.SerialNodeMagGrad)
        node.firmware_info = {"project": "MagGrad", "board": "MagGrad_AK_TMAG_V1", "protocol": "maggrad_binary_v1"}
        node.firmware_caps = {"profiles": "AUTO,LOW_NOISE,STABLE"}
        node.firmware_status = {
            "target_hz": "200",
            "actual_hz": "199",
            "profile": "LOW_NOISE",
            "warning": "AK_INIT_PARTIAL",
            "ak_bitmap": "0xEFF",
        }
        node.hardware_config_name = "ak09973d"
        node.startup_strategy = "cont"
        node.startup_sensors = "AUTO"
        node.trigger_rate_hz = 200
        node.icm_rate_hz = 480
        node.profile = "AUTO"

        metadata = self.mod.SerialNodeMagGrad._recording_metadata(node)

        self.assertEqual(metadata["status_warning"], "AK_INIT_PARTIAL")
        self.assertEqual(metadata["status_actual_hz"], "199")
        self.assertEqual(metadata["status_ak_bitmap"], "0xEFF")

    def test_noise_analysis_outputs_required_columns(self):
        spec = importlib.util.spec_from_file_location(
            "analyze_maggrad_noise",
            SCRIPT.parent / "analyze_maggrad_noise.py",
        )
        analyzer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(analyzer)
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "sample.csv"
            path.write_text(
                "# board,MagGrad_AK_TMAG_V1\n"
                "# hardware_config,ak09973d\n"
                "# status_target_hz,200\n"
                "# status_actual_hz,200\n"
                "# status_profile,LOW_NOISE\n"
                "# status_warning,NONE\n"
                "pc_time,seq,tick_ms,bitmap,sensor_1_x_gs,sensor_1_y_gs,sensor_1_z_gs\n"
                "0.00,1,0,0x0001,1.0,2.0,3.0\n"
                "0.01,2,5,0x0001,2.0,4.0,6.0\n"
                "0.02,4,10,0x0000,,,\n"
            )

            rows = analyzer.analyze_file(path)

        self.assertEqual(rows[0]["sensor_type"], "ak09973d")
        self.assertEqual(rows[0]["sensor_id/group"], "1")
        self.assertEqual(rows[0]["target_hz"], "200")
        self.assertEqual(rows[0]["warning"], "NONE")
        self.assertIn("std_x", rows[0])
        self.assertEqual(rows[0]["drop_rate"], "0.000000")

    def test_noise_analysis_falls_back_to_seq_drop_rate(self):
        spec = importlib.util.spec_from_file_location(
            "analyze_maggrad_noise",
            SCRIPT.parent / "analyze_maggrad_noise.py",
        )
        analyzer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(analyzer)
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "sample.csv"
            path.write_text(
                "# board,MagGrad_AK_TMAG_V1\n"
                "# hardware_config,ak09973d\n"
                "pc_time,seq,tick_ms,bitmap,sensor_1_x_gs,sensor_1_y_gs,sensor_1_z_gs\n"
                "0.00,1,0,0x0001,1.0,2.0,3.0\n"
                "0.01,2,5,0x0001,2.0,4.0,6.0\n"
                "0.02,4,10,0x0000,,,\n"
            )

            rows = analyzer.analyze_file(path)

        self.assertEqual(rows[0]["drop_rate"], "0.250000")


if __name__ == "__main__":
    unittest.main()
