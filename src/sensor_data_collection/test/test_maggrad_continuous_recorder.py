import importlib.util
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "maggrad_continuous_collection_node.py"
)


def load_collection_module():
    rospy = types.ModuleType("rospy")
    rospy.loginfo = lambda *_args, **_kwargs: None
    rospy.logwarn = lambda *_args, **_kwargs: None
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Imu = type("Imu", (), {})
    serial_processor = types.ModuleType("serial_processor")
    serial_processor_msg = types.ModuleType("serial_processor.msg")
    serial_processor_msg.MagGradImuRaw = type("MagGradImuRaw", (), {})
    serial_processor_msg.StmUplink = type("StmUplink", (), {})
    sensor_array_config = types.ModuleType("sensor_array_config")
    sensor_array_config.get_hardware_config = lambda _name: None
    sensor_array_config.resolve_runtime_value = lambda value, _key, default: default if value == "runtime" else value
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = type("Bool", (), {})
    std_msgs_msg.String = type("String", (), {})
    tf2_ros = types.ModuleType("tf2_ros")
    tf2_ros.Buffer = type("Buffer", (), {})
    tf2_ros.TransformListener = type("TransformListener", (), {})

    for name, module in {
        "rospy": rospy,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "serial_processor": serial_processor,
        "serial_processor.msg": serial_processor_msg,
        "sensor_array_config": sensor_array_config,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "tf2_ros": tf2_ros,
    }.items():
        sys.modules[name] = module

    spec = importlib.util.spec_from_file_location("maggrad_continuous_collection_node", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ContinuousRecorderTest(unittest.TestCase):
    def test_start_creates_experiment_directory_with_data_file_inside(self):
        module = load_collection_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            recorder = module.ContinuousRecorder(tmpdir, 1, "csv")

            recorder.start()
            path = recorder.path
            recorder.stop()

            self.assertTrue(os.path.isdir(os.path.dirname(path)))
            self.assertEqual(os.path.dirname(os.path.dirname(path)), tmpdir)
            self.assertEqual(
                os.path.basename(path),
                os.path.basename(os.path.dirname(path)) + ".csv",
            )

    def test_csv_stores_experiment_metadata_in_comments_not_each_row(self):
        module = load_collection_module()
        with tempfile.TemporaryDirectory() as tmpdir:
            recorder = module.ContinuousRecorder(
                tmpdir,
                1,
                "csv",
                experiment_metadata={"experiment_id": "trial_001", "warning": "NONE", "ak_topic": "stm_uplink_raw"},
                min_valid_rows=0,
            )

            recorder.start()
            recorder.write({
                "pc_time": 1.25,
                "ak_seq": 7,
                "ak_tick_ms": 123,
                "ak_topic": "stm_uplink_raw",
                "coil": {
                    "name": "coil_1_on",
                    "channel": 1,
                    "index": 0,
                    "state_time": 1.0,
                    "dwell_s": 0.25,
                    "frequency_hz": 0.2,
                    "amplitude_v": 5.0,
                    "offset_v": 2.5,
                },
                "sensors": [{"id": 1, "x": 1.0, "y": 2.0, "z": 3.0}],
                "imu_raw": {},
                "imu": {},
                "experiment": {"experiment_id": "trial_001", "warning": "NONE"},
            })
            path = recorder.path
            recorder.stop()

            lines = Path(path).read_text(encoding="utf-8").splitlines()
            self.assertIn("# experiment_id,trial_001", lines)
            self.assertIn("# ak_topic,stm_uplink_raw", lines)
            header = next(line for line in lines if not line.startswith("#"))
            row = lines[lines.index(header) + 1]
            self.assertNotIn("experiment_id", header)
            self.assertNotIn("warning", header)
            self.assertNotIn("ak_topic", header)
            self.assertTrue(header.startswith("pc_time,ak_seq,ak_tick_ms,coil_state"))
            self.assertTrue(row.startswith("1.250000000,7,123,coil_1_on"))


if __name__ == "__main__":
    unittest.main()
