import unittest
from pathlib import Path

import yaml


CONFIG = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "experiment"
    / "maggrad_continuous.yaml"
)
LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "mi_gels.launch"


class MagGradExperimentConfigTest(unittest.TestCase):
    def test_signal_and_collection_timing_share_one_experiment_config(self):
        with CONFIG.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        frequency = float(config["coil_frequency_hz"])
        initial_by_channel = {
            int(item["channel_index"]): item for item in config["initial_configs"]
        }
        timing_by_channel = {
            int(item["channel"]): item for item in config["coil_timing"]
        }

        self.assertEqual(sorted(initial_by_channel), [1, 2, 3])
        self.assertEqual(sorted(timing_by_channel), [1, 2, 3])
        for channel in sorted(initial_by_channel):
            initial = initial_by_channel[channel]
            timing = timing_by_channel[channel]
            self.assertEqual(float(initial["frequency"]), frequency)
            self.assertEqual(float(initial["phase"]), float(timing["phase_deg"]))
            self.assertEqual(float(initial["duty_cycle"]), float(timing["duty_cycle"]))
            self.assertEqual(float(initial["amplitude"]), float(config["default_amplitude_v"]))
            self.assertEqual(float(initial["offset"]), float(config["default_offset_v"]))

    def test_main_launch_defaults_to_unified_experiment_config(self):
        launch_text = LAUNCH.read_text(encoding="utf-8")

        self.assertIn(
            '<arg name="experiment_config" default="$(find mi_gels_bringup)/config/experiment/maggrad_continuous.yaml"/>',
            launch_text,
        )
        self.assertIn('<arg name="signal_config" default="$(arg experiment_config)"/>', launch_text)
        self.assertIn('<arg name="record_collection_config" default="$(arg experiment_config)"/>', launch_text)
        self.assertIn('<arg name="record_output_dir" default="$(find sensor_data_collection)/../../data/maggrad_continuous"/>', launch_text)
        self.assertIn('<arg name="orchestrator_coil_frequency_hz" default="runtime"/>', launch_text)
        self.assertNotIn('<param name="coil_frequency_hz" value="0.2"/>', launch_text)


if __name__ == "__main__":
    unittest.main()
