#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path

import numpy as np

from calibration.ak_consistency_analysis import (
    analyze_ak_rotation_capture,
    load_ak_rotation_csv,
)


def _write_capture(path: Path, rows):
    header = ["pc_time", "ak_seq"]
    for sid in range(1, 13):
        header.extend([f"sensor_{sid}_x_gs", f"sensor_{sid}_y_gs", f"sensor_{sid}_z_gs"])
    with path.open("w", newline="") as f:
        f.write("# experiment_id,ak_unit\n")
        f.write("# sensor_type,AK09973D\n")
        f.write("# board,MagGrad_AK_TMAG_V1\n")
        f.write("# profile,LOW_NOISE\n")
        f.write("# mag_rate_hz,100\n")
        f.write("# icm_rate_hz,500\n")
        f.write("# coil_current_a,1.25\n")
        f.write("# rotation_run_id,run_001\n")
        f.write("# warning,NONE\n")
        f.write("# ak_bitmap,0xFFF\n")
        f.write("# ak_count,12\n")
        f.write(",".join(header) + "\n")
        for idx, sensors in enumerate(rows):
            values = [f"{idx * 0.01:.6f}", str(idx)]
            for sensor in sensors:
                values.extend(f"{value:.9f}" for value in sensor)
            f.write(",".join(values) + "\n")


def _synthetic_ak_rows(n_rows=96):
    theta = np.linspace(0.0, 2.0 * np.pi, n_rows, endpoint=False)
    common = np.column_stack([
        0.8 * np.cos(theta),
        0.8 * np.sin(theta),
        np.full(n_rows, 0.2),
    ])
    rows = []
    for idx, b in enumerate(common):
        sensors = []
        for sid in range(12):
            scale = 1.0 + 0.015 * (sid - 5.5)
            bias = np.array([
                0.02 * (sid % 3),
                -0.015 * (sid % 4),
                0.01 * (sid % 5),
            ])
            noise = 0.002 * np.array([
                np.sin(0.17 * idx + sid),
                np.cos(0.13 * idx + sid),
                np.sin(0.11 * idx + sid),
            ])
            sensors.append(scale * b + bias + noise)
        rows.append(np.array(sensors))
    return rows


class AkConsistencyAnalysisTest(unittest.TestCase):
    def test_loads_metadata_and_ak_sensor_matrix(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "ak_rotation.csv"
            _write_capture(path, _synthetic_ak_rows(8))

            capture = load_ak_rotation_csv(path)

        self.assertEqual(capture.metadata["sensor_type"], "AK09973D")
        self.assertEqual(capture.metadata["ak_bitmap"], "0xFFF")
        self.assertEqual(capture.samples.shape, (8, 12, 3))
        self.assertEqual(capture.seq.tolist(), list(range(8)))

    def test_affine_consistency_reduces_held_out_residual(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "ak_rotation.csv"
            _write_capture(path, _synthetic_ak_rows())

            result = analyze_ak_rotation_capture(path, train_fraction=0.5)

        raw = result.calibration_rows["raw/R_CORR"]
        corrected = result.calibration_rows["consistency_affine"]
        self.assertGreater(raw["heldout_residual_rms"], 0.03)
        self.assertLess(corrected["heldout_residual_rms"], raw["heldout_residual_rms"] * 0.35)
        self.assertEqual(corrected["train_rows"], 48)
        self.assertEqual(corrected["heldout_rows"], 48)

    def test_denoise_rows_include_noise_and_delay_cost(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "ak_rotation.csv"
            _write_capture(path, _synthetic_ak_rows())

            result = analyze_ak_rotation_capture(path, sample_rate_hz=100.0)

        filters = {row["filter"] for row in result.denoise_rows}
        self.assertIn("none", filters)
        self.assertIn("moving_average_5", filters)
        self.assertIn("iir_lowpass_alpha_0.20", filters)
        self.assertIn("savgol_9_2", filters)
        for row in result.denoise_rows:
            self.assertIn("residual_rms", row)
            self.assertIn("estimated_delay_ms", row)
            self.assertIn("trajectory_distortion_rms", row)


if __name__ == "__main__":
    unittest.main()
