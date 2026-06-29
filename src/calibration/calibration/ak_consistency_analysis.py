"""AK09973D rotation consistency calibration and denoise analysis."""

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


CALIBRATION_COLUMNS = [
    "experiment_id",
    "sensor_type",
    "board",
    "profile",
    "mag_rate_hz",
    "icm_rate_hz",
    "coil_current_a",
    "rotation_run_id",
    "warning",
    "ak_bitmap",
    "ak_count",
    "stage",
    "train_rows",
    "heldout_rows",
    "train_residual_rms",
    "heldout_residual_rms",
    "heldout_residual_p2p",
    "field_norm_std",
    "drop_rate",
]

DENOISE_COLUMNS = [
    "experiment_id",
    "sensor_type",
    "board",
    "profile",
    "mag_rate_hz",
    "icm_rate_hz",
    "coil_current_a",
    "rotation_run_id",
    "warning",
    "ak_bitmap",
    "ak_count",
    "filter",
    "residual_rms",
    "residual_p2p",
    "field_norm_std",
    "trajectory_distortion_rms",
    "estimated_delay_ms",
]

RECOMMENDATION_COLUMNS = [
    "sensor_type",
    "recommended_profile",
    "recommended_mag_rate_hz",
    "recommended_filter",
    "calibration_stage",
    "reason",
    "warning_policy",
]


@dataclass
class AkRotationCapture:
    path: Path
    metadata: dict
    samples: np.ndarray
    seq: np.ndarray
    pc_time: np.ndarray


@dataclass
class AkConsistencyResult:
    metadata: dict
    calibration_rows: dict
    denoise_rows: list
    affine: dict


def load_ak_rotation_csv(path):
    """Load a MagGrad AK rotation CSV with optional '# key,value' metadata."""
    path = Path(path)
    metadata = {}
    rows = []
    header = None
    with path.open(newline="") as f:
        for raw in f:
            if raw.startswith("#"):
                item = raw[1:].strip()
                if "," in item:
                    key, value = item.split(",", 1)
                    metadata[key.strip()] = value.strip()
                continue
            if not raw.strip():
                continue
            parsed = next(csv.reader([raw]))
            if header is None:
                header = parsed
            else:
                rows.append(dict(zip(header, parsed)))

    if header is None:
        raise ValueError(f"{path} has no CSV header")
    sensor_ids = _sensor_ids(header)
    if len(sensor_ids) != 12:
        raise ValueError(f"{path} has {len(sensor_ids)} AK sensors, expected 12")

    samples = np.zeros((len(rows), 12, 3), dtype=float)
    seq = np.zeros(len(rows), dtype=int)
    pc_time = np.zeros(len(rows), dtype=float)
    for row_idx, row in enumerate(rows):
        seq[row_idx] = _int_or_default(row.get("ak_seq", row.get("seq", row_idx)), row_idx)
        pc_time[row_idx] = _float_or_default(row.get("pc_time", row_idx), float(row_idx))
        for sensor_idx, sid in enumerate(sensor_ids):
            samples[row_idx, sensor_idx, 0] = float(row[f"sensor_{sid}_x_gs"])
            samples[row_idx, sensor_idx, 1] = float(row[f"sensor_{sid}_y_gs"])
            samples[row_idx, sensor_idx, 2] = float(row[f"sensor_{sid}_z_gs"])
    return AkRotationCapture(path=path, metadata=metadata, samples=samples, seq=seq, pc_time=pc_time)


def analyze_ak_rotation_capture(path, train_fraction=0.67, sample_rate_hz=None):
    capture = load_ak_rotation_csv(path)
    samples = capture.samples
    if samples.shape[0] < 4:
        raise ValueError("AK rotation capture needs at least 4 rows")
    split = int(round(samples.shape[0] * float(train_fraction)))
    split = max(2, min(samples.shape[0] - 2, split))
    train_idx = np.arange(split)
    heldout_idx = np.arange(split, samples.shape[0])
    sample_rate_hz = _sample_rate_hz(capture, sample_rate_hz)

    raw_train = _residual_stats(samples[train_idx])
    raw_heldout = _residual_stats(samples[heldout_idx])
    affine = _fit_consistency_affine(samples[train_idx])
    corrected = _apply_affine(samples, affine)
    corr_train = _residual_stats(corrected[train_idx])
    corr_heldout = _residual_stats(corrected[heldout_idx])

    common_meta = _common_row_metadata(capture.metadata)
    calibration_rows = {
        "raw/R_CORR": {
            **common_meta,
            "stage": "raw/R_CORR",
            "train_rows": len(train_idx),
            "heldout_rows": len(heldout_idx),
            "train_residual_rms": raw_train["residual_rms"],
            "heldout_residual_rms": raw_heldout["residual_rms"],
            "heldout_residual_p2p": raw_heldout["residual_p2p"],
            "field_norm_std": raw_heldout["field_norm_std"],
            "drop_rate": _drop_rate(capture.seq),
        },
        "consistency_affine": {
            **common_meta,
            "stage": "consistency_affine",
            "train_rows": len(train_idx),
            "heldout_rows": len(heldout_idx),
            "train_residual_rms": corr_train["residual_rms"],
            "heldout_residual_rms": corr_heldout["residual_rms"],
            "heldout_residual_p2p": corr_heldout["residual_p2p"],
            "field_norm_std": corr_heldout["field_norm_std"],
            "drop_rate": _drop_rate(capture.seq),
        },
    }
    denoise_rows = _denoise_rows(corrected, sample_rate_hz, common_meta)
    return AkConsistencyResult(
        metadata=capture.metadata,
        calibration_rows=calibration_rows,
        denoise_rows=denoise_rows,
        affine=affine,
    )


def write_analysis_outputs(result, output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    calibration_rows = list(result.calibration_rows.values())
    denoise_rows = result.denoise_rows
    recommendation_rows = [_recommendation_row(result)]
    paths = {
        "calibration_csv": output_dir / "ak_calibration_comparison.csv",
        "calibration_md": output_dir / "ak_calibration_comparison.md",
        "denoise_csv": output_dir / "ak_denoise_comparison.csv",
        "denoise_md": output_dir / "ak_denoise_comparison.md",
        "recommendation_csv": output_dir / "ak_recommended_runtime_config.csv",
        "recommendation_md": output_dir / "ak_recommended_runtime_config.md",
    }
    _write_csv(paths["calibration_csv"], calibration_rows, CALIBRATION_COLUMNS)
    _write_markdown(paths["calibration_md"], "AK Calibration Comparison", calibration_rows, CALIBRATION_COLUMNS)
    _write_csv(paths["denoise_csv"], denoise_rows, DENOISE_COLUMNS)
    _write_markdown(paths["denoise_md"], "AK Denoise Comparison", denoise_rows, DENOISE_COLUMNS)
    _write_csv(paths["recommendation_csv"], recommendation_rows, RECOMMENDATION_COLUMNS)
    _write_markdown(
        paths["recommendation_md"],
        "AK Recommended Runtime Config",
        recommendation_rows,
        RECOMMENDATION_COLUMNS,
    )
    return paths


def _fit_consistency_affine(train_samples):
    target = np.mean(train_samples, axis=1)
    affine = {}
    for sensor_idx in range(train_samples.shape[1]):
        x = train_samples[:, sensor_idx, :]
        A = np.column_stack([x, np.ones(x.shape[0])])
        coeff = []
        for axis in range(3):
            c, _, _, _ = np.linalg.lstsq(A, target[:, axis], rcond=None)
            coeff.append(c)
        coeff = np.array(coeff)
        affine[sensor_idx + 1] = {
            "D": coeff[:, 0:3],
            "e": coeff[:, 3],
        }
    return affine


def _apply_affine(samples, affine):
    corrected = np.zeros_like(samples, dtype=float)
    for sensor_idx in range(samples.shape[1]):
        params = affine[sensor_idx + 1]
        corrected[:, sensor_idx, :] = samples[:, sensor_idx, :] @ params["D"].T + params["e"]
    return corrected


def _residual_stats(samples):
    common = np.mean(samples, axis=1)
    residual = samples - common[:, None, :]
    residual_norm = np.linalg.norm(residual, axis=2)
    common_norm = np.linalg.norm(common, axis=1)
    return {
        "residual_rms": _rms(residual_norm),
        "residual_p2p": float(np.max(residual_norm) - np.min(residual_norm)),
        "field_norm_std": float(np.std(common_norm, ddof=1)) if common_norm.size > 1 else 0.0,
    }


def _denoise_rows(samples, sample_rate_hz, metadata):
    common = np.mean(samples, axis=1)
    filters = [
        ("none", samples, 0.0),
        ("moving_average_5", _moving_average(samples, 5), 2.0 / sample_rate_hz),
        ("iir_lowpass_alpha_0.20", _iir_lowpass(samples, 0.20), 4.0 / sample_rate_hz),
        ("savgol_9_2", _savgol(samples, 9, 2), 4.0 / sample_rate_hz),
    ]
    rows = []
    for name, filtered, delay_s in filters:
        stats = _residual_stats(filtered)
        filtered_common = np.mean(filtered, axis=1)
        distortion = _rms(np.linalg.norm(filtered_common - common, axis=1))
        rows.append({
            **metadata,
            "filter": name,
            "residual_rms": stats["residual_rms"],
            "residual_p2p": stats["residual_p2p"],
            "field_norm_std": stats["field_norm_std"],
            "trajectory_distortion_rms": distortion,
            "estimated_delay_ms": delay_s * 1000.0,
        })
    return rows


def _moving_average(values, window):
    pad = window // 2
    padded = np.pad(values, [(pad, pad), (0, 0), (0, 0)], mode="edge")
    out = np.zeros_like(values, dtype=float)
    for idx in range(values.shape[0]):
        out[idx] = np.mean(padded[idx:idx + window], axis=0)
    return out


def _iir_lowpass(values, alpha):
    out = np.zeros_like(values, dtype=float)
    out[0] = values[0]
    for idx in range(1, values.shape[0]):
        out[idx] = alpha * values[idx] + (1.0 - alpha) * out[idx - 1]
    return out


def _savgol(values, window, order):
    if window % 2 == 0:
        raise ValueError("Savitzky-Golay window must be odd")
    half = window // 2
    out = np.zeros_like(values, dtype=float)
    x_full = np.arange(-half, half + 1, dtype=float)
    flat = values.reshape(values.shape[0], -1)
    out_flat = out.reshape(values.shape[0], -1)
    for idx in range(values.shape[0]):
        start = max(0, idx - half)
        end = min(values.shape[0], idx + half + 1)
        x = np.arange(start - idx, end - idx, dtype=float)
        if len(x) < order + 1:
            out_flat[idx] = flat[idx]
            continue
        for col in range(flat.shape[1]):
            coeff = np.polyfit(x, flat[start:end, col], min(order, len(x) - 1))
            out_flat[idx, col] = np.polyval(coeff, 0.0)
    return out


def _sample_rate_hz(capture, explicit):
    if explicit is not None:
        return float(explicit)
    for key in ("mag_rate_hz", "status_actual_hz", "actual_hz", "status_target_hz"):
        value = capture.metadata.get(key)
        if value:
            parsed = _float_or_default(value, None)
            if parsed and parsed > 0:
                return parsed
    if capture.pc_time.size > 2:
        diffs = np.diff(capture.pc_time)
        valid = diffs[diffs > 0]
        if valid.size:
            return 1.0 / float(np.median(valid))
    return 100.0


def _drop_rate(seq):
    if len(seq) < 2:
        return 0.0
    expected = int(seq[-1] - seq[0] + 1)
    if expected <= 0:
        return 0.0
    missing = expected - len(set(int(v) for v in seq))
    return max(0, missing) / expected


def _common_row_metadata(metadata):
    return {
        "experiment_id": metadata.get("experiment_id", ""),
        "sensor_type": metadata.get("sensor_type", "AK09973D"),
        "board": metadata.get("board", ""),
        "profile": metadata.get("profile", metadata.get("status_profile", "")),
        "mag_rate_hz": metadata.get("mag_rate_hz", metadata.get("status_target_hz", "")),
        "icm_rate_hz": metadata.get("icm_rate_hz", metadata.get("status_icm_target_hz", "")),
        "coil_current_a": metadata.get("coil_current_a", ""),
        "rotation_run_id": metadata.get("rotation_run_id", ""),
        "warning": metadata.get("warning", metadata.get("status_warning", "NONE")),
        "ak_bitmap": metadata.get("ak_bitmap", metadata.get("status_ak_bitmap", "")),
        "ak_count": metadata.get("ak_count", metadata.get("status_ak_count", "")),
    }


def _recommendation_row(result):
    meta = _common_row_metadata(result.metadata)
    best = min(result.denoise_rows, key=lambda row: (row["estimated_delay_ms"] > 25.0, row["residual_rms"]))
    return {
        "sensor_type": "AK09973D",
        "recommended_profile": meta.get("profile", "AUTO") or "AUTO",
        "recommended_mag_rate_hz": meta.get("mag_rate_hz", ""),
        "recommended_filter": best["filter"],
        "calibration_stage": "consistency_affine",
        "reason": "lowest residual among low-delay candidates in this capture",
        "warning_policy": "AK_INIT_PARTIAL/RATE_UNVERIFIED are retained as warnings, not fatal analysis errors",
    }


def _write_csv(path, rows, columns):
    with Path(path).open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _fmt(row.get(key, "")) for key in columns})


def _write_markdown(path, title, rows, columns):
    lines = [f"# {title}", "", _markdown_table(rows, columns), ""]
    Path(path).write_text("\n".join(lines))


def _markdown_table(rows, columns):
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(["---"] * len(columns)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(_fmt(row.get(column, "")) for column in columns) + " |")
    return "\n".join(lines)


def _sensor_ids(header):
    ids = []
    for name in header:
        if name.startswith("sensor_") and name.endswith("_x_gs"):
            ids.append(name[len("sensor_"):-len("_x_gs")])
    return ids


def _float_or_default(value, default):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _int_or_default(value, default):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _rms(values):
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return math.nan
    return float(np.sqrt(np.mean(values * values)))


def _fmt(value):
    if isinstance(value, (np.floating, float)):
        if math.isnan(float(value)):
            return ""
        return f"{float(value):.9g}"
    if isinstance(value, (np.integer, int)):
        return str(int(value))
    return str(value)


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Analyze AK09973D rotation consistency captures.")
    parser.add_argument("csv", type=Path, help="AK rotation CSV capture")
    parser.add_argument("--output-dir", type=Path, required=True, help="Output directory for CSV/Markdown tables")
    parser.add_argument("--train-fraction", type=float, default=0.67, help="Fraction of rows used for affine fitting")
    parser.add_argument("--sample-rate-hz", type=float, default=None, help="Override sample rate used for delay estimates")
    return parser


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    result = analyze_ak_rotation_capture(
        args.csv,
        train_fraction=args.train_fraction,
        sample_rate_hz=args.sample_rate_hz,
    )
    paths = write_analysis_outputs(result, args.output_dir)
    for name, path in sorted(paths.items()):
        print(f"{name}: {path}")


if __name__ == "__main__":
    main()
