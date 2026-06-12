#!/usr/bin/env python3
"""Analyze static MagGrad CSV captures and emit noise summary tables."""

import argparse
import csv
import math
from pathlib import Path


NOISE_COLUMNS = [
    "board",
    "sensor_type",
    "sensor_id/group",
    "target_hz",
    "actual_hz",
    "profile",
    "low_noise_enabled",
    "fallback_from",
    "warning",
    "std_x",
    "std_y",
    "std_z",
    "rms_xyz",
    "p2p_x",
    "p2p_y",
    "p2p_z",
    "error_rate",
    "drop_rate",
    "sample_count",
]

PROFILE_COLUMNS = [
    "sensor_type",
    "target_hz_range",
    "recommended_profile",
    "register_mode/ODR/OSR",
    "expected_actual_hz",
    "verified",
    "fallback_profile",
    "notes",
]

FALLBACK_COLUMNS = [
    "condition",
    "observed_warning",
    "active_profile",
    "data_continues",
    "expected_ros_behavior",
    "impact_on_noise",
]


PROFILE_RECOMMENDATIONS = [
    ["AK09973D", "1..200", "LOW_NOISE", "SDR=0,ODR=500Hz", "target<=200", "verified-static-30s", "STABLE", "best default from AK matrix; lower ODR candidates produced partial bitmap on this board"],
    ["AK09973D", "201..500", "LOW_NOISE", "SDR=0,ODR=500Hz", "actual<target at 500Hz", "unverified-rate", "LOW_NOISE@ODR500", "data continues with warning; not a default low-noise recommendation"],
    ["AK09973D", "501..1000", "LOW_NOISE", "fallback SDR=0,ODR=500Hz", "actual about 469Hz in 1000Hz request", "unverified-not-default", "LOW_NOISE@ODR500", "ODR1000 candidate caused severe partial bitmap and is not used by AUTO"],
    ["TMAG3001", "1..280", "LP_LN1_AVG32X", "DEV_CFG2.LP_LN=1,DEV_CFG1.CONV_AVG=32X", "target<=280", "verified-static-30s", "LC_AVG32X", "highest averaging profile verified through 280Hz; lowest measured RMS"],
    ["TMAG3001", "281..500", "LP_LN1_AVG32X", "DEV_CFG2.LP_LN=1,DEV_CFG1.CONV_AVG=32X", "actual about 281Hz", "unverified-rate", "LC_AVG32X", "data remains complete, but effective rate does not reach target; keep lowest noise and warn"],
    ["QMC6309", "1..200", "LOW_NOISE", "ODR=200Hz,OSR=current,range=current", "target<=200", "unverified", "STABLE", "current profile; lower-noise OSR matrix pending"],
    ["QMC6309", "201..500", "STABLE", "ODR=current,OSR=current,range=current", "target", "unverified", "STABLE", "requires QMC board verification"],
]

FALLBACK_ROWS = [
    ["AK 12/12", "NONE", "LOW_NOISE", "yes", "no warning", "baseline noise"],
    ["AK 11/12 partial", "AK_INIT_PARTIAL", "LOW_NOISE", "yes", "warn only", "missing channel excluded or marked by bitmap"],
    ["TMAG LP_LN=1 AVG32X success", "NONE", "LP_LN1_AVG32X", "yes", "no warning", "lowest measured profile, verified through 280Hz"],
    ["TMAG LP_LN=1 init failed", "TMAG_LP_LN_INIT_FAILED", "LC same AVG", "yes", "warn only", "higher noise expected; data should continue"],
    ["High frequency unverified", "NONE or profile warning", "AUTO-selected profile", "yes if stable", "publish STATUS", "must be measured before verified AUTO default"],
]


def _fmt(value):
    if value is None:
        return ""
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if math.isnan(value):
            return ""
        return f"{value:.6f}"
    return str(value)


def read_capture(path):
    metadata = {}
    rows = []
    header = None
    with Path(path).open(newline="") as f:
        for raw in f:
            if raw.startswith("#"):
                item = raw[1:].strip()
                if "," in item:
                    key, value = item.split(",", 1)
                    metadata[key.strip()] = value.strip()
                continue
            if not raw.strip():
                continue
            if header is None:
                header = next(csv.reader([raw]))
                continue
            rows.append(dict(zip(header, next(csv.reader([raw])))))
    return metadata, rows


def _float_values(rows, column):
    out = []
    for row in rows:
        value = row.get(column, "")
        if value == "":
            continue
        try:
            out.append(float(value))
        except ValueError:
            continue
    return out


def _std(values):
    if len(values) < 2:
        return 0.0 if values else math.nan
    mean = sum(values) / len(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / (len(values) - 1))


def _p2p(values):
    return (max(values) - min(values)) if values else math.nan


def _drop_rate(rows):
    seqs = []
    for row in rows:
        try:
            seqs.append(int(row.get("seq", "")))
        except ValueError:
            continue
    if len(seqs) < 2:
        return 0.0
    expected = seqs[-1] - seqs[0] + 1
    if expected <= 0:
        return 0.0
    missing = expected - len(set(seqs))
    return max(0, missing) / expected


def _rate_deficit(target_hz, actual_hz):
    try:
        target = float(target_hz)
        actual = float(actual_hz)
    except (TypeError, ValueError):
        return None
    if target <= 0.0:
        return None
    return max(0.0, target - actual) / target


def _sensor_ids(header):
    ids = []
    for name in header:
        if name.startswith("sensor_") and name.endswith("_x_gs"):
            ids.append(name[len("sensor_"):-len("_x_gs")])
    return ids


def analyze_file(path):
    metadata, rows = read_capture(path)
    if not rows:
        return []
    header = list(rows[0].keys())
    board = metadata.get("board", metadata.get("project", ""))
    sensor_type = metadata.get("hardware_config", metadata.get("sensor_type", ""))
    target_hz = metadata.get("status_target_hz", metadata.get("rate_hz", ""))
    actual_hz = metadata.get("actual_hz", metadata.get("status_actual_hz", target_hz))
    drop_rate = _rate_deficit(target_hz, actual_hz)
    if drop_rate is None:
        drop_rate = _drop_rate(rows)
    profile = metadata.get("status_profile_id", metadata.get("status_profile", metadata.get("profile", "")))
    fallback_from = metadata.get("status_fallback_from", metadata.get("fallback_from", "NONE"))
    warning = metadata.get("status_warning", metadata.get("warning", "NONE"))
    low_noise_enabled = "yes" if (
        profile in ("AUTO", "LOW_NOISE") or "lp_ln1" in str(profile).lower()
    ) and fallback_from in ("", "NONE") else "no"

    result = []
    for sid in _sensor_ids(header):
        xs = _float_values(rows, f"sensor_{sid}_x_gs")
        ys = _float_values(rows, f"sensor_{sid}_y_gs")
        zs = _float_values(rows, f"sensor_{sid}_z_gs")
        valid_count = min(len(xs), len(ys), len(zs))
        total = len(rows)
        error_rate = 1.0 - (valid_count / total) if total else 0.0
        std_x = _std(xs)
        std_y = _std(ys)
        std_z = _std(zs)
        rms_xyz = math.sqrt(sum(v * v for v in (std_x, std_y, std_z) if not math.isnan(v)))
        result.append({
            "board": board,
            "sensor_type": sensor_type,
            "sensor_id/group": sid,
            "target_hz": target_hz,
            "actual_hz": actual_hz,
            "profile": profile,
            "low_noise_enabled": low_noise_enabled,
            "fallback_from": fallback_from or "NONE",
            "warning": warning or "NONE",
            "std_x": _fmt(std_x),
            "std_y": _fmt(std_y),
            "std_z": _fmt(std_z),
            "rms_xyz": _fmt(rms_xyz),
            "p2p_x": _fmt(_p2p(xs)),
            "p2p_y": _fmt(_p2p(ys)),
            "p2p_z": _fmt(_p2p(zs)),
            "error_rate": _fmt(error_rate),
            "drop_rate": _fmt(drop_rate),
            "sample_count": str(valid_count),
        })
    return result


def write_csv(path, rows, columns):
    with Path(path).open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in columns})


def markdown_table(rows, columns):
    lines = ["| " + " | ".join(columns) + " |", "| " + " | ".join(["---"] * len(columns)) + " |"]
    for row in rows:
        lines.append("| " + " | ".join(str(row.get(col, "")) for col in columns) + " |")
    return "\n".join(lines)


def write_markdown(path, noise_rows):
    profile_rows = [dict(zip(PROFILE_COLUMNS, row)) for row in PROFILE_RECOMMENDATIONS]
    fallback_rows = [dict(zip(FALLBACK_COLUMNS, row)) for row in FALLBACK_ROWS]
    text = "\n\n".join([
        "# MagGrad Noise And Profile Report",
        "## Profile Recommendation Table",
        markdown_table(profile_rows, PROFILE_COLUMNS),
        "## Noise Measurement Table",
        markdown_table(noise_rows, NOISE_COLUMNS) if noise_rows else "No capture rows analyzed yet.",
        "## Fallback And Warning Table",
        markdown_table(fallback_rows, FALLBACK_COLUMNS),
    ])
    Path(path).write_text(text + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("captures", nargs="*", type=Path)
    parser.add_argument("--noise-csv", type=Path, default=Path("maggrad_noise_measurements.csv"))
    parser.add_argument("--profile-csv", type=Path, default=Path("maggrad_profile_recommendations.csv"))
    parser.add_argument("--report-md", type=Path, default=Path("maggrad_noise_report.md"))
    args = parser.parse_args()

    noise_rows = []
    for capture in args.captures:
        noise_rows.extend(analyze_file(capture))
    write_csv(args.noise_csv, noise_rows, NOISE_COLUMNS)
    profile_rows = [dict(zip(PROFILE_COLUMNS, row)) for row in PROFILE_RECOMMENDATIONS]
    write_csv(args.profile_csv, profile_rows, PROFILE_COLUMNS)
    write_markdown(args.report_md, noise_rows)


if __name__ == "__main__":
    main()
