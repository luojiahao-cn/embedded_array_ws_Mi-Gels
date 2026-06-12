# High Field Sweep Analysis

Date: 2026-06-10

Remote host: `zlab`

Workspace: `/home/zhang/embedded_array_ws_Mi-Gels`

ROS topic: `/stm_uplink_raw`

Message type: `serial_processor/StmUplink`

Each level was collected for about 20 s. The over-temperature protection run
captured at `20260610_225235_level09` was deleted and is not included.

## Data Files

Raw JSONL and summary JSON files are stored on `zlab`:

```text
/home/zhang/embedded_array_ws_Mi-Gels/data/high_field_sweep/
```

Included summary files:

```text
20260610_224202_field_around_10_summary.json
20260610_224338_level02_summary.json
20260610_224531_level03_summary.json
20260610_224649_level04_summary.json
20260610_224806_level05_summary.json
20260610_224925_level06_summary.json
20260610_225026_level07_summary.json
20260610_225130_level08_summary.json
20260610_225433_level09_summary.json
20260610_225555_level10_summary.json
```

## Summary Table

`mean_mag` is the average magnitude across all 12 sensors. `between max-min`
is the spread between the highest and lowest sensor mean magnitude at that
field level. `temporal std` is the average per-sensor magnitude standard
deviation over the 20 s capture.

| Level | mean_mag | between std | between max-min | max-min / mean | temporal std |
| --- | ---: | ---: | ---: | ---: | ---: |
| field_around_10 | 10.0133 | 0.0645 | 0.2343 | 2.340% | 0.0546 |
| level02 | 20.2487 | 0.0793 | 0.2881 | 1.423% | 0.0548 |
| level03 | 28.0751 | 0.0896 | 0.3177 | 1.132% | 0.0550 |
| level04 | 33.8839 | 0.0984 | 0.3443 | 1.016% | 0.0555 |
| level05 | 40.4639 | 0.1086 | 0.3807 | 0.941% | 0.0559 |
| level06 | 46.5940 | 0.1193 | 0.4233 | 0.909% | 0.0560 |
| level07 | 50.9858 | 0.1258 | 0.4528 | 0.888% | 0.0569 |
| level08 | 58.9005 | 0.1395 | 0.5059 | 0.859% | 0.0574 |
| level09 | 70.2590 | 0.2091 | 0.7792 | 1.109% | 0.0638 |
| level10 | 78.0104 | 0.2241 | 0.8302 | 1.064% | 0.0581 |

## Main Findings

### 1. The First Eight Levels Are Stable

From about `10` to `59`, the array behaves consistently:

- Temporal noise stays close to `0.055` to `0.057`.
- Absolute sensor-to-sensor spread increases slowly with field strength.
- Relative sensor-to-sensor spread decreases from `2.34%` to `0.86%`.
- Sampling rate stays near `200 Hz`.

This range does not show evidence of saturation or unstable noise growth.

### 2. Levels 9 and 10 Changed Field Direction

Levels 9 and 10 are not just stronger versions of the first eight levels. The
array mean field vector changed direction significantly.

| Level | array mean vector | vector magnitude | z fraction |
| --- | ---: | ---: | ---: |
| field_around_10 | `(-6.229, -7.799, +0.624)` | 10.001 | +0.0624 |
| level02 | `(-12.632, -15.780, +0.790)` | 20.228 | +0.0390 |
| level03 | `(-17.526, -21.880, +0.909)` | 28.048 | +0.0324 |
| level04 | `(-21.160, -26.406, +0.996)` | 33.853 | +0.0294 |
| level05 | `(-25.279, -31.530, +1.098)` | 40.427 | +0.0271 |
| level06 | `(-29.117, -36.303, +1.182)` | 46.552 | +0.0254 |
| level07 | `(-31.870, -39.720, +1.244)` | 50.940 | +0.0244 |
| level08 | `(-36.822, -45.885, +1.362)` | 58.849 | +0.0232 |
| level09 | `(-54.245, +44.516, +0.819)` | 70.178 | +0.0117 |
| level10 | `(-60.245, +49.411, +0.848)` | 77.920 | +0.0109 |

The sign of the Y component flips between level08 and level09. Therefore the
larger spread at levels 9 and 10 should not be interpreted as pure high-field
nonlinearity. It includes the effect of field direction and local spatial
gradient direction.

### 3. Temporal Noise Is Mostly Common-Mode

Per-sensor temporal noise is similar across all sensors. In the high two levels,
all sensors show a comparable increase rather than one sensor failing.

| Sensor | noise mean | noise max | low8 mean | high2 mean | high2 / low8 |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 0.0571 | 0.0636 | 0.0561 | 0.0610 | 1.089 |
| 2 | 0.0565 | 0.0629 | 0.0556 | 0.0602 | 1.083 |
| 3 | 0.0567 | 0.0642 | 0.0555 | 0.0616 | 1.110 |
| 4 | 0.0564 | 0.0633 | 0.0555 | 0.0603 | 1.086 |
| 5 | 0.0575 | 0.0632 | 0.0566 | 0.0610 | 1.078 |
| 6 | 0.0566 | 0.0638 | 0.0556 | 0.0607 | 1.093 |
| 7 | 0.0571 | 0.0631 | 0.0563 | 0.0604 | 1.073 |
| 8 | 0.0568 | 0.0637 | 0.0558 | 0.0609 | 1.091 |
| 9 | 0.0570 | 0.0650 | 0.0559 | 0.0614 | 1.098 |
| 10 | 0.0564 | 0.0640 | 0.0554 | 0.0608 | 1.099 |
| 11 | 0.0565 | 0.0638 | 0.0552 | 0.0615 | 1.113 |
| 12 | 0.0569 | 0.0646 | 0.0558 | 0.0613 | 1.098 |

This supports the interpretation that short-term random noise is not dominated
by a single defective sensor. The high-field noise increase is more likely
related to field state, thermal state, or common acquisition conditions.

### 4. Sensor-to-Sensor Differences Are Mostly Linear

Each sensor mean magnitude was regressed against the array mean magnitude.
All sensors have very high `R^2`, so most differences can be modeled as fixed
gain/offset/pose effects.

| Sensor | slope | gain error | intercept | R^2 | RMSE | mean residual |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 1.000106 | +0.011% | +0.0668 | 0.99999978 | 0.0097 | +0.0715 |
| 2 | 0.998401 | -0.160% | -0.0625 | 0.99999896 | 0.0209 | -0.1324 |
| 3 | 1.000173 | +0.017% | +0.0245 | 0.99999957 | 0.0134 | +0.0320 |
| 4 | 1.001609 | +0.161% | -0.1701 | 0.99999570 | 0.0426 | -0.0998 |
| 5 | 1.001317 | +0.132% | -0.0125 | 0.99999969 | 0.0114 | +0.0451 |
| 6 | 1.001572 | +0.157% | -0.1370 | 0.99999175 | 0.0590 | -0.0682 |
| 7 | 0.993985 | -0.602% | +0.0973 | 0.99998904 | 0.0675 | -0.1658 |
| 8 | 0.993306 | -0.669% | +0.2722 | 0.99996288 | 0.1242 | -0.0207 |
| 9 | 1.001430 | +0.143% | +0.0216 | 0.99999972 | 0.0108 | +0.0841 |
| 10 | 1.002761 | +0.276% | -0.1427 | 0.99999235 | 0.0569 | -0.0219 |
| 11 | 1.001647 | +0.165% | -0.0513 | 0.99999965 | 0.0122 | +0.0208 |
| 12 | 1.003693 | +0.369% | +0.0937 | 0.99999997 | 0.0035 | +0.2553 |

The most important fixed patterns are:

- Sensor 12 is consistently high, with about `+0.369%` gain error and
  `+0.255` average residual.
- Sensors 7 and 8 have the largest negative gain errors, about `-0.602%` and
  `-0.669%`.
- Sensors 7 and 8 become the lowest sensors when the field direction changes
  at levels 9 and 10.

### 5. Lowest and Highest Sensors

| Level | Lowest sensor | Highest sensor | Gap |
| --- | ---: | ---: | ---: |
| field_around_10 | 4: 9.9074 | 12: 10.1417 | 0.2343 |
| level02 | 4: 20.1305 | 12: 20.4185 | 0.2881 |
| level03 | 4: 27.9525 | 12: 28.2702 | 0.3177 |
| level04 | 4: 33.7579 | 12: 34.1022 | 0.3443 |
| level05 | 2: 40.3260 | 12: 40.7067 | 0.3807 |
| level06 | 2: 46.4385 | 12: 46.8618 | 0.4233 |
| level07 | 2: 50.8177 | 12: 51.2706 | 0.4528 |
| level08 | 2: 58.7095 | 12: 59.2153 | 0.5059 |
| level09 | 7: 69.8371 | 12: 70.6163 | 0.7792 |
| level10 | 7: 77.5539 | 12: 78.3841 | 0.8302 |

The low sensor changes from 4 to 2 and then to 7 as the field grows and changes
direction. The high sensor remains 12 throughout. This is consistent with a
combination of fixed gain/pose differences plus direction-dependent local
gradient effects.

## Interpretation

The array is not primarily limited by short-term random noise in these captures.
The dominant effects are:

1. Sensor placement and orientation differences.
2. Per-sensor gain differences.
3. Local magnetic field gradient across the array.
4. Field direction changes, especially between level08 and level09.
5. Possible thermal/common-mode effects around level09.

The sensors are only about 2 mm apart, so in a uniform field the difference
should be small. However, the data show that the field is not behaving like a
single uniform scalar field across all captures. Direction-dependent residuals
are visible, especially for sensors 7 and 8 at levels 9 and 10.

## Saturation Check

There is no clear saturation evidence in this dataset:

- All 12 sensors remain present.
- Sampling rate remains near `200 Hz`.
- Magnitude continues increasing at the final level.
- No axis appears stuck at a constant value.
- No single sensor develops a much larger variance than the others.

Therefore, saturation is not the current leading explanation. Direction change,
spatial gradient, and per-sensor calibration/pose differences are more likely.

## Recommendations

1. Repeat one field level three times without changing the field.
   This separates random/thermal drift from field-position effects.

2. Repeat level08 and level10 directions separately.
   The direction flip between level08 and level09 is the biggest confounder.

3. Fit per-sensor calibration parameters using vector data, not only magnitude.
   Magnitude hides orientation errors. A better model is:

   ```text
   B_sensor_i = A_i * B_reference + b_i
   ```

   where `A_i` captures gain and cross-axis/pose errors and `b_i` captures
   offset.

4. Add a controlled zero-field capture before and after the sweep.
   This helps separate offset drift from field-dependent response.

5. If the hardware allows it, log temperature or thermal protection state.
   The deleted over-temperature run and the level09 noise bump suggest thermal
   state may matter.

6. For comparing sensors 2 mm apart, compare vector residuals after subtracting
   the array mean vector:

   ```text
   residual_i = B_i - mean(B_all_sensors)
   ```

   This will show whether the difference is a fixed sensor calibration error or
   a spatial gradient pattern that rotates with the field.

