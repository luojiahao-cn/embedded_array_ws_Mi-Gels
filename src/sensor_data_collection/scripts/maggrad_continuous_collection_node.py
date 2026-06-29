#!/usr/bin/env python3
"""Continuous MagGrad collection with host-side electromagnet state labels."""

import csv
import json
import os
import threading
from datetime import datetime

import rospy
from sensor_msgs.msg import Imu
from serial_processor.msg import MagGradImuRaw, StmUplink
from sensor_array_config import get_hardware_config, resolve_runtime_value
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


class ContinuousRecorder:
    def __init__(
        self,
        output_dir,
        n_sensors,
        record_format,
        tf_frames=None,
        experiment_metadata=None,
        min_valid_rows=1,
        delete_invalid=True,
    ):
        self.output_dir = os.path.expanduser(output_dir)
        self.n_sensors = int(n_sensors)
        self.record_format = str(record_format).lower()
        self.tf_frames = list(tf_frames or [])
        self.experiment_metadata = dict(experiment_metadata or {})
        self.min_valid_rows = int(min_valid_rows)
        self.delete_invalid = bool(delete_invalid)
        self.file = None
        self.writer = None
        self.path = None
        self.rows_written = 0
        self.last_closed_path = None
        self.last_closed_rows = 0
        self.last_closed_valid = True
        os.makedirs(self.output_dir, exist_ok=True)

    @property
    def is_open(self):
        return self.file is not None

    def start(self):
        self.stop()
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        ext = "jsonl" if self.record_format == "jsonl" else "csv"
        self.path = os.path.join(self.output_dir, f"maggrad_continuous_{ts}.{ext}")
        self.rows_written = 0
        self.file = open(self.path, "w", newline="")
        if ext == "csv":
            for key in sorted(self.experiment_metadata):
                self.file.write(f"# {key},{self.experiment_metadata[key]}\n")
            self.writer = csv.writer(self.file)
            self.writer.writerow(self._csv_header())
        rospy.loginfo(f"[MagGradContinuous] Recording started: {self.path}")

    def stop(self):
        path = self.path
        rows = self.rows_written
        if self.file is not None:
            self.file.flush()
            os.fsync(self.file.fileno())
            self.file.close()
        self.file = None
        self.writer = None
        self.last_closed_path = path
        self.last_closed_rows = rows
        self.last_closed_valid = rows >= self.min_valid_rows
        if path and not self.last_closed_valid:
            message = (
                f"[MagGradContinuous] Recording has only {rows} data rows "
                f"(< min_valid_rows={self.min_valid_rows}): {path}"
            )
            if self.delete_invalid:
                try:
                    os.remove(path)
                    rospy.logwarn(f"{message}; deleted invalid file")
                except OSError as exc:
                    rospy.logwarn(f"{message}; failed to delete invalid file: {exc}")
            else:
                rospy.logwarn(f"{message}; keeping invalid file")

    def _csv_header(self):
        header = [
            "experiment_id",
            "sensor_type",
            "board",
            "profile",
            "mag_rate_hz",
            "icm_rate_hz",
            "coil_current_a",
            "fy8300_voltage_v",
            "current_at_5v_a",
            "estimated_current_a",
            "current_mapping",
            "rotation_run_id",
            "firmware_status",
            "warning",
            "ak_bitmap",
            "ak_count",
            "pc_time",
            "ak_seq",
            "ak_tick_ms",
            "ak_topic",
            "coil_state",
            "coil_channel",
            "coil_state_index",
            "coil_state_time",
            "coil_dwell_s",
            "coil_frequency_hz",
            "coil_amplitude_v",
            "coil_offset_v",
        ]
        for sid in range(1, self.n_sensors + 1):
            header.extend([f"sensor_{sid}_x_gs", f"sensor_{sid}_y_gs", f"sensor_{sid}_z_gs"])
        header.extend([
            "imu_raw_seq",
            "imu_raw_tick_ms",
            "imu_raw_ax",
            "imu_raw_ay",
            "imu_raw_az",
            "imu_raw_gx",
            "imu_raw_gy",
            "imu_raw_gz",
            "imu_raw_temp",
            "imu_seq",
            "imu_av_x",
            "imu_av_y",
            "imu_av_z",
            "imu_la_x",
            "imu_la_y",
            "imu_la_z",
        ])
        for frame in self.tf_frames:
            prefix = self._csv_name(frame)
            header.extend([
                f"{prefix}_pos_x",
                f"{prefix}_pos_y",
                f"{prefix}_pos_z",
                f"{prefix}_rot_x",
                f"{prefix}_rot_y",
                f"{prefix}_rot_z",
                f"{prefix}_rot_w",
            ])
        return header

    def _csv_name(self, value):
        return str(value).replace("/", "_").replace(" ", "_")

    def write(self, record):
        if self.file is None:
            return
        if self.record_format == "jsonl":
            self.file.write(json.dumps(record, sort_keys=True) + "\n")
            self.rows_written += 1
            return

        sensors = {int(item["id"]): item for item in record["sensors"]}
        coil = record["coil"]
        imu_raw = record.get("imu_raw") or {}
        imu = record.get("imu") or {}
        meta = record.get("experiment") or {}
        row = [
            meta.get("experiment_id", ""),
            meta.get("sensor_type", ""),
            meta.get("board", ""),
            meta.get("profile", ""),
            meta.get("mag_rate_hz", ""),
            meta.get("icm_rate_hz", ""),
            meta.get("coil_current_a", ""),
            meta.get("fy8300_voltage_v", ""),
            meta.get("current_at_5v_a", ""),
            meta.get("estimated_current_a", ""),
            meta.get("current_mapping", ""),
            meta.get("rotation_run_id", ""),
            meta.get("firmware_status", ""),
            meta.get("warning", ""),
            meta.get("ak_bitmap", ""),
            meta.get("ak_count", ""),
            f"{record['pc_time']:.9f}",
            record["ak_seq"],
            record["ak_tick_ms"],
            record["ak_topic"],
            coil["name"],
            coil["channel"],
            coil["index"],
            f"{coil['state_time']:.9f}",
            coil["dwell_s"],
            coil["frequency_hz"],
            coil["amplitude_v"],
            coil["offset_v"],
        ]
        for sid in range(1, self.n_sensors + 1):
            sensor = sensors.get(sid)
            if sensor is None:
                row.extend(["", "", ""])
            else:
                row.extend([f"{sensor['x']:.9f}", f"{sensor['y']:.9f}", f"{sensor['z']:.9f}"])
        row.extend([
            imu_raw.get("seq", ""),
            imu_raw.get("tick_ms", ""),
            imu_raw.get("ax", ""),
            imu_raw.get("ay", ""),
            imu_raw.get("az", ""),
            imu_raw.get("gx", ""),
            imu_raw.get("gy", ""),
            imu_raw.get("gz", ""),
            imu_raw.get("temp", ""),
            imu.get("seq", ""),
            imu.get("angular_velocity", {}).get("x", ""),
            imu.get("angular_velocity", {}).get("y", ""),
            imu.get("angular_velocity", {}).get("z", ""),
            imu.get("linear_acceleration", {}).get("x", ""),
            imu.get("linear_acceleration", {}).get("y", ""),
            imu.get("linear_acceleration", {}).get("z", ""),
        ])
        tf_data = record.get("tf") or {}
        for frame in self.tf_frames:
            pose = tf_data.get(frame)
            if pose is None:
                row.extend(["", "", "", "", "", "", ""])
                continue
            position = pose.get("position", {})
            rotation = pose.get("rotation", {})
            row.extend([
                position.get("x", ""),
                position.get("y", ""),
                position.get("z", ""),
                rotation.get("x", ""),
                rotation.get("y", ""),
                rotation.get("z", ""),
                rotation.get("w", ""),
            ])
        self.writer.writerow(row)
        self.rows_written += 1


class MagGradContinuousCollectionNode:
    def __init__(self):
        rospy.init_node("maggrad_continuous_collection_node", anonymous=True)

        self.hardware_config_name = resolve_runtime_value(
            rospy.get_param("~hardware_config", "runtime"),
            "hardware_config",
            "qmc6309",
        )
        n_sensors_param = rospy.get_param("~n_sensors", "runtime")
        if resolve_runtime_value(n_sensors_param, "n_sensors", "runtime") == "runtime":
            self.n_sensors = int(get_hardware_config(self.hardware_config_name).magnetometer.n_sensors)
        else:
            self.n_sensors = int(resolve_runtime_value(n_sensors_param, "n_sensors", 12))
        default_output_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "..", "data")
        )
        self.output_dir = rospy.get_param(
            "~output_dir",
            default_output_dir,
        )
        self.record_format = rospy.get_param("~record_format", "csv")
        self.min_valid_rows = int(rospy.get_param("~min_valid_rows", 100))
        self.delete_invalid_recordings = self._param_bool(rospy.get_param("~delete_invalid_recordings", True))
        self.use_calibrated = self._param_bool(rospy.get_param("~use_calibrated", False))
        self.ak_topic = "stm_uplink" if self.use_calibrated else "stm_uplink_raw"
        self.record_tf = self._param_bool(rospy.get_param("~record_tf", True))
        self.tf_reference_frame = rospy.get_param("~tf_reference_frame", "lab_table")
        self.tf_frames = list(rospy.get_param("~tf_frames", [
            "sensor_array_filt",
            "diana7_em_tcp_filt",
            "arm1_em_tcp_filt",
            "arm2_em_tcp_filt",
        ]))
        self.coil_frequency_hz = float(rospy.get_param("~coil_frequency_hz", 1.0))
        self.experiment_metadata = self._load_experiment_metadata()
        if self.coil_frequency_hz <= 0:
            raise ValueError("coil_frequency_hz must be positive")
        self.coil_period_s = 1.0 / self.coil_frequency_hz
        self.coil_timing = self._load_coil_timing()

        self.lock = threading.Lock()
        self.recording = False
        self.record_start_time = None
        self.latest_imu_raw = None
        self.latest_imu = None
        self.latest_firmware_status = {}

        self.tf_buffer = None
        self.tf_listener = None
        self._last_tf_warn = {}
        self._init_tf()

        self.recorder = ContinuousRecorder(
            self.output_dir,
            self.n_sensors,
            self.record_format,
            self.tf_frames if self.record_tf else [],
            self.experiment_metadata,
            self.min_valid_rows,
            self.delete_invalid_recordings,
        )

        self.pub_coil_state = rospy.Publisher("maggrad/coil_state", String, queue_size=10, latch=True)
        self.pub_record_status = rospy.Publisher("maggrad/continuous_record/status", String, queue_size=10, latch=True)

        self.sub_trigger = rospy.Subscriber("~record_trigger", Bool, self._on_record_trigger)
        self.sub_experiment_metadata = rospy.Subscriber(
            "~experiment_metadata",
            String,
            self._on_experiment_metadata,
        )
        self.sub_ak = rospy.Subscriber(self.ak_topic, StmUplink, self._on_ak)
        self.sub_imu_raw = rospy.Subscriber("maggrad/imu_raw", MagGradImuRaw, self._on_imu_raw)
        self.sub_imu = rospy.Subscriber("maggrad/imu", Imu, self._on_imu)
        self.sub_firmware_status = rospy.Subscriber("maggrad/firmware/status", String, self._on_firmware_status)
        self.timer = rospy.Timer(rospy.Duration(0.02), self._on_timer)
        rospy.on_shutdown(self._on_shutdown)

        self._publish_coil_state(self._all_off_state(rospy.Time.now().to_sec()))
        self._publish_status()
        rospy.loginfo(
            f"MagGrad continuous collection initialized: ak_topic={self.ak_topic}, "
            f"coil_frequency={self.coil_frequency_hz}Hz, record_tf={self.record_tf}, "
            f"experiment_id={self.experiment_metadata.get('experiment_id', '')}, "
            f"rotation_run_id={self.experiment_metadata.get('rotation_run_id', '')}, "
            f"output_dir={os.path.expanduser(self.output_dir)}"
        )

    def _param_bool(self, value):
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _load_coil_timing(self):
        raw = rospy.get_param("~coil_timing", [])
        if not raw:
            raw = [
                {"name": "coil_1_on", "channel": 1, "phase_deg": 0.0, "duty_cycle": 33.0},
                {"name": "coil_2_on", "channel": 2, "phase_deg": 120.0, "duty_cycle": 33.0},
                {"name": "coil_3_on", "channel": 3, "phase_deg": 240.0, "duty_cycle": 33.0},
            ]

        default_amplitude = float(rospy.get_param("~default_amplitude_v", 5.0))
        default_offset = float(rospy.get_param("~default_offset_v", 2.5))
        timing = []
        for index, item in enumerate(raw):
            channel = int(item.get("channel", 0))
            if channel < 1 or channel > 3:
                raise ValueError(f"Invalid coil channel at index {index}: {channel}")
            duty_cycle = float(item.get("duty_cycle", 0.0))
            if duty_cycle <= 0.0 or duty_cycle > 100.0:
                raise ValueError(f"Invalid duty_cycle at index {index}: {duty_cycle}")
            phase_s = (float(item.get("phase_deg", 0.0)) % 360.0) / 360.0 * self.coil_period_s
            active_s = duty_cycle / 100.0 * self.coil_period_s
            timing.append({
                "index": index,
                "name": str(item.get("name", f"state_{index}")),
                "channel": channel,
                "phase_deg": float(item.get("phase_deg", 0.0)) % 360.0,
                "phase_s": phase_s,
                "duty_cycle": duty_cycle,
                "dwell_s": active_s,
                "frequency_hz": self.coil_frequency_hz,
                "amplitude_v": float(item.get("amplitude_v", default_amplitude)),
                "offset_v": float(item.get("offset_v", default_offset)),
            })
        return timing

    def _load_experiment_metadata(self):
        return {
            "experiment_id": rospy.get_param("~experiment_id", ""),
            "sensor_type": rospy.get_param("~sensor_type", "AK09973D" if self.hardware_config_name == "ak09973d" else ""),
            "board": rospy.get_param("~board", "MagGrad_AK_TMAG_V1" if self.hardware_config_name == "ak09973d" else ""),
            "profile": rospy.get_param("~profile", ""),
            "mag_rate_hz": rospy.get_param("~mag_rate_hz", ""),
            "icm_rate_hz": rospy.get_param("~icm_rate_hz", ""),
            "coil_current_a": rospy.get_param("~coil_current_a", ""),
            "fy8300_voltage_v": rospy.get_param("~fy8300_voltage_v", ""),
            "current_at_5v_a": rospy.get_param("~current_at_5v_a", ""),
            "estimated_current_a": rospy.get_param("~estimated_current_a", ""),
            "current_mapping": rospy.get_param("~current_mapping", ""),
            "coil_channel": rospy.get_param("~coil_channel", ""),
            "rotation_run_id": rospy.get_param("~rotation_run_id", ""),
            "firmware_status": rospy.get_param("~firmware_status", ""),
            "warning": rospy.get_param("~warning", "NONE"),
            "ak_bitmap": rospy.get_param("~ak_bitmap", ""),
            "ak_count": rospy.get_param("~ak_count", ""),
        }

    def _init_tf(self):
        if not self.record_tf:
            return
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        rospy.loginfo(
            f"[MagGradContinuous] TF recording enabled: reference={self.tf_reference_frame}, "
            f"frames={self.tf_frames}"
        )

    def _state_payload(self, state, state_time):
        payload = dict(state)
        payload["state_time"] = state_time
        payload["recording"] = self.recording
        return payload

    def _all_off_state(self, state_time):
        return {
            "index": -1,
            "name": "all_off",
            "channel": 0,
            "phase_deg": None,
            "phase_s": None,
            "duty_cycle": 0.0,
            "dwell_s": 0.0,
            "frequency_hz": self.coil_frequency_hz,
            "amplitude_v": 0.0,
            "offset_v": 0.0,
            "state_time": state_time,
            "recording": self.recording,
        }

    def _state_for_time(self, pc_time):
        if self.record_start_time is None:
            return self._all_off_state(pc_time)
        elapsed = max(0.0, pc_time - self.record_start_time.to_sec())
        cycle_pos = elapsed % self.coil_period_s
        for state in self.coil_timing:
            start = state["phase_s"]
            end = start + state["dwell_s"]
            active = start <= cycle_pos < end
            if end > self.coil_period_s:
                active = cycle_pos >= start or cycle_pos < (end - self.coil_period_s)
            if active:
                cycle_start = pc_time - cycle_pos
                return self._state_payload(state, cycle_start + start)
        return self._all_off_state(pc_time - cycle_pos)

    def _publish_coil_state(self, state):
        self.pub_coil_state.publish(String(data=json.dumps(state, sort_keys=True)))

    def _publish_status(self):
        now = rospy.Time.now().to_sec()
        status = {
            "recording": self.recording,
            "path": self.recorder.path,
            "rows_written": self.recorder.rows_written,
            "last_closed_path": self.recorder.last_closed_path,
            "last_closed_rows": self.recorder.last_closed_rows,
            "last_closed_valid": self.recorder.last_closed_valid,
            "coil_state": self._state_for_time(now),
        }
        self.pub_record_status.publish(String(data=json.dumps(status, sort_keys=True)))

    def _on_record_trigger(self, msg):
        with self.lock:
            enable = bool(msg.data)
            if enable and not self.recording:
                self.recorder.start()
                self.recording = True
                self.record_start_time = rospy.Time.now()
                self._publish_coil_state(self._state_for_time(self.record_start_time.to_sec()))
            elif not enable and self.recording:
                self.recording = False
                self.recorder.stop()
                self._publish_coil_state(self._all_off_state(rospy.Time.now().to_sec()))
            self._publish_status()

    def _on_experiment_metadata(self, msg):
        updates = self._parse_metadata_update(msg.data)
        if not updates:
            rospy.logwarn(f"[MagGradContinuous] Ignoring empty experiment metadata update: {msg.data}")
            return
        with self.lock:
            self.experiment_metadata.update(updates)
            self.recorder.experiment_metadata.update(updates)
        rospy.loginfo(f"[MagGradContinuous] Updated experiment metadata: {updates}")

    def _parse_metadata_update(self, text):
        text = str(text).strip()
        if not text:
            return {}
        if text.startswith("{"):
            try:
                parsed = json.loads(text)
            except ValueError as exc:
                rospy.logwarn(f"[MagGradContinuous] Invalid metadata JSON: {exc}")
                return {}
            return {str(key): str(value) for key, value in parsed.items()}
        updates = {}
        for token in text.replace(",", " ").split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            updates[key.strip()] = value.strip()
        return updates

    def _on_timer(self, _event):
        with self.lock:
            if not self.recording:
                return
            self._publish_coil_state(self._state_for_time(rospy.Time.now().to_sec()))
            self._publish_status()

    def _on_imu_raw(self, msg):
        with self.lock:
            self.latest_imu_raw = msg

    def _on_imu(self, msg):
        with self.lock:
            self.latest_imu = msg

    def _on_firmware_status(self, msg):
        parsed = self._parse_kv_status(msg.data)
        if not parsed:
            return
        with self.lock:
            self.latest_firmware_status = parsed

    def _parse_kv_status(self, line):
        result = {}
        for token in str(line).strip().split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            result[key.strip()] = value.strip()
        return result

    def _runtime_experiment_metadata(self):
        meta = dict(self.experiment_metadata)
        status = dict(self.latest_firmware_status)
        if status:
            meta["firmware_status"] = " ".join(f"{key}={status[key]}" for key in sorted(status))
            meta["warning"] = status.get("warning", meta.get("warning", "NONE"))
            meta["ak_bitmap"] = status.get("ak_bitmap", status.get("bitmap", meta.get("ak_bitmap", "")))
            meta["ak_count"] = status.get("ak_count", meta.get("ak_count", ""))
            meta["profile"] = status.get("profile", status.get("profile_id", meta.get("profile", "")))
            meta["mag_rate_hz"] = status.get("target_hz", meta.get("mag_rate_hz", ""))
        return meta

    def _on_ak(self, msg):
        pc_time = rospy.Time.now().to_sec()
        with self.lock:
            if not self.recording:
                return
            state = self._state_for_time(pc_time)
            imu_raw = self._imu_raw_to_dict(self.latest_imu_raw)
            imu = self._imu_to_dict(self.latest_imu)
            experiment = self._runtime_experiment_metadata()
            if experiment.get("coil_channel") not in ("", None):
                state = dict(state)
                state["channel"] = experiment["coil_channel"]
                state["name"] = f"manual_coil_{experiment['coil_channel']}"
        tf_data = self._lookup_tf_frames()

        record = {
            "pc_time": pc_time,
            "ak_seq": msg.header.seq,
            "ak_tick_ms": msg.timestamp,
            "ak_topic": self.ak_topic,
            "experiment": experiment,
            "coil": state,
            "sensors": [
                {"id": int(s.id), "x": float(s.x), "y": float(s.y), "z": float(s.z)}
                for s in msg.sensor_data
            ],
            "imu_raw": imu_raw,
            "imu": imu,
            "tf": tf_data,
        }
        self.recorder.write(record)

    def _lookup_tf_frames(self):
        if not self.record_tf or self.tf_buffer is None:
            return {}
        result = {}
        for frame in self.tf_frames:
            result[frame] = self._lookup_tf(frame)
        return result

    def _lookup_tf(self, frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.tf_reference_frame,
                frame,
                rospy.Time(0),
                rospy.Duration(0.001),
            )
            tr = transform.transform.translation
            rot = transform.transform.rotation
            return {
                "reference_frame": self.tf_reference_frame,
                "position": {"x": tr.x, "y": tr.y, "z": tr.z},
                "rotation": {"x": rot.x, "y": rot.y, "z": rot.z, "w": rot.w},
            }
        except Exception as exc:
            now = rospy.Time.now().to_sec()
            last = self._last_tf_warn.get(frame, 0.0)
            if now - last > 5.0:
                rospy.logwarn(f"[MagGradContinuous] TF lookup failed: {self.tf_reference_frame}->{frame}: {exc}")
                self._last_tf_warn[frame] = now
            return None

    def _imu_raw_to_dict(self, msg):
        if msg is None:
            return None
        return {
            "seq": int(msg.seq),
            "tick_ms": int(msg.tick_ms),
            "ax": int(msg.ax),
            "ay": int(msg.ay),
            "az": int(msg.az),
            "gx": int(msg.gx),
            "gy": int(msg.gy),
            "gz": int(msg.gz),
            "temp": int(msg.temp),
        }

    def _imu_to_dict(self, msg):
        if msg is None:
            return None
        return {
            "seq": int(msg.header.seq),
            "angular_velocity": {
                "x": float(msg.angular_velocity.x),
                "y": float(msg.angular_velocity.y),
                "z": float(msg.angular_velocity.z),
            },
            "linear_acceleration": {
                "x": float(msg.linear_acceleration.x),
                "y": float(msg.linear_acceleration.y),
                "z": float(msg.linear_acceleration.z),
            },
        }

    def _on_shutdown(self):
        with self.lock:
            self.recording = False
            self.recorder.stop()
            self._publish_coil_state(self._all_off_state(rospy.Time.now().to_sec()))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        MagGradContinuousCollectionNode().run()
    except rospy.ROSInterruptException:
        pass
