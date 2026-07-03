#!/usr/bin/env python3
"""Coordinate MI-GELS experiment startup before triggering robot motion."""

import json
import math
import os
import time

import rospy
import yaml
from sensor_msgs.msg import CameraInfo, Image
from signal_generator.msg import ChannelStatus
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError("YAML file must contain a mapping: {}".format(path))
    return data


def _expected_signal_channels(config):
    channels = {}
    for item in config.get("initial_configs", []):
        channel = int(item.get("channel_index", 0))
        if channel <= 0:
            continue
        channels[channel] = {
            "waveform": int(item.get("waveform", 0)),
            "frequency": float(item.get("frequency", 0.0)),
            "amplitude": float(item.get("amplitude", 0.0)),
            "offset": float(item.get("offset", 0.0)),
            "phase": float(item.get("phase", 0.0)),
            "duty_cycle": float(item.get("duty_cycle", 0.0)),
            "output_enabled": bool(item.get("output_en", False)),
        }
    return channels


def _runtime_float(value, config, key, default):
    if isinstance(value, str) and value.strip().lower() == "runtime":
        value = config.get(key, default)
    return float(value)


def _param_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _bytes_from_image_data(data):
    if isinstance(data, bytes):
        return data
    if isinstance(data, bytearray):
        return bytes(data)
    return bytes(bytearray(data))


def _row_slice(data, offset, width):
    return data[offset:offset + width]


def save_ros_image(image, path_stem):
    encoding = str(getattr(image, "encoding", "")).lower()
    width = int(image.width)
    height = int(image.height)
    step = int(getattr(image, "step", 0))
    data = _bytes_from_image_data(image.data)
    if width <= 0 or height <= 0:
        raise ValueError("Image has invalid size: {}x{}".format(width, height))

    if encoding in ("mono8", "8uc1"):
        row_width = width
        step = step or row_width
        payload = b"".join(_row_slice(data, row * step, row_width) for row in range(height))
        path = path_stem + ".pgm"
        with open(path, "wb") as stream:
            stream.write("P5\n{} {}\n255\n".format(width, height).encode("ascii"))
            stream.write(payload)
        return path

    if encoding in ("rgb8", "bgr8"):
        row_width = width * 3
        step = step or row_width
        rows = []
        for row in range(height):
            row_data = bytearray(_row_slice(data, row * step, row_width))
            if encoding == "bgr8":
                for index in range(0, len(row_data), 3):
                    row_data[index], row_data[index + 2] = row_data[index + 2], row_data[index]
            rows.append(bytes(row_data))
        path = path_stem + ".ppm"
        with open(path, "wb") as stream:
            stream.write("P6\n{} {}\n255\n".format(width, height).encode("ascii"))
            stream.write(b"".join(rows))
        return path

    raise ValueError("Unsupported image encoding for experiment photo: {}".format(image.encoding))


def _close(actual, expected, tolerance):
    return math.isclose(float(actual), float(expected), abs_tol=float(tolerance))


def channel_status_matches(status, expected, tolerances):
    if not status.valid:
        return False, "status.valid is false"
    if int(status.waveform) != int(expected["waveform"]):
        return False, "waveform={} expected={}".format(status.waveform, expected["waveform"])
    if bool(status.output_enabled) != bool(expected["output_enabled"]):
        return False, "output_enabled={} expected={}".format(status.output_enabled, expected["output_enabled"])
    for field in ("frequency", "amplitude", "offset", "phase", "duty_cycle"):
        if not _close(getattr(status, field), expected[field], tolerances[field]):
            return False, "{}={} expected={} tol={}".format(
                field,
                getattr(status, field),
                expected[field],
                tolerances[field],
            )
    return True, "ready"


def _status_summary(status):
    return (
        "waveform={waveform} frequency={frequency:.3f}Hz amplitude={amplitude:.3f}V "
        "offset={offset:.3f}V phase={phase:.3f}deg duty={duty:.3f}% output={output}"
    ).format(
        waveform=int(status.waveform),
        frequency=float(status.frequency),
        amplitude=float(status.amplitude),
        offset=float(status.offset),
        phase=float(status.phase),
        duty=float(status.duty_cycle),
        output=bool(status.output_enabled),
    )


class ExperimentOrchestrator:
    def __init__(self):
        rospy.init_node("mi_gels_experiment_orchestrator")
        self.camera_image_topic = rospy.get_param("~camera_image_topic", "/zed2i/zed_node/left/image_rect_gray")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/zed2i/zed_node/left/camera_info")
        self.signal_config = rospy.get_param("~signal_config")
        self.motion_service = rospy.get_param("~motion_service", "/mi_gels_motion/run_experiment")
        self.prepare_start_service = rospy.get_param("~prepare_start_service", "/mi_gels_motion/prepare_start")
        self.run_trajectory_service = rospy.get_param("~run_trajectory_service", "/mi_gels_motion/run_trajectory")
        self.record_trigger_topic = rospy.get_param("~record_trigger_topic", "/maggrad_continuous_collection/record_trigger")
        self.record_status_topic = rospy.get_param("~record_status_topic", "/maggrad/continuous_record/status")
        self.camera_snapshot_topic = rospy.get_param("~camera_snapshot_topic", self.camera_image_topic)
        self.capture_experiment_photos = _param_bool(rospy.get_param("~capture_experiment_photos", True))
        self.photo_before_motion_name = rospy.get_param("~photo_before_motion_name", "before_motion")
        self.photo_after_motion_name = rospy.get_param("~photo_after_motion_name", "after_motion")
        self.timeout_s = float(rospy.get_param("~timeout_s", 120.0))
        self.settle_time_s = float(rospy.get_param("~settle_time_s", 2.0))
        self.initial_settle_time_s = float(rospy.get_param("~initial_settle_time_s", self.settle_time_s))
        self.pre_motion_cycles = int(rospy.get_param("~pre_motion_cycles", 5))
        coil_frequency_param = rospy.get_param("~coil_frequency_hz", "runtime")
        self.frequency_tolerance = float(rospy.get_param("~frequency_tolerance", 0.01))
        self.voltage_tolerance = float(rospy.get_param("~voltage_tolerance", 0.05))
        self.phase_tolerance = float(rospy.get_param("~phase_tolerance", 0.5))
        self.duty_tolerance = float(rospy.get_param("~duty_tolerance", 0.5))
        self.signal_status_prefix = rospy.get_param("~signal_status_prefix", "/fy8300")
        self.stage_prefix = rospy.get_param("~stage_prefix", "[MI-GELS]")

        self.signal_config_data = _load_yaml(self.signal_config)
        self.signal_expected = _expected_signal_channels(self.signal_config_data)
        if not self.signal_expected:
            raise ValueError("No initial_configs found in signal config: {}".format(self.signal_config))
        self.coil_frequency_hz = _runtime_float(
            coil_frequency_param,
            self.signal_config_data,
            "coil_frequency_hz",
            1.0,
        )
        self.tolerances = {
            "frequency": self.frequency_tolerance,
            "amplitude": self.voltage_tolerance,
            "offset": self.voltage_tolerance,
            "phase": self.phase_tolerance,
            "duty_cycle": self.duty_tolerance,
        }
        if self.pre_motion_cycles < 0:
            raise ValueError("pre_motion_cycles must be non-negative")
        if self.coil_frequency_hz <= 0.0:
            raise ValueError("coil_frequency_hz must be positive")
        self.record_trigger_pub = rospy.Publisher(self.record_trigger_topic, Bool, queue_size=1, latch=True)
        self.signal_output_pubs = {
            channel: rospy.Publisher(
                "{}/ch{}/output_en".format(self.signal_status_prefix.rstrip("/"), channel),
                Bool,
                queue_size=1,
                latch=True,
            )
            for channel in self.signal_expected
        }

    def stage(self, number, message):
        return "{}[{}/6] {}".format(getattr(self, "stage_prefix", "[MI-GELS]"), number, message)

    def _remaining_timeout(self, deadline):
        remaining = deadline - time.time()
        if remaining <= 0:
            raise RuntimeError("Timed out waiting for experiment prerequisites")
        return remaining

    def wait_for_camera(self, deadline):
        rospy.loginfo("%s: %s", self.stage(1, "Waiting for camera info"), self.camera_info_topic)
        camera_info = rospy.wait_for_message(
            self.camera_info_topic,
            CameraInfo,
            timeout=self._remaining_timeout(deadline),
        )
        if camera_info.width <= 0 or camera_info.height <= 0:
            raise RuntimeError("CameraInfo has invalid resolution: {}x{}".format(camera_info.width, camera_info.height))

        rospy.loginfo("%s: %s", self.stage(2, "Waiting for camera image"), self.camera_image_topic)
        image = rospy.wait_for_message(
            self.camera_image_topic,
            Image,
            timeout=self._remaining_timeout(deadline),
        )
        if image.width <= 0 or image.height <= 0 or not image.data:
            raise RuntimeError("Camera image is invalid or empty")
        rospy.loginfo(
            "%s: image=%dx%d camera_info=%dx%d",
            self.stage(2, "Camera ready"),
            image.width,
            image.height,
            camera_info.width,
            camera_info.height,
        )

    def wait_for_signal_generator(self, deadline, output_enabled=None):
        rospy.loginfo("%s", self.stage(3, "Waiting for FY8300 channel status"))
        for channel in sorted(self.signal_expected):
            topic = "{}/ch{}/status".format(self.signal_status_prefix.rstrip("/"), channel)
            expected = dict(self.signal_expected[channel])
            if output_enabled is not None:
                expected["output_enabled"] = bool(output_enabled)
            last_error = "no status received"
            rospy.loginfo("%s: channel=%d topic=%s", self.stage(3, "Waiting for FY8300 status"), channel, topic)
            while not rospy.is_shutdown():
                try:
                    status = rospy.wait_for_message(topic, ChannelStatus, timeout=self._remaining_timeout(deadline))
                except (rospy.ROSException, RuntimeError) as exc:
                    raise RuntimeError(
                        "Timed out waiting for FY8300 channel {} ready on {}; last mismatch: {}".format(
                            channel,
                            topic,
                            last_error,
                        )
                    ) from exc
                matches, reason = channel_status_matches(status, expected, self.tolerances)
                if matches:
                    rospy.loginfo("%s: channel=%d %s", self.stage(3, "FY8300 channel ready"), channel, _status_summary(status))
                    break
                last_error = reason
                rospy.logwarn("%s: channel=%d mismatch=%s", self.stage(3, "FY8300 channel not ready yet"), channel, reason)
            else:
                raise RuntimeError("Shutdown while waiting for FY8300 channel {}".format(channel))
            if last_error != "no status received":
                rospy.logdebug("FY8300 channel %d previous status mismatch: %s", channel, last_error)

    def set_signal_outputs(self, enabled):
        rospy.loginfo("%s: output_enabled=%s", self.stage(5, "Setting FY8300 outputs"), enabled)
        for channel in sorted(self.signal_output_pubs):
            self.signal_output_pubs[channel].publish(Bool(data=bool(enabled)))

    def trigger_motion(self, deadline):
        rospy.loginfo("%s: %s", self.stage(4, "Waiting for motion service"), self.motion_service)
        rospy.wait_for_service(self.motion_service, timeout=self._remaining_timeout(deadline))
        if self.settle_time_s > 0.0:
            rospy.loginfo(
                "%s: All prerequisites ready. Starting robot motion in %.2f seconds.",
                self.stage(5, "Starting robot motion"),
                self.settle_time_s,
            )
            rospy.sleep(self.settle_time_s)
        else:
            rospy.loginfo("%s: All prerequisites ready. Starting robot motion now.", self.stage(5, "Starting robot motion"))
        proxy = rospy.ServiceProxy(self.motion_service, Trigger)
        response = proxy()
        if not response.success:
            raise RuntimeError("Motion service failed: {}".format(response.message))
        rospy.loginfo("%s: %s", self.stage(5, "Motion service completed"), response.message)

    def wait_for_recording_node(self, deadline):
        rospy.loginfo("%s: %s", self.stage(4, "Waiting for recording status"), self.record_status_topic)
        rospy.wait_for_message(
            self.record_status_topic,
            String,
            timeout=self._remaining_timeout(deadline),
        )

    def wait_for_recording_path(self, deadline):
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message(
                self.record_status_topic,
                String,
                timeout=self._remaining_timeout(deadline),
            )
            try:
                status = json.loads(msg.data)
            except ValueError:
                rospy.logwarn("%s: invalid recording status JSON: %s", self.stage(5, "Waiting for recording path"), msg.data)
                continue
            path = status.get("path")
            if status.get("recording") and path:
                return os.path.dirname(os.path.expanduser(path))
        raise RuntimeError("Shutdown while waiting for recording path")

    def call_trigger_service(self, service_name, deadline, label):
        rospy.loginfo("%s: %s", self.stage(4, "Waiting for {}".format(label)), service_name)
        rospy.wait_for_service(service_name, timeout=self._remaining_timeout(deadline))
        proxy = rospy.ServiceProxy(service_name, Trigger)
        response = proxy()
        if not response.success:
            raise RuntimeError("{} failed: {}".format(label, response.message))
        rospy.loginfo("%s: %s", self.stage(4, "{} completed".format(label)), response.message)

    def set_recording(self, enabled):
        rospy.loginfo("%s: recording=%s topic=%s", self.stage(5, "Setting data recording"), enabled, self.record_trigger_topic)
        self.record_trigger_pub.publish(Bool(data=bool(enabled)))

    def pre_motion_wait_s(self):
        return float(self.pre_motion_cycles) / self.coil_frequency_hz

    def capture_experiment_photo(self, directory, name, deadline):
        if not self.capture_experiment_photos:
            return None
        if not directory:
            raise RuntimeError("Cannot capture experiment photo without a recording directory")
        os.makedirs(directory, exist_ok=True)
        image = rospy.wait_for_message(
            self.camera_snapshot_topic,
            Image,
            timeout=self._remaining_timeout(deadline),
        )
        path = save_ros_image(image, os.path.join(directory, name))
        rospy.loginfo("%s: %s", self.stage(5, "Saved experiment photo"), path)
        return path

    def run_experiment_sequence(self, deadline):
        recording_started = False
        signal_enabled = False
        recording_dir = None
        try:
            self.call_trigger_service(self.prepare_start_service, deadline, "prepare_start")
            settle_deadline = deadline
            if self.initial_settle_time_s > 0.0:
                settle_deadline = min(deadline, time.time() + self.initial_settle_time_s)
            self.set_signal_outputs(True)
            signal_enabled = True
            self.wait_for_signal_generator(settle_deadline, output_enabled=True)
            if self.initial_settle_time_s > 0.0:
                remaining_settle_s = max(0.0, settle_deadline - time.time())
                rospy.loginfo(
                    "%s: waiting %.2fs before recording",
                    self.stage(5, "Initial pose settling"),
                    remaining_settle_s,
                )
                rospy.sleep(remaining_settle_s)
            self.set_recording(True)
            recording_started = True
            if self.capture_experiment_photos:
                recording_dir = self.wait_for_recording_path(deadline)
            wait_s = self.pre_motion_wait_s()
            if wait_s > 0.0:
                rospy.loginfo(
                    "%s: cycles=%d coil_frequency=%.3fHz wait=%.2fs",
                    self.stage(5, "Pre-motion data collection"),
                    self.pre_motion_cycles,
                    self.coil_frequency_hz,
                    wait_s,
                )
                rospy.sleep(wait_s)
            self.capture_experiment_photo(recording_dir, self.photo_before_motion_name, deadline)
            self.call_trigger_service(self.run_trajectory_service, deadline, "run_trajectory")
            self.capture_experiment_photo(recording_dir, self.photo_after_motion_name, deadline)
        finally:
            if recording_started:
                self.set_recording(False)
            if signal_enabled:
                self.set_signal_outputs(False)

    def run(self):
        deadline = time.time() + self.timeout_s
        current_stage = "initialization"
        try:
            current_stage = "camera"
            self.wait_for_camera(deadline)
            current_stage = "data recording"
            self.wait_for_recording_node(deadline)
            current_stage = "experiment sequence"
            self.run_experiment_sequence(deadline)
            rospy.loginfo("%s", self.stage(6, "Experiment complete"))
        except Exception as exc:
            rospy.logerr("%s failed during %s: %s", self.stage(6, "Experiment failed"), current_stage, exc)
            raise


if __name__ == "__main__":
    try:
        ExperimentOrchestrator().run()
    except rospy.ROSInterruptException:
        pass
