#!/usr/bin/env python3

import csv
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleOdometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


from .state_comparison import SIM_CLOCK_TOLERANCE_S, Sample, SamplePairs, world_to_geodetic, geodetic_to_px4_ned


CompareSpec = tuple[str, str, str, str, str]

STATE_COMPARE_SPECS: tuple[CompareSpec, ...] = (
    (
        "position",
        "px4_position_ned",
        "truth_position_ned",
        "position_error_ned",
        "px4_local_ned",
    ),
    (
        "velocity",
        "px4_velocity_ned",
        "truth_velocity_ned",
        "velocity_error_ned",
        "px4_local_ned",
    ),
    (
        "acceleration",
        "px4_acceleration_ned",
        "truth_acceleration_ned",
        "acceleration_error_ned",
        "px4_local_ned",
    ),
    (
        "rpy",
        "px4_rpy_ned_frd",
        "truth_rpy_ned_frd",
        "rpy_error_ned_frd",
        "px4_local_ned_body_frd",
    ),
    (
        "angular_velocity",
        "px4_angular_velocity_body_frd",
        "truth_angular_velocity_body_frd",
        "angular_velocity_error_body_frd",
        "body_frd",
    ),
)
STATE_COMPARE_BY_KEY: dict[str, CompareSpec] = {spec[0]: spec for spec in STATE_COMPARE_SPECS}
STATE_COMPARE_TOPICS: tuple[str, ...] = tuple(
    topic
    for _, px4_topic, truth_topic, error_topic, _ in STATE_COMPARE_SPECS
    for topic in (px4_topic, truth_topic, error_topic)
)


class TrajectoryLogger(Node):
    """Write trajectory CSV files and optional online state comparison topics."""

    def __init__(self) -> None:
        super().__init__("trajectory_logger")

        self.declare_parameter("config_file", "")
        self.declare_parameter("log_root", "")
        self.declare_parameter("run_id", "")
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("vehicle_attitude_topic", "/fmu/out/vehicle_attitude")
        self.declare_parameter("vehicle_odometry_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("gazebo_odometry_topic", "/model/x500_0/odometry_with_covariance")
        self.declare_parameter("publish_state_compare_topics", True)
        self.declare_parameter("state_compare_topic_prefix", "state_compare")
        self.declare_parameter("control_diagnostics_topic", "")

        self.latest_attitude: VehicleAttitude | None = None
        self.latest_px4_odometry: VehicleOdometry | None = None
        self._configure_comparison()
        self.last_truth_time_s: float | None = None
        self.last_truth_velocity: tuple[float, float, float] | None = None
        self.ros_start_time_s = self._ros_now_s()
        self.first_px4_timestamp_us: int | None = None
        self.first_gazebo_time_s: float | None = None

        self.log_dir = self._make_log_dir()
        metadata = {
            "schema_version": 2,
            "comparison_time_domain": "gazebo_sim",
            "reference_point": "px4_body_origin",
            "world_origin_lat_lon_alt": list(self.world_origin),
            "truth_body_offset_flu_m": list(self.body_offset),
            **{name: self.get_parameter(name).value for name in (
                "comparison_max_gap_s", "comparison_max_age_s",
                "vehicle_local_position_topic", "vehicle_attitude_topic", "vehicle_odometry_topic",
                "gazebo_odometry_topic", "state_compare_topic_prefix")},
        }
        (self.log_dir / "alignment_config.yaml").write_text(
            yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
        self.px4_file, self.px4_writer = self._open_writer("px4_estimate.csv", px4_fieldnames())
        self.truth_file, self.truth_writer = self._open_writer("gazebo_truth.csv", truth_fieldnames())
        self.compare_file, self.compare_writer = self._open_writer(
            "state_comparison.csv", comparison_fieldnames())
        self.control_file = None
        control_topic = str(self.get_parameter("control_diagnostics_topic").value)
        if control_topic:
            self.create_subscription(DiagnosticArray, control_topic,
                                     self._control_diagnostics_callback, 10)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        local_position_topic = self.get_parameter("vehicle_local_position_topic").value
        attitude_topic = self.get_parameter("vehicle_attitude_topic").value
        px4_odometry_topic = self.get_parameter("vehicle_odometry_topic").value
        gazebo_odometry_topic = self.get_parameter("gazebo_odometry_topic").value
        self.publish_state_compare_topics = bool(
            self.get_parameter("publish_state_compare_topics").value
        )
        self.state_compare_topic_prefix = normalize_topic_prefix(
            str(self.get_parameter("state_compare_topic_prefix").value)
        )

        self.create_subscription(
            VehicleLocalPosition,
            str(local_position_topic),
            self._vehicle_local_position_callback,
            qos_profile,
        )
        self.create_subscription(
            VehicleAttitude,
            str(attitude_topic),
            self._vehicle_attitude_callback,
            qos_profile,
        )
        self.create_subscription(
            VehicleOdometry,
            str(px4_odometry_topic),
            self._vehicle_odometry_callback,
            qos_profile,
        )
        self.create_subscription(
            Odometry,
            str(gazebo_odometry_topic),
            self._gazebo_odometry_callback,
            qos_profile,
        )

        self.state_compare_publishers = self._make_state_compare_publishers()
        self.comparison_status_pub = self.create_publisher(
            DiagnosticArray, f"{self.state_compare_topic_prefix}/status", 10)
        self.status_timer = self.create_timer(
            0.5, self._publish_comparison_status, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(f"Logging trajectories to {self.log_dir}")
        self.get_logger().info(
            "PX4 estimate: "
            f"{local_position_topic}, {attitude_topic}, {px4_odometry_topic}; "
            f"Gazebo truth: {gazebo_odometry_topic}"
        )
        if self.publish_state_compare_topics:
            self.get_logger().info(
                f"Publishing online state comparison topics under {self.state_compare_topic_prefix}"
            )

    def _control_diagnostics_callback(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != "visual_pursuit_interceptor":
                continue
            row = {"sample_sim_time_s": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                   **{item.key: item.value for item in status.values}}
            if self.control_file is None:
                self.control_file, self.control_writer = self._open_writer(
                    "visual_control.csv", list(row))
            self.control_writer.writerow(row)

    def _configure_comparison(self) -> None:
        config_path = str(self.get_parameter("config_file").value)
        if not config_path:
            config_path = str(Path(get_package_share_directory("uav_trajectory_tracking"))
                              / "config" / "trajectory_logging.yaml")
        config = yaml.safe_load(Path(config_path).expanduser().read_text())
        for name in ("world_origin_lat_lon_alt", "truth_body_offset_flu_m",
                     "comparison_max_gap_s", "comparison_max_age_s"):
            self.declare_parameter(name, config[name])
        self.world_origin = tuple(self.get_parameter("world_origin_lat_lon_alt").value)
        self.body_offset = tuple(self.get_parameter("truth_body_offset_flu_m").value)
        for value in (self.world_origin, self.body_offset):
            if len(value) != 3 or not all(math.isfinite(x) for x in value):
                raise ValueError("World origin and body offset must be finite three-vectors.")
        if not -90 < self.world_origin[0] < 90 or not -180 <= self.world_origin[1] <= 180:
            raise ValueError("Invalid world latitude/longitude.")
        for name in ("comparison_max_gap_s", "comparison_max_age_s"):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive.")
            setattr(self, name, value)
        self.pairs = SamplePairs(self.comparison_max_gap_s, self.comparison_max_age_s)
        self.reference: tuple[float, float, float] | None = None
        self.reference_key = None
        self.last_px4_message_times = {}
        self.last_pair_by_key = {}
        self.local_reset_counters = None
        self.attitude_reset_counter = None
        self.odometry_reset_counter = None
        self.last_compare_truth_time_s: float | None = None
        self.last_body_velocity = None
        self.last_body_velocity_time_s = None
        self.last_pair_received_s: float | None = None
        self.pair_count = 0
        self.rejected_count = 0
        self.comparison_epoch = 0
        self.last_reset_reason = "startup"

    def _monotonic_s(self) -> float:
        return time.monotonic()

    def _reset_comparison(self, reason: str) -> None:
        self.pairs.clear()
        self.last_body_velocity = None
        self.last_body_velocity_time_s = None
        self.last_pair_received_s = None
        self.last_pair_by_key.clear()
        self.comparison_epoch += 1
        self.last_reset_reason = reason

    def _accept_px4_message(self, source: str, timestamp_sample_us: int) -> float | None:
        # Native PX4 timestamps already use Gazebo time with UXRCE_DDS_SYNCT=0.
        sample_time = int(timestamp_sample_us) * 1e-6
        age = self._ros_now_s() - sample_time
        previous = self.last_px4_message_times.get(source)
        if (timestamp_sample_us <= 0 or not -SIM_CLOCK_TOLERANCE_S <= age <= self.comparison_max_age_s
                or (previous is not None and sample_time <= previous)):
            self.rejected_count += 1
            return None
        self.last_px4_message_times[source] = sample_time
        return sample_time

    def _update_reference(self, msg: VehicleLocalPosition) -> None:
        reference = (float(msg.ref_lat), float(msg.ref_lon), float(msg.ref_alt))
        valid = (msg.xy_global and msg.z_global and msg.ref_timestamp > 0
                 and all(math.isfinite(v) for v in reference)
                 and -90 < reference[0] < 90 and -180 <= reference[1] <= 180)
        key = (int(msg.ref_timestamp), *reference) if valid else None
        counters = (msg.xy_reset_counter, msg.z_reset_counter,
                    msg.vxy_reset_counter, msg.vz_reset_counter, msg.heading_reset_counter)
        if key != self.reference_key:
            self._reset_comparison("local_reference_changed")
        elif self.local_reset_counters is not None and counters != self.local_reset_counters:
            self._reset_comparison("local_estimator_reset")
        self.reference_key = key
        self.reference = reference if valid else None
        self.local_reset_counters = counters

    def _queue_px4(self, key: str, sample_time: float, value: tuple) -> None:
        sample = Sample(sample_time, tuple(float(v) for v in value), self._monotonic_s(),
                        self.reference)
        if not self.pairs.add("px4", key, sample):
            self.rejected_count += 1

    def _queue_truth(self, stamp_s, position, quaternion, velocity_body, omega_body) -> None:
        values = (*position, *quaternion, *velocity_body, *omega_body)
        if not all(math.isfinite(v) for v in values) or sum(v*v for v in quaternion) < 1e-12:
            self.rejected_count += 1
            self.pairs.truth.clear()
            self.last_body_velocity = None
            return
        if stamp_s == self.last_compare_truth_time_s:
            self.rejected_count += 1
            return
        self.last_compare_truth_time_s = stamp_s
        rotation = quaternion_to_matrix(quaternion)
        offset_world = matvec(rotation, self.body_offset)
        position_body = tuple(p+r for p,r in zip(position, offset_world))
        # Rigid-body transport: v_body_origin = v_model_origin + omega cross r.
        wx, wy, wz = omega_body
        rx, ry, rz = self.body_offset
        cross = (wy*rz-wz*ry, wz*rx-wx*rz, wx*ry-wy*rx)
        corrected_velocity = tuple(v+c for v,c in zip(velocity_body, cross))
        velocity_ned = body_flu_vector_to_ned(quaternion, corrected_velocity)
        values = {
            "position": position_body,  # world ENU; mapped with each PX4 sample's origin
            "velocity": velocity_ned,
            "rpy": enu_flu_quaternion_to_ned_frd(quaternion),
            "angular_velocity": body_flu_vector_to_body_frd(omega_body),
        }
        received_s = self._monotonic_s()
        for key, value in values.items():
            self.pairs.add("truth", key, Sample(stamp_s, value, received_s))
        if self.last_body_velocity is not None:
            dt = stamp_s-self.last_body_velocity_time_s
            if 0 < dt <= self.comparison_max_gap_s:
                acceleration = tuple((v-old)/dt for v,old in zip(velocity_ned,self.last_body_velocity))
                # A finite difference represents the interval midpoint, not its end.
                midpoint = (stamp_s+self.last_body_velocity_time_s)/2
                self.pairs.add("truth", "acceleration", Sample(midpoint, acceleration, received_s))
        self.last_body_velocity = velocity_ned
        self.last_body_velocity_time_s = stamp_s

    def _flush_pairs(self) -> None:
        now_s = self._monotonic_s()
        for key, sample, truth, left_s, right_s in self.pairs.ready(now_s):
            px4 = sample.value
            if key == "position":
                if sample.reference is None:
                    continue
                truth = geodetic_to_px4_ned(world_to_geodetic(truth, self.world_origin), sample.reference)
            elif key == "rpy":
                px4, truth = quaternion_to_rpy(px4), quaternion_to_rpy(truth)
            error = tuple(a-b for a,b in zip(px4,truth))
            if key == "rpy":
                error = tuple(wrap_pi(v) for v in error)
            _, px4_topic, truth_topic, error_topic, frame = STATE_COMPARE_BY_KEY[key]
            # Distinct frame names for each vehicle's independent local origin.
            frame_id = f"{self.state_compare_topic_prefix.strip('/')}/{frame}"
            for topic, value in ((px4_topic,px4),(truth_topic,truth),(error_topic,error)):
                self._publish_vector(topic, value, frame_id, sample.time_s)
            row = {
                "sample_sim_time_s": fmt_time(sample.time_s), "quantity": key,
                "truth_left_sim_time_s": fmt_time(left_s), "truth_right_sim_time_s": fmt_time(right_s),
                "truth_bracket_s": fmt_time(right_s-left_s),
                "px4_queue_age_s": fmt_time(now_s-sample.received_s),
                "alignment_epoch": self.comparison_epoch, "frame_id": frame_id,
                "ros_publish_time_s": fmt_time(self._ros_now_s()),
            }
            if sample.reference is not None:
                row.update(zip(("ref_lat", "ref_lon", "ref_alt"), sample.reference))
            for name, values in (("px4",px4),("truth",truth),("error",error)):
                row.update({f"{name}_{axis}": fmt_float(value) for axis,value in zip("xyz",values)})
            self.compare_writer.writerow(row)
            self.pair_count += 1
            self.last_pair_received_s = now_s
            self.last_pair_by_key[key] = now_s

    def _publish_comparison_status(self) -> None:
        now_s = self._monotonic_s()
        # Expire pending samples even if one or both data sources stop.
        self._flush_pairs()
        if self._ros_now_s() <= 0:
            state = "waiting_for_sim_clock"
        elif self.reference is None:
            state = "waiting_for_global_reference"
        elif self.last_pair_received_s is None or now_s-self.last_pair_received_s > self.comparison_max_age_s:
            state = "waiting_for_paired_samples"
        elif any(key not in self.last_pair_by_key or now_s-self.last_pair_by_key[key] > self.comparison_max_age_s
                 for key in STATE_COMPARE_BY_KEY):
            state = "partially_paired"
        else:
            state = "paired"
        status = DiagnosticStatus()
        status.name = f"{self.state_compare_topic_prefix}/alignment"
        status.level = DiagnosticStatus.OK if state == "paired" else DiagnosticStatus.WARN
        status.message = state
        values = {"time_domain": "gazebo_sim", "position_alignment": "px4_global_reference",
                  "reference_point": "body_origin", "alignment_epoch": self.comparison_epoch,
                  "last_reset_reason": self.last_reset_reason, "paired_samples": self.pair_count,
                  "rejected_samples": self.rejected_count, "unpaired_samples": self.pairs.dropped,
                  "fresh_quantities": ",".join(key for key,stamp in self.last_pair_by_key.items()
                                              if now_s-stamp <= self.comparison_max_age_s)}
        status.values = [KeyValue(key=k, value=str(v)) for k,v in values.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.comparison_status_pub.publish(msg)

    def _make_log_dir(self) -> Path:
        log_root = str(self.get_parameter("log_root").value)
        run_id = str(self.get_parameter("run_id").value)

        root = Path(log_root).expanduser() if log_root else Path.cwd() / "log" / "trajectory_runs"
        if not run_id:
            run_id = datetime.now().strftime("%Y%m%d_%H%M%S")

        log_dir = root / run_id
        if not log_dir.exists():
            log_dir.mkdir(parents=True, exist_ok=False)
            return log_dir

        suffix = 1
        while True:
            candidate = root / f"{run_id}_{suffix:02d}"
            if not candidate.exists():
                candidate.mkdir(parents=True, exist_ok=False)
                return candidate
            suffix += 1

    def _open_writer(self, filename: str, fieldnames: list[str]) -> tuple[Any, csv.DictWriter]:
        path = self.log_dir / filename
        csv_file = path.open("w", encoding="utf-8", newline="", buffering=1)
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        return csv_file, writer

    def _ros_now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _vehicle_attitude_callback(self, msg: VehicleAttitude) -> None:
        self.latest_attitude = msg
        sample_time = self._accept_px4_message("attitude", msg.timestamp_sample)
        if sample_time is None:
            return
        if self.attitude_reset_counter is not None and self.attitude_reset_counter != msg.quat_reset_counter:
            self._reset_comparison("attitude_reset")
        self.attitude_reset_counter = msg.quat_reset_counter
        self._queue_px4("rpy", sample_time, tuple(msg.q))
        self._flush_pairs()

    def _vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        self.latest_px4_odometry = msg
        sample_time = self._accept_px4_message("odometry", msg.timestamp_sample)
        if sample_time is None:
            return
        if self.odometry_reset_counter is not None and self.odometry_reset_counter != msg.reset_counter:
            self._reset_comparison("odometry_reset")
        self.odometry_reset_counter = msg.reset_counter
        self._queue_px4("angular_velocity", sample_time, tuple(msg.angular_velocity))
        self._flush_pairs()

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        q = self._latest_px4_quaternion()
        roll, pitch, yaw = quaternion_to_rpy(q) if q is not None else ("", "", "")
        angular_velocity = (
            tuple(float(value) for value in self.latest_px4_odometry.angular_velocity)
            if self.latest_px4_odometry is not None
            else ("", "", "")
        )

        ros_time_s = self._ros_now_s()
        if self.ros_start_time_s <= 0:
            self.ros_start_time_s = ros_time_s
        px4_time_s = float(msg.timestamp) * 1e-6
        if self.first_px4_timestamp_us is None:
            self.first_px4_timestamp_us = int(msg.timestamp)
        px4_elapsed_s = (int(msg.timestamp) - self.first_px4_timestamp_us) * 1e-6

        row = {
            "ros_time_s": fmt_time(ros_time_s),
            "ros_elapsed_s": fmt_time(ros_time_s - self.ros_start_time_s),
            "px4_time_s": fmt_time(px4_time_s),
            "px4_elapsed_s": fmt_time(px4_elapsed_s),
            "px4_timestamp_us": msg.timestamp,
            "px4_timestamp_sample_us": msg.timestamp_sample,
            "sample_sim_time_s": fmt_time(msg.timestamp_sample * 1e-6) if msg.timestamp_sample else "",
            "ref_timestamp": msg.ref_timestamp,
            "ref_lat": msg.ref_lat, "ref_lon": msg.ref_lon, "ref_alt": msg.ref_alt,
            "xy_valid": msg.xy_valid, "z_valid": msg.z_valid,
            "v_xy_valid": msg.v_xy_valid, "v_z_valid": msg.v_z_valid,
            "xy_global": msg.xy_global, "z_global": msg.z_global,
            "xy_reset_counter": msg.xy_reset_counter, "z_reset_counter": msg.z_reset_counter,
            "vehicle_attitude_timestamp_sample_us": self.latest_attitude.timestamp_sample if self.latest_attitude else "",
            "vehicle_odometry_timestamp_sample_us": self.latest_px4_odometry.timestamp_sample if self.latest_px4_odometry else "",
            "frame_position": "px4_local_ned",
            "frame_body": "body_frd",
            "x_ned_m": fmt_float(msg.x),
            "y_ned_m": fmt_float(msg.y),
            "z_ned_m": fmt_float(msg.z),
            "vx_ned_mps": fmt_float(msg.vx),
            "vy_ned_mps": fmt_float(msg.vy),
            "vz_ned_mps": fmt_float(msg.vz),
            "ax_ned_mps2": fmt_float(msg.ax),
            "ay_ned_mps2": fmt_float(msg.ay),
            "az_ned_mps2": fmt_float(msg.az),
            "heading_rad": fmt_float(msg.heading),
            "q_w": fmt_float(q[0]) if q is not None else "",
            "q_x": fmt_float(q[1]) if q is not None else "",
            "q_y": fmt_float(q[2]) if q is not None else "",
            "q_z": fmt_float(q[3]) if q is not None else "",
            "roll_rad": fmt_float(roll) if roll != "" else "",
            "pitch_rad": fmt_float(pitch) if pitch != "" else "",
            "yaw_rad": fmt_float(yaw) if yaw != "" else "",
            "angular_velocity_x_body_frd_radps": fmt_float(angular_velocity[0])
            if angular_velocity[0] != ""
            else "",
            "angular_velocity_y_body_frd_radps": fmt_float(angular_velocity[1])
            if angular_velocity[1] != ""
            else "",
            "angular_velocity_z_body_frd_radps": fmt_float(angular_velocity[2])
            if angular_velocity[2] != ""
            else "",
            "vehicle_attitude_timestamp_us": self.latest_attitude.timestamp if self.latest_attitude else "",
            "vehicle_odometry_timestamp_us": self.latest_px4_odometry.timestamp
            if self.latest_px4_odometry
            else "",
        }
        self.px4_writer.writerow(row)

        sample_time = self._accept_px4_message("local_position", msg.timestamp_sample)
        if sample_time is None:
            return
        self._update_reference(msg)
        if msg.xy_valid and msg.z_valid and self.reference is not None:
            self._queue_px4("position", sample_time, (msg.x, msg.y, msg.z))
        else:
            self.pairs.pending.pop("position", None)
        if msg.v_xy_valid and msg.v_z_valid:
            self._queue_px4("velocity", sample_time, (msg.vx, msg.vy, msg.vz))
            self._queue_px4("acceleration", sample_time, (msg.ax, msg.ay, msg.az))
        else:
            self.pairs.pending.pop("velocity", None)
            self.pairs.pending.pop("acceleration", None)
        self._flush_pairs()

    def _latest_px4_quaternion(self) -> tuple[float, float, float, float] | None:
        if self.latest_attitude is not None:
            return tuple(float(value) for value in self.latest_attitude.q)
        if self.latest_px4_odometry is not None:
            return tuple(float(value) for value in self.latest_px4_odometry.q)
        return None

    def _gazebo_odometry_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        stamp_s = stamp_to_seconds(msg.header.stamp)
        # Gazebo epoch zero is valid; never substitute wall time for sim time.
        sample_time_s = stamp_s
        if self.first_gazebo_time_s is None:
            self.first_gazebo_time_s = sample_time_s
        gazebo_elapsed_s = sample_time_s - self.first_gazebo_time_s

        velocity = (
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.linear.z),
        )

        q = (
            float(pose.orientation.w),
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
        )
        roll, pitch, yaw = quaternion_to_rpy(q)
        q_ned_frd = enu_flu_quaternion_to_ned_frd(q)
        roll_ned, pitch_ned, yaw_ned = quaternion_to_rpy(q_ned_frd)

        x_enu = float(pose.position.x)
        y_enu = float(pose.position.y)
        z_enu = float(pose.position.z)
        vx_body_flu, vy_body_flu, vz_body_flu = velocity
        vx_ned, vy_ned, vz_ned = body_flu_vector_to_ned(q, velocity)
        ax_ned, ay_ned, az_ned = self._truth_acceleration(
            sample_time_s,
            (vx_ned, vy_ned, vz_ned),
        )
        wx_body_flu = float(twist.angular.x)
        wy_body_flu = float(twist.angular.y)
        wz_body_flu = float(twist.angular.z)
        wx_body_frd, wy_body_frd, wz_body_frd = body_flu_vector_to_body_frd(
            (wx_body_flu, wy_body_flu, wz_body_flu)
        )
        ros_time_s = self._ros_now_s()
        if self.ros_start_time_s <= 0:
            self.ros_start_time_s = ros_time_s

        row = {
            "ros_time_s": fmt_time(ros_time_s),
            "ros_elapsed_s": fmt_time(ros_time_s - self.ros_start_time_s),
            "gazebo_time_s": fmt_time(sample_time_s),
            "gazebo_elapsed_s": fmt_time(gazebo_elapsed_s),
            "stamp_sec": msg.header.stamp.sec,
            "stamp_nanosec": msg.header.stamp.nanosec,
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x_enu_m": fmt_float(x_enu),
            "y_enu_m": fmt_float(y_enu),
            "z_enu_m": fmt_float(z_enu),
            "vx_body_flu_mps": fmt_float(vx_body_flu),
            "vy_body_flu_mps": fmt_float(vy_body_flu),
            "vz_body_flu_mps": fmt_float(vz_body_flu),
            "x_ned_equiv_m": fmt_float(y_enu),
            "y_ned_equiv_m": fmt_float(x_enu),
            "z_ned_equiv_m": fmt_float(-z_enu),
            "vx_ned_equiv_mps": fmt_float(vx_ned),
            "vy_ned_equiv_mps": fmt_float(vy_ned),
            "vz_ned_equiv_mps": fmt_float(vz_ned),
            "ax_ned_equiv_mps2": fmt_float(ax_ned),
            "ay_ned_equiv_mps2": fmt_float(ay_ned),
            "az_ned_equiv_mps2": fmt_float(az_ned),
            "q_enu_flu_w": fmt_float(q[0]),
            "q_enu_flu_x": fmt_float(q[1]),
            "q_enu_flu_y": fmt_float(q[2]),
            "q_enu_flu_z": fmt_float(q[3]),
            "roll_enu_flu_rad": fmt_float(roll),
            "pitch_enu_flu_rad": fmt_float(pitch),
            "yaw_enu_flu_rad": fmt_float(yaw),
            "q_ned_frd_w": fmt_float(q_ned_frd[0]),
            "q_ned_frd_x": fmt_float(q_ned_frd[1]),
            "q_ned_frd_y": fmt_float(q_ned_frd[2]),
            "q_ned_frd_z": fmt_float(q_ned_frd[3]),
            "roll_ned_frd_rad": fmt_float(roll_ned),
            "pitch_ned_frd_rad": fmt_float(pitch_ned),
            "yaw_ned_frd_rad": fmt_float(yaw_ned),
            "angular_velocity_x_body_flu_radps": fmt_float(wx_body_flu),
            "angular_velocity_y_body_flu_radps": fmt_float(wy_body_flu),
            "angular_velocity_z_body_flu_radps": fmt_float(wz_body_flu),
            "angular_velocity_x_body_frd_radps": fmt_float(wx_body_frd),
            "angular_velocity_y_body_frd_radps": fmt_float(wy_body_frd),
            "angular_velocity_z_body_frd_radps": fmt_float(wz_body_frd),
        }
        self.truth_writer.writerow(row)

        self._queue_truth(sample_time_s, (x_enu, y_enu, z_enu), q, velocity,
                          (wx_body_flu, wy_body_flu, wz_body_flu))
        self._flush_pairs()

    def _truth_acceleration(
        self, sample_time_s: float, velocity: tuple[float, float, float]
    ) -> tuple[float | str, float | str, float | str]:
        if self.last_truth_time_s is None or self.last_truth_velocity is None:
            self.last_truth_time_s = sample_time_s
            self.last_truth_velocity = velocity
            return ("", "", "")

        dt = sample_time_s - self.last_truth_time_s
        previous_velocity = self.last_truth_velocity
        self.last_truth_time_s = sample_time_s
        self.last_truth_velocity = velocity

        if dt <= 1e-9:
            return ("", "", "")

        return (
            (velocity[0] - previous_velocity[0]) / dt,
            (velocity[1] - previous_velocity[1]) / dt,
            (velocity[2] - previous_velocity[2]) / dt,
        )

    def _make_state_compare_publishers(self) -> dict[str, Any]:
        if not self.publish_state_compare_topics:
            return {}
        return {
            topic: self.create_publisher(
                Vector3Stamped,
                f"{self.state_compare_topic_prefix}/{topic}",
                10,
            )
            for topic in STATE_COMPARE_TOPICS
        }

    def _publish_vector(
        self,
        topic_key: str,
        vector: tuple[float, float, float],
        frame_id: str,
        sample_time_s: float,
    ) -> None:
        publisher = self.state_compare_publishers.get(topic_key)
        if publisher is None:
            return
        msg = Vector3Stamped()
        stamp_ns = round(sample_time_s * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        msg.header.frame_id = frame_id
        msg.vector.x = float(vector[0])
        msg.vector.y = float(vector[1])
        msg.vector.z = float(vector[2])
        publisher.publish(msg)

    def destroy_node(self) -> bool:
        if self.control_file is not None:
            self.control_file.close()
        for csv_file in (self.px4_file, self.truth_file, self.compare_file):
            csv_file.flush()
            csv_file.close()
        return super().destroy_node()


def comparison_fieldnames() -> list[str]:
    return [
        "sample_sim_time_s", "quantity", "truth_left_sim_time_s", "truth_right_sim_time_s",
        "truth_bracket_s", "px4_queue_age_s", "alignment_epoch",
        "ref_lat", "ref_lon", "ref_alt", "frame_id", "ros_publish_time_s",
        *[f"{source}_{axis}" for source in ("px4", "truth", "error") for axis in "xyz"],
    ]


def px4_fieldnames() -> list[str]:
    return [
        "ros_time_s",
        "ros_elapsed_s",
        "px4_time_s",
        "px4_elapsed_s",
        "px4_timestamp_us",
        "px4_timestamp_sample_us", "sample_sim_time_s",
        "ref_timestamp", "ref_lat", "ref_lon", "ref_alt", "xy_global", "z_global",
        "xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "xy_reset_counter", "z_reset_counter",
        "vehicle_attitude_timestamp_sample_us", "vehicle_odometry_timestamp_sample_us",
        "frame_position",
        "frame_body",
        "x_ned_m",
        "y_ned_m",
        "z_ned_m",
        "vx_ned_mps",
        "vy_ned_mps",
        "vz_ned_mps",
        "ax_ned_mps2",
        "ay_ned_mps2",
        "az_ned_mps2",
        "heading_rad",
        "q_w",
        "q_x",
        "q_y",
        "q_z",
        "roll_rad",
        "pitch_rad",
        "yaw_rad",
        "angular_velocity_x_body_frd_radps",
        "angular_velocity_y_body_frd_radps",
        "angular_velocity_z_body_frd_radps",
        "vehicle_attitude_timestamp_us",
        "vehicle_odometry_timestamp_us",
    ]


def truth_fieldnames() -> list[str]:
    return [
        "ros_time_s",
        "ros_elapsed_s",
        "gazebo_time_s",
        "gazebo_elapsed_s",
        "stamp_sec",
        "stamp_nanosec",
        "frame_id",
        "child_frame_id",
        "x_enu_m",
        "y_enu_m",
        "z_enu_m",
        "vx_body_flu_mps",
        "vy_body_flu_mps",
        "vz_body_flu_mps",
        "x_ned_equiv_m",
        "y_ned_equiv_m",
        "z_ned_equiv_m",
        "vx_ned_equiv_mps",
        "vy_ned_equiv_mps",
        "vz_ned_equiv_mps",
        "ax_ned_equiv_mps2",
        "ay_ned_equiv_mps2",
        "az_ned_equiv_mps2",
        "q_enu_flu_w",
        "q_enu_flu_x",
        "q_enu_flu_y",
        "q_enu_flu_z",
        "roll_enu_flu_rad",
        "pitch_enu_flu_rad",
        "yaw_enu_flu_rad",
        "q_ned_frd_w",
        "q_ned_frd_x",
        "q_ned_frd_y",
        "q_ned_frd_z",
        "roll_ned_frd_rad",
        "pitch_ned_frd_rad",
        "yaw_ned_frd_rad",
        "angular_velocity_x_body_flu_radps",
        "angular_velocity_y_body_flu_radps",
        "angular_velocity_z_body_flu_radps",
        "angular_velocity_x_body_frd_radps",
        "angular_velocity_y_body_frd_radps",
        "angular_velocity_z_body_frd_radps",
    ]


def quaternion_to_rpy(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = normalize_quaternion(q)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_to_matrix(q: tuple[float, float, float, float]) -> tuple[tuple[float, ...], ...]:
    w, x, y, z = normalize_quaternion(q)
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def matrix_to_quaternion(matrix: tuple[tuple[float, ...], ...]) -> tuple[float, float, float, float]:
    m = matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m[2][1] - m[1][2]) / scale
        y = (m[0][2] - m[2][0]) / scale
        z = (m[1][0] - m[0][1]) / scale
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        scale = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / scale
        x = 0.25 * scale
        y = (m[0][1] + m[1][0]) / scale
        z = (m[0][2] + m[2][0]) / scale
    elif m[1][1] > m[2][2]:
        scale = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / scale
        x = (m[0][1] + m[1][0]) / scale
        y = 0.25 * scale
        z = (m[1][2] + m[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / scale
        x = (m[0][2] + m[2][0]) / scale
        y = (m[1][2] + m[2][1]) / scale
        z = 0.25 * scale
    return normalize_quaternion((w, x, y, z))


def matmul(
    left: tuple[tuple[float, ...], ...],
    right: tuple[tuple[float, ...], ...],
) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(sum(left[row][idx] * right[idx][col] for idx in range(3)) for col in range(3))
        for row in range(3)
    )


def matvec(
    matrix: tuple[tuple[float, ...], ...],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(sum(matrix[row][idx] * vector[idx] for idx in range(3)) for row in range(3))


def enu_to_ned_matrix() -> tuple[tuple[float, ...], ...]:
    return (
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, -1.0),
    )


def flu_to_frd_matrix() -> tuple[tuple[float, ...], ...]:
    return (
        (1.0, 0.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, -1.0),
    )


def enu_flu_quaternion_to_ned_frd(
    q_enu_flu: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    rotation_ned_frd = matmul(
        matmul(enu_to_ned_matrix(), quaternion_to_matrix(q_enu_flu)),
        flu_to_frd_matrix(),
    )
    return matrix_to_quaternion(rotation_ned_frd)


def body_flu_vector_to_body_frd(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    return matvec(flu_to_frd_matrix(), vector)


def body_flu_vector_to_ned(
    q_enu_flu: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    rotation_ned_flu = matmul(enu_to_ned_matrix(), quaternion_to_matrix(q_enu_flu))
    return matvec(rotation_ned_flu, vector)


def normalize_quaternion(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(value / norm for value in q)


def wrap_pi(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def normalize_topic_prefix(prefix: str) -> str:
    normalized = prefix.strip().rstrip("/")
    return normalized if normalized else "state_compare"


def stamp_to_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def fmt_float(value: float | str) -> str:
    if value == "":
        return ""
    return f"{float(value):.9g}"


def fmt_time(value: float | str) -> str:
    if value == "":
        return ""
    return f"{float(value):.6f}"


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
