#!/usr/bin/env python3

import csv
import math
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleOdometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


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

        self.declare_parameter("log_root", "")
        self.declare_parameter("run_id", "")
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("vehicle_attitude_topic", "/fmu/out/vehicle_attitude")
        self.declare_parameter("vehicle_odometry_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("gazebo_odometry_topic", "/model/x500_0/odometry_with_covariance")
        self.declare_parameter("publish_state_compare_topics", True)
        self.declare_parameter("state_compare_topic_prefix", "state_compare")

        self.latest_attitude: VehicleAttitude | None = None
        self.latest_px4_odometry: VehicleOdometry | None = None
        self.latest_px4_compare: dict[str, tuple[float, float, float]] = {}
        self.latest_truth_compare: dict[str, tuple[float, float, float]] = {}
        self.last_truth_time_s: float | None = None
        self.last_truth_velocity: tuple[float, float, float] | None = None
        self.ros_start_time_s = self._ros_now_s()
        self.first_px4_timestamp_us: int | None = None
        self.first_gazebo_time_s: float | None = None

        self.log_dir = self._make_log_dir()
        self.px4_file, self.px4_writer = self._open_writer("px4_estimate.csv", px4_fieldnames())
        self.truth_file, self.truth_writer = self._open_writer("gazebo_truth.csv", truth_fieldnames())

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

    def _vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        self.latest_px4_odometry = msg

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        q = self._latest_px4_quaternion()
        roll, pitch, yaw = quaternion_to_rpy(q) if q is not None else ("", "", "")
        angular_velocity = (
            tuple(float(value) for value in self.latest_px4_odometry.angular_velocity)
            if self.latest_px4_odometry is not None
            else ("", "", "")
        )

        ros_time_s = self._ros_now_s()
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

        self._publish_px4_compare_topics(
            position=(float(msg.x), float(msg.y), float(msg.z)),
            velocity=(float(msg.vx), float(msg.vy), float(msg.vz)),
            acceleration=(float(msg.ax), float(msg.ay), float(msg.az)),
            rpy=(roll, pitch, yaw) if q is not None else None,
            angular_velocity=angular_velocity if angular_velocity[0] != "" else None,
        )

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
        sample_time_s = stamp_s if stamp_s > 0.0 else self.get_clock().now().nanoseconds * 1e-9
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

        self._publish_truth_compare_topics(
            position=(float(y_enu), float(x_enu), float(-z_enu)),
            velocity=(vx_ned, vy_ned, vz_ned),
            acceleration=(ax_ned, ay_ned, az_ned),
            rpy=(roll_ned, pitch_ned, yaw_ned),
            angular_velocity=(wx_body_frd, wy_body_frd, wz_body_frd),
        )

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

    def _publish_px4_compare_topics(
        self,
        *,
        position: tuple[float, float, float],
        velocity: tuple[float, float, float],
        acceleration: tuple[float, float, float],
        rpy: tuple[float, float, float] | None,
        angular_velocity: tuple[float, float, float] | None,
    ) -> None:
        if not self.publish_state_compare_topics:
            return
        values: dict[str, tuple[float, float, float]] = {
            "position": position,
            "velocity": velocity,
            "acceleration": acceleration,
        }
        if rpy is not None:
            values["rpy"] = rpy
        if angular_velocity is not None:
            values["angular_velocity"] = angular_velocity
        for key, vector in values.items():
            self.latest_px4_compare[key] = vector
            self._publish_compare_vector("px4", key, vector)
        self._publish_compare_errors()

    def _publish_truth_compare_topics(
        self,
        *,
        position: tuple[float, float, float],
        velocity: tuple[float, float, float],
        acceleration: tuple[float | str, float | str, float | str],
        rpy: tuple[float, float, float],
        angular_velocity: tuple[float, float, float],
    ) -> None:
        if not self.publish_state_compare_topics:
            return
        values: dict[str, tuple[float, float, float]] = {
            "position": position,
            "velocity": velocity,
            "rpy": rpy,
            "angular_velocity": angular_velocity,
        }
        if all(value != "" for value in acceleration):
            values["acceleration"] = (
                float(acceleration[0]),
                float(acceleration[1]),
                float(acceleration[2]),
            )
        else:
            self.latest_truth_compare.pop("acceleration", None)
        for key, vector in values.items():
            self.latest_truth_compare[key] = vector
            self._publish_compare_vector("truth", key, vector)
        self._publish_compare_errors()

    def _publish_compare_errors(self) -> None:
        for key, _, _, error_topic, frame_id in STATE_COMPARE_SPECS:
            if key not in self.latest_px4_compare or key not in self.latest_truth_compare:
                continue
            px4 = self.latest_px4_compare[key]
            truth = self.latest_truth_compare[key]
            if key == "rpy":
                error = tuple(wrap_pi(px4[idx] - truth[idx]) for idx in range(3))
            else:
                error = tuple(px4[idx] - truth[idx] for idx in range(3))
            self._publish_vector(error_topic, error, frame_id)

    def _publish_compare_vector(
        self,
        source: str,
        key: str,
        vector: tuple[float, float, float],
    ) -> None:
        _, px4_topic, truth_topic, _, frame_id = STATE_COMPARE_BY_KEY[key]
        if source == "px4":
            topic_key = px4_topic
        elif source == "truth":
            topic_key = truth_topic
        else:
            raise ValueError(f"Unknown state compare source: {source}")
        self._publish_vector(topic_key, vector, frame_id)

    def _publish_vector(
        self,
        topic_key: str,
        vector: tuple[float, float, float],
        frame_id: str,
    ) -> None:
        publisher = self.state_compare_publishers.get(topic_key)
        if publisher is None:
            return
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.vector.x = float(vector[0])
        msg.vector.y = float(vector[1])
        msg.vector.z = float(vector[2])
        publisher.publish(msg)

    def destroy_node(self) -> bool:
        for csv_file in (self.px4_file, self.truth_file):
            csv_file.flush()
            csv_file.close()
        return super().destroy_node()


def px4_fieldnames() -> list[str]:
    return [
        "ros_time_s",
        "ros_elapsed_s",
        "px4_time_s",
        "px4_elapsed_s",
        "px4_timestamp_us",
        "px4_timestamp_sample_us",
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
