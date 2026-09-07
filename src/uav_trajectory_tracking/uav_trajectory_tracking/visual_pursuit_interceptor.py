#!/usr/bin/env python3
from __future__ import annotations

import math
from collections import deque
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleOdometry,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from .fixed_camera_target_tracker import valid_camera_info
from .state_comparison import SIM_CLOCK_TOLERANCE_S, slerp


class InterceptorState:
    INITIALIZING = "initializing"
    TAKEOFF = "takeoff"
    TRANSIT = "transit_to_hover"
    HOLD = "hold"
    ACQUIRING = "acquiring_target"
    COAST = "coast_on_lock_loss"
    PURSUIT = "pursuit"
    TARGET_LOST = "target_lost"


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]
GUIDANCE_STATES = {InterceptorState.PURSUIT, InterceptorState.COAST}
# Commander publishes unchanged VehicleStatus every 500 ms; allow scheduling jitter.
VEHICLE_STATUS_TIMEOUT_S = 1.0


@dataclass
class KalmanAxisState:
    position_rad: float
    rate_rad_s: float
    p00: float
    p01: float
    p11: float
    time_s: float


class AngleKalmanFilter:
    """Constant-angular-velocity Kalman filter at observation acquisition times."""

    def __init__(
        self,
        measurement_noise_std_rad: float,
        process_noise_std_rad_s2: float,
        max_prediction_s: float,
    ) -> None:
        self.measurement_variance = (
            measurement_noise_std_rad * measurement_noise_std_rad
        )
        self.accel_variance = process_noise_std_rad_s2 * process_noise_std_rad_s2
        self.max_prediction_s = max_prediction_s
        self.state: KalmanAxisState | None = None

    def reset(self) -> None:
        self.state = None

    def update(self, measurement_rad: float, measurement_time_s: float) -> None:
        if self.state is None or measurement_time_s < self.state.time_s - 1e-6:
            self._initialize(measurement_rad, measurement_time_s)
            return

        self._predict_in_place(measurement_time_s - self.state.time_s)
        assert self.state is not None
        residual = measurement_rad - self.state.position_rad

        innovation_variance = self.state.p00 + self.measurement_variance
        if innovation_variance <= 1e-12:
            return

        gain_position = self.state.p00 / innovation_variance
        gain_rate = self.state.p01 / innovation_variance
        p00 = self.state.p00
        p01 = self.state.p01
        p11 = self.state.p11

        position_rad = self.state.position_rad + gain_position * residual

        self.state = KalmanAxisState(
            position_rad=position_rad,
            rate_rad_s=self.state.rate_rad_s + gain_rate * residual,
            p00=max((1.0 - gain_position) * p00, 1e-12),
            p01=(1.0 - gain_position) * p01,
            p11=max(p11 - gain_rate * p01, 1e-12),
            time_s=measurement_time_s,
        )

    def predict(self, target_time_s: float) -> tuple[float, float, float] | None:
        if self.state is None:
            return None

        dt_s = target_time_s - self.state.time_s
        if not 0.0 <= dt_s <= self.max_prediction_s:
            return None
        position_rad = self.state.position_rad + self.state.rate_rad_s * dt_s
        return position_rad, self.state.rate_rad_s, dt_s

    def _initialize(self, measurement_rad: float, measurement_time_s: float) -> None:
        initial_rate_std_rad_s = max(
            1.0,
            math.sqrt(self.accel_variance) * self.max_prediction_s,
        )
        self.state = KalmanAxisState(
            position_rad=measurement_rad,
            rate_rad_s=0.0,
            p00=max(self.measurement_variance, 1e-12),
            p01=0.0,
            p11=initial_rate_std_rad_s * initial_rate_std_rad_s,
            time_s=measurement_time_s,
        )

    def _predict_in_place(self, dt_s: float) -> None:
        assert self.state is not None
        if dt_s <= 1e-9:
            return

        q00 = 0.25 * self.accel_variance * dt_s ** 4
        q01 = 0.5 * self.accel_variance * dt_s ** 3
        q11 = self.accel_variance * dt_s * dt_s
        position_rad = self.state.position_rad + self.state.rate_rad_s * dt_s

        self.state = KalmanAxisState(
            position_rad=position_rad,
            rate_rad_s=self.state.rate_rad_s,
            p00=(
                self.state.p00
                + 2.0 * dt_s * self.state.p01
                + dt_s * dt_s * self.state.p11
                + q00
            ),
            p01=self.state.p01 + dt_s * self.state.p11 + q01,
            p11=self.state.p11 + q11,
            time_s=self.state.time_s + dt_s,
        )


class LosAngleKalmanFilter:
    """Acquisition-time LOS angle filtering with bounded forward prediction.

    This is not the paper's joint IMU/relative-state delayed Kalman filter.
    """

    def __init__(
        self,
        measurement_noise_std_rad: float,
        process_noise_std_rad_s2: float,
        max_prediction_s: float,
    ) -> None:
        self.yaw = AngleKalmanFilter(
            measurement_noise_std_rad,
            process_noise_std_rad_s2,
            max_prediction_s,
        )
        self.pitch = AngleKalmanFilter(
            measurement_noise_std_rad,
            process_noise_std_rad_s2,
            max_prediction_s,
        )

    def reset(self) -> None:
        self.yaw.reset()
        self.pitch.reset()

    def update(
        self,
        yaw_error_rad: float,
        pitch_error_rad: float,
        measurement_time_s: float,
    ) -> None:
        self.yaw.update(yaw_error_rad, measurement_time_s)
        self.pitch.update(pitch_error_rad, measurement_time_s)

    def predict(
        self,
        target_time_s: float,
    ) -> tuple[float, float, float, float, float] | None:
        yaw = self.yaw.predict(target_time_s)
        pitch = self.pitch.predict(target_time_s)
        if yaw is None or pitch is None:
            return None
        yaw_error_rad, yaw_rate_rad_s, yaw_horizon_s = yaw
        pitch_error_rad, pitch_rate_rad_s, pitch_horizon_s = pitch
        return (
            yaw_error_rad,
            pitch_error_rad,
            yaw_rate_rad_s,
            pitch_rate_rad_s,
            max(yaw_horizon_s, pitch_horizon_s),
        )


class VisualPursuitInterceptor(Node):
    """Use calibrated observations and unified PX4 odometry samples for guidance."""

    def __init__(self) -> None:
        super().__init__("visual_pursuit_interceptor")

        self.declare_parameter("config_file", "")
        self.declare_parameter("observation_timeout_s", 0.2)
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status_v4")
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("vehicle_odometry_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("camera_info_topic", "/x500_0/camera/camera_info")
        self.declare_parameter("bearing_topic", "/x500_0/fixed_camera_target_tracker/bearing")
        self.declare_parameter("optical_frame_id", "x500_0/camera_optical_frame")
        self.declare_parameter("tracking_active_topic", "/x500_0/fixed_camera_target_tracker/tracking_active")
        self.declare_parameter("lock_active_topic", "/x500_0/fixed_camera_target_tracker/lock_active")
        self.declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode")
        self.declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")
        self.declare_parameter("diagnostics_topic", "/x500_0/visual_pursuit_interceptor/diagnostics")
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("source_system", 1)
        self.declare_parameter("source_component", 1)

        config = self._load_config()
        camera_mount_rpy_rad = parse_point(
            config.get("camera_mount_rpy_rad", [0.0, 0.0, 0.0]),
            "camera_mount_rpy_rad",
        )
        self.camera_to_body_flu = rotation_from_rpy(*camera_mount_rpy_rad)
        self.body_to_camera_flu = tuple(zip(*self.camera_to_body_flu))
        self.search_yaw_rate_rad_s = math.radians(nonnegative_float(
            config.get("search_yaw_rate_deg_s", 20.0), "search_yaw_rate_deg_s"
        ))
        self.search_yaw_rad: float | None = None
        self.last_search_yaw_time_s: float | None = None
        self.search_vertical_amplitude_m = nonnegative_float(
            config.get("search_vertical_amplitude_m", 2.0), "search_vertical_amplitude_m")
        self.search_vertical_period_s = positive_float(
            config.get("search_vertical_period_s", 12.0), "search_vertical_period_s")
        self.search_vertical_min_z_ned = float(config.get("search_vertical_min_z_ned", -8.0))
        self.search_vertical_max_z_ned = float(config.get("search_vertical_max_z_ned", -2.0))
        if (not all(math.isfinite(z) for z in
                    (self.search_vertical_min_z_ned, self.search_vertical_max_z_ned))
                or self.search_vertical_min_z_ned >= self.search_vertical_max_z_ned):
            raise ValueError("Vertical search requires finite min_z_ned < max_z_ned.")
        self.vertical_search_start_time_s: float | None = None
        self.control_rate_hz = positive_float(
            config.get("control_rate_hz", 20.0),
            "control_rate_hz",
        )
        self.takeoff_warmup_s = nonnegative_float(
            config.get("takeoff_warmup_s", 1.5),
            "takeoff_warmup_s",
        )
        self.initial_hover_position = parse_point(
            config.get("initial_hover_position_ned", [1.0, 2.0, -5.0]),
            "initial_hover_position_ned",
        )
        self.hover_acceptance_radius_m = positive_float(
            config.get("hover_acceptance_radius_m", 0.3),
            "hover_acceptance_radius_m",
        )
        self.max_pursuit_speed_mps = positive_float(
            config.get("max_pursuit_speed_mps", 2.0),
            "max_pursuit_speed_mps",
        )
        self.speed_accel_mps2 = positive_float(
            config.get("speed_accel_mps2", 1.0), "speed_accel_mps2")
        self.png_vertical_gain = positive_float(
            config.get("png_vertical_gain", 3.5),
            "png_vertical_gain",
        )
        self.png_horizontal_gain = positive_float(
            config.get("png_horizontal_gain", 3.5),
            "png_horizontal_gain",
        )
        self.observation_timeout_s = positive_float(
            self.get_parameter("observation_timeout_s").value, "observation_timeout_s")
        self.los_filter_enabled = bool(config.get("los_filter_enabled", True))
        self.los_filter_measurement_noise_std_rad = positive_float(
            config.get("los_filter_measurement_noise_std_rad", math.radians(1.0)),
            "los_filter_measurement_noise_std_rad",
        )
        self.los_filter_process_noise_std_rad_s2 = positive_float(
            config.get("los_filter_process_noise_std_rad_s2", 4.0),
            "los_filter_process_noise_std_rad_s2",
        )
        self.los_filter_max_prediction_s = positive_float(
            config.get("los_filter_max_prediction_s", self.observation_timeout_s),
            "los_filter_max_prediction_s",
        )
        self.min_velocity_direction_mps = nonnegative_float(
            config.get("min_velocity_direction_mps", 0.2),
            "min_velocity_direction_mps",
        )
        self.coast_velocity_decay_s = positive_float(
            config.get("coast_velocity_decay_s", 0.6),
            "coast_velocity_decay_s",
        )
        self.lock_loss_grace_s = nonnegative_float(
            config.get("lock_loss_grace_s", 0.3),
            "lock_loss_grace_s",
        )
        self.tracking_status_timeout_s = positive_float(
            config.get("tracking_status_timeout_s", 0.2), "tracking_status_timeout_s")
        self.hold_position_on_loss = bool(config.get("hold_position_on_loss", True))
        self.yaw_mode = str(config.get("yaw_mode", "face_los")).strip().lower()
        self._validate_yaw_mode()

        self.target_system = int(self.get_parameter("target_system").value)
        self.target_component = int(self.get_parameter("target_component").value)
        self.source_system = int(self.get_parameter("source_system").value)
        self.source_component = int(self.get_parameter("source_component").value)

        self.state_timeout_s = positive_float(config.get("state_timeout_s", 0.5), "state_timeout_s")
        self.attitude_max_gap_s = positive_float(config.get("attitude_max_gap_s", 0.1), "attitude_max_gap_s")
        self.camera_info: CameraInfo | None = None
        self.guidance_observation: Vector3 | None = None
        self.last_projection_sample_us: int | None = None
        self.last_guidance_sample_us: int | None = None
        self.last_pursuit_time_s: float | None = None
        self.state_received = {}
        self.attitudes = deque(maxlen=256)
        self.pending_bearings = deque(maxlen=16)
        self.odometry_reset_counter = None
        self.measured_los_ned = None
        self.last_camera_stamp_s = None
        self.vehicle_status: VehicleStatus | None = None
        self.vehicle_local_position: VehicleLocalPosition | None = None
        self.vehicle_odometry: VehicleOdometry | None = None
        self.tracking_active = False
        self.last_tracking_active_time_s: float | None = None
        self.tracking_true_since_s: float | None = None
        self.last_tracking_true_time_s: float | None = None
        self.lock_active = False
        self.last_lock_active_time_s: float | None = None
        self.lock_true_since_s: float | None = None
        self.last_lock_true_time_s: float | None = None
        self.image_yaw_error_rad: float | None = None
        self.image_pitch_error_rad: float | None = None
        self.last_bearing_time_s: float | None = None
        self.los_filter = LosAngleKalmanFilter(
            self.los_filter_measurement_noise_std_rad,
            self.los_filter_process_noise_std_rad_s2,
            self.los_filter_max_prediction_s,
        )
        self.los_filter_horizontal_rad: float | None = None
        self.los_filter_vertical_rad: float | None = None
        self.los_filter_horizontal_rate_rad_s: float | None = None
        self.los_filter_vertical_rate_rad_s: float | None = None
        self.los_filter_prediction_horizon_s: float | None = None
        self.los_filter_last_horizontal_rad: float | None = None
        self.los_filter_last_measurement_stamp_s: float | None = None
        self.los_filter_measurement_time_source = "none"
        self.los_filter_measurement_delay_observed_s: float | None = None
        self.takeoff_position: tuple[float, float, float] | None = None
        self.takeoff_altitude_reached = False
        self.hold_position: tuple[float, float, float] | None = None
        self.initial_hover_reached = False
        self.state = InterceptorState.INITIALIZING
        self.previous_state = InterceptorState.INITIALIZING
        self.setpoint_counter = 0
        self.last_mode_request_us = 0
        self.last_arm_request_us = 0
        self.last_los_body = (1.0, 0.0, 0.0)
        self.last_los_ned = (1.0, 0.0, 0.0)
        self.last_closing_speed_mps: float | None = None
        self.last_commanded_closing_speed_mps = 0.0
        self.last_velocity_setpoint_ned = (0.0, 0.0, 0.0)
        self.last_png_los_vertical_angle_rad: float | None = None
        self.last_png_los_horizontal_angle_rad: float | None = None
        self.last_png_velocity_vertical_angle_rad: float | None = None
        self.last_png_velocity_horizontal_angle_rad: float | None = None
        self.last_png_desired_vertical_angle_rad: float | None = None
        self.last_png_desired_horizontal_angle_rad: float | None = None
        self.last_control_time_s: float | None = None

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            str(self.get_parameter("offboard_control_mode_topic").value),
            px4_qos,
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            str(self.get_parameter("trajectory_setpoint_topic").value),
            px4_qos,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            str(self.get_parameter("vehicle_command_topic").value),
            px4_qos,
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )

        self.create_subscription(
            VehicleStatus,
            str(self.get_parameter("vehicle_status_topic").value),
            self._vehicle_status_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            str(self.get_parameter("vehicle_local_position_topic").value),
            self._vehicle_local_position_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleOdometry,
            str(self.get_parameter("vehicle_odometry_topic").value),
            self._vehicle_odometry_callback,
            px4_qos,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback, sensor_qos,
        )
        self.create_subscription(
            Vector3Stamped,
            str(self.get_parameter("bearing_topic").value),
            self._bearing_callback,
            sensor_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("tracking_active_topic").value),
            self._tracking_active_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("lock_active_topic").value),
            self._lock_active_callback,
            10,
        )

        self.timer = self.create_timer(
            1.0 / max(self.control_rate_hz, 1.0),
            self._timer_callback,
        )

        self.get_logger().info(
            "Visual pursuit interceptor ready: "
            f"rate={self.control_rate_hz:.1f} Hz, "
            f"initial_hover=({self.initial_hover_position[0]:.2f}, "
            f"{self.initial_hover_position[1]:.2f}, "
            f"{self.initial_hover_position[2]:.2f}) m NED, "
            f"speed_limit={self.max_pursuit_speed_mps:.2f} m/s, "
            f"png_gains=({self.png_vertical_gain:.2f}, {self.png_horizontal_gain:.2f}), "
            f"los_filter_enabled={self.los_filter_enabled}, "
            f"target_system={self.target_system}"
        )

    def _load_config(self) -> dict[str, Any]:
        config_path = str(self.get_parameter("config_file").value)
        if not config_path:
            share_dir = Path(get_package_share_directory("uav_trajectory_tracking"))
            config_path = str(share_dir / "config" / "visual_interception.yaml")

        path = Path(config_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Visual interception config does not exist: {path}")

        with path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        if str(config.get("frame", "NED")).upper() != "NED":
            raise ValueError("Only PX4 local NED interception configs are supported.")

        return config

    def _state_fresh(self, source: str, timestamp_us: int) -> bool:
        received = self.state_received.get(source)
        age = self._now_s() - timestamp_us * 1e-6
        timeout = VEHICLE_STATUS_TIMEOUT_S if source == "status" else self.state_timeout_s
        return (timestamp_us > 0 and -SIM_CLOCK_TOLERANCE_S <= age <= timeout
                and received is not None and 0.0 <= time.monotonic()-received <= timeout)

    def _vehicle_status_callback(self, msg: VehicleStatus) -> None:
        if (msg.timestamp == 0 or not -SIM_CLOCK_TOLERANCE_S
                <= self._now_s()-msg.timestamp*1e-6 <= VEHICLE_STATUS_TIMEOUT_S):
            return
        if self.vehicle_status is not None and msg.timestamp <= self.vehicle_status.timestamp:
            return
        self.vehicle_status = msg
        self.state_received["status"] = time.monotonic()

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        if (msg.timestamp_sample == 0 or not -SIM_CLOCK_TOLERANCE_S
                <= self._now_s()-msg.timestamp_sample*1e-6 <= self.state_timeout_s):
            return
        if (self.vehicle_local_position is not None
                and msg.timestamp_sample <= self.vehicle_local_position.timestamp_sample):
            return
        self.vehicle_local_position = msg
        self.state_received["position"] = time.monotonic()

    def _vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        q = tuple(float(v) for v in msg.q)
        if (msg.pose_frame != VehicleOdometry.POSE_FRAME_NED
                or msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED
                or not all(math.isfinite(v) for v in (*msg.position, *msg.velocity, *q))
                or abs(sum(v*v for v in q)-1.0) > 0.01
                or not -SIM_CLOCK_TOLERANCE_S <= self._now_s()-msg.timestamp_sample*1e-6 <= self.state_timeout_s
                or msg.timestamp_sample == 0):
            return
        if (self.vehicle_odometry is not None
                and msg.timestamp_sample <= self.vehicle_odometry.timestamp_sample):
            return
        if self.odometry_reset_counter is not None and msg.reset_counter != self.odometry_reset_counter:
            self._reset_observations()
        self.odometry_reset_counter = msg.reset_counter
        norm = math.sqrt(sum(v*v for v in q))
        q = tuple(v/norm for v in q)
        msg.q = list(q)
        self.vehicle_odometry = msg
        self.state_received["odometry"] = time.monotonic()
        self.attitudes.append((msg.timestamp_sample*1e-6, q))
        self._drain_bearings()

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = (msg if valid_camera_info(msg)
                            and msg.header.frame_id == self.get_parameter("optical_frame_id").value
                            else None)

    def _reset_observations(self) -> None:
        self.guidance_observation = None
        self.last_projection_sample_us = None
        self.last_pursuit_time_s = None
        self.attitudes.clear()
        self.pending_bearings.clear()
        self.last_bearing_time_s = None
        self.last_camera_stamp_s = None
        self.measured_los_ned = None
        self._reset_los_filter()
        self._reset_guidance_state()

    def _bearing_callback(self, msg: Vector3Stamped) -> None:
        # The header is Gazebo acquisition time; the vector is an optical unit ray.
        stamp = stamp_to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec)
        ray = (msg.vector.x, msg.vector.y, msg.vector.z)
        if (stamp is None or msg.header.frame_id != self.get_parameter("optical_frame_id").value
                or not all(math.isfinite(v) for v in ray) or ray[2] <= 0
                or abs(vector_norm(ray)-1.0) > 1e-3):
            return
        if self.last_camera_stamp_s is not None and stamp <= self.last_camera_stamp_s:
            return
        # PX4 and image headers share Gazebo time (UXRCE_DDS_SYNCT=0).
        if not -SIM_CLOCK_TOLERANCE_S <= self._now_s()-stamp <= self._observation_horizon():
            return
        self.last_camera_stamp_s = stamp
        self.pending_bearings.append((stamp, ray))
        self._drain_bearings()

    def _observation_horizon(self) -> float:
        return min(self.observation_timeout_s, self.los_filter_max_prediction_s) if self.los_filter_enabled else self.observation_timeout_s

    def _drain_bearings(self) -> None:
        now = self._now_s()
        while self.pending_bearings:
            stamp, optical = self.pending_bearings[0]
            if now < stamp:
                break  # Wait for /clock; never predict backward.
            if now-stamp > self._observation_horizon():
                self.pending_bearings.popleft()
                continue
            if not self.attitudes or stamp > self.attitudes[-1][0]:
                break  # Wait for the right-hand attitude sample; never extrapolate.
            self.pending_bearings.popleft()
            if (stamp < self.attitudes[0][0] or (self.last_bearing_time_s is not None
                    and stamp <= self.last_bearing_time_s)):
                continue
            right = next(i for i, sample in enumerate(self.attitudes) if sample[0] >= stamp)
            tb, qb = self.attitudes[right]
            ta, qa = (tb, qb) if tb == stamp else self.attitudes[right-1]
            if tb-ta > self.attitude_max_gap_s:
                continue
            q = slerp(qa, qb, (stamp-ta)/(tb-ta) if tb > ta else 0.0)
            body = fixed_camera_bearing_to_body_los(optical, self.camera_to_body_flu)
            los = normalize(rotate_body_to_ned(q, body))
            vertical, horizontal = ned_direction_angles(los)
            if self.last_bearing_time_s is None or stamp-self.last_bearing_time_s > self._observation_horizon():
                self._reset_los_filter()
            if self.los_filter_last_horizontal_rad is not None:
                horizontal = self.los_filter_last_horizontal_rad + angle_delta_rad(horizontal, self.los_filter_last_horizontal_rad)
            self.los_filter_last_horizontal_rad = horizontal
            self.image_yaw_error_rad = math.atan2(optical[0], optical[2])
            self.image_pitch_error_rad = math.atan2(optical[1], optical[2])
            self.last_los_body = body
            self.measured_los_ned = los
            self.last_bearing_time_s = stamp
            self.los_filter_last_measurement_stamp_s = stamp
            self.los_filter_measurement_time_source = "acquisition_sim_attitude_slerp"
            self.los_filter_measurement_delay_observed_s = now-stamp
            if self.los_filter_enabled:
                self.los_filter.update(horizontal, vertical, stamp)

    def _tracking_active_callback(self, msg: Bool) -> None:
        now_s = self._now_s()
        self.tracking_active = bool(msg.data)
        self.last_tracking_active_time_s = now_s
        if self.tracking_active:
            if self.tracking_true_since_s is None:
                self.tracking_true_since_s = now_s
            self.last_tracking_true_time_s = now_s
        else:
            self.tracking_true_since_s = None

    def _lock_active_callback(self, msg: Bool) -> None:
        now_s = self._now_s()
        self.lock_active = bool(msg.data)
        self.last_lock_active_time_s = now_s
        if self.lock_active:
            if self.lock_true_since_s is None:
                self.lock_true_since_s = now_s
            self.last_lock_true_time_s = now_s
        else:
            self.lock_true_since_s = None

    def _timer_callback(self) -> None:
        now_s = self._now_s()
        now_us = int(now_s * 1_000_000)
        dt_s = self._control_dt_s(now_s)
        self.previous_state = self.state
        self._drain_bearings()
        vehicle_ready = self._vehicle_ready()
        if not vehicle_ready:
            self.vertical_search_start_time_s = None
            self.state = InterceptorState.INITIALIZING
            self.setpoint_counter = 0
            self._reset_guidance_state()
            self._publish_diagnostics(now_us, False, False, vehicle_ready=vehicle_ready)
            return
        pursuing = self._ready_to_pursue(now_s)
        velocity_control = self.state in GUIDANCE_STATES

        self._publish_offboard_control_mode(
            now_us,
            velocity_control,
        )
        self._publish_setpoint(now_us, pursuing, velocity_control, dt_s)

        warmup_cycles = max(1, int(self.takeoff_warmup_s * self.control_rate_hz))
        if (
            self.setpoint_counter >= warmup_cycles
            and self._has_setpoint_available()
        ):
            self._request_offboard_and_arm_if_needed(now_us)

        self._publish_diagnostics(
            now_us,
            pursuing,
            velocity_control,
            vehicle_ready=vehicle_ready,
        )
        self.setpoint_counter += 1

    def _ready_to_pursue(self, now_s: float) -> bool:
        """Advance pursuit state after the control cycle's vehicle readiness check."""
        if not self.takeoff_altitude_reached:
            self._capture_takeoff_position_if_needed()
            self.state = InterceptorState.TAKEOFF
            if (
                self.takeoff_position is not None
                and abs(
                    float(self.vehicle_local_position.z)
                    - self.takeoff_position[2]
                )
                <= self.hover_acceptance_radius_m
            ):
                self.takeoff_altitude_reached = True
                self.hold_position = self.initial_hover_position
                self.state = InterceptorState.TRANSIT
            return False

        if not self.initial_hover_reached:
            self.state = InterceptorState.TRANSIT
            self.hold_position = self.initial_hover_position
            if (
                self._distance_to(self.initial_hover_position)
                <= self.hover_acceptance_radius_m
            ):
                self.initial_hover_reached = True
                self.state = InterceptorState.HOLD
            return False

        if self._prepare_guidance_observation(now_s) is None:
            if (
                self.previous_state in GUIDANCE_STATES
                and self._pursuit_recently_active(now_s)
            ):
                self.state = InterceptorState.COAST
                return False
            self._capture_loss_hold_position_if_needed()
            self.state = (
                InterceptorState.ACQUIRING
                if self._tracking_signal_active(now_s) or self._lock_signal_active(now_s)
                else InterceptorState.TARGET_LOST
            )
            return False

        if self._lock_signal_active(now_s):
            self.state = InterceptorState.PURSUIT
            self.last_pursuit_time_s = self.vehicle_odometry.timestamp_sample * 1e-6
            return True

        if (
            self.previous_state in GUIDANCE_STATES
            and self._pursuit_recently_active(now_s)
        ):
            self.state = InterceptorState.COAST
            return False

        if self._tracking_signal_active(now_s) or self._lock_signal_active(now_s):
            self._capture_loss_hold_position_if_needed()
            self.state = InterceptorState.ACQUIRING
            return False

        self._capture_loss_hold_position_if_needed()
        self.state = InterceptorState.TARGET_LOST
        return False

    def _vehicle_ready(self) -> bool:
        p, a, status = self.vehicle_local_position, self.vehicle_odometry, self.vehicle_status
        return bool(p is not None and a is not None and status is not None
                    and self._state_fresh("status", status.timestamp)
                    and self._state_fresh("position", p.timestamp_sample)
                    and self._state_fresh("odometry", a.timestamp_sample)
                    and p.xy_valid and p.z_valid and p.v_xy_valid and p.v_z_valid
                    and all(math.isfinite(v) for v in (p.x, p.y, p.z, p.vx, p.vy, p.vz))
                    and all(math.isfinite(v) for v in a.q) and sum(v*v for v in a.q) > 1e-12)

    def _has_setpoint_available(self) -> bool:
        return (
            self.hold_position is not None
            or self.state in GUIDANCE_STATES
        )

    def _capture_loss_hold_position_if_needed(self) -> None:
        if not self.hold_position_on_loss:
            return
        if self.previous_state not in GUIDANCE_STATES:
            return
        if (
            self.vehicle_local_position is None
            or not self.vehicle_local_position.xy_valid
            or not self.vehicle_local_position.z_valid
        ):
            return
        self.hold_position = (
            float(self.vehicle_local_position.x),
            float(self.vehicle_local_position.y),
            float(self.vehicle_local_position.z),
        )

    def _capture_takeoff_position_if_needed(self) -> None:
        if self.takeoff_position is not None:
            return
        if (
            self.vehicle_local_position is None
            or not self.vehicle_local_position.xy_valid
            or not self.vehicle_local_position.z_valid
        ):
            return
        self.takeoff_position = (
            float(self.vehicle_local_position.x),
            float(self.vehicle_local_position.y),
            self.initial_hover_position[2],
        )
        self.hold_position = self.takeoff_position

    def _distance_to(self, target: tuple[float, float, float]) -> float:
        assert self.vehicle_local_position is not None
        dx = float(self.vehicle_local_position.x) - target[0]
        dy = float(self.vehicle_local_position.y) - target[1]
        dz = float(self.vehicle_local_position.z) - target[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _tracking_signal_active(self, now_s: float) -> bool:
        if not self.tracking_active or self.last_tracking_active_time_s is None:
            return False
        return now_s - self.last_tracking_active_time_s <= self.tracking_status_timeout_s

    def _lock_signal_active(self, now_s: float) -> bool:
        if not self.lock_active or self.last_lock_active_time_s is None:
            return False
        return now_s - self.last_lock_active_time_s <= self.tracking_status_timeout_s

    def _pursuit_recently_active(self, now_s: float) -> bool:
        return (self.last_pursuit_time_s is not None
                and now_s - self.last_pursuit_time_s <= self.lock_loss_grace_s)

    def _fresh_bearing(self, now_s: float) -> bool:
        if (
            self.image_yaw_error_rad is None
            or self.image_pitch_error_rad is None
            or self.last_bearing_time_s is None
        ):
            return False
        return (self.measured_los_ned is not None
                and 0.0 <= now_s - self.last_bearing_time_s <= self._observation_horizon())

    def _publish_offboard_control_mode(
        self, now_us: int, velocity_control: bool,
    ) -> None:
        msg = OffboardControlMode()
        msg.position = not velocity_control
        msg.velocity = velocity_control
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = now_us
        self.offboard_mode_pub.publish(msg)

    def _publish_setpoint(
        self,
        now_us: int,
        pursuing: bool,
        velocity_control: bool,
        dt_s: float,
    ) -> None:
        assert self.vehicle_local_position is not None or not pursuing

        if pursuing or velocity_control:
            self._stop_vertical_search()

        if pursuing:
            self._publish_pursuit_setpoint(now_us)
            return

        if velocity_control:
            self._publish_coast_setpoint(now_us, dt_s)
            return

        self._reset_guidance_state()
        self._publish_hold_setpoint(now_us)

    def _publish_pursuit_setpoint(self, now_us: int) -> None:
        assert self.vehicle_odometry is not None and self.guidance_observation is not None
        sample_us = self.vehicle_odometry.timestamp_sample
        if sample_us != self.last_guidance_sample_us:
            # Advance all differences together, once per accepted state sample.
            dt_s = (0.0 if self.last_guidance_sample_us is None else
                    (sample_us - self.last_guidance_sample_us) * 1e-6)
            self._visual_png_velocity_setpoint(self.guidance_observation, dt_s)
            self.last_guidance_sample_us = sample_us
            self.last_los_ned = self.guidance_observation
            if not self.hold_position_on_loss:
                self.hold_position = tuple(float(v) for v in self.vehicle_odometry.position)

        # Publication cadence maintains velocity and yaw without advancing guidance.
        self._publish_velocity_setpoint(now_us, self.last_velocity_setpoint_ned)

    def _publish_coast_setpoint(self, now_us: int, dt_s: float) -> None:
        if self.previous_state == InterceptorState.PURSUIT:
            # Initialize braking from measured motion, not the pursuit reference.
            self._reset_guidance_state()
            self.last_velocity_setpoint_ned = self._current_velocity_ned()
        decay = math.exp(-max(dt_s, 0.0) / self.coast_velocity_decay_s)
        self.last_velocity_setpoint_ned = scale(self.last_velocity_setpoint_ned, decay)
        self.last_commanded_closing_speed_mps = 0.0
        self._publish_velocity_setpoint(now_us, self.last_velocity_setpoint_ned)

    def _publish_velocity_setpoint(self, now_us: int, velocity_ned: Vector3) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = now_us
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = list(velocity_ned)
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = self._yaw_from_los(self.last_los_ned)
        msg.yawspeed = math.nan
        self.trajectory_pub.publish(msg)

    def _prepare_guidance_observation(self, now_s: float) -> Vector3 | None:
        """Predict and project at the same epoch as the attitude and velocity."""
        info, state = self.camera_info, self.vehicle_odometry
        if info is None or state is None or not self._fresh_bearing(now_s):
            self.guidance_observation = None
            return None
        sample_s = state.timestamp_sample * 1e-6
        if not self._fresh_bearing(sample_s):
            self.guidance_observation = None
            return None
        if state.timestamp_sample != self.last_projection_sample_us:
            self.last_projection_sample_us = state.timestamp_sample
            self.guidance_observation = None
            los = self._guidance_los_ned(sample_s)
            optical = ned_los_to_optical(tuple(state.q), los, self.body_to_camera_flu)
            if all(math.isfinite(v) for v in optical) and optical[2] > 0.0:
                u = info.k[0] * optical[0] / optical[2] + info.k[2]
                v = info.k[4] * optical[1] / optical[2] + info.k[5]
                if 0.0 <= u < info.width and 0.0 <= v < info.height:
                    self.guidance_observation = los
        return self.guidance_observation

    def _guidance_los_ned(self, now_s: float) -> Vector3:
        assert self._fresh_bearing(now_s)
        if self.los_filter_enabled:
            self._update_los_prediction(now_s)
            horizontal, vertical = self.los_filter_horizontal_rad, self.los_filter_vertical_rad
            assert horizontal is not None and vertical is not None
            return direction_from_ned_angles(vertical, horizontal)
        return self.measured_los_ned

    def _update_los_prediction(self, now_s: float) -> None:
        estimate = self.los_filter.predict(now_s) if self._fresh_bearing(now_s) else None
        if estimate is None:
            self.los_filter_horizontal_rad = None
            self.los_filter_vertical_rad = None
            self.los_filter_horizontal_rate_rad_s = None
            self.los_filter_vertical_rate_rad_s = None
            self.los_filter_prediction_horizon_s = None
            return

        (
            self.los_filter_horizontal_rad,
            self.los_filter_vertical_rad,
            self.los_filter_horizontal_rate_rad_s,
            self.los_filter_vertical_rate_rad_s,
            self.los_filter_prediction_horizon_s,
        ) = estimate

    def _reset_los_filter(self) -> None:
        self.los_filter.reset()
        self.los_filter_horizontal_rad = None
        self.los_filter_vertical_rad = None
        self.los_filter_horizontal_rate_rad_s = None
        self.los_filter_vertical_rate_rad_s = None
        self.los_filter_prediction_horizon_s = None
        self.los_filter_last_horizontal_rad = None
        self.los_filter_last_measurement_stamp_s = None
        self.los_filter_measurement_time_source = "none"
        self.los_filter_measurement_delay_observed_s = None

    def _visual_png_velocity_setpoint(self, los_ned: Vector3, dt_s: float) -> Vector3:
        los_vertical_rad, los_horizontal_rad = ned_direction_angles(los_ned)
        current_velocity_ned = self._current_velocity_ned()
        current_speed_mps = vector_norm(current_velocity_ned)

        # Zero/low speed has no usable direction. Do not carry an older direction
        # across a low-speed interval into the next PNG difference.
        if current_speed_mps > 0.0 and current_speed_mps >= self.min_velocity_direction_mps:
            velocity_vertical_rad, velocity_horizontal_rad = ned_direction_angles(
                current_velocity_ned
            )
        else:
            velocity_vertical_rad = velocity_horizontal_rad = None

        if (
            velocity_vertical_rad is None
            or self.last_png_velocity_vertical_angle_rad is None
            or self.last_png_velocity_horizontal_angle_rad is None
            or self.last_png_desired_vertical_angle_rad is None
            or self.last_png_desired_horizontal_angle_rad is None
            or self.last_png_los_vertical_angle_rad is None
            or self.last_png_los_horizontal_angle_rad is None
            or dt_s <= 1e-6
        ):
            desired_vertical_rad = los_vertical_rad
            desired_horizontal_rad = los_horizontal_rad
        else:
            # Integrate the guidance reference; measured velocity can lag the command.
            desired_vertical_rad = self.last_png_desired_vertical_angle_rad + self.png_vertical_gain * angle_delta_rad(
                los_vertical_rad,
                self.last_png_los_vertical_angle_rad,
            )
            desired_horizontal_rad = self.last_png_desired_horizontal_angle_rad + self.png_horizontal_gain * angle_delta_rad(
                los_horizontal_rad,
                self.last_png_los_horizontal_angle_rad,
            )

        desired_vertical_rad = clamp(
            desired_vertical_rad,
            -0.5 * math.pi + 1e-3,
            0.5 * math.pi - 1e-3,
        )
        desired_horizontal_rad = wrap_angle_rad(desired_horizontal_rad)
        desired_direction_ned = direction_from_ned_angles(
            desired_vertical_rad,
            desired_horizontal_rad,
        )
        if dot(desired_direction_ned, los_ned) <= 0.0:
            # A large LOS innovation must not command motion away from the target.
            desired_vertical_rad, desired_horizontal_rad = los_vertical_rad, los_horizontal_rad
            desired_direction_ned = los_ned
        # Ramp the reference independently of measured speed so the PX4 velocity
        # loop receives a sustained tracking error, including under disturbances.
        previous_speed = (current_speed_mps if self.last_png_los_vertical_angle_rad is None
                          else vector_norm(self.last_velocity_setpoint_ned))
        desired_speed = min(self.max_pursuit_speed_mps,
                            previous_speed + self.speed_accel_mps2 * dt_s)
        velocity_setpoint_ned = scale(desired_direction_ned, desired_speed)

        self.last_png_los_vertical_angle_rad = los_vertical_rad
        self.last_png_los_horizontal_angle_rad = los_horizontal_rad
        self.last_png_velocity_vertical_angle_rad = velocity_vertical_rad
        self.last_png_velocity_horizontal_angle_rad = velocity_horizontal_rad
        self.last_png_desired_vertical_angle_rad = desired_vertical_rad
        self.last_png_desired_horizontal_angle_rad = desired_horizontal_rad
        self.last_commanded_closing_speed_mps = dot(velocity_setpoint_ned, los_ned)
        self.last_closing_speed_mps = dot(current_velocity_ned, los_ned)
        self.last_velocity_setpoint_ned = velocity_setpoint_ned
        return velocity_setpoint_ned

    def _current_velocity_ned(self) -> Vector3:
        if self.vehicle_odometry is not None:
            return tuple(float(v) for v in self.vehicle_odometry.velocity)
        return (0.0, 0.0, 0.0)

    def _reset_guidance_state(self) -> None:
        self.last_guidance_sample_us = None
        self.last_velocity_setpoint_ned = (0.0, 0.0, 0.0)
        self.last_commanded_closing_speed_mps = 0.0
        self.last_png_los_vertical_angle_rad = None
        self.last_png_los_horizontal_angle_rad = None
        self.last_png_velocity_vertical_angle_rad = None
        self.last_png_velocity_horizontal_angle_rad = None
        self.last_png_desired_vertical_angle_rad = None
        self.last_png_desired_horizontal_angle_rad = None

    def _update_hold_position_from_current(self) -> None:
        if (
            self.vehicle_local_position is None
            or not self.vehicle_local_position.xy_valid
            or not self.vehicle_local_position.z_valid
        ):
            return
        self.hold_position = (
            float(self.vehicle_local_position.x),
            float(self.vehicle_local_position.y),
            float(self.vehicle_local_position.z),
        )

    def _stop_vertical_search(self) -> None:
        if self.vertical_search_start_time_s is not None:
            self.hold_position = (
                self.hold_position[0], self.hold_position[1], float(self.vehicle_local_position.z))
            self.vertical_search_start_time_s = None

    def _publish_hold_setpoint(self, now_us: int) -> None:
        if self.takeoff_position is None:
            self._capture_takeoff_position_if_needed()

        if (not self.hold_position_on_loss and self.vertical_search_start_time_s is None
                or self.hold_position is None):
            self._update_hold_position_from_current()

        if self.hold_position is None:
            return

        now_s = now_us * 1e-6
        observation = self._prepare_guidance_observation(now_s)
        bearing_fresh = observation is not None
        if not self.initial_hover_reached or bearing_fresh:
            self._stop_vertical_search()
        elif self.search_vertical_amplitude_m > 0.0 and self.vehicle_odometry is not None:
            if self.vertical_search_start_time_s is None:
                self.hold_position = (
                    self.hold_position[0], self.hold_position[1], float(self.vehicle_local_position.z))
                self.vertical_search_start_time_s = now_s

        self.state = (
            InterceptorState.HOLD
            if self.state == InterceptorState.INITIALIZING
            else self.state
        )
        msg = TrajectorySetpoint()
        msg.position = [self.hold_position[0], self.hold_position[1], self.hold_position[2]]
        if self.vertical_search_start_time_s is not None:
            phase = 2.0 * math.pi * (now_s - self.vertical_search_start_time_s) / self.search_vertical_period_s
            msg.position[2] = clamp(
                self.hold_position[2] - self.search_vertical_amplitude_m * math.sin(phase),
                self.search_vertical_min_z_ned, self.search_vertical_max_z_ned)
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = self._yaw_from_los(self.last_los_ned)
        msg.yawspeed = math.nan
        if self.initial_hover_reached:
            if bearing_fresh and self.vehicle_odometry is not None:
                self.last_los_ned = observation
                msg.yaw = self._yaw_from_los(self.last_los_ned)
                self.search_yaw_rad = msg.yaw
            elif self.vehicle_odometry is not None:
                if (self.search_yaw_rad is None or self.last_search_yaw_time_s is None
                        or now_s - self.last_search_yaw_time_s > 0.25):
                    forward = rotate_body_to_ned(tuple(self.vehicle_odometry.q), (1.0, 0.0, 0.0))
                    self.search_yaw_rad = math.atan2(forward[1], forward[0])
                dt_s = (0.0 if self.last_search_yaw_time_s is None else
                        clamp(now_s - self.last_search_yaw_time_s, 0.0, 0.25))
                self.search_yaw_rad = wrap_angle_rad(
                    self.search_yaw_rad + self.search_yaw_rate_rad_s * dt_s
                )
                msg.yaw = self.search_yaw_rad
                msg.yawspeed = self.search_yaw_rate_rad_s
            self.last_search_yaw_time_s = now_s
        msg.timestamp = now_us
        self.trajectory_pub.publish(msg)

    def _yaw_from_los(self, los_ned: tuple[float, float, float]) -> float:
        if self.yaw_mode == "fixed_north":
            return 0.0
        if self.yaw_mode != "face_los":
            raise ValueError(f"Unsupported yaw_mode: {self.yaw_mode!r}")
        return math.atan2(los_ned[1], los_ned[0])

    def _validate_yaw_mode(self) -> None:
        valid_modes = {"fixed_north", "face_los"}
        if self.yaw_mode not in valid_modes:
            raise ValueError(
                f"yaw_mode must be one of {sorted(valid_modes)}; "
                f"got {self.yaw_mode!r}"
            )

    def _request_offboard_and_arm_if_needed(self, now_us: int) -> None:
        request_interval_us = 1_000_000

        if (
            self.vehicle_status is None
            or self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            if now_us - self.last_mode_request_us >= request_interval_us:
                self._publish_vehicle_command(
                    now_us,
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,
                    param2=6.0,
                )
                self.last_mode_request_us = now_us
                self.get_logger().info("Requested Offboard mode.")

        if (
            self.vehicle_status is None
            or self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED
        ):
            if now_us - self.last_arm_request_us >= request_interval_us:
                self._publish_vehicle_command(
                    now_us,
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0,
                )
                self.last_arm_request_us = now_us
                self.get_logger().info("Sent arm command.")

    def _publish_vehicle_command(
        self,
        now_us: int,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = now_us
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = int(command)
        msg.target_system = self.target_system
        msg.target_component = self.target_component
        msg.source_system = self.source_system
        msg.source_component = self.source_component
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def _publish_diagnostics(
        self,
        now_us: int,
        pursuing: bool,
        velocity_control: bool,
        *,
        vehicle_ready: bool,
    ) -> None:
        msg = DiagnosticArray()
        msg.header.stamp.sec = int(now_us // 1_000_000)
        msg.header.stamp.nanosec = int((now_us % 1_000_000) * 1000)

        status = DiagnosticStatus()
        status.name = "visual_pursuit_interceptor"
        status.hardware_id = f"target_system={self.target_system}"
        status.level = DiagnosticStatus.OK if pursuing else DiagnosticStatus.WARN
        status.message = self.state
        now_s = now_us * 1e-6
        current_velocity_ned = self._current_velocity_ned()
        bearing_fresh = self._fresh_bearing(now_s)
        los_filter_ready = (
            bearing_fresh
            and self.los_filter_horizontal_rad is not None
            and self.los_filter_vertical_rad is not None
        )
        status.values = [
            diagnostic_value("state", self.state),
            diagnostic_value("vehicle_state_fresh", vehicle_ready),
            diagnostic_value("pursuing", pursuing),
            diagnostic_value("velocity_control_active", velocity_control),
            diagnostic_value("takeoff_altitude_reached", self.takeoff_altitude_reached),
            diagnostic_value("takeoff_x_m", point_value(self.takeoff_position, 0)),
            diagnostic_value("takeoff_y_m", point_value(self.takeoff_position, 1)),
            diagnostic_value("takeoff_z_m", point_value(self.takeoff_position, 2)),
            diagnostic_value("initial_hover_reached", self.initial_hover_reached),
            diagnostic_value("hold_x_m", point_value(self.hold_position, 0)),
            diagnostic_value("hold_y_m", point_value(self.hold_position, 1)),
            diagnostic_value("hold_z_m", point_value(self.hold_position, 2)),
            diagnostic_value("detection_active", self.tracking_active),
            diagnostic_value("lock_active", self.lock_active),
            diagnostic_value(
                "detection_true_duration_s",
                self._detection_true_duration_s(now_s),
            ),
            diagnostic_value(
                "lock_true_duration_s",
                self._lock_true_duration_s(now_s),
            ),
            diagnostic_value(
                "detection_loss_age_s",
                self._detection_loss_age_s(now_s),
            ),
            diagnostic_value("lock_loss_grace_s", self.lock_loss_grace_s),
            diagnostic_value(
                "lock_loss_age_s",
                self._lock_loss_age_s(now_s),
            ),
            diagnostic_value("png_vertical_gain", self.png_vertical_gain),
            diagnostic_value("png_horizontal_gain", self.png_horizontal_gain),
            diagnostic_value(
                "commanded_closing_speed_mps",
                self.last_commanded_closing_speed_mps,
            ),
            diagnostic_value("bearing_fresh", bearing_fresh),
            diagnostic_value("guidance_projection_valid",
                             vehicle_ready and bearing_fresh and self.guidance_observation is not None),
            diagnostic_value("guidance_sample_timestamp_us", self.last_guidance_sample_us),
            diagnostic_value(
                "image_yaw_error_deg",
                degrees_or_none(self.image_yaw_error_rad),
            ),
            diagnostic_value(
                "image_pitch_error_deg",
                degrees_or_none(self.image_pitch_error_rad),
            ),
            diagnostic_value(
                "bearing_age_s",
                self._bearing_age_s(now_s),
            ),
            diagnostic_value("los_filter_enabled", self.los_filter_enabled),
            diagnostic_value("los_filter_ready", los_filter_ready),
            diagnostic_value(
                "los_filter_measurement_time_source",
                self.los_filter_measurement_time_source,
            ),
            diagnostic_value(
                "los_filter_measurement_delay_observed_s",
                self.los_filter_measurement_delay_observed_s,
            ),
            diagnostic_value(
                "los_filter_measurement_stamp_s",
                self.los_filter_last_measurement_stamp_s,
            ),
            diagnostic_value(
                "los_filter_prediction_horizon_s",
                self.los_filter_prediction_horizon_s,
            ),
            diagnostic_value(
                "los_filter_horizontal_deg",
                degrees_or_none(self.los_filter_horizontal_rad),
            ),
            diagnostic_value(
                "los_filter_vertical_deg",
                degrees_or_none(self.los_filter_vertical_rad),
            ),
            diagnostic_value(
                "los_filter_horizontal_rate_deg_s",
                degrees_or_none(self.los_filter_horizontal_rate_rad_s),
            ),
            diagnostic_value(
                "los_filter_vertical_rate_deg_s",
                degrees_or_none(self.los_filter_vertical_rate_rad_s),
            ),
            diagnostic_value("closing_speed_mps", self.last_closing_speed_mps),
            diagnostic_value("los_body_x", self.last_los_body[0]),
            diagnostic_value("los_body_y", self.last_los_body[1]),
            diagnostic_value("los_body_z", self.last_los_body[2]),
            diagnostic_value("los_ned_x", self.last_los_ned[0]),
            diagnostic_value("los_ned_y", self.last_los_ned[1]),
            diagnostic_value("los_ned_z", self.last_los_ned[2]),
            diagnostic_value(
                "png_los_vertical_angle_deg",
                degrees_or_none(self.last_png_los_vertical_angle_rad),
            ),
            diagnostic_value(
                "png_los_horizontal_angle_deg",
                degrees_or_none(self.last_png_los_horizontal_angle_rad),
            ),
            diagnostic_value(
                "png_desired_vertical_angle_deg",
                degrees_or_none(self.last_png_desired_vertical_angle_rad),
            ),
            diagnostic_value(
                "png_desired_horizontal_angle_deg",
                degrees_or_none(self.last_png_desired_horizontal_angle_rad),
            ),
            diagnostic_value("png_actual_vertical_angle_deg",
                             degrees_or_none(self.last_png_velocity_vertical_angle_rad)),
            diagnostic_value("png_actual_horizontal_angle_deg",
                             degrees_or_none(self.last_png_velocity_horizontal_angle_rad)),
            diagnostic_value(
                "velocity_setpoint_ned_x_mps",
                self.last_velocity_setpoint_ned[0],
            ),
            diagnostic_value(
                "velocity_setpoint_ned_y_mps",
                self.last_velocity_setpoint_ned[1],
            ),
            diagnostic_value(
                "velocity_setpoint_ned_z_mps",
                self.last_velocity_setpoint_ned[2],
            ),
            diagnostic_value("velocity_ned_x_mps", current_velocity_ned[0]),
            diagnostic_value("velocity_ned_y_mps", current_velocity_ned[1]),
            diagnostic_value("velocity_ned_z_mps", current_velocity_ned[2]),
        ]
        msg.status.append(status)
        self.diagnostics_pub.publish(msg)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _control_dt_s(self, now_s: float) -> float:
        if self.last_control_time_s is None:
            self.last_control_time_s = now_s
            return 1.0 / max(self.control_rate_hz, 1.0)

        dt_s = max(0.0, now_s - self.last_control_time_s)
        self.last_control_time_s = now_s
        return min(dt_s, 0.25)

    def _detection_true_duration_s(self, now_s: float) -> float | None:
        if (
            not self._tracking_signal_active(now_s)
            or self.tracking_true_since_s is None
        ):
            return None
        return max(0.0, now_s - self.tracking_true_since_s)

    def _lock_true_duration_s(self, now_s: float) -> float | None:
        if not self._lock_signal_active(now_s) or self.lock_true_since_s is None:
            return None
        return max(0.0, now_s - self.lock_true_since_s)

    def _detection_loss_age_s(self, now_s: float) -> float | None:
        if self.tracking_active or self.last_tracking_true_time_s is None:
            return None
        return max(0.0, now_s - self.last_tracking_true_time_s)

    def _lock_loss_age_s(self, now_s: float) -> float | None:
        if self.lock_active or self.last_lock_true_time_s is None:
            return None
        return max(0.0, now_s - self.last_lock_true_time_s)

    def _bearing_age_s(self, now_s: float) -> float | None:
        if self.last_bearing_time_s is None:
            return None
        return max(0.0, now_s - self.last_bearing_time_s)


def fixed_camera_bearing_to_body_los(
    optical_ray: Vector3,
    camera_to_body_flu: Matrix3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
) -> Vector3:
    """Optical right/down/forward -> body FRD; ray origin remains the camera.

    Translation cannot re-anchor the ray without a target range measurement.
    """
    ray_flu = (optical_ray[2], -optical_ray[0], -optical_ray[1])
    return normalize(gazebo_flu_to_px4_frd(matvec3(camera_to_body_flu, ray_flu)))


def gazebo_flu_to_px4_frd(vector: Vector3) -> Vector3:
    return (vector[0], -vector[1], -vector[2])


def rotation_from_rpy(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> Matrix3:
    return matmul3(
        rotation_z(yaw_rad),
        matmul3(rotation_y(pitch_rad), rotation_x(roll_rad)),
    )


def rotation_x(angle_rad: float) -> Matrix3:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def rotation_y(angle_rad: float) -> Matrix3:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def rotation_z(angle_rad: float) -> Matrix3:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def matmul3(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            sum(left[row][index] * right[index][col] for index in range(3))
            for col in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


def matvec3(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(
        sum(matrix[row][col] * vector[col] for col in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def rotate_body_to_ned(
    q_body_to_ned: tuple[float, float, float, float],
    vector_body: tuple[float, float, float],
) -> tuple[float, float, float]:
    w, x, y, z = q_body_to_ned
    vx, vy, vz = vector_body

    # q * [0, v] * q_conjugate
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def ned_los_to_optical(q: tuple[float, float, float, float], los: Vector3,
                       body_to_camera_flu: Matrix3) -> Vector3:
    body = rotate_body_to_ned((q[0], -q[1], -q[2], -q[3]), los)
    camera_flu = matvec3(body_to_camera_flu, gazebo_flu_to_px4_frd(body))
    return (-camera_flu[1], -camera_flu[2], camera_flu[0])


def ned_direction_angles(vector: Vector3) -> tuple[float, float]:
    normalized = normalize(vector)
    horizontal_norm = math.hypot(normalized[0], normalized[1])
    vertical_angle_rad = math.atan2(normalized[2], horizontal_norm)
    horizontal_angle_rad = math.atan2(normalized[1], normalized[0])
    return vertical_angle_rad, horizontal_angle_rad


def direction_from_ned_angles(
    vertical_angle_rad: float,
    horizontal_angle_rad: float,
) -> Vector3:
    horizontal_scale = math.cos(vertical_angle_rad)
    return normalize(
        (
            horizontal_scale * math.cos(horizontal_angle_rad),
            horizontal_scale * math.sin(horizontal_angle_rad),
            math.sin(vertical_angle_rad),
        )
    )


def wrap_angle_rad(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def angle_delta_rad(current_rad: float, previous_rad: float) -> float:
    return wrap_angle_rad(current_rad - previous_rad)


def normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = vector_norm(vector)
    if norm <= 1e-9:
        return (1.0, 0.0, 0.0)
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def vector_norm(vector: Vector3) -> float:
    return math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)


def dot(left: Vector3, right: Vector3) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def scale(vector: tuple[float, float, float], scalar: float) -> tuple[float, float, float]:
    return (vector[0] * scalar, vector[1] * scalar, vector[2] * scalar)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def positive_float(value: Any, name: str) -> float:
    number = float(value)
    if number <= 0.0:
        raise ValueError(f"{name} must be positive.")
    return number


def nonnegative_float(value: Any, name: str) -> float:
    number = float(value)
    if number < 0.0:
        raise ValueError(f"{name} must be non-negative.")
    return number


def parse_point(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"{name} must be [x, y, z].")
    return (float(value[0]), float(value[1]), float(value[2]))


def degrees_or_none(value: float | None) -> float | None:
    return None if value is None else math.degrees(value)


def point_value(point: tuple[float, float, float] | None, index: int) -> float | None:
    return None if point is None else point[index]


def stamp_to_seconds(sec: int, nanosec: int) -> float | None:
    if int(sec) == 0 and int(nanosec) == 0:
        return None
    return float(sec) + float(nanosec) * 1e-9


def diagnostic_value(key: str, value: bool | float | str | None) -> KeyValue:
    item = KeyValue()
    item.key = key
    if isinstance(value, bool):
        item.value = str(value).lower()
    elif value is None:
        item.value = "nan"
    elif isinstance(value, float):
        item.value = f"{value:.6g}"
    else:
        item.value = str(value)
    return item


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VisualPursuitInterceptor()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
