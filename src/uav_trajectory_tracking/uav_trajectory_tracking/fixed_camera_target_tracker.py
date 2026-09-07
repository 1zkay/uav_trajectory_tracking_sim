#!/usr/bin/env python3
"""Convert tracked image coordinates to calibrated fixed-camera observations.

bearing is a unit vector in the camera optical frame, stamped at acquisition.
The scene contains one target; track IDs do not gate observations.
Confidence remains in Detection2DArray and is used only for target selection.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray

from .state_comparison import SIM_CLOCK_TOLERANCE_S


def valid_camera_info(msg: CameraInfo) -> bool:
    """Full-resolution, undistorted simulation pinhole camera contract."""
    fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
    return (all(math.isfinite(v) for v in (fx, fy, cx, cy))
            and fx > 0.0 and fy > 0.0 and msg.width > 0 and msg.height > 0
            and msg.header.frame_id.endswith("/camera_optical_frame")
            and all(math.isfinite(v) and v == 0.0 for v in msg.d)
            and msg.binning_x in (0, 1) and msg.binning_y in (0, 1)
            and msg.roi.x_offset == 0 and msg.roi.y_offset == 0
            and msg.roi.width in (0, msg.width) and msg.roi.height in (0, msg.height))


class FixedCameraTargetTracker(Node):
    def __init__(self) -> None:
        super().__init__("fixed_camera_target_tracker")
        defaults = {
            "detections_topic": "/x500_0/yolo/tracks",
            "camera_info_topic": "/x500_0/camera/camera_info",
            "bearing_topic": "/x500_0/fixed_camera_target_tracker/bearing",
            "tracking_active_topic": "/x500_0/fixed_camera_target_tracker/tracking_active",
            "lock_active_topic": "/x500_0/fixed_camera_target_tracker/lock_active",
            "target_class_id": "",
            "min_score": 0.2,
            "observation_timeout_s": 0.2,
            "lock_confirm_s": 0.2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.min_score = float(self.get_parameter("min_score").value)
        self.timeout_s = float(self.get_parameter("observation_timeout_s").value)
        self.confirm_s = float(self.get_parameter("lock_confirm_s").value)
        if not (0.0 <= self.min_score <= 1.0 and math.isfinite(self.timeout_s)
                and self.timeout_s > 0.0 and math.isfinite(self.confirm_s)
                and self.confirm_s >= 0.0):
            raise ValueError("Invalid observation score, timeout or confirmation interval")
        self.target_class_id = str(self.get_parameter("target_class_id").value)
        self.camera_info: CameraInfo | None = None
        self.first_stamp_s: float | None = None
        self.last_stamp_s: float | None = None
        self.bearing_pub = self.create_publisher(
            Vector3Stamped, str(self.get_parameter("bearing_topic").value), 10
        )
        self.active_pub = self.create_publisher(
            Bool, str(self.get_parameter("tracking_active_topic").value), 10
        )
        self.lock_pub = self.create_publisher(
            Bool, str(self.get_parameter("lock_active_topic").value), 10
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback, qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection2DArray, str(self.get_parameter("detections_topic").value),
            self._detections_callback, qos_profile_sensor_data,
        )
        self.create_timer(1.0 / 30.0, self._publish_status)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if valid_camera_info(msg):
            self.camera_info = msg
        else:
            self.camera_info = None
            self._reset()

    def _reset(self) -> None:
        self.first_stamp_s = None
        self.last_stamp_s = None

    def _detections_callback(self, msg: Detection2DArray) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self.last_stamp_s is not None and now_s - self.last_stamp_s > self.timeout_s:
            self._reset()
        info = self.camera_info
        if info is None:
            return
        candidates = []
        for detection in msg.detections:
            if not detection.results:
                continue
            result = max(detection.results, key=lambda item: item.hypothesis.score)
            score = float(result.hypothesis.score)
            if not math.isfinite(score) or not self.min_score <= score <= 1.0:
                continue
            if self.target_class_id and result.hypothesis.class_id != self.target_class_id:
                continue
            header = detection.header
            if header.stamp.sec == 0 and header.stamp.nanosec == 0:
                header = msg.header
            stamp_s = header.stamp.sec + header.stamp.nanosec * 1e-9
            # Both detections and this node use Gazebo acquisition/simulation time.
            if stamp_s <= 0.0 or not -SIM_CLOCK_TOLERANCE_S <= now_s - stamp_s <= self.timeout_s:
                continue
            if self.last_stamp_s is not None and stamp_s <= self.last_stamp_s:
                continue
            u, v = detection.bbox.center.position.x, detection.bbox.center.position.y
            if not (0.0 <= u < info.width and 0.0 <= v < info.height
                    and math.isfinite(detection.bbox.size_x)
                    and math.isfinite(detection.bbox.size_y)
                    and detection.bbox.size_x > 0.0 and detection.bbox.size_y > 0.0):
                continue
            if header.frame_id != info.header.frame_id:
                continue
            candidates.append((score, stamp_s, detection, header))
        if not candidates:
            return
        _, stamp_s, detection, header = max(candidates, key=lambda item: item[0])
        if self.first_stamp_s is None:
            self.first_stamp_s = stamp_s
        self.last_stamp_s = stamp_s
        bearing = Vector3Stamped()
        bearing.header = header
        ray = ((detection.bbox.center.position.x-info.k[2])/info.k[0],
               (detection.bbox.center.position.y-info.k[5])/info.k[4], 1.0)
        norm = math.sqrt(sum(v*v for v in ray))
        bearing.vector.x, bearing.vector.y, bearing.vector.z = (v/norm for v in ray)
        self.bearing_pub.publish(bearing)
        self._publish_status()

    def _publish_status(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        active = (self.last_stamp_s is not None
                  and -SIM_CLOCK_TOLERANCE_S <= now_s - self.last_stamp_s <= self.timeout_s)
        # Confirmation measures actual distinct observations, not timer ticks.
        locked = (active and self.first_stamp_s is not None
                  and self.last_stamp_s - self.first_stamp_s >= self.confirm_s)
        self.active_pub.publish(Bool(data=active))
        self.lock_pub.publish(Bool(data=locked))
        if not active:
            self._reset()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FixedCameraTargetTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
