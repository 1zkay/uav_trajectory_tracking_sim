#!/usr/bin/env python3

import math
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as NavPath
from px4_msgs.msg import VehicleLocalPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray

from uav_trajectory_tracking.parametric_trajectory import ParametricTrajectory, Point3
from uav_trajectory_tracking.state_comparison import SIM_CLOCK_TOLERANCE_S, px4_ned_to_world


Color = tuple[float, float, float, float]


class TrajectoryVisualizer(Node):
    """Publish standard ROS paths and markers in a shared ENU map frame."""

    def __init__(self) -> None:
        super().__init__("trajectory_visualizer")

        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("publish_planned_path", True)
        self.declare_parameter("reference_config_file", "")
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("trajectory_markers_topic", "/trajectory_markers")
        self.declare_parameter("trajectory_path_topic", "/trajectory_path")
        self.declare_parameter("vehicle_path_topic", "/vehicle_path")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("vehicle_path_min_distance_m", 0.15)
        self.declare_parameter("vehicle_path_max_points", 2000)

        self.publish_planned_path = bool(self.get_parameter("publish_planned_path").value)
        self.planned_points = []
        if self.publish_planned_path:
            trajectory = ParametricTrajectory.from_config(self._load_config())
            self.planned_points = trajectory.sample_points(include_return=True)
        reference_config = str(self.get_parameter("reference_config_file").value)
        if not reference_config:
            reference_config = str(Path(get_package_share_directory("uav_trajectory_tracking"))
                                   / "config" / "trajectory_logging.yaml")
        reference_settings = yaml.safe_load(Path(reference_config).expanduser().read_text())
        self.world_origin = tuple(reference_settings["world_origin_lat_lon_alt"])
        if (len(self.world_origin) != 3 or not all(math.isfinite(v) for v in self.world_origin)
                or not -90 < self.world_origin[0] < 90 or not -180 <= self.world_origin[1] <= 180):
            raise ValueError("Invalid world_origin_lat_lon_alt for the visualization map.")
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.path_min_distance_m = float(
            self.get_parameter("vehicle_path_min_distance_m").get_parameter_value().double_value
        )
        self.path_max_points = max(1, int(
            self.get_parameter("vehicle_path_max_points").get_parameter_value().integer_value
        ))

        self.vehicle_local_position: VehicleLocalPosition | None = None
        self.vehicle_path_poses: list[PoseStamped] = []
        self.reference = None
        self.reference_key = None
        self.last_sample_timestamp_us = 0
        self.last_ros_time_ns = None

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        vehicle_local_position_topic = (
            self.get_parameter("vehicle_local_position_topic").get_parameter_value().string_value
        )
        trajectory_markers_topic = self.get_parameter("trajectory_markers_topic").get_parameter_value().string_value
        trajectory_path_topic = self.get_parameter("trajectory_path_topic").get_parameter_value().string_value
        vehicle_path_topic = self.get_parameter("vehicle_path_topic").get_parameter_value().string_value

        self.create_subscription(
            VehicleLocalPosition,
            vehicle_local_position_topic,
            self._vehicle_local_position_callback,
            px4_qos,
        )

        # Each message is a full snapshot; keep one for late-joining viewers.
        visualization_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, trajectory_markers_topic, visualization_qos)
        self.planned_path_pub = (self.create_publisher(NavPath, trajectory_path_topic, visualization_qos)
                                 if self.publish_planned_path else None)
        self.vehicle_path_pub = self.create_publisher(NavPath, vehicle_path_topic, visualization_qos)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").get_parameter_value().double_value)
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self._timer_callback)

        self.get_logger().info(
            f"Visualizing {len(self.planned_points)} planned points and actual flight in "
            f"frame '{self.frame_id}'. Waiting for a valid PX4 global reference."
        )
        self.get_logger().info(
            f"Publishing visualization topics: {trajectory_markers_topic}, {vehicle_path_topic}"
            + (f", {trajectory_path_topic}" if self.publish_planned_path else "")
        )

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("trajectory_file").get_parameter_value().string_value

        if not config_path:
            share_dir = Path(get_package_share_directory("uav_trajectory_tracking"))
            config_path = str(share_dir / "config" / "trajectory_hold.yaml")

        path = Path(config_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Trajectory config does not exist: {path}")

        with path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        if str(config.get("frame", "NED")).upper() != "NED":
            raise ValueError("Only PX4 local NED trajectory configs are supported.")

        return config

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        now_ns = self._check_clock_reset()
        sample_ns = int(msg.timestamp_sample) * 1000
        # A late packet from before a clock reset must not advance the sample watermark.
        # Allow the same small /clock delivery skew as the other simulation consumers.
        if (now_ns == 0 or msg.timestamp_sample <= self.last_sample_timestamp_us
                or sample_ns > now_ns + int(SIM_CLOCK_TOLERANCE_S * 1e9)):
            return
        reference = (msg.ref_lat, msg.ref_lon, msg.ref_alt)
        if (not msg.xy_valid or not msg.z_valid or not msg.xy_global or not msg.z_global
                or msg.ref_timestamp == 0
                or not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z, *reference))
                or not -90 < msg.ref_lat < 90 or not -180 <= msg.ref_lon <= 180):
            self.vehicle_local_position = None
            self.reference = None
            return
        key = (msg.ref_timestamp, *reference, msg.xy_reset_counter, msg.z_reset_counter)
        if key != self.reference_key:
            self.vehicle_path_poses.clear()
        self.reference_key = key
        self.reference = reference
        self.vehicle_local_position = msg
        self.last_sample_timestamp_us = msg.timestamp_sample

        point = self._ned_to_map((msg.x, msg.y, msg.z))
        if self.vehicle_path_poses:
            previous = self.vehicle_path_poses[-1].pose.position
            distance = math.dist((previous.x, previous.y, previous.z), (point.x, point.y, point.z))
            if distance < self.path_min_distance_m:
                return

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        # PX4 and ROS share Gazebo time (UXRCE_DDS_SYNCT=0 in the SITL scripts).
        pose.header.stamp = Time(nanoseconds=sample_ns).to_msg()
        pose.pose.position = point
        pose.pose.orientation.w = 1.0  # Position-only trail, not a body attitude.
        self.vehicle_path_poses.append(pose)
        self.vehicle_path_poses = self.vehicle_path_poses[-self.path_max_points :]

    def _check_clock_reset(self) -> int:
        now_ns = self.get_clock().now().nanoseconds
        if self.last_ros_time_ns is not None and now_ns < self.last_ros_time_ns:
            self.vehicle_path_poses.clear()
            self.vehicle_local_position = None
            self.reference = None
            self.reference_key = None
            self.last_sample_timestamp_us = 0
        self.last_ros_time_ns = now_ns
        return now_ns

    def _timer_callback(self) -> None:
        self._check_clock_reset()
        now = self.get_clock().now().to_msg()
        planned_path_points = ([self._ned_to_map(point) for point in self.planned_points]
                               if self.reference is not None else [])

        self.marker_pub.publish(self._make_markers(planned_path_points, now))
        if self.planned_path_pub is not None:
            self.planned_path_pub.publish(self._make_path(planned_path_points, now))
        path = NavPath()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.vehicle_path_poses[-1].header.stamp if self.vehicle_path_poses else now
        path.poses = list(self.vehicle_path_poses)
        self.vehicle_path_pub.publish(path)

    def _make_markers(self, planned_points: list[Point], stamp) -> MarkerArray:
        markers = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.frame_id = self.frame_id
        clear_marker.header.stamp = stamp
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        marker_id = 1
        if planned_points:
            markers.markers.append(
                self._sphere(
                    marker_id,
                    "trajectory_start",
                    planned_points[0],
                    0.42,
                    (0.05, 0.35, 1.0, 0.95),
                    stamp,
                )
            )
            marker_id += 1
            markers.markers.append(
                self._sphere(
                    marker_id,
                    "trajectory_end",
                    planned_points[-1],
                    0.46,
                    (0.1, 0.75, 0.25, 1.0),
                    stamp,
                )
            )
            marker_id += 1

        if self.vehicle_local_position is not None and self.vehicle_local_position.xy_valid:
            vehicle_point = self._ned_to_map(
                (
                    self.vehicle_local_position.x,
                    self.vehicle_local_position.y,
                    self.vehicle_local_position.z,
                )
            )
            markers.markers.append(
                self._sphere(marker_id, "vehicle", vehicle_point, 0.38, (0.95, 0.95, 0.95, 1.0), stamp)
            )

        return markers

    def _make_path(self, points: list[Point], stamp) -> NavPath:
        path = NavPath()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp

        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position = point
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def _base_marker(self, marker_id: int, namespace: str, marker_type: int, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _sphere(self, marker_id: int, namespace: str, point: Point, diameter: float, color: Color, stamp) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.SPHERE, stamp)
        marker.pose.position = point
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        self._set_color(marker, color)
        return marker

    def _set_color(self, marker: Marker, color: Color) -> None:
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

    def _ned_to_map(self, point: Point3) -> Point:
        east, north, up = px4_ned_to_world(point, self.reference, self.world_origin)
        return Point(x=east, y=north, z=up)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryVisualizer()

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
