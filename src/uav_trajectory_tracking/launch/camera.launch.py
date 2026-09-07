"""Official Gazebo camera bridge and fixed optical-frame transforms."""
import math
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_camera(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    config = yaml.safe_load(Path(value("camera_config_file")).expanduser().read_text())
    translation = config["camera_mount_translation_flu_m"]
    rpy = config["camera_mount_rpy_rad"]
    if (len(translation) != 3 or len(rpy) != 3
            or not all(math.isfinite(v) for v in (*translation, *rpy))):
        raise ValueError("Camera mount must contain finite FLU translation and RPY triples")

    parameters = {"use_sim_time": True, "bridge_names": ["image", "info"],
                  "override_frame_id": "x500_0/camera_optical_frame"}
    for name, kind, gz_topic, ros_topic in (
        ("image", "Image", "camera_gazebo_topic", "camera_image_topic"),
        ("info", "CameraInfo", "camera_info_gazebo_topic", "camera_info_topic"),
    ):
        parameters.update({
            f"bridges.{name}.ros_type_name": f"sensor_msgs/msg/{kind}",
            f"bridges.{name}.gz_type_name": f"gz.msgs.{kind}",
            f"bridges.{name}.gz_topic_name": value(gz_topic),
            f"bridges.{name}.ros_topic_name": value(ros_topic),
            f"bridges.{name}.direction": "GZ_TO_ROS",
            # Default ROS QoS is reliable/volatile; bound both queues to the latest frame.
            f"bridges.{name}.publisher_queue": 1,
            f"bridges.{name}.subscriber_queue": 1,
        })
    mount_args = [arg for flag, number in zip(
        ("--x", "--y", "--z", "--roll", "--pitch", "--yaw"), (*translation, *rpy))
        for arg in (flag, str(number))]
    return [
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="camera_bridge",
             namespace=value("node_namespace"), output="screen", parameters=[parameters]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="camera_mount_tf",
             namespace=value("node_namespace"), parameters=[{"use_sim_time": True}],
             arguments=mount_args + ["--frame-id", "x500_0/base_link", "--child-frame-id", "x500_0/camera_link"]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="camera_optical_tf",
             namespace=value("node_namespace"), parameters=[{"use_sim_time": True}],
             arguments=["--qx", "-0.5", "--qy", "0.5", "--qz", "-0.5", "--qw", "0.5",
                        "--frame-id", "x500_0/camera_link", "--child-frame-id", "x500_0/camera_optical_frame"]),
    ]


def generate_launch_description():
    camera = "/world/trajectory_tracking/model/x500_0/link/camera_link/sensor/camera/"
    defaults = {
        "node_namespace": "",
        "camera_config_file": PathJoinSubstitution([
            FindPackageShare("uav_trajectory_tracking"), "config", "visual_interception.yaml"]),
        "camera_gazebo_topic": camera + "image",
        "camera_image_topic": "/x500_0/camera/image_raw",
        "camera_info_gazebo_topic": camera + "camera_info",
        "camera_info_topic": "/x500_0/camera/camera_info",
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()),
        OpaqueFunction(function=launch_camera),
    ])
