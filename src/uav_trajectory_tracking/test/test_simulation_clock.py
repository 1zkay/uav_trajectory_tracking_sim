import os
import signal
import subprocess
import time
import uuid
from pathlib import Path

import rclpy
import pytest
from gz.msgs10.clock_pb2 import Clock as GzClock
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Path as NavPath
from px4_msgs.msg import OffboardControlMode, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

from uav_trajectory_tracking.state_comparison import world_to_geodetic


def test_target_clock_survives_visual_launch_restart(monkeypatch, tmp_path):
    """Keep the target control timer running across an actual visual launch restart."""
    monkeypatch.setenv("ROS_DOMAIN_ID", "84")
    monkeypatch.setenv("GZ_PARTITION", "clock_test_" + uuid.uuid4().hex)
    monkeypatch.setenv("PX4_GZ_WORLD", "trajectory_tracking")
    root = Path(__file__).resolve().parents[3]
    rclpy.init(domain_id=84)
    probe = Node("simulation_clock_test")
    heartbeats, clocks, processes = [], [], []
    probe.create_subscription(OffboardControlMode, "/px4_1/fmu/in/offboard_control_mode",
                              lambda msg: heartbeats.append(msg.timestamp), qos_profile_sensor_data)
    probe.create_subscription(Clock, "/clock", clocks.append, qos_profile_sensor_data)
    paths = {"host": [], "target": []}
    path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for name, topic in (("host", "/vehicle_path"), ("target", "/target/vehicle_path")):
        probe.create_subscription(NavPath, topic, paths[name].append, path_qos)
    origin = (47.397971057728974, 8.546163739800146, 0.)
    position_sources = [
        (probe.create_publisher(VehicleLocalPosition, topic, qos_profile_sensor_data),
         world_to_geodetic(point, origin))
        for topic, point in (("/fmu/out/vehicle_local_position_v1", (0., 0., .24)),
                             ("/px4_1/fmu/out/vehicle_local_position_v1", (0., 5., .24)))
    ]
    gz_node = GzNode()
    publisher = gz_node.advertise("/world/trajectory_tracking/clock", GzClock)
    start = time.monotonic()

    def start_process(name, args):
        with (tmp_path / (name + ".log")).open("w") as output:
            process = subprocess.Popen(args, cwd=root, stdout=output,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    def wait_for(predicate):
        deadline = time.monotonic() + 15
        while not predicate() and time.monotonic() < deadline:
            stamp = int((100 + time.monotonic() - start) * 1e9)
            msg = GzClock()
            msg.sim.sec, msg.sim.nsec = divmod(stamp, 1_000_000_000)
            publisher.publish(msg)
            for source, reference in position_sources:
                source.publish(VehicleLocalPosition(
                    timestamp=stamp//1000, timestamp_sample=stamp//1000,
                    xy_valid=True, z_valid=True, xy_global=True, z_global=True,
                    ref_timestamp=1, ref_lat=reference[0], ref_lon=reference[1], ref_alt=reference[2],
                    x=0., y=0., z=-5.))
            rclpy.spin_once(probe, timeout_sec=.02)
        assert predicate(), "\n".join(p.read_text() for p in tmp_path.glob("*.log"))

    def check_target():
        previous = heartbeats[-1] if heartbeats else 0
        wait_for(lambda: sum(stamp > previous for stamp in heartbeats) >= 10)
        assert all(process.poll() is None for process in (clock_process, target_process))
        endpoints = probe.get_publishers_info_by_topic("/clock")
        assert len(endpoints) == 1
        assert endpoints[0].node_name == "simulation_clock_bridge"
        assert clocks[-1].clock.sec >= 100
        return endpoints[0].endpoint_gid

    def check_paths():
        wait_for(lambda: all(messages and messages[-1].poses for messages in paths.values()))
        host, target = (paths[name][-1] for name in ("host", "target"))
        assert host.header.frame_id == target.header.frame_id == "map"
        assert target.poses[-1].pose.position.y - host.poses[-1].pose.position.y == pytest.approx(5., abs=1e-4)
        for path in (host, target):
            assert path.poses[-1].pose.position.z == pytest.approx(5.24, abs=1e-4)
            assert path.poses[-1].header.stamp.sec >= 100
        assert len(probe.get_publishers_info_by_topic("/vehicle_path")) == 1
        assert not probe.get_publishers_info_by_topic("/trajectory_path")

    visual_args = [str(root / "scripts/start_visual_interception.sh"),
                   "enable_camera_bridge:=false", "enable_yolo_tracking:=false",
                   "enable_yolo_annotation:=false", "enable_csv_logging:=false",
                   "enable_truth_odometry_bridge:=false"]
    try:
        clock_process = start_process("clock", [str(root / "scripts/start_simulation_clock.sh")])
        target_process = start_process("target", [str(root / "scripts/start_target_trajectory_tracking.sh"),
                                                   "enable_csv_logging:=false"])
        clock_gid = check_target()  # The target starts without the host visual launch.
        visual = start_process("visual", visual_args)
        wait_for(lambda: "visual_pursuit_interceptor" in probe.get_node_names())
        assert check_target() == clock_gid
        check_paths()
        visual.send_signal(signal.SIGINT)
        wait_for(lambda: visual.poll() is not None)
        assert check_target() == clock_gid
        paths["host"].clear()
        restarted = start_process("visual_restarted", visual_args)
        wait_for(lambda: "visual_pursuit_interceptor" in probe.get_node_names())
        assert check_target() == clock_gid
        check_paths()
        assert restarted.poll() is None
        # No loss of the target timer while the visual launch shuts down or restarts.
        assert max(b - a for a, b in zip(heartbeats, heartbeats[1:])) < 500_000
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                if process is clock_process:
                    # ros2 run needs terminal-style SIGINT for its child executable.
                    os.killpg(process.pid, signal.SIGINT)
                else:
                    process.send_signal(signal.SIGINT)
                process.wait(timeout=10)
        probe.destroy_node()
        rclpy.shutdown()
