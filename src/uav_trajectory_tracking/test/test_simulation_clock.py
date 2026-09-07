import os
import signal
import subprocess
import time
import uuid
from pathlib import Path

import rclpy
from gz.msgs10.clock_pb2 import Clock as GzClock
from gz.transport13 import Node as GzNode
from px4_msgs.msg import OffboardControlMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


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
        visual.send_signal(signal.SIGINT)
        wait_for(lambda: visual.poll() is not None)
        assert check_target() == clock_gid
        restarted = start_process("visual_restarted", visual_args)
        wait_for(lambda: "visual_pursuit_interceptor" in probe.get_node_names())
        assert check_target() == clock_gid
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
