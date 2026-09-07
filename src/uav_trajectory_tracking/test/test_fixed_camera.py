import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from uav_trajectory_tracking.fixed_camera_target_tracker import FixedCameraTargetTracker
from uav_trajectory_tracking.visual_pursuit_interceptor import (
    InterceptorState, VisualPursuitInterceptor,
    fixed_camera_image_error_to_body_los, rotate_body_to_ned,
)


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def observation(monkeypatch):
    rclpy.init()
    node = FixedCameraTargetTracker()
    time = SimpleNamespace(value=100.0)
    monkeypatch.setattr(node, "get_clock", lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(time.value * 1e9))))
    node.error_pub = Capture()
    node.active_pub = Capture()
    node.lock_pub = Capture()
    info = CameraInfo(width=1280, height=960)
    info.header.frame_id = "camera_link"
    fx = 640.0 / math.tan(1.74 / 2.0)
    info.k = [fx, 0.0, 640.0, 0.0, fx, 480.0, 0.0, 0.0, 1.0]
    node._camera_info_callback(info)
    yield node, time
    node.destroy_node()
    rclpy.shutdown()


def detection(stamp, track="7", u=900.0, v=700.0):
    msg = Detection2DArray()
    msg.header.frame_id = "camera_link"
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    item = Detection2D()
    item.header = msg.header
    item.id = track
    item.bbox.center.position.x = u
    item.bbox.center.position.y = v
    item.bbox.size_x = 80.0
    item.bbox.size_y = 60.0
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = "4"
    hypothesis.hypothesis.score = 0.9
    item.results = [hypothesis]
    msg.detections = [item]
    return msg


def test_projection_right_down_and_off_axis():
    assert fixed_camera_image_error_to_body_los(0.0, 0.0) == (1.0, 0.0, 0.0)
    ray = fixed_camera_image_error_to_body_los(math.atan(0.5), math.atan(0.25))
    assert ray == pytest.approx(tuple(v / math.sqrt(1.3125) for v in (1, 0.5, 0.25)))


def test_body_rotation_and_fixed_mount():
    # PX4 +90 deg yaw rotates forward from North to East.
    ray = fixed_camera_image_error_to_body_los(0.0, 0.0)
    assert rotate_body_to_ned((math.sqrt(0.5), 0, 0, math.sqrt(0.5)), ray) == pytest.approx((0, 1, 0))
    # Gazebo +90 deg camera mount yaw faces left, i.e. -Y in PX4 FRD.
    assert fixed_camera_image_error_to_body_los(0, 0, (0, 0, math.pi / 2)) == pytest.approx((0, -1, 0))


def test_calibrated_observation_and_off_axis_lock(observation):
    node, time = observation
    for offset in (0.0, 0.1, 0.21):
        time.value = 100.0 + offset
        node._detections_callback(detection(time.value))
    msg = node.error_pub.messages[-1]
    assert msg.vector.x == pytest.approx(math.atan2(260, node.camera_info.k[0]))
    assert msg.vector.y == pytest.approx(math.atan2(220, node.camera_info.k[4]))
    assert msg.header.stamp.sec == 100
    assert node.lock_pub.messages[-1].data  # no centered-gimbal gate


@pytest.mark.parametrize("stamp,u", [(99.0, 640.0), (101.0, 640.0), (100.0, float("nan")), (100.0, 1280.0)])
def test_invalid_observations_do_not_refresh_tracking(observation, stamp, u):
    node, _ = observation
    node._detections_callback(detection(stamp, u=u))
    node._publish_status()
    assert not node.error_pub.messages
    assert not node.active_pub.messages[-1].data


def test_no_intrinsics_no_observation(observation):
    node, _ = observation
    node._camera_info_callback(CameraInfo())
    node._detections_callback(detection(100))
    assert not node.error_pub.messages


def test_repeated_frames_do_not_confirm_lock_and_expire(observation):
    node, time = observation
    node._detections_callback(detection(100))
    time.value = 100.1
    node._detections_callback(detection(100))
    assert len(node.error_pub.messages) == 1
    assert not node.lock_pub.messages[-1].data
    time.value = 100.3
    node._publish_status()
    assert not node.active_pub.messages[-1].data
    assert node.track_id is None


def test_track_identity_not_silently_switched(observation):
    node, time = observation
    node._detections_callback(detection(100))
    time.value = 100.1
    node._detections_callback(detection(100.1, track="8"))
    assert node.track_id == "7"
    assert len(node.error_pub.messages) == 1


def test_controller_uses_fixed_observation_without_joint_feedback(observation):
    observer, time = observation
    controller = VisualPursuitInterceptor()
    try:
        controller.get_clock = observer.get_clock
        controller.vehicle_status = VehicleStatus()
        controller.vehicle_local_position = VehicleLocalPosition(xy_valid=True, z_valid=True)
        controller.vehicle_attitude = VehicleAttitude(q=[1.0, 0.0, 0.0, 0.0])
        controller.takeoff_altitude_reached = True
        controller.initial_hover_reached = True
        controller.trajectory_pub = Capture()
        observer._detections_callback(detection(100))
        controller._visual_error_callback(observer.error_pub.messages[-1])
        controller._tracking_active_callback(Bool(data=True))
        controller._lock_active_callback(Bool(data=True))
        assert controller._ready_to_pursue(time.value)
        assert controller.gimbal_yaw_rad is None
        assert all("gimbal" not in sub.topic_name for sub in controller.subscriptions)
        controller._publish_pursuit_setpoint(100_000_000, 1.0 / 30.0)
        assert controller.last_visual_los_body[1] > 0
        assert controller.last_visual_los_body[2] > 0
        assert all(math.isfinite(v) for v in controller.trajectory_pub.messages[-1].velocity)
        time.value += 1.0
        assert not controller._ready_to_pursue(time.value)
        assert controller.state == InterceptorState.TARGET_LOST
    finally:
        controller.destroy_node()


def test_world_uses_fixed_camera_wrapper():
    root = Path(__file__).resolve().parents[3]
    world = ET.parse(root / "px4_overlays/worlds/trajectory_tracking.sdf")
    host = next(n for n in world.findall(".//world/include") if n.findtext("name") == "x500_0")
    assert host.findtext("uri") == "model://x500_mono_cam_trajectory_wind"
    wrapper = ET.parse(root / "px4_overlays/models/x500_mono_cam_trajectory_wind/model.sdf")
    assert wrapper.findtext(".//include/uri") == "model://x500_mono_cam"


def test_fixed_search_holds_position_and_starts_from_current_yaw(observation):
    observer, time = observation
    controller = VisualPursuitInterceptor()
    try:
        controller.get_clock = observer.get_clock
        controller.vehicle_attitude = VehicleAttitude(
            q=[math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)])
        controller.takeoff_position = (0.0, 0.0, -5.0)
        controller.hold_position = (1.0, 2.0, -5.0)
        controller.initial_hover_reached = True
        controller.state = InterceptorState.TARGET_LOST
        controller.trajectory_pub = Capture()
        controller._publish_hold_setpoint(100_000_000)
        first = controller.trajectory_pub.messages[-1]
        assert first.position == pytest.approx((1.0, 2.0, -5.0))
        assert first.yaw == pytest.approx(math.pi / 2)
        controller._publish_hold_setpoint(100_100_000)
        second = controller.trajectory_pub.messages[-1]
        assert second.position == pytest.approx(first.position)
        assert second.yaw - first.yaw == pytest.approx(math.radians(20.0) * 0.1, abs=1e-6)
    finally:
        controller.destroy_node()
