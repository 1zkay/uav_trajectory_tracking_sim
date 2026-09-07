import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import rclpy
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from uav_trajectory_tracking.fixed_camera_target_tracker import FixedCameraTargetTracker
from uav_trajectory_tracking.visual_pursuit_interceptor import (
    InterceptorState, VisualPursuitInterceptor,
    fixed_camera_bearing_to_body_los, rotate_body_to_ned, rotation_from_rpy,
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
    node.bearing_pub = Capture()
    node.active_pub = Capture()
    node.lock_pub = Capture()
    info = CameraInfo(width=1280, height=960)
    info.header.frame_id = "x500_0/camera_optical_frame"
    fx = 640.0 / math.tan(1.74 / 2.0)
    info.k = [fx, 0.0, 640.0, 0.0, fx, 480.0, 0.0, 0.0, 1.0]
    node._camera_info_callback(info)
    yield node, time
    node.destroy_node()
    rclpy.shutdown()


def detection(stamp, track="7", u=900.0, v=700.0):
    msg = Detection2DArray()
    msg.header.frame_id = "x500_0/camera_optical_frame"
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


def odometry_sample(**kwargs):
    defaults = dict(timestamp_sample=100_000_000, q=[1., 0., 0., 0.],
                    pose_frame=VehicleOdometry.POSE_FRAME_NED,
                    velocity_frame=VehicleOdometry.VELOCITY_FRAME_NED,
                    position=[0., 0., 0.], velocity=[0., 0., 0.])
    defaults.update(kwargs)
    return VehicleOdometry(**defaults)


def prime_controller(node, now, position=(0., 0., 0.)):
    info = CameraInfo(width=1280, height=960)
    info.header.frame_id = "x500_0/camera_optical_frame"
    fx = 640.0 / math.tan(1.74 / 2.0)
    info.k = [fx, 0., 640., 0., fx, 480., 0., 0., 1.]
    node._camera_info_callback(info)
    stamp = round(now*1e6)
    node._vehicle_status_callback(VehicleStatus(timestamp=stamp))
    node._vehicle_local_position_callback(VehicleLocalPosition(
        timestamp_sample=stamp, xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True,
        x=position[0], y=position[1], z=position[2]))
    node._vehicle_odometry_callback(odometry_sample(timestamp_sample=stamp, q=[1., 0., 0., 0.]))


def test_projection_right_down_and_off_axis():
    assert fixed_camera_bearing_to_body_los((0., 0., 1.)) == (1.0, 0.0, 0.0)
    ray = fixed_camera_bearing_to_body_los((0.5, 0.25, 1.))
    assert ray == pytest.approx(tuple(v / math.sqrt(1.3125) for v in (1, 0.5, 0.25)))


def test_body_rotation_and_fixed_mount():
    # PX4 +90 deg yaw rotates forward from North to East.
    ray = fixed_camera_bearing_to_body_los((0., 0., 1.))
    assert rotate_body_to_ned((math.sqrt(0.5), 0, 0, math.sqrt(0.5)), ray) == pytest.approx((0, 1, 0))
    # Gazebo +90 deg camera mount yaw faces left, i.e. -Y in PX4 FRD.
    assert fixed_camera_bearing_to_body_los((0., 0., 1.), rotation_from_rpy(0, 0, math.pi / 2)) == pytest.approx((0, -1, 0))


def test_calibrated_observation_and_off_axis_lock(observation):
    node, time = observation
    for offset in (0.0, 0.1, 0.21):
        time.value = 100.0 + offset
        node._detections_callback(detection(time.value))
    msg = node.bearing_pub.messages[-1]
    assert msg.vector.x / msg.vector.z == pytest.approx(260 / node.camera_info.k[0])
    assert msg.vector.y / msg.vector.z == pytest.approx(220 / node.camera_info.k[4])
    assert msg.header.stamp.sec == 100
    assert node.lock_pub.messages[-1].data  # no centered-gimbal gate


@pytest.mark.parametrize("stamp,u", [(99.0, 640.0), (101.0, 640.0), (100.0, float("nan")), (100.0, 1280.0)])
def test_invalid_observations_do_not_refresh_tracking(observation, stamp, u):
    node, _ = observation
    node._detections_callback(detection(stamp, u=u))
    node._publish_status()
    assert not node.bearing_pub.messages
    assert not node.active_pub.messages[-1].data


def test_no_intrinsics_no_observation(observation):
    node, _ = observation
    node._camera_info_callback(CameraInfo())
    node._detections_callback(detection(100))
    assert not node.bearing_pub.messages


def test_repeated_frames_do_not_confirm_lock_and_expire(observation):
    node, time = observation
    node._detections_callback(detection(100))
    time.value = 100.1
    node._detections_callback(detection(100))
    assert len(node.bearing_pub.messages) == 1
    assert not node.lock_pub.messages[-1].data
    time.value = 100.3
    node._publish_status()
    assert not node.active_pub.messages[-1].data
    assert node.first_stamp_s is None and node.last_stamp_s is None


def test_single_target_id_changes_preserve_bearing_and_lock(observation):
    node, time = observation
    for offset, track in [(0., "7"), (.1, "8"), (.21, "9"), (.25, "10")]:
        time.value = 100. + offset
        node._detections_callback(detection(time.value, track=track))
    assert len(node.bearing_pub.messages) == 4
    assert node.first_stamp_s == 100.
    assert node.last_stamp_s == 100.25
    assert node.lock_pub.messages[-1].data
    assert node.lock_pub.messages[-2].data


def test_single_target_new_id_after_empty_frame_is_accepted_immediately(observation):
    node, time = observation
    for stamp in (100., 100.1, 100.21):
        time.value = stamp
        node._detections_callback(detection(stamp, track="19"))
    time.value = 100.242
    node._detections_callback(Detection2DArray())
    node._publish_status()
    assert node.lock_pub.messages[-1].data
    time.value = 100.278
    node._detections_callback(detection(time.value, track="20"))
    assert len(node.bearing_pub.messages) == 4
    assert node.last_stamp_s == pytest.approx(time.value)
    assert node.first_stamp_s == 100.
    assert node.lock_pub.messages[-1].data
    # A genuine observation timeout still clears confirmation history.
    time.value = 100.5
    node._publish_status()
    assert not node.lock_pub.messages[-1].data
    node._detections_callback(detection(time.value, track="21"))
    assert not node.lock_pub.messages[-1].data


def test_single_target_does_not_require_id_but_rejects_repeated_stamp(observation):
    node, time = observation
    node._detections_callback(detection(100., track=""))
    time.value = 100.1
    node._detections_callback(detection(100., track="new"))
    assert len(node.bearing_pub.messages) == 1
    assert node.last_stamp_s == 100.


def test_controller_uses_fixed_observation_without_joint_feedback(observation):
    observer, time = observation
    controller = VisualPursuitInterceptor()
    try:
        controller.get_clock = observer.get_clock
        prime_controller(controller, time.value)
        controller.vehicle_status = VehicleStatus(timestamp=100_000_000)
        controller.vehicle_local_position = VehicleLocalPosition(timestamp_sample=100_000_000, xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True)
        controller.vehicle_odometry = odometry_sample(timestamp_sample=100_000_000, q=[1.0, 0.0, 0.0, 0.0])
        controller.takeoff_altitude_reached = True
        controller.initial_hover_reached = True
        controller.trajectory_pub = Capture()
        observer._detections_callback(detection(100))
        controller._bearing_callback(observer.bearing_pub.messages[-1])
        controller._tracking_active_callback(Bool(data=True))
        controller._lock_active_callback(Bool(data=True))
        assert controller._ready_to_pursue(time.value)
        assert not controller.has_parameter("gimbal_joint_state_topic")
        assert not controller.has_parameter("gimbal_search_active_topic")
        assert all("gimbal" not in sub.topic_name for sub in controller.subscriptions)
        controller._publish_pursuit_setpoint(100_000_000)
        assert controller.last_los_body[1] > 0
        assert controller.last_los_body[2] > 0
        msg = controller.trajectory_pub.messages[-1]
        assert all(math.isfinite(v) for v in msg.velocity)
        assert all(math.isnan(v) for v in (*msg.position, *msg.acceleration))
        assert math.isfinite(msg.yaw) and math.isnan(msg.yawspeed)
        assert not controller.has_parameter("vehicle_rates_setpoint_topic")
        assert not any(p.topic_name == '/fmu/in/vehicle_rates_setpoint' for p in controller.publishers)
        controller.diagnostics_pub = Capture()
        controller._publish_diagnostics(100_000_000, True, True, vehicle_ready=True)
        values = controller.diagnostics_pub.messages[-1].status[0].values
        assert all("gimbal" not in item.key and "vertical_search" not in item.key for item in values)
        time.value += 1.0
        prime_controller(controller, time.value)
        assert not controller._ready_to_pursue(time.value)
        assert controller.state == InterceptorState.TARGET_LOST
    finally:
        controller.destroy_node()


def test_lock_loss_coasts_then_holds_current_position(observation):
    observer, time = observation
    controller = VisualPursuitInterceptor()
    try:
        controller.get_clock = observer.get_clock
        prime_controller(controller, time.value)
        controller.vehicle_status = VehicleStatus(timestamp=100_000_000)
        controller.vehicle_local_position = VehicleLocalPosition(
            timestamp_sample=100_000_000, xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True, x=3.0, y=4.0, z=-5.0)
        controller.vehicle_odometry = odometry_sample(timestamp_sample=100_000_000, q=[1.0, 0.0, 0.0, 0.0])
        controller.takeoff_altitude_reached = True
        controller.initial_hover_reached = True
        controller.takeoff_position = (0.0, 0.0, -5.0)
        controller.hold_position = controller.takeoff_position
        controller.trajectory_pub = Capture()
        controller.previous_state = InterceptorState.PURSUIT
        controller.last_pursuit_time_s = time.value
        controller.last_velocity_setpoint_ned = (2.0, 0.0, 0.0)
        controller.vehicle_odometry.velocity[0] = 2.0
        time.value += controller.lock_loss_grace_s / 2
        assert not controller._ready_to_pursue(time.value)
        assert controller.state == InterceptorState.COAST
        controller._publish_setpoint(int(time.value * 1e6), False, True, 0.1)
        coast = controller.trajectory_pub.messages[-1]
        assert all(math.isnan(v) for v in coast.position)
        assert 0.0 < coast.velocity[0] < 2.0

        controller.previous_state = InterceptorState.COAST
        time.value += controller.lock_loss_grace_s
        prime_controller(controller, time.value, position=(3., 4., -5.))
        assert not controller._ready_to_pursue(time.value)
        assert controller.state == InterceptorState.TARGET_LOST
        controller._publish_setpoint(int(time.value * 1e6), False, False, 0.1)
        hold = controller.trajectory_pub.messages[-1]
        assert hold.position == pytest.approx((3.0, 4.0, -5.0))
        assert all(math.isnan(v) for v in hold.velocity)
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
        prime_controller(controller, time.value)
        controller.vehicle_odometry = odometry_sample(
            q=[math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)])
        controller.takeoff_position = (0.0, 0.0, -5.0)
        controller.hold_position = (1.0, 2.0, -5.0)
        controller.initial_hover_reached = True
        controller.state = InterceptorState.TARGET_LOST
        controller.trajectory_pub = Capture()
        controller.search_vertical_amplitude_m = 0.0
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


@pytest.fixture
def controller(observation):
    observer, clock = observation
    node = VisualPursuitInterceptor()
    node.get_clock = observer.get_clock
    prime_controller(node, clock.value)
    node.search_vertical_amplitude_m = 2.0
    yield node, clock
    node.destroy_node()


def bearing(stamp, ray=(0., 0., 1.)):
    from geometry_msgs.msg import Vector3Stamped
    msg = Vector3Stamped()
    msg.header.frame_id = "x500_0/camera_optical_frame"
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    msg.vector.x, msg.vector.y, msg.vector.z = ray
    return msg


@pytest.mark.parametrize("center", [-5.0, -7.5, -2.5])
@pytest.mark.parametrize("hold_on_loss", [True, False])
def test_vertical_search_keeps_xy_and_respects_ned_limits(controller, center, hold_on_loss):
    node, clock = controller
    node.hold_position_on_loss = hold_on_loss
    prime_controller(node, clock.value + .01, position=(1., 2., center))
    node.takeoff_position = node.hold_position = (1., 2., center)
    node.initial_hover_reached = True
    node.trajectory_pub = Capture()
    for elapsed, offset in [(0., 0.), (3., -2.), (6., 0.), (9., 2.), (12., 0.)]:
        node._publish_hold_setpoint(round((100. + elapsed) * 1e6))
        msg = node.trajectory_pub.messages[-1]
        assert msg.position == pytest.approx((1., 2., max(-8., min(-2., center + offset))))
        assert msg.yawspeed == pytest.approx(math.radians(20.))
        assert all(math.isnan(v) for v in msg.velocity)
        node.vehicle_local_position.x += .1
        node.vehicle_local_position.z += .1


@pytest.mark.parametrize("locked", [False, True])
def test_vertical_search_stops_on_observation_and_reanchors_on_loss(controller, locked):
    node, clock = controller
    clock.value = 100.125
    prime_controller(node, clock.value, position=(1., 2., -5.))
    node.takeoff_altitude_reached = node.initial_hover_reached = True
    node.takeoff_position = node.hold_position = (1., 2., -5.)
    for publisher in ("trajectory_pub", "offboard_mode_pub", "vehicle_command_pub", "diagnostics_pub"):
        setattr(node, publisher, Capture())
    node._timer_callback()
    assert node.vertical_search_start_time_s == pytest.approx(clock.value)
    clock.value += 3.
    prime_controller(node, clock.value, position=(1., 2., -6.))
    node._bearing_callback(bearing(clock.value))
    assert node._fresh_bearing(clock.value)
    node._tracking_active_callback(Bool(data=True))
    node._lock_active_callback(Bool(data=locked))
    node._timer_callback()
    assert node.vertical_search_start_time_s is None
    assert node.hold_position[2] == pytest.approx(-6.)
    if not locked:
        assert node.trajectory_pub.messages[-1].position == pytest.approx((1., 2., -6.))
    clock.value += 1.
    prime_controller(node, clock.value, position=(1., 2., -6.))
    node._timer_callback()
    assert node.vertical_search_start_time_s == pytest.approx(clock.value)
    assert node.trajectory_pub.messages[-1].position == pytest.approx((1., 2., -6.))
    previous_count = len(node.trajectory_pub.messages)
    clock.value += .6  # PX4 state stops updating.
    node._timer_callback()
    assert node.vertical_search_start_time_s is None
    assert len(node.trajectory_pub.messages) == previous_count


def test_vertical_search_does_not_run_during_takeoff_or_coast(controller):
    node, clock = controller
    node.takeoff_position = node.hold_position = (0., 0., -5.)
    node.trajectory_pub = Capture()
    node._publish_hold_setpoint(100_000_000)
    assert node.vertical_search_start_time_s is None
    assert node.trajectory_pub.messages[-1].position[2] == -5.
    node.initial_hover_reached = True
    node._publish_hold_setpoint(100_000_000)
    assert node.vertical_search_start_time_s is not None
    node._publish_setpoint(100_100_000, False, True, .1)
    assert node.vertical_search_start_time_s is None
    assert all(math.isnan(v) for v in node.trajectory_pub.messages[-1].position)


def test_rejected_delayed_frames_cannot_keep_old_filter_alive(controller):
    node, clock = controller
    node._bearing_callback(bearing(100.))
    assert node._fresh_bearing(100.)
    for received, stamp in [(100.15, 100.02), (100.25, 100.10), (100.35, 100.20)]:
        clock.value = received
        node._bearing_callback(bearing(stamp, (0.6, 0., 0.8)))
    assert node.last_bearing_time_s == 100.
    assert not node._fresh_bearing(clock.value)
    node._update_los_prediction(clock.value)
    assert node.los_filter_horizontal_rad is None


@pytest.mark.parametrize("stamp,ray", [(99., (0.,0.,1.)), (101., (0.,0.,1.)),
                                      (100., (0.,0.,0.)), (100., (float('nan'),0.,1.))])
def test_invalid_bearings_do_not_refresh_guidance(controller, stamp, ray):
    node, _ = controller
    node._bearing_callback(bearing(stamp, ray))
    assert node.last_bearing_time_s is None


def test_rotation_uses_exposure_attitude_and_waits_for_right_sample(controller):
    node, clock = controller
    clock.value = 100.02
    # At exposure the body yaw is 45 deg; a stationary north target lies left.
    node._bearing_callback(bearing(100.02, (-math.sqrt(.5), 0., math.sqrt(.5))))
    assert node.last_bearing_time_s is None
    clock.value = 100.04
    node._vehicle_odometry_callback(odometry_sample(
        timestamp_sample=100_040_000, q=[math.sqrt(.5), 0., 0., math.sqrt(.5)]))
    assert node._guidance_los_ned(clock.value) == pytest.approx((1., 0., 0.), abs=1e-8)
    # Rotating the camera further does not rotate the already inertial LOS.
    clock.value = 100.06
    node._vehicle_odometry_callback(odometry_sample(timestamp_sample=100_060_000, q=[0.,0.,0.,1.]))
    assert node._guidance_los_ned(clock.value) == pytest.approx((1., 0., 0.), abs=1e-8)


def test_common_sim_clock_pairs_without_timesync_messages(controller):
    node, clock = controller
    node._bearing_callback(bearing(100.))
    assert node.last_bearing_time_s == pytest.approx(100.)
    assert node._fresh_bearing(clock.value)


def test_exposure_ahead_of_clock_waits_without_backward_prediction(controller):
    node, clock = controller
    node._vehicle_odometry_callback(odometry_sample(timestamp_sample=100_005_000,q=[1.,0.,0.,0.]))
    node._bearing_callback(bearing(100.005))
    assert node.last_bearing_time_s is None
    clock.value = 100.005
    node._drain_bearings()
    assert node._fresh_bearing(clock.value)


def test_state_timeout_stops_offboard_publication(controller):
    node, clock = controller
    assert node._vehicle_ready()
    node.offboard_mode_pub = Capture()
    node.trajectory_pub = Capture()
    node.vehicle_command_pub = Capture()
    node.diagnostics_pub = Capture()
    clock.value = 100.6
    node._timer_callback()
    assert not node._vehicle_ready()
    assert not node.offboard_mode_pub.messages
    assert not node.trajectory_pub.messages
    assert not node.vehicle_command_pub.messages
    assert node.setpoint_counter == 0


def test_control_and_diagnostics_share_readiness_until_next_cycle(controller, monkeypatch):
    from unittest.mock import Mock

    node, clock = controller
    for publisher in ("offboard_mode_pub", "trajectory_pub", "vehicle_command_pub", "diagnostics_pub"):
        setattr(node, publisher, Capture())
    readiness = Mock(wraps=node._vehicle_ready)
    monkeypatch.setattr(node, "_vehicle_ready", readiness)
    publish_setpoint = node._publish_setpoint

    def publish_then_expire(*args):
        publish_setpoint(*args)
        clock.value += .6  # Cross the freshness boundary before diagnostics are published.

    monkeypatch.setattr(node, "_publish_setpoint", publish_then_expire)
    node._timer_callback()
    assert readiness.call_count == 1
    assert len(node.offboard_mode_pub.messages) == len(node.trajectory_pub.messages) == 1
    node._timer_callback()
    assert readiness.call_count == 2
    assert len(node.offboard_mode_pub.messages) == len(node.trajectory_pub.messages) == 1
    assert not node.vehicle_command_pub.messages
    assert node.state == InterceptorState.INITIALIZING
    assert node.setpoint_counter == 0
    diagnostics = [dict((item.key, item.value) for item in msg.status[0].values)
                   for msg in node.diagnostics_pub.messages]
    assert [values["vehicle_state_fresh"] for values in diagnostics] == ["true", "false"]


def test_invalid_or_replayed_odometry_does_not_replace_state(controller):
    node, clock = controller
    valid = node.vehicle_odometry
    clock.value = 100.1
    for stamp,q in [(100_100_000,[0.,0.,0.,0.]),(100_100_000,[float('nan'),0.,0.,1.]),
                    (1,[1.,0.,0.,0.]),(100_000_000,[0.,0.,0.,1.])]:
        node._vehicle_odometry_callback(odometry_sample(timestamp_sample=stamp,q=q))
    assert node.vehicle_odometry is valid


def test_attitude_gap_invalidates_observation(controller):
    node, clock = controller
    node.attitude_max_gap_s = .01
    clock.value = 100.02
    node._vehicle_odometry_callback(odometry_sample(timestamp_sample=100_020_000,q=[1.,0.,0.,0.]))
    node._bearing_callback(bearing(100.01))
    assert node.last_bearing_time_s is None
    node._bearing_callback(bearing(100.02))
    assert node._fresh_bearing(clock.value)


@pytest.mark.parametrize("setting", ["distortion", "roi", "binning", "frame"])
def test_unsupported_camera_calibration_is_rejected(observation, setting):
    node, _ = observation
    info = node.camera_info
    if setting == "distortion": info.d = [.1, 0., 0., 0., 0.]
    elif setting == "roi": info.roi.x_offset = 1
    elif setting == "binning": info.binning_x = 2
    else: info.header.frame_id = "camera_link"
    node._camera_info_callback(info)
    assert node.camera_info is None


def test_yolo_export_keeps_image_acquisition_header():
    from sensor_msgs.msg import Image
    from uav_trajectory_tracking.yolo_tracker import YoloTracker, YoloTrack
    msg = Image()
    msg.header.frame_id = "x500_0/camera_optical_frame"
    msg.header.stamp.sec, msg.header.stamp.nanosec = 42, 123456789
    output = YoloTracker._to_detection_array(None, msg, [YoloTrack("7", "4", .9, 640.,480.,80.,60.)])
    assert output.header == msg.header
    assert output.detections[0].header == msg.header


def test_ros_camera_observation_pipeline(monkeypatch, tmp_path):
    """Exercise actual DDS QoS, optical TF, /clock and exposure attitude pairing."""
    import time
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import QoSProfile, DurabilityPolicy
    import signal
    import subprocess
    import uuid
    from gz.transport13 import Node as GzNode
    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.msgs10.camera_info_pb2 import CameraInfo as GzInfo
    monkeypatch.setenv("GZ_PARTITION", "camera_test_" + uuid.uuid4().hex)
    rclpy.init()
    observer, control, source = FixedCameraTargetTracker(), VisualPursuitInterceptor(), Node("camera_pipeline_test")
    control.timer.cancel()  # This test observes the input path without issuing flight commands.
    control.set_parameters([Parameter("use_sim_time", value=True)])
    observer.set_parameters([Parameter("use_sim_time", value=True)])
    executor = SingleThreadedExecutor()
    nodes = [observer, control, source]
    for node in nodes: executor.add_node(node)
    received_images, transforms = [], []
    source.create_subscription(Image, "/x500_0/camera/image_raw", received_images.append, 1)
    source.create_subscription(TFMessage, "/tf_static", lambda m: transforms.extend(m.transforms),
                               QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publishers = {
        'clock': source.create_publisher(Clock, '/clock', 10),
        'tracks': source.create_publisher(Detection2DArray, '/x500_0/yolo/tracks', qos_profile_sensor_data),
        'odom': source.create_publisher(VehicleOdometry, '/fmu/out/vehicle_odometry', qos_profile_sensor_data),
    }
    gz_node = GzNode()
    gz_image = GzImage(width=1280, height=960, step=3840, data=b'\x01\x02\x03' * (1280*960))
    gz_image.pixel_format_type = gz_image.DESCRIPTOR.fields_by_name['pixel_format_type'].enum_type.values_by_name['RGB_INT8'].number
    gz_info = GzInfo(width=1280, height=960)
    gz_info.intrinsics.k.extend([540.,0.,640.,0.,540.,480.,0.,0.,1.])
    gz_info.projection.p.extend([540.,0.,640.,0.,0.,540.,480.,0.,0.,0.,1.,0.])
    gz_info.rectification_matrix.extend([1.,0.,0.,0.,1.,0.,0.,0.,1.])
    gz_publishers = []
    for topic, kind, msg in [('image', GzImage, gz_image), ('info', GzInfo, gz_info)]:
        msg.header.stamp.sec, msg.header.stamp.nsec = 100, 50_000_000
        field = msg.header.data.add(); field.key = 'frame_id'; field.value.append('camera_link')
        gz_publishers.append((gz_node.advertise('/test_camera/'+topic, kind), msg))
    launch_path = Path(__file__).resolve().parents[1] / 'launch/camera.launch.py'
    process = subprocess.Popen(['ros2', 'launch', str(launch_path),
        'camera_gazebo_topic:=/test_camera/image', 'camera_info_gazebo_topic:=/test_camera/info'],
        stdout=(tmp_path/'camera_launch.log').open('w'), stderr=subprocess.STDOUT, start_new_session=True)
    def spin_until(predicate):
        deadline = time.monotonic()+5
        next_publish = 0.
        while not predicate() and time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                for publisher, msg in gz_publishers: publisher.publish(msg)
                next_publish = time.monotonic()+.05
            executor.spin_once(timeout_sec=.01)
        assert predicate(), (tmp_path/'camera_launch.log').read_text()
    try:
        spin_until(lambda: all(p.get_subscription_count() for p in publishers.values())
                   and {t.child_frame_id for t in transforms}
                   >= {'x500_0/camera_link', 'x500_0/camera_optical_frame'})
        clock = Clock(); clock.clock.sec, clock.clock.nanosec = 100,100_000_000
        publishers['clock'].publish(clock)
        spin_until(lambda: observer.get_clock().now().nanoseconds == 100_100_000_000)
        spin_until(lambda: observer.camera_info is not None and control.camera_info is not None
                   and bool(received_images))
        image = received_images[0]
        assert image.header.frame_id == 'x500_0/camera_optical_frame'
        assert (image.header.stamp.sec, image.header.stamp.nanosec) == (100, 50_000_000)
        assert image.header == observer.camera_info.header
        assert (image.width, image.height, image.encoding) == (1280, 960, 'rgb8')
        assert bytes(image.data) == gz_image.data
        from rclpy.qos import ReliabilityPolicy
        endpoints = source.get_publishers_info_by_topic('/x500_0/camera/image_raw')
        assert len(endpoints) == 1
        assert endpoints[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
        # DDS discovery does not report history depth with this RMW; query bridge configuration.
        from rclpy.parameter_client import AsyncParameterClient
        client = AsyncParameterClient(source, '/camera_bridge')
        spin_until(client.services_are_ready)
        future = client.get_parameters(['bridges.image.publisher_queue', 'bridges.image.subscriber_queue'])
        spin_until(future.done)
        assert [v.integer_value for v in future.result().values] == [1, 1]
        mount = next(t for t in transforms if t.child_frame_id == 'x500_0/camera_link')
        assert (mount.transform.translation.x, mount.transform.translation.y,
                mount.transform.translation.z) == pytest.approx((.12,.03,.002))
        optical = next(t for t in transforms if t.child_frame_id.endswith('optical_frame'))
        q = optical.transform.rotation
        assert rotate_body_to_ned((q.w,q.x,q.y,q.z),(0.,0.,1.)) == pytest.approx((1.,0.,0.))
        publishers['odom'].publish(odometry_sample(timestamp_sample=100_000_000,q=[1.,0.,0.,0.]))
        spin_until(lambda: len(control.attitudes)==1)
        publishers['odom'].publish(odometry_sample(timestamp_sample=100_100_000,q=[1.,0.,0.,0.]))
        spin_until(lambda: len(control.attitudes)==2)
        publishers['tracks'].publish(detection(100.05, u=640.,v=480.))
        spin_until(lambda: control.last_bearing_time_s is not None)
        assert control._prepare_guidance_observation(100.1) == pytest.approx((1.,0.,0.))
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        executor.shutdown()
        for node in nodes: node.destroy_node()
        rclpy.shutdown()


def test_future_state_does_not_block_later_valid_samples(controller):
    node, clock = controller
    node._vehicle_status_callback(VehicleStatus(timestamp=200_000_000))
    node._vehicle_local_position_callback(VehicleLocalPosition(timestamp_sample=200_000_000))
    clock.value = 100.1
    prime_controller(node, clock.value)
    assert node._vehicle_ready()


def test_small_sim_clock_delivery_skew_keeps_state_valid(controller):
    node, clock = controller
    prime_controller(node, clock.value + .005)
    assert node.vehicle_status.timestamp == 100_005_000
    assert node.vehicle_local_position.timestamp_sample == 100_005_000
    assert node.vehicle_odometry.timestamp_sample == 100_005_000
    assert node._vehicle_ready()


def test_status_freshness_allows_500ms_publication_with_jitter(controller):
    node, clock = controller
    clock.value = 100.51
    node._vehicle_local_position_callback(VehicleLocalPosition(
        timestamp_sample=100_510_000, xy_valid=True, z_valid=True,
        v_xy_valid=True, v_z_valid=True))
    node._vehicle_odometry_callback(odometry_sample(
        timestamp_sample=100_510_000, q=[1., 0., 0., 0.]))
    assert node._vehicle_ready()
    clock.value = 101.01
    assert not node._state_fresh("status", node.vehicle_status.timestamp)


def test_estimator_reset_discards_previous_los(controller):
    node, clock = controller
    node._bearing_callback(bearing(100.))
    clock.value = 100.01
    node._vehicle_odometry_callback(odometry_sample(timestamp_sample=100_010_000,
        q=[1.,0.,0.,0.], reset_counter=1))
    assert not node._fresh_bearing(clock.value)
    assert node.los_filter.predict(clock.value) is None
    assert len(node.attitudes) == 1


def test_inertial_azimuth_crosses_pi_without_filter_jump(controller):
    node, clock = controller
    for stamp, yaw in [(100.01, math.radians(179)), (100.02, math.radians(-179))]:
        clock.value = stamp
        node._vehicle_odometry_callback(odometry_sample(timestamp_sample=round(stamp*1e6),
            q=[math.cos(yaw/2),0.,0.,math.sin(yaw/2)]))
        node._bearing_callback(bearing(stamp))
    los = node._guidance_los_ned(clock.value)
    assert los[0] < -.99  # The target remains south instead of sweeping through north.
    assert node.los_filter_last_horizontal_rad == pytest.approx(math.radians(181))


def test_observer_tolerates_clock_delivery_skew_without_restamping(observation):
    node, clock = observation
    node.bearing_pub = Capture()
    node._detections_callback(detection(clock.value + .005))
    assert len(node.bearing_pub.messages) == 1
    assert node.bearing_pub.messages[0].header.stamp.nanosec == 5_000_000
    node._publish_status()
    assert node.active_pub.messages[-1].data
    clock.value += node.timeout_s + .006
    node._publish_status()
    assert not node.active_pub.messages[-1].data


def test_png_integrates_direction_reference_despite_velocity_response_lag(controller):
    from uav_trajectory_tracking.visual_pursuit_interceptor import direction_from_ned_angles
    node, _ = controller
    node.vehicle_odometry.velocity[0] = 1.0
    node._visual_png_velocity_setpoint(direction_from_ned_angles(0., 0.), .01)
    node._visual_png_velocity_setpoint(direction_from_ned_angles(.1, 0.), .01)
    assert node.last_png_desired_vertical_angle_rad == pytest.approx(.3)
    assert node.last_png_velocity_vertical_angle_rad == pytest.approx(0.)
    node.vehicle_odometry.velocity[0] = math.cos(.05)
    node.vehicle_odometry.velocity[2] = math.sin(.05)
    node._visual_png_velocity_setpoint(direction_from_ned_angles(.1, 0.), .01)
    assert node.last_png_desired_vertical_angle_rad == pytest.approx(.3)
    node._visual_png_velocity_setpoint(direction_from_ned_angles(.1, 0.), .01)
    assert node.last_png_desired_vertical_angle_rad == pytest.approx(.3)


@pytest.mark.parametrize('dt', [.004, .01, .02, 1/30])
def test_speed_increment_has_acceleration_units(controller, dt):
    from uav_trajectory_tracking.visual_pursuit_interceptor import vector_norm
    node, _ = controller
    node.vehicle_odometry.velocity[0] = 1.0
    command = node._visual_png_velocity_setpoint((1., 0., 0.), dt)
    assert (vector_norm(command) - 1.) / dt == pytest.approx(node.speed_accel_mps2)


def test_png_reacquisition_starts_from_measured_speed(controller):
    from uav_trajectory_tracking.visual_pursuit_interceptor import vector_norm
    node, _ = controller
    node.vehicle_odometry.velocity[0] = 2.0
    node.last_velocity_setpoint_ned = (0., 0., 0.)
    node._reset_guidance_state()
    command = node._visual_png_velocity_setpoint((1., 0., 0.), .01)
    assert vector_norm(command) == pytest.approx(2.01)


def test_camera_projection_inverse_with_mount_and_attitude():
    from uav_trajectory_tracking.visual_pursuit_interceptor import ned_los_to_optical
    ray = (.3, -.4, math.sqrt(.75))
    mount = rotation_from_rpy(.2, -.1, .3)
    q = (math.cos(.3), 0., 0., math.sin(.3))
    los = rotate_body_to_ned(q, fixed_camera_bearing_to_body_los(ray, mount))
    assert ned_los_to_optical(q, los, tuple(zip(*mount))) == pytest.approx(ray)


@pytest.mark.parametrize('pursuit,coast,expected', [
    (True, False, (False, True, False)),
    (False, True, (False, True, False)),
    (False, False, (True, False, False)),
])
def test_offboard_selects_one_control_level(controller, pursuit, coast, expected):
    node, _ = controller
    node.offboard_mode_pub = Capture()
    node._publish_offboard_control_mode(100_000_000, pursuit or coast)
    msg = node.offboard_mode_pub.messages[-1]
    assert (msg.position, msg.velocity, msg.body_rate) == expected
    assert not msg.acceleration and not msg.attitude and not msg.direct_actuator


def test_pursuit_loss_reacquisition_switches_output_and_clears_direction(controller):
    node, clock = controller
    node.takeoff_altitude_reached = node.initial_hover_reached = True
    node.takeoff_position = node.hold_position = (0., 0., -5.)
    node.vehicle_odometry.velocity[0] = 1.
    node.trajectory_pub = Capture()
    node.offboard_mode_pub = Capture()
    node.diagnostics_pub = Capture()
    node.vehicle_command_pub = Capture()
    node._bearing_callback(bearing(clock.value))
    node._lock_active_callback(Bool(data=True))
    node._timer_callback()
    assert node.state == InterceptorState.PURSUIT
    assert len(node.trajectory_pub.messages) == 1
    clock.value += .01
    node._lock_active_callback(Bool(data=False))
    node._timer_callback()
    assert node.state == InterceptorState.COAST
    assert len(node.trajectory_pub.messages) == 2
    assert node.last_png_velocity_vertical_angle_rad is None
    assert 0. < node.trajectory_pub.messages[-1].velocity[0] < 1.
    clock.value += .01
    node._lock_active_callback(Bool(data=True))
    node._timer_callback()
    assert node.state == InterceptorState.PURSUIT
    assert len(node.trajectory_pub.messages) == 3


def test_guidance_reuses_mount_and_predicts_once_per_state_sample(controller, monkeypatch):
    from unittest.mock import Mock
    import uav_trajectory_tracking.visual_pursuit_interceptor as interceptor

    node, clock = controller
    node.takeoff_altitude_reached = node.initial_hover_reached = True
    for publisher in ('trajectory_pub', 'offboard_mode_pub',
                      'diagnostics_pub', 'vehicle_command_pub'):
        setattr(node, publisher, Capture())
    prediction = Mock(wraps=node.los_filter.predict)
    monkeypatch.setattr(node.los_filter, 'predict', prediction)
    rotation = Mock(side_effect=AssertionError('Fixed mount must be computed at initialization'))
    monkeypatch.setattr(interceptor, 'rotation_from_rpy', rotation)
    node._bearing_callback(bearing(clock.value, (.6, 0., .8)))
    assert prediction.call_count == 0  # Acquisition callback only updates the filter.
    node._lock_active_callback(Bool(data=True))
    node._timer_callback()
    assert prediction.call_count == 1
    assert len(node.trajectory_pub.messages) == 1
    assert node.last_los_body == pytest.approx((.8, .6, 0.))
    values = {v.key: v.value for v in node.diagnostics_pub.messages[-1].status[0].values}
    assert values['los_filter_ready'] == 'true'
    assert not any('dkf' in key for key in values)
    clock.value += .01
    node._timer_callback()
    assert prediction.call_count == 1
    first, held = node.trajectory_pub.messages
    assert held.timestamp > first.timestamp
    assert list(held.velocity) == list(first.velocity)
    assert held.yaw == first.yaw
    node._vehicle_odometry_callback(odometry_sample(timestamp_sample=100_010_000))
    node._timer_callback()
    assert prediction.call_count == 2
    assert node.last_guidance_sample_us == 100_010_000
    assert rotation.call_count == 0


def start_pursuit(node, clock):
    node.takeoff_altitude_reached = node.initial_hover_reached = True
    node.takeoff_position = node.hold_position = (0., 0., -5.)
    for name in ('trajectory_pub', 'offboard_mode_pub',
                 'diagnostics_pub', 'vehicle_command_pub'):
        setattr(node, name, Capture())
    node._bearing_callback(bearing(clock.value))
    node._lock_active_callback(Bool(data=True))
    node._timer_callback()
    assert node.state == InterceptorState.PURSUIT


def test_guidance_uses_odometry_epoch_velocity_and_sample_dt(controller, monkeypatch):
    from unittest.mock import Mock
    node, clock = controller
    guidance = Mock(wraps=node._visual_png_velocity_setpoint)
    monkeypatch.setattr(node, '_visual_png_velocity_setpoint', guidance)
    start_pursuit(node, clock)
    assert guidance.call_args.args[1] == 0.  # Initialize histories without a derivative kick.
    clock.value = 100.008
    node._timer_callback()
    assert guidance.call_count == 1
    # A separately updated position topic must not enter the guidance velocity.
    node._vehicle_local_position_callback(VehicleLocalPosition(
        timestamp_sample=100_008_000, xy_valid=True, z_valid=True,
        v_xy_valid=True, v_z_valid=True, vx=9., vy=9., vz=9.))
    node._vehicle_odometry_callback(odometry_sample(
        timestamp_sample=100_020_000, velocity=[1., 0., 0.]))
    clock.value = 100.024
    node._timer_callback()
    assert guidance.call_count == 2
    assert guidance.call_args.args[1] == pytest.approx(.020)  # Not .016 publication dt.
    assert node._current_velocity_ned() == (1., 0., 0.)
    assert node.last_velocity_setpoint_ned == pytest.approx((.020, 0., 0.))  # Reference ramps from zero.
    assert node.los_filter_prediction_horizon_s == pytest.approx(.020)
    clock.value = 100.032
    node._timer_callback()
    assert guidance.call_count == 2
    a, b = node.trajectory_pub.messages[-2:]
    assert list(a.velocity) == list(b.velocity) and a.yaw == b.yaw


@pytest.mark.parametrize('yaw,pitch', [(math.pi, 0.), (math.pi / 2, 0.),
                                      (1.0, 0.), (0., .9)])
def test_fresh_exposure_can_leave_current_camera_view(controller, yaw, pitch):
    node, clock = controller
    start_pursuit(node, clock)
    clock.value = 100.02
    # q = yaw * pitch. The exposure ray was centered at identity attitude.
    q = [math.cos(yaw/2)*math.cos(pitch/2), -math.sin(yaw/2)*math.sin(pitch/2),
         math.cos(yaw/2)*math.sin(pitch/2), math.sin(yaw/2)*math.cos(pitch/2)]
    node._vehicle_odometry_callback(odometry_sample(timestamp_sample=100_020_000, q=q))
    node._timer_callback()
    assert node._fresh_bearing(clock.value)
    assert node.guidance_observation is None
    assert node.state == InterceptorState.COAST
    assert len(node.trajectory_pub.messages) == 2
    mode = node.offboard_mode_pub.messages[-1]
    assert mode.velocity and not mode.body_rate and not mode.position
    assert node.last_guidance_sample_us is None


def test_predicted_los_outside_image_expires_coast_despite_lock_true(controller):
    node, clock = controller
    start_pursuit(node, clock)
    # A valid acquired image can predict beyond the right edge at the next state epoch.
    node.los_filter.yaw.state.rate_rad_s = 10.
    clock.value = 100.1
    node._vehicle_odometry_callback(odometry_sample(timestamp_sample=100_100_000))
    node._lock_active_callback(Bool(data=True))
    node._timer_callback()
    assert node._fresh_bearing(clock.value)
    assert node.state == InterceptorState.COAST
    assert node.guidance_observation is None
    clock.value = 100.41
    prime_controller(node, clock.value, position=(1., 2., -5.))
    node._lock_active_callback(Bool(data=True))
    node._timer_callback()
    assert node.state == InterceptorState.ACQUIRING
    assert node.offboard_mode_pub.messages[-1].position
    assert node.vertical_search_start_time_s == pytest.approx(clock.value)
    assert len(node.trajectory_pub.messages) == 3
    # A new centered observation restores pursuit using freshly initialized differences.
    clock.value = 100.42
    prime_controller(node, clock.value, position=(1., 2., -5.))
    node._bearing_callback(bearing(clock.value))
    node._timer_callback()
    assert node.state == InterceptorState.PURSUIT
    assert node.last_velocity_setpoint_ned == (0., 0., 0.)
    assert len(node.trajectory_pub.messages) == 4


@pytest.mark.parametrize('field,value', [
    ('pose_frame', VehicleOdometry.POSE_FRAME_FRD),
    ('velocity_frame', VehicleOdometry.VELOCITY_FRAME_BODY_FRD),
    ('position', [math.nan, 0., 0.]), ('velocity', [0., math.nan, 0.]),
])
def test_odometry_requires_finite_unified_ned_state(controller, field, value):
    node, clock = controller
    old = node.vehicle_odometry
    clock.value = 100.01
    node._vehicle_odometry_callback(odometry_sample(timestamp_sample=100_010_000, **{field: value}))
    assert node.vehicle_odometry is old


def test_missing_calibration_and_expired_bearing_stop_held_pursuit(controller):
    node, clock = controller
    start_pursuit(node, clock)
    node._camera_info_callback(CameraInfo())
    node._timer_callback()
    assert node.state == InterceptorState.COAST
    assert len(node.trajectory_pub.messages) == 2
    clock.value = 100.13
    prime_controller(node, clock.value)
    node._timer_callback()
    assert not node._fresh_bearing(clock.value)
    assert node.state == InterceptorState.COAST
    assert len(node.trajectory_pub.messages) == 3


def test_clock_delivery_skew_does_not_trigger_false_loss(controller):
    node, clock = controller
    start_pursuit(node, clock)
    # DDS odometry may arrive just ahead of the corresponding /clock message.
    node._vehicle_odometry_callback(odometry_sample(timestamp_sample=100_004_000))
    node._timer_callback()
    assert node.state == InterceptorState.PURSUIT
    assert node.last_guidance_sample_us == 100_004_000
    assert node.los_filter_prediction_horizon_s == pytest.approx(.004)


@pytest.mark.parametrize('angle_deg', [-60., -25., 15., 25., 60.])
def test_png_starts_after_initial_sample_without_alignment_gate(controller, angle_deg):
    from uav_trajectory_tracking.visual_pursuit_interceptor import direction_from_ned_angles
    node, _ = controller
    direction = direction_from_ned_angles(0., math.radians(angle_deg))
    node.vehicle_odometry.velocity = list(direction)
    los = (1., 0., 0.)
    assert node._visual_png_velocity_setpoint(los, 0.) == pytest.approx(los)
    # LOS moves by .02 rad; PNG adds .06 rad even outside the former 10/20 deg gates.
    command = node._visual_png_velocity_setpoint(direction_from_ned_angles(0., .02), .01)
    expected = direction_from_ned_angles(0., .06)
    assert command == pytest.approx(tuple(1.01 * v for v in expected))


@pytest.mark.parametrize('initial_velocity', [(0., 0., 0.), (1., 0., 0.),
                                              (0., 1., 0.), (-1., 0., 0.), (0., 0., -1.)])
@pytest.mark.parametrize('dt', [.01, .02])
def test_guidance_closes_on_target_with_velocity_response_and_crosswind(controller, initial_velocity, dt):
    """Guidance-only plant with finite velocity response; not a PX4 dynamics model."""
    import numpy as np
    node, _ = controller
    position = np.zeros(3)
    velocity = np.array(initial_velocity)
    target = np.array([15., 0., 0.])
    node.vehicle_odometry.velocity = velocity.tolist()
    node._visual_png_velocity_setpoint((1., 0., 0.), 0.)
    closest = 15.
    for _ in range(round(20. / dt)):
        relative = target - position
        distance = np.linalg.norm(relative)
        closest = min(closest, distance)
        if distance < 1.:
            break
        node.vehicle_odometry.velocity = velocity.tolist()
        command = node._visual_png_velocity_setpoint(tuple(relative / distance), dt)
        assert np.dot(command, relative) > 0.
        acceleration = (np.array(command) - velocity) / .35 + np.array([0., .35, .08])
        acceleration *= min(1., 3. / max(np.linalg.norm(acceleration), 1e-9))
        velocity += acceleration * dt
        position += velocity * dt
    assert closest < 1., (position, velocity, closest)


@pytest.mark.parametrize('low_speed', [0., .1])
def test_png_low_speed_recovery_requires_adjacent_valid_direction_samples(controller, low_speed):
    from uav_trajectory_tracking.visual_pursuit_interceptor import direction_from_ned_angles
    node, _ = controller
    direction = direction_from_ned_angles(0., .4)
    los = (1., 0., 0.)
    node.vehicle_odometry.velocity = list(direction)
    node._visual_png_velocity_setpoint(los, 0.)
    node._visual_png_velocity_setpoint(los, .01)
    node.vehicle_odometry.velocity = [low_speed, 0., 0.]
    command = node._visual_png_velocity_setpoint(los, .01)
    assert command[0] > 0. and command[1:] == pytest.approx((0., 0.))
    assert node.last_png_velocity_horizontal_angle_rad is None
    node.vehicle_odometry.velocity = list(direction)
    command = node._visual_png_velocity_setpoint(los, .01)
    assert command[0] > 0. and command[1:] == pytest.approx((0., 0.))
    node._visual_png_velocity_setpoint(direction_from_ned_angles(0., .02), .01)
    assert node.last_png_desired_horizontal_angle_rad == pytest.approx(.06)
    node._reset_guidance_state()
    node._visual_png_velocity_setpoint(los, 0.)
    assert node.last_png_desired_horizontal_angle_rad == pytest.approx(0.)


def test_zero_speed_is_not_a_direction_when_threshold_is_zero(controller):
    node, _ = controller
    node.min_velocity_direction_mps = 0.
    node.vehicle_odometry.velocity = [0., 0., 0.]
    for _ in range(2):
        command = node._visual_png_velocity_setpoint((0., 1., 0.), .01)
        assert command[0] == pytest.approx(0.) and command[1] > 0.
        assert node.last_png_velocity_horizontal_angle_rad is None


def test_png_large_los_innovation_realigns_in_same_sample(controller):
    node, _ = controller
    node.vehicle_odometry.velocity = [1., 0., 0.]
    # Previous reference was 50 deg left; a 50 deg LOS jump gives a 100 deg candidate.
    node._visual_png_velocity_setpoint((math.cos(math.radians(50.)),
                                       -math.sin(math.radians(50.)), 0.), .01)
    command = node._visual_png_velocity_setpoint((1., 0., 0.), .01)
    assert command == pytest.approx((1.02, 0., 0.))
    assert node.last_png_desired_horizontal_angle_rad == pytest.approx(0.)




@pytest.mark.parametrize('dt', [.008, .012, .02])
def test_velocity_reference_accumulates_when_actual_speed_stalls(controller, dt):
    from uav_trajectory_tracking.visual_pursuit_interceptor import vector_norm
    node, _ = controller
    node.vehicle_odometry.velocity = [.1, 0., 0.]
    node._visual_png_velocity_setpoint((1., 0., 0.), 0.)
    for step in range(1, round(4. / dt) + 1):
        command = node._visual_png_velocity_setpoint((1., 0., 0.), dt)
        assert vector_norm(command) == pytest.approx(min(3., .1 + step * dt))
    node._reset_guidance_state()
    node.vehicle_odometry.velocity = [.4, 0., 0.]
    assert node._visual_png_velocity_setpoint((1., 0., 0.), 0.) == pytest.approx((.4, 0., 0.))


def test_pursuit_sends_ned_velocity_and_los_yaw_without_acceleration_feedforward(controller):
    node, clock = controller
    start_pursuit(node, clock)
    clock.value += .02
    node._vehicle_odometry_callback(odometry_sample(timestamp_sample=100_020_000))
    node._bearing_callback(bearing(clock.value, (.3, .4, math.sqrt(.75))))
    node._timer_callback()
    msg = node.trajectory_pub.messages[-1]
    assert list(msg.velocity) == pytest.approx([.02*v for v in node.last_los_ned])
    assert msg.yaw == pytest.approx(math.atan2(node.last_los_ned[1], node.last_los_ned[0]))
    assert all(math.isnan(v) for v in (*msg.position, *msg.acceleration, *msg.jerk))
    assert math.isnan(msg.yawspeed)
    mode = node.offboard_mode_pub.messages[-1]
    assert mode.velocity and not any((mode.position, mode.acceleration, mode.attitude,
                                     mode.body_rate, mode.thrust_and_torque, mode.direct_actuator))
    values = {v.key: v.value for v in node.diagnostics_pub.messages[-1].status[0].values}
    assert not any('thrust' in k or 'body_rate' in k or 'guidance_accel' in k for k in values)


@pytest.mark.parametrize('namespace', ['/', '/verification'])
def test_observation_timeout_shared_ros_parameters(tmp_path, namespace):
    import yaml
    config_path = Path(__file__).parents[1] / 'config' / 'fixed_camera_tracking.yaml'
    params = yaml.safe_load(config_path.read_text())
    params['/**']['ros__parameters']['observation_timeout_s'] = .37
    shared = tmp_path / 'shared.yaml'
    shared.write_text(yaml.safe_dump(params))
    nodes = []
    rclpy.init(args=['--ros-args', '--params-file', str(shared), '-r', f'__ns:={namespace}'])
    try:
        observer = FixedCameraTargetTracker()
        nodes.append(observer)
        control = VisualPursuitInterceptor()
        nodes.append(control)
        assert observer.timeout_s == pytest.approx(.37)
        assert control.observation_timeout_s == pytest.approx(.37)
        assert control._observation_horizon() == pytest.approx(.12)
    finally:
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()


def test_shared_status_timeout_keeps_topic_reception_times_independent(controller):
    node, clock = controller
    node.tracking_status_timeout_s = .2
    node._tracking_active_callback(Bool(data=True))
    clock.value += .1
    node._lock_active_callback(Bool(data=True))
    assert node._tracking_signal_active(100.19)
    assert node._lock_signal_active(100.19)
    assert not node._tracking_signal_active(100.25)
    assert node._lock_signal_active(100.25)
    assert not node._lock_signal_active(100.31)
