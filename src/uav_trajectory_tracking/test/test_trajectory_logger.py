import csv
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleOdometry

from uav_trajectory_tracking.state_comparison import (
    Sample, SamplePairs, geodetic_to_px4_ned, world_to_geodetic,
)
from uav_trajectory_tracking.trajectory_logger import (
    TrajectoryLogger, enu_flu_quaternion_to_ned_frd, quaternion_to_rpy,
)


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_truth_interpolates_at_px4_measurement_time_once():
    pairs = SamplePairs()
    pairs.add('truth', 'position', Sample(10.0, (0., 0., 0.), 100.0))
    pairs.add('px4', 'position', Sample(10.05, (0.15, 0., 0.), 100.03))
    assert list(pairs.ready(100.03)) == []  # no extrapolation
    pairs.add('truth', 'position', Sample(10.1, (0.3, 0., 0.), 100.09))
    result = list(pairs.ready(100.1))
    assert len(result) == 1
    assert result[0][2] == pytest.approx((0.15, 0, 0))
    assert list(pairs.ready(100.11)) == []
    assert not pairs.add('px4', 'position', Sample(10.05, (99., 0., 0.), 100.2))


def test_quaternion_interpolation_crosses_pi_without_false_full_turn():
    pairs = SamplePairs(max_gap_s=0.2)
    q = lambda angle: (math.cos(angle/2), 0., 0., math.sin(angle/2))
    pairs.add('truth', 'rpy', Sample(10., q(math.radians(179)), 100.))
    pairs.add('truth', 'rpy', Sample(10.1, q(math.radians(-179)), 100.))
    pairs.add('px4', 'rpy', Sample(10.05, q(math.pi), 100.))
    truth = list(pairs.ready(100.))[0][2]
    assert abs(quaternion_to_rpy(truth)[2]) == pytest.approx(math.pi)


@pytest.mark.parametrize('right,now', [(10.5, 100.1), (10.1, 101.)])
def test_large_gap_and_stale_truth_are_rejected(right, now):
    pairs = SamplePairs()
    pairs.add('truth', 'velocity', Sample(10., (0., 0., 0.), 100.))
    pairs.add('truth', 'velocity', Sample(right, (1., 0., 0.), 100.))
    pairs.add('px4', 'velocity', Sample(10.05, (0., 0., 0.), now))
    assert list(pairs.ready(now)) == []
    assert pairs.dropped == 1


def test_invalid_values_and_zero_quaternions_rejected():
    pairs = SamplePairs()
    assert not pairs.add('truth', 'position', Sample(10., (math.nan, 0, 0), 100.))
    assert not pairs.add('px4', 'rpy', Sample(10., (0., 0., 0., 0.), 100.))


def test_geodesy_matches_gazebo_wgs84():
    # Independent oracle: the same WGS84 conversion used by Gazebo NavSat.
    from gz.math7 import Angle, SphericalCoordinates, Vector3d
    origin = (47.397971057728974, 8.546163739800146, 0.)
    sphere = SphericalCoordinates(SphericalCoordinates.EARTH_WGS84,
                                  Angle(math.radians(origin[0])), Angle(math.radians(origin[1])),
                                  origin[2], Angle(0.))
    for point in [(0., 0., .24), (0., 5., .24), (30., -20., 10.)]:
        # NavSat calls PositionTransform(LOCAL2, SPHERICAL); the legacy
        # SphericalFromLocalPosition wrapper retains an XY inversion bug.
        expected = sphere.position_transform(Vector3d(*point), SphericalCoordinates.LOCAL2,
                                             SphericalCoordinates.SPHERICAL)
        actual = world_to_geodetic(point, origin)
        assert actual[:2] == pytest.approx((math.degrees(expected.x()), math.degrees(expected.y())), abs=1e-9)
        assert actual[2] == pytest.approx(expected.z(), abs=1e-6)
        assert geodetic_to_px4_ned(actual, actual) == pytest.approx((0,0,0), abs=1e-9)


@pytest.fixture
def logger(tmp_path, monkeypatch):
    rclpy.init()
    monkeypatch.setattr(TrajectoryLogger, '_make_log_dir', lambda self: tmp_path)
    node = TrajectoryLogger()
    clock = SimpleNamespace(sim=10., steady=100.)
    monkeypatch.setattr(node, '_ros_now_s', lambda: clock.sim)
    monkeypatch.setattr(node, '_monotonic_s', lambda: clock.steady)
    node.state_compare_publishers = {key: Capture() for key in node.state_compare_publishers}
    node.comparison_status_pub = Capture()
    yield node, clock
    node.destroy_node()
    rclpy.shutdown()


def local(node, stamp=10., bias=0., reference=None):
    if reference is None:
        reference = world_to_geodetic((0.,5.,.24), node.world_origin)
    return VehicleLocalPosition(
        timestamp=int(stamp*1e6), timestamp_sample=int(stamp*1e6),
        xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True,
        xy_global=True, z_global=True, ref_timestamp=1,
        ref_lat=reference[0], ref_lon=reference[1], ref_alt=reference[2],
        x=bias, y=0., z=0.)


def truth(stamp, position=(0.,5.,0.), q=(1.,0.,0.,0.), omega=(0.,0.,0.)):
    msg = Odometry()
    stamp_ns = round(stamp*1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    msg.header.frame_id = 'x500_1/odom'
    msg.child_frame_id = 'x500_1/base_footprint'
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = position
    msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z = q
    msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = omega
    return msg


def test_target_origin_alignment_preserves_real_estimation_bias(logger):
    node, clock = logger
    node._vehicle_local_position_callback(local(node, bias=.4))
    node._gazebo_odometry_callback(truth(9.99))
    node._gazebo_odometry_callback(truth(10.01))
    px4 = node.state_compare_publishers['px4_position_ned'].messages[-1]
    actual = node.state_compare_publishers['truth_position_ned'].messages[-1]
    error = node.state_compare_publishers['position_error_ned'].messages[-1]
    assert (actual.vector.x, actual.vector.y, actual.vector.z) == pytest.approx((0,0,0), abs=1e-6)
    assert error.vector.x == pytest.approx(.4)
    assert px4.header == actual.header == error.header
    assert error.header.stamp.sec == 10
    assert error.header.stamp.nanosec == 0
    rows = list(csv.DictReader((node.log_dir/'state_comparison.csv').open()))
    position = next(row for row in rows if row['quantity'] == 'position')
    assert float(position['truth_bracket_s']) == pytest.approx(.02)
    assert float(position['error_x']) == pytest.approx(.4)
    assert 'dds_estimated_offset_us' not in position


def test_missing_origin_does_not_publish_position_error(logger):
    node, _ = logger
    msg = local(node)
    msg.xy_global = False
    node._vehicle_local_position_callback(msg)
    node._gazebo_odometry_callback(truth(9.99))
    node._gazebo_odometry_callback(truth(10.01))
    assert not node.state_compare_publishers['position_error_ned'].messages
    assert node.state_compare_publishers['velocity_error_ned'].messages
    node._publish_comparison_status()
    assert node.comparison_status_pub.messages[-1].status[0].message == 'waiting_for_global_reference'


def test_body_point_transport_accounts_for_rotation_and_angular_velocity(logger):
    node, _ = logger
    q = (math.sqrt(.5), 0., math.sqrt(.5), 0.)  # model FLU pitch +90
    node._queue_truth(10., (0.,0.,0.), q, (0.,0.,0.), (0.,1.,0.))
    assert node.pairs.truth['position'][-1].value == pytest.approx((.24,0.,0.), abs=1e-9)
    # omega cross r = (+.24,0,0) FLU; rotated into world -Z, hence NED +Z.
    assert node.pairs.truth['velocity'][-1].value == pytest.approx((0.,0.,.24), abs=1e-9)


def test_attitude_and_angular_velocity_keep_their_own_sample_times(logger):
    node, clock = logger
    node._vehicle_local_position_callback(local(node))
    node._gazebo_odometry_callback(truth(10.))
    node._gazebo_odometry_callback(truth(10.02))
    clock.sim += .02
    node._vehicle_attitude_callback(VehicleAttitude(
        timestamp=10_020_000, timestamp_sample=10_010_000,
        q=enu_flu_quaternion_to_ned_frd((1.,0.,0.,0.))))
    node._vehicle_odometry_callback(VehicleOdometry(
        timestamp=10_020_000, timestamp_sample=10_015_000,
        angular_velocity=[0.,0.,0.]))
    attitude = node.state_compare_publishers['rpy_error_ned_frd'].messages[-1]
    angular = node.state_compare_publishers['angular_velocity_error_body_frd'].messages[-1]
    assert attitude.header.stamp.nanosec == 10_000_000
    assert angular.header.stamp.nanosec == 15_000_000
    assert attitude.vector.z == pytest.approx(0, abs=1e-6)


def test_reference_and_estimator_resets_clear_pending_pairs(logger):
    node, clock = logger
    node._vehicle_local_position_callback(local(node))
    first_epoch = node.comparison_epoch
    clock.sim += .01
    msg = local(node, stamp=clock.sim)
    msg.ref_timestamp = 2
    msg.ref_lon += .0001
    node._vehicle_local_position_callback(msg)
    assert node.comparison_epoch == first_epoch+1
    assert len(node.pairs.pending['position']) == 1
    assert node.pairs.pending['position'][0].reference[1] == msg.ref_lon
    clock.sim += .01
    msg.timestamp_sample = int(clock.sim*1e6)
    msg.xy_reset_counter = 1
    node._vehicle_local_position_callback(msg)
    assert node.comparison_epoch == first_epoch+2
    assert len(node.pairs.pending['position']) == 1


def test_delayed_px4_packet_cannot_roll_back_reference(logger):
    node, clock = logger
    node._vehicle_local_position_callback(local(node))
    key = node.reference_key
    old = local(node, stamp=9.99)
    old.ref_timestamp = 99
    node._vehicle_local_position_callback(old)
    assert node.reference_key == key


def test_paused_simulation_expires_pairs(logger):
    node, clock = logger
    node._vehicle_local_position_callback(local(node))
    node._gazebo_odometry_callback(truth(10.))
    clock.steady += node.comparison_max_age_s+1
    node._publish_comparison_status()
    assert node.comparison_status_pub.messages[-1].status[0].message == 'waiting_for_paired_samples'
    assert not any(node.pairs.pending.values())


def test_truth_acceleration_is_at_interval_midpoint(logger):
    node, _ = logger
    node._queue_truth(10., (0.,0.,0.), (1.,0.,0.,0.), (0.,0.,0.), (0.,0.,0.))
    node._queue_truth(10.02, (0.,0.,0.), (1.,0.,0.,0.), (.02,0.,0.), (0.,0.,0.))
    sample = node.pairs.truth['acceleration'][-1]
    assert sample.time_s == pytest.approx(10.01)
    assert sample.value == pytest.approx((0.,1.,0.))


def test_common_clock_rejects_wrong_epoch_old_and_missing_samples(logger):
    node, clock = logger
    assert node._accept_px4_message('position', 1_000_000_000) is None
    assert node._accept_px4_message('position', 0) is None
    assert node._accept_px4_message('position', 9_000_000) is None
    assert node._accept_px4_message('position', 10_000_000) == pytest.approx(10.)


def test_default_geographic_reference_matches_world():
    import xml.etree.ElementTree as ET
    import yaml
    root = Path(__file__).resolve().parents[3]
    config = yaml.safe_load((root/'src/uav_trajectory_tracking/config/trajectory_logging.yaml').read_text())
    sphere = ET.parse(root/'px4_overlays/worlds/trajectory_tracking.sdf').find('.//spherical_coordinates')
    assert sphere.findtext('world_frame_orientation') == 'ENU'
    assert float(sphere.findtext('heading_deg', '0')) == 0
    assert config['world_origin_lat_lon_alt'] == pytest.approx([
        float(sphere.findtext(field)) for field in ('latitude_deg', 'longitude_deg', 'elevation')])


def test_real_ros_subscriptions_deliver_paired_position(logger):
    import time
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    node, clock = logger
    source = Node('logger_test_source')
    pubs = {}
    for key, msg_type in [('vehicle_local_position_topic', VehicleLocalPosition),
                          ('gazebo_odometry_topic', Odometry)]:
        pubs[key] = source.create_publisher(msg_type, node.get_parameter(key).value, qos_profile_sensor_data)
    try:
        deadline = time.monotonic()+3
        while any(pub.get_subscription_count() == 0 for pub in pubs.values()) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert all(pub.get_subscription_count() > 0 for pub in pubs.values())
        pubs['vehicle_local_position_topic'].publish(local(node, bias=.25))
        deadline = time.monotonic()+2
        while node.reference is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert node.reference is not None
        pubs['gazebo_odometry_topic'].publish(truth(9.99))
        pubs['gazebo_odometry_topic'].publish(truth(10.01))
        out = node.state_compare_publishers['position_error_ned'].messages
        deadline = time.monotonic()+2
        while not out and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert out
        assert out[-1].vector.x == pytest.approx(.25)
        assert out[-1].header.stamp.sec == 10
    finally:
        source.destroy_node()


def test_control_log_preserves_controller_sample_time_and_mode(logger):
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    node, _ = logger
    msg = DiagnosticArray()
    msg.header.stamp.sec = 9
    msg.header.stamp.nanosec = 250_000_000
    msg.status = [DiagnosticStatus(name='visual_pursuit_interceptor', values=[
        KeyValue(key='state', value='pursuit'),
        KeyValue(key='body_rate_control', value='true'),
        KeyValue(key='collective_thrust_normalized', value='0.74'),
    ])]
    node._control_diagnostics_callback(msg)
    rows = list(csv.DictReader((node.log_dir/'visual_control.csv').open()))
    assert len(rows) == 1
    assert float(rows[0]['sample_sim_time_s']) == 9.25
    assert rows[0]['body_rate_control'] == 'true'
    assert rows[0]['state'] == 'pursuit'
