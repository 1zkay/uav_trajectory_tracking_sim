from types import SimpleNamespace

import pytest
import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSCompatibility, qos_check_compatible
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from uav_trajectory_tracking.state_comparison import geodetic_to_px4_ned, world_to_geodetic
from uav_trajectory_tracking.trajectory_visualizer import TrajectoryVisualizer


WORLD_ORIGIN = (47.397971057728974, 8.546163739800146, 0.)


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def visualizer(request, monkeypatch):
    parameters = {'publish_planned_path': False, 'vehicle_path_max_points': 3}
    parameters.update(getattr(request, 'param', {}))
    args = ['--ros-args']
    for name, value in parameters.items():
        args += ['-p', f'{name}:={str(value).lower() if isinstance(value, bool) else value}']
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryVisualizer()
        node.timer.cancel()
        clock = SimpleNamespace(seconds=100.)
        monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(
            now=lambda: Time(nanoseconds=int(clock.seconds * 1e9))))
        node.marker_pub = Capture()
        node.vehicle_path_pub = Capture()
        if node.planned_path_pub is not None:
            node.planned_path_pub = Capture()
        yield node, clock
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def local(point=(0., 5., 5.24), stamp=10., reference_point=(0., 5., .24), **overrides):
    reference = world_to_geodetic(reference_point, WORLD_ORIGIN)
    position = geodetic_to_px4_ned(world_to_geodetic(point, WORLD_ORIGIN), reference)
    values = dict(timestamp=int(stamp * 1e6), timestamp_sample=int(stamp * 1e6),
                  xy_valid=True, z_valid=True, xy_global=True, z_global=True,
                  ref_timestamp=1, ref_lat=reference[0], ref_lon=reference[1], ref_alt=reference[2],
                  x=position[0], y=position[1], z=position[2])
    values.update(overrides)
    return VehicleLocalPosition(**values)


def coordinates(pose):
    point = pose.pose.position
    return point.x, point.y, point.z


@pytest.mark.parametrize('visualizer', [{'trajectory_file': '/not-used-in-actual-only-mode.yaml'}], indirect=True)
def test_actual_only_needs_no_planned_trajectory(visualizer):
    node, _ = visualizer
    assert node.planned_path_pub is None
    assert not any(sub.topic_name.endswith('/current_stage') for sub in node.subscriptions)
    assert not any(pub.topic_name.startswith('/fmu/in/') for pub in node.publishers)
    node._vehicle_local_position_callback(local())
    node._timer_callback()
    assert {marker.ns for marker in node.marker_pub.messages[-1].markers} == {'', 'vehicle'}
    assert len(node.vehicle_path_pub.messages[-1].poses) == 1


@pytest.mark.parametrize('reference_point', [(0., 0., .24), (0., 5., .24)])
def test_common_map_and_original_sample_stamps(visualizer, reference_point):
    node, clock = visualizer
    points = [(1., 7., 5.24), (2., 8., 6.24)]
    for point, stamp in zip(points, (10.123456, 10.5)):
        node._vehicle_local_position_callback(local(point, stamp, reference_point))
    for now in (100., 101.):
        clock.seconds = now
        node._timer_callback()
        path = node.vehicle_path_pub.messages[-1]
        assert path.header.frame_id == 'map'
        assert path.header.stamp.sec == 10
        assert path.header.stamp.nanosec == 500_000_000
        for pose, point in zip(path.poses, points):
            assert pose.header.frame_id == 'map'
            assert coordinates(pose) == pytest.approx(point, abs=2e-6)
            assert pose.pose.orientation.w == 1.
        assert path.poses[0].header.stamp.nanosec == 123_456_000


@pytest.mark.parametrize('invalid', [
    {'xy_valid': False}, {'z_valid': False}, {'xy_global': False}, {'z_global': False},
    {'x': float('nan')}, {'ref_alt': float('inf')}, {'ref_timestamp': 0}, {'timestamp_sample': 0},
])
def test_invalid_positions_do_not_form_a_path(visualizer, invalid):
    node, _ = visualizer
    node._vehicle_local_position_callback(local(**invalid))
    node._timer_callback()
    assert node.vehicle_path_pub.messages[-1].poses == []
    assert not any(marker.ns == 'vehicle' for marker in node.marker_pub.messages[-1].markers)


def test_decimation_bounded_history_and_out_of_order_samples(visualizer):
    node, _ = visualizer
    for index, east in enumerate((0., .01, 1., 2., 3.)):
        node._vehicle_local_position_callback(local((east, 5., 5.24), 10. + index))
    node._vehicle_local_position_callback(local((100., 5., 5.24), 13.))
    node._vehicle_local_position_callback(local((100., 5., 5.24), 14.))
    node._timer_callback()
    path = node.vehicle_path_pub.messages[-1]
    assert len(path.poses) == 3
    assert [pose.pose.position.x for pose in path.poses] == pytest.approx([1., 2., 3.], abs=2e-6)


@pytest.mark.parametrize('change', [
    {'xy_reset_counter': 1}, {'z_reset_counter': 1}, {'ref_timestamp': 2},
])
def test_estimator_resets_start_a_new_path_segment(visualizer, change):
    node, _ = visualizer
    node._vehicle_local_position_callback(local())
    node._vehicle_local_position_callback(local((10., 5., 5.24), 11., **change))
    node._timer_callback()
    assert len(node.vehicle_path_pub.messages[-1].poses) == 1


def test_simulation_clock_reset_drops_previous_run(visualizer):
    node, clock = visualizer
    node._vehicle_local_position_callback(local())
    node._timer_callback()
    clock.seconds = 1.
    node._timer_callback()
    assert node.vehicle_path_pub.messages[-1].poses == []
    node._vehicle_local_position_callback(local(stamp=1.))
    node._timer_callback()
    assert len(node.vehicle_path_pub.messages[-1].poses) == 1


@pytest.mark.parametrize('reset_by_timer', [False, True])
def test_clock_reset_rejects_late_previous_run_sample(visualizer, reset_by_timer):
    node, clock = visualizer
    node._vehicle_local_position_callback(local(stamp=100.))
    clock.seconds = 1.
    if reset_by_timer:
        node._timer_callback()
    node._vehicle_local_position_callback(local(stamp=100.01))
    node._timer_callback()
    assert node.vehicle_path_pub.messages[-1].poses == []
    assert not any(marker.ns == 'vehicle' for marker in node.marker_pub.messages[-1].markers)

    node._vehicle_local_position_callback(local((1., 5., 5.24), stamp=1.01))
    node._timer_callback()
    path = node.vehicle_path_pub.messages[-1]
    assert len(path.poses) == 1
    assert coordinates(path.poses[0]) == pytest.approx((1., 5., 5.24), abs=2e-6)
    assert path.header.stamp.sec == 1
    assert path.header.stamp.nanosec == 10_000_000


@pytest.mark.parametrize('lead_us, accepted', [(10_000, True), (20_000, True), (20_001, False)])
def test_sample_clock_skew_tolerance(visualizer, lead_us, accepted):
    node, _ = visualizer
    node._vehicle_local_position_callback(local(stamp=100.))
    node._vehicle_local_position_callback(local((1., 5., 5.24), stamp=100. + lead_us * 1e-6,
                                               timestamp_sample=100_000_000 + lead_us))
    node._timer_callback()
    path = node.vehicle_path_pub.messages[-1]
    assert len(path.poses) == (2 if accepted else 1)
    assert path.header.stamp.nanosec == (lead_us * 1000 if accepted else 0)
    vehicle = next(marker for marker in node.marker_pub.messages[-1].markers if marker.ns == 'vehicle')
    assert vehicle.pose.position.x == pytest.approx(1. if accepted else 0., abs=2e-6)


def test_waits_for_initialized_simulation_clock(visualizer):
    node, clock = visualizer
    clock.seconds = 0.
    node._vehicle_local_position_callback(local(stamp=.01))
    node._timer_callback()
    assert node.vehicle_path_pub.messages[-1].poses == []

    clock.seconds = .01
    node._vehicle_local_position_callback(local(stamp=.01))
    node._timer_callback()
    assert len(node.vehicle_path_pub.messages[-1].poses) == 1


@pytest.mark.parametrize('visualizer', [{'publish_planned_path': True}], indirect=True)
def test_planned_path_uses_the_same_map_reference(visualizer):
    node, _ = visualizer
    node._timer_callback()
    assert node.planned_path_pub.messages[-1].poses == []
    node._vehicle_local_position_callback(local())
    node._timer_callback()
    path = node.planned_path_pub.messages[-1]
    assert path.poses
    for pose in path.poses:
        assert coordinates(pose) == pytest.approx((0., 5., 5.24), abs=2e-5)


def test_qos_receives_volatile_px4_and_retains_latest_snapshot(visualizer):
    node, _ = visualizer
    subscription = next(sub for sub in node.subscriptions if sub.topic_name.endswith('vehicle_local_position_v1'))
    assert subscription.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
    assert subscription.qos_profile.durability == DurabilityPolicy.VOLATILE
    compatibility, reason = qos_check_compatible(qos_profile_sensor_data, subscription.qos_profile)
    assert compatibility != QoSCompatibility.ERROR, reason
    for publisher in node.publishers:
        if publisher.topic_name in ('/vehicle_path', '/trajectory_markers'):
            assert publisher.qos_profile.reliability == ReliabilityPolicy.RELIABLE
            assert publisher.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
            assert publisher.qos_profile.depth == 1
