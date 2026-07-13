import math

from sensor_msgs.msg import LaserScan

from jetbot_governor.predictive_governor import min_front_arc_distance


def make_scan(
    ranges, angle_min=-math.pi, angle_max=math.pi, range_min=0.05, range_max=10.0
):
    scan = LaserScan()
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = (angle_max - angle_min) / max(len(ranges) - 1, 1)
    scan.range_min = range_min
    scan.range_max = range_max
    scan.ranges = ranges
    return scan


def test_wall_test_obstacle_directly_ahead_is_detected():
    # 5-point scan spanning -90deg..+90deg; middle point (index 2) is dead ahead.
    scan = make_scan(
        [2.0, 2.0, 0.3, 2.0, 2.0], angle_min=-math.pi / 2, angle_max=math.pi / 2
    )
    dist = min_front_arc_distance(scan, half_angle_rad=math.radians(60))
    assert math.isclose(dist, 0.3, abs_tol=1e-6)


def test_obstacle_behind_robot_is_ignored():
    # Obstacle at +170 degrees (behind the robot) should not affect the
    # front-arc check - this is the bug the original min(all ranges) had.
    scan = make_scan(
        [2.0, 2.0, 2.0, 2.0, 0.1], angle_min=-math.pi, angle_max=math.pi
    )
    dist = min_front_arc_distance(scan, half_angle_rad=math.radians(60))
    assert dist > 0.4  # nothing in the front arc that close


def test_empty_or_all_invalid_scan_does_not_crash():
    scan = make_scan([0.0, 0.0, 0.0, 0.0, 0.0])
    dist = min_front_arc_distance(scan, half_angle_rad=math.radians(60))
    assert dist == math.inf


def test_inf_reading_treated_as_clear_at_range_max():
    scan = make_scan(
        [math.inf, math.inf, math.inf],
        angle_min=-math.radians(10),
        angle_max=math.radians(10),
        range_max=8.0,
    )
    dist = min_front_arc_distance(scan, half_angle_rad=math.radians(60))
    assert dist == 8.0


def test_nan_reading_is_skipped_not_crashing():
    scan = make_scan([math.nan, math.nan, math.nan])
    dist = min_front_arc_distance(scan, half_angle_rad=math.radians(60))
    assert dist == math.inf
