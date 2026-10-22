import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "navigation_interface"))
from wall_follower import WallFollower


def scene(distance=0.45, front=3.0):
    wall = np.column_stack((np.linspace(-1, 2, 300), np.full(300, -distance)))
    end = np.column_stack((np.full(300, front), np.linspace(-distance, 2, 300)))
    return np.concatenate((wall, end))


def test_parallel_wall_is_followed_straight():
    d = WallFollower().step(scene())
    assert d.forward > 0.2 and abs(d.angular) < 0.08


def test_close_wall_steers_away_and_far_wall_steers_towards():
    assert WallFollower().step(scene(0.32)).angular > 0
    assert WallFollower().step(scene(0.8)).angular < 0


def test_blocked_front_turns_left_in_place():
    d = WallFollower().step(scene(front=0.5))
    assert d.state == "turn_left" and d.forward == 0 and d.angular > 0


def test_followed_side_opening_turns_right():
    f = WallFollower()
    f.step(scene())
    d = f.step(scene(distance=2.0))
    assert d.state == "turn_right" and d.angular < 0


def test_imminent_contact_and_invalid_sensor_stop():
    assert WallFollower().step(scene(0.23)).forward == 0
    assert WallFollower().step(np.zeros((0, 2))).state == "sensor_stop"


def test_left_hand_rule_is_mirrored():
    right = scene(0.35)
    left = right.copy()
    left[:, 1] *= -1
    a = WallFollower().step(right)
    b = WallFollower(side="left").step(left)
    assert abs(a.forward - b.forward) < 1e-8 and abs(a.angular + b.angular) < 1e-8


def test_starting_away_from_wall_acquires_instead_of_circling():
    f = WallFollower()
    pose = np.array([0.0, 0.0, 0.0])
    states = set()
    world = np.column_stack((np.linspace(-20, 20, 2000), np.full(2000, -1.5)))
    for _ in range(500):
        delta = world - pose[:2]
        co, si = np.cos(pose[2]), np.sin(pose[2])
        points = np.column_stack(
            (co * delta[:, 0] + si * delta[:, 1], -si * delta[:, 0] + co * delta[:, 1])
        )
        d = f.step(points, pose[2], pose[:2])
        states.add(d.state)
        pose[0] += 0.1 * d.forward * np.cos(pose[2])
        pose[1] += 0.1 * d.forward * np.sin(pose[2])
        pose[2] += 0.1 * d.angular
        assert pose[1] > -1.25
    assert "approach_wall" in states and "follow_wall" in states
    assert pose[0] > 4 and abs(pose[1] + 1.05) < 0.15


def test_right_opening_has_priority_over_a_distant_front_wall():
    f = WallFollower()
    f.step(scene())
    d = f.step(scene(distance=2.0, front=0.5))
    assert d.state == "turn_right" and d.angular < 0


def test_rounds_a_wall_end_instead_of_looping_in_start_room():
    # Boundary-connected obstacle with a 1 m passage above it, matching a
    # common maze junction. Scan rays select only the first visible surface.
    rectangles = [
        (-10.1, -10.0, -10.0, 10.0),
        (-10.0, 10.0, -10.1, -10.0),
        (-7.0, -6.0, -10.0, -8.0),
        (-10.0, -7.0, -7.0, -6.0),
    ]
    segments = []
    for x0, x1, y0, y1 in rectangles:
        segments.extend(
            [
                ((x0, y0), (x1, y0)),
                ((x1, y0), (x1, y1)),
                ((x1, y1), (x0, y1)),
                ((x0, y1), (x0, y0)),
            ]
        )
    starts = np.array([a for a, b in segments])
    edges = np.array([b for a, b in segments]) - starts
    angles = np.linspace(-np.pi, np.pi, 640, endpoint=False)
    f = WallFollower()
    pose = np.array([-8.5, -8.5, 0.0])
    escaped = False
    for _ in range(1200):
        directions = np.column_stack(
            (np.cos(angles + pose[2]), np.sin(angles + pose[2]))
        )
        difference = starts - pose[:2]
        cross = (
            directions[:, 0, None] * edges[None, :, 1]
            - directions[:, 1, None] * edges[None, :, 0]
        )
        cross = np.where(abs(cross) < 1e-10, np.nan, cross)
        distance = (difference[:, 0] * edges[:, 1] - difference[:, 1] * edges[:, 0])[
            None, :
        ] / cross
        u = (
            difference[None, :, 0] * directions[:, 1, None]
            - difference[None, :, 1] * directions[:, 0, None]
        ) / cross
        distance = np.where((distance > 0) & (u >= 0) & (u <= 1), distance, np.inf)
        ranges = np.min(distance, axis=1)
        valid = np.isfinite(ranges) & (ranges < 12)
        points = np.column_stack(
            (
                np.cos(angles[valid]) * ranges[valid],
                np.sin(angles[valid]) * ranges[valid],
            )
        )
        d = f.step(points, pose[2], pose[:2])
        pose[0] += 0.1 * d.forward * np.cos(pose[2])
        pose[1] += 0.1 * d.forward * np.sin(pose[2])
        pose[2] += 0.1 * d.angular
        for x0, x1, y0, y1 in rectangles:
            assert (
                np.hypot(
                    max(x0 - pose[0], 0, pose[0] - x1),
                    max(y0 - pose[1], 0, pose[1] - y1),
                )
                > 0.23
            )
        if pose[0] > -5.8:
            escaped = True
            break
    assert escaped, pose


def test_recovery_reverse_requires_clear_space_behind():
    assert WallFollower.safe_reverse(scene()) < 0
    points = np.concatenate((scene(), np.tile([-0.30, 0], (20, 1))))
    assert WallFollower.safe_reverse(points) == 0
