import math

import numpy as np

from ail_tb4_maze.path_follower import PathFollower


def wall(a, b):

    return np.linspace(a, b, max(2, math.ceil(math.dist(a, b) / 0.015)))


def test_turn_then_drive_around_a_narrow_right_angle_without_contact():
    obstacles = np.vstack(
        [
            wall((-1.0, -0.4), (1.4, -0.4)),
            wall((-1.0, 0.4), (0.6, 0.4)),
            wall((0.6, 0.4), (0.6, 2.0)),
            wall((1.4, -0.4), (1.4, 2.0)),
            wall((0.6, 2.0), (1.4, 2.0)),
        ]
    )
    pose = np.array([-0.5, 0.0, 0.0])
    f = PathFollower()
    f.set_route([(-0.5, 0.0), (0.95, 0.0), (1.0, 1.5)], pose)
    for _ in range(650):
        a = pose[2]
        delta = obstacles - pose[:2]
        points = delta @ np.array(
            [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
        )
        v, w, _ = f.step(pose, points)
        dt = 0.05
        if abs(w) < 1e-6:
            pose[:2] += v * dt * np.array([math.cos(a), math.sin(a)])
        else:
            pose[:2] += (
                v
                / w
                * np.array(
                    [
                        math.sin(a + w * dt) - math.sin(a),
                        -math.cos(a + w * dt) + math.cos(a),
                    ]
                )
            )
        pose[2] += w * dt
        assert np.linalg.norm(obstacles - pose[:2], axis=1).min() > 0.205
        if f.done:
            break
    assert f.done and np.linalg.norm(pose[:2] - (1.0, 1.5)) < 0.08


def test_recovery_can_back_away_from_a_soft_safety_margin():
    points = wall((0.22, -0.5), (0.22, 0.5))
    assert PathFollower.safe(points, -0.1, 0.0)
    assert not PathFollower.safe(points, 0.1, 0.0)
    assert PathFollower.safe(points, 0.0, 0.8)


def test_missing_lidar_blocks_motion():
    f = PathFollower()
    f.set_route([(1.0, 0.0)], (0.0, 0.0, 0.0))
    v, w, state = f.step((0.0, 0.0, 0.0), np.empty((0, 2)))
    assert (v, w, state) == (0.0, 0.0, "blocked")
