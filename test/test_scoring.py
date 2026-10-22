import json
from pathlib import Path

import pytest

from ail_tb4_maze.scoring import Scoring

C = json.loads(
    (Path(__file__).resolve().parents[1] / "config/challenge.json").read_text()
)
C["time_limit"] = 300.0  # Explicit deadline for the historical bounded-trial cases.


def trial():
    s = Scoring(C)
    s.start(20, (9, -2, 0))
    return s


def test_complete_crossing_only():
    s = trial()
    s.update(21, 10.1, -2)
    assert s.status == "running"
    s.update(22, 10.3, -2)
    assert s.status == "success"
    assert abs(s.distance - 1.3) < 1e-9


def test_outside_aperture_fails():
    s = trial()
    s.update(21, 10.5, 0)
    assert s.status == "failure"


def test_timeout_sim_time():
    s = trial()
    s.tick(319.99)
    assert s.status == "running"
    s.tick(320)
    assert s.status == "timeout"
    assert s.elapsed == 300


def test_unlimited_trial_can_cross_exit_after_more_than_five_minutes():
    s = Scoring(dict(C, time_limit=0.0))
    s.start(0, (9, -2, 0))
    s.tick(3600)
    assert s.status == "running"
    assert s.elapsed == 3600
    s.contact(3600, "wall")
    assert s.collisions == 1
    s.update(3601, 10.1, -2)
    assert s.status == "running"
    s.update(3602, 10.3, -2)
    assert s.status == "success"
    assert s.result()["time_limit_sim_seconds"] == 0.0


def test_custom_deadline_keeps_running_past_the_previous_default():
    s = Scoring(dict(C, time_limit=600.0))
    s.start(0, (9, -2, 0))
    s.tick(300)
    assert s.status == "running"
    s.tick(600)
    assert s.status == "timeout"


@pytest.mark.parametrize("limit", [-1.0, float("nan"), float("inf")])
def test_invalid_time_limit_is_rejected(limit):
    with pytest.raises(ValueError, match="time_limit"):
        Scoring(dict(C, time_limit=limit))


def test_unlimited_trial_still_detects_clock_reversal():
    s = Scoring(dict(C, time_limit=0.0))
    s.start(20, (9, -2, 0))
    s.tick(19)
    assert s.reason == "simulation_time_reversed"


def test_referee_captures_selected_limit_at_start_and_after_reset(tmp_path):
    import rclpy
    from rclpy.parameter import Parameter
    from std_srvs.srv import Trigger

    from ail_tb4_maze.referee import Referee

    config = Path(__file__).resolve().parents[1] / "config/challenge.json"
    rclpy.init(
        domain_id=119,
        args=[
            "--ros-args",
            "-p",
            f"config:={config}",
            "-p",
            f"results_dir:={tmp_path}",
        ],
    )
    node = Referee()
    try:
        node.pose = (9, -2, 0)
        assert node.start(Trigger.Request(), Trigger.Response()).success
        node.score.tick(node.score.started + 601)
        assert node.score.status == "running"
        assert node.set_parameters([Parameter("time_limit", value=30.0)])[0].successful
        assert (
            node.score.c["time_limit"] == 0.0
        )  # Active attempt keeps its captured limit.
        assert not node.set_parameters([Parameter("time_limit", value=-1.0)])[
            0
        ].successful
        node.reset(Trigger.Request(), Trigger.Response())
        node.pose = (9, -2, 0)
        assert node.start(Trigger.Request(), Trigger.Response()).success
        node.score.tick(node.score.started + 30)
        assert node.score.status == "timeout"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_time_reversal_fails():
    s = trial()
    s.tick(19)
    assert s.reason == "simulation_time_reversed"


def test_contact_episodes_and_multiple_walls():
    s = trial()
    s.contact(20, "wall1")
    s.contact(20.1, "wall1")
    assert s.collisions == 1
    s.contact(20.5, "wall1")
    s.contact(20.5, "wall2")
    assert s.collisions == 3


def test_terminal_result_immutable():
    s = trial()
    s.tick(320)
    before = s.result()
    s.update(321, 10.5, -2)
    s.contact(321, "wall")
    assert s.result() == before


def test_spawn_outside_does_not_win():
    s = Scoring(C)
    s.start(0, (11, -2, 0))
    s.update(1, 11, -2)
    assert s.status == "failure"


def test_return_inside_cancels_crossing():
    s = trial()
    s.update(21, 10.1, -2)
    s.update(22, 9.9, -2)
    assert not s.crossed


def test_fallen_robot_fails():
    s = trial()
    s.update(21, 9, -2, 1)
    assert s.reason == "robot_fell_or_lifted"
