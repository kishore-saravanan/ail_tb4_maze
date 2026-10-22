import math

import numpy as np

from ail_tb4_maze.dead_end import closed_left_opening
from ail_tb4_maze.pledge import Pledge


def scan(front=3.0, side=0.38):

    return np.concatenate(
        (
            np.column_stack((np.linspace(-2, 3, 200), np.full(200, side))),
            np.column_stack((np.linspace(-2, 3, 200), np.full(200, -side))),
            np.column_stack((np.full(200, front), np.linspace(-side, side, 200))),
        )
    )


def test_preferred_direction_speed_cap():
    p = Pledge()
    v, w = p.step(scan(), (0, 0, 0))
    assert v == 0.30 and w == 0 and not p.following


def test_obstacle_starts_left_hand_following():
    p = Pledge()
    v, w = p.step(scan(front=0.35), (0, 0, 0))
    assert p.following and v == 0 and w < 0


def test_full_revolution_is_not_zero_turns():
    p = Pledge()
    p.step(scan(front=0.35), (0, 0, 0))
    for angle in np.linspace(0, -2 * math.pi, 100):
        p.observe_heading(math.atan2(math.sin(angle), math.cos(angle)))
    p.step(scan(), (0, 0, 0))
    assert p.following and abs(p.net_turn + 2 * math.pi) < 1e-8


def test_returns_to_preferred_only_after_signed_turns_cancel():
    p = Pledge()
    p.step(scan(front=0.35), (0, 0, 0))
    p.step(scan(), (0, 0, -math.pi / 2))
    p.step(scan(), (0.5, 0, -math.pi / 2))
    for angle in [-1.0, -0.5, 0]:
        p.observe_heading(angle)
    p.step(scan(), (0.5, 0, 0))
    assert not p.following


def room():
    # Horizontal corridor and one short, fully observed vertical dead end.
    cells = np.full((100, 100), -1, dtype=np.int8)
    cells[30:51, 10:90] = 100
    cells[32:49, 12:88] = 0
    cells[45:81, 40:61] = 100
    cells[45:78, 42:59] = 0
    # Keep the doorway open into the main corridor.
    cells[40:51, 42:59] = 0

    return cells


def test_fully_observed_dead_end_has_a_proof():
    cells = room()
    assert closed_left_opening(cells, 0.05, (0, 0), (2.5, 2.0, 0)) is not None


def test_unseen_space_invalidates_proof():
    cells = room()
    cells[72:81, 43:58] = -1
    assert closed_left_opening(cells, 0.05, (0, 0), (2.5, 2.0, 0)) is None


def test_second_entrance_invalidates_proof():
    cells = room()
    cells[30:81, 68:90] = 100
    cells[59:81, 40:90] = 100
    cells[32:49, 12:88] = 0
    cells[40:78, 42:59] = 0
    cells[40:78, 70:88] = 0
    cells[61:78, 42:88] = 0
    assert closed_left_opening(cells, 0.05, (0, 0), (2.5, 2.0, 0)) is None


def test_branch_extending_beyond_doorway_is_not_pruned():
    cells = room()
    cells[59:81, 55:88] = 100
    cells[61:78, 55:85] = 0
    assert closed_left_opening(cells, 0.05, (0, 0), (2.5, 2.0, 0)) is None


def test_new_unknown_cell_invalidates_previous_positive_result():
    cells = room()
    assert closed_left_opening(cells, 0.05, (0, 0), (2.5, 2.0, 0)) is not None
    cells[60, 50] = -1
    assert closed_left_opening(cells, 0.05, (0, 0), (2.5, 2.0, 0)) is None


def test_turn_is_committed_despite_side_opening_mid_rotation():
    p = Pledge()
    p.step(scan(front=0.35), (0, 0, 0))
    assert p.turn_target is not None
    # An apparent left opening must not reverse the ongoing right turn.
    v, w = p.step(scan(front=3.0, side=2.0), (0, 0, -0.4))
    assert v == 0 and w < 0 and p.state == "committed_turn_right"
    v, w = p.step(scan(), (0, 0, -math.pi / 2))
    assert v > 0 and p.state == "clear_junction"


def test_wrapped_sensor_heading_keeps_signed_rotation():
    p = Pledge()
    p.observe_heading(0.0)
    p.following = True
    for angle in np.linspace(0, 4 * math.pi, 200):
        p.observe_heading(math.atan2(math.sin(angle), math.cos(angle)))
    assert abs(p.net_turn - 4 * math.pi) < 1e-8


def following_controller():
    p = Pledge()
    p.observe_heading(0.0)
    p.preferred = p.turn_origin = math.pi
    p.following = True

    return p


def test_open_side_starts_corner_but_virtual_proven_wall_skips_it():
    a = following_controller()
    b = following_controller()
    a.step(scan(), (0, 0, 0))
    b.step(scan(), (0, 0, 0))
    opening = scan(side=2.0)
    assert a.step(opening, (0.1, 0, 0))[0] > 0 and a.advance_origin is not None
    virtual = np.column_stack((np.linspace(-0.4, 0.6, 100), np.full(100, 0.48)))
    b.step(opening, (0.1, 0, 0), np.concatenate((opening, virtual)))
    assert b.advance_origin is None and b.state == "follow_left_wall"
    assert a.net_turn == b.net_turn


def test_recovery_does_not_clear_turn_history():
    from ail_tb4_maze.pledge import PledgeMotion

    p = following_controller()
    motion = PledgeMotion(p)
    motion.anchor = (0, 0)
    motion.progress_time = 0.0
    motion.step(scan(), (0, 0, 0), 9.0)
    assert motion.recoveries == 1 and abs(p.net_turn + math.pi) < 1e-8


def test_dead_end_proof_is_translation_invariant():
    cells = room()
    assert closed_left_opening(cells, 0.05, (-20.0, 13.0), (-17.5, 15.0, 0)) is not None


def test_late_dead_end_proof_cancels_pending_corner():
    p = following_controller()
    p.step(scan(), (0, 0, 0))
    opening = scan(side=2.0)
    p.step(opening, (0.1, 0, 0))
    assert p.advance_origin is not None
    virtual = np.column_stack((np.linspace(-0.4, 2.0, 100), np.full(100, 0.48)))
    p.step(opening, (0.3, 0, 0), np.concatenate((opening, virtual)))
    assert p.advance_origin is None and p.state == "pass_proven_dead_end"


def test_proven_portal_and_unknown_portal_produce_different_choices():
    p = following_controller()
    p.step(scan(), (0, 0, 0))
    proof = closed_left_opening(room(), 0.05, (0, 0), (2.5, 2.0, 0))
    assert proof is not None
    opening = scan(side=2.0)
    p.step(opening, (0.1, 0, 0), np.concatenate((opening, proof)))
    assert p.advance_origin is None
    unknown = room()
    unknown[60, 50] = -1
    assert closed_left_opening(unknown, 0.05, (0, 0), (2.5, 2.0, 0)) is None
    p.step(opening, (0.2, 0, 0))
    assert p.advance_origin is not None


def test_virtual_wall_never_masks_real_collision_guard():
    from ail_tb4_maze.pledge import PledgeMotion

    p = following_controller()
    motion = PledgeMotion(p)
    real = scan(front=0.15)
    virtual = np.column_stack((np.linspace(-0.4, 2.0, 100), np.full(100, 0.48)))
    assert motion.step(real, (0, 0, 0), 0.0, np.concatenate((real, virtual))) == (
        0.0,
        0.0,
    )


def test_thin_lidar_wall_surface_with_unknown_behind_it_is_closed():
    cells = room()
    # Keep only the surfaces bounding free space, as a LiDAR occupancy map
    # does; the unobserved volume inside each solid wall remains unknown.
    from scipy import ndimage

    surface = (cells >= 50) & ndimage.binary_dilation(cells == 0)
    cells[(cells >= 50) & ~surface] = -1
    assert closed_left_opening(cells, 0.05, (0, 0), (2.5, 2.0, 0)) is not None


def test_unseen_gap_too_narrow_for_robot_does_not_create_a_fake_route():
    cells = room()
    cells[77:81, 48:52] = -1
    assert closed_left_opening(cells, 0.05, (0, 0), (2.5, 2.0, 0)) is not None


def test_closed_corridor_is_one_record_across_approaches():
    from ail_tb4_maze.dead_end import ClosedCorridors

    memory = ClosedCorridors()
    cells = room()
    assert memory.update(cells, 0.05, (0, 0), (2.5, 2.0, 0)) is not None
    first = memory.records[0]["id"]
    memory.update(cells, 0.05, (0, 0), (2.6, 2.0, 0))
    assert len(memory.records) == 1 and memory.records[0]["id"] == first
    # It remains one known edge while the robot is elsewhere.
    assert memory.update(cells, 0.05, (0, 0), (0.8, 2.0, 0)) is None
    assert len(memory.records) == 1
    assert memory.update(cells, 0.05, (0, 0), (2.5, 2.0, 0)) is not None
    assert memory.selected_id == first


def test_new_reachable_unknown_invalidates_remembered_corridor():
    from ail_tb4_maze.dead_end import ClosedCorridors

    memory = ClosedCorridors()
    cells = room()
    memory.update(cells, 0.05, (0, 0), (2.5, 2.0, 0))
    cells[72:81, 43:58] = -1
    assert memory.update(cells, 0.05, (0, 0), (2.5, 2.0, 0)) is None
    assert not memory.records and memory.invalidated == 1


def test_remembered_left_portal_does_not_block_opposite_approach():
    from ail_tb4_maze.dead_end import ClosedCorridors

    memory = ClosedCorridors()
    cells = room()
    memory.update(cells, 0.05, (0, 0), (2.5, 2.0, 0))
    assert memory.update(cells, 0.05, (0, 0), (2.5, 2.0, math.pi)) is None
    assert len(memory.records) == 1


def test_unknown_robot_side_of_doorway_cannot_prove_a_closed_corridor():
    cells = room()
    cells[40, 50] = -1
    assert closed_left_opening(cells, 0.05, (0, 0), (2.5, 2.0, 0)) is None


def test_startup_uses_first_visible_opening_without_circling_start_room():
    p = Pledge()
    p.step(scan(side=0.88), (0, 0, 0))
    v, w = p.step(scan(side=2.0), (0.1, 0, 0))
    assert p.state == "select_initial_corridor" and p.entry_phase and not p.following
    assert v > 0 and w == 0
    v, w = p.step(scan(side=2.0), (0.5, 0, 0))
    assert v == 0 and w > 0 and p.state == "committed_turn_left"
    p.step(scan(), (0.5, 0, math.pi / 2))
    v, w = p.step(scan(), (0.5, 0.5, math.pi / 2))
    assert not p.entry_phase and not p.following
    assert (
        abs(p.preferred - math.pi / 2) < 1e-8 and abs(p.unwrapped - math.pi / 2) < 1e-8
    )
    assert v == 0.3 and abs(w) < 1e-8


def test_far_parallel_wall_is_not_a_startup_opening():
    p = Pledge()
    for x in [0, 0.1, 0.2]:
        p.step(scan(side=0.88), (x, 0, 0))
    assert not p.entry_phase and p.state == "preferred_direction"


def test_closed_virtual_portal_prevents_initial_corridor_selection():
    p = Pledge()
    p.step(scan(side=0.88), (0, 0, 0))
    opening = scan(side=2.0)
    virtual = np.column_stack((np.linspace(-0.4, 2.0, 100), np.full(100, 0.4)))
    p.step(opening, (0.1, 0, 0), np.concatenate((opening, virtual)))
    assert not p.entry_phase and not p.following


def test_wall_mode_revolution_is_preserved_after_initial_heading_selection():
    p = Pledge()
    p.step(scan(side=0.88), (0, 0, 0))
    p.step(scan(side=2.0), (0.1, 0, 0))
    p.step(scan(side=2.0), (0.5, 0, 0))
    p.step(scan(), (0.5, 0, math.pi / 2))
    p.step(scan(), (0.5, 0.5, math.pi / 2))
    p.following = True
    for a in np.linspace(math.pi / 2, math.pi / 2 + 2 * math.pi, 100):
        p.observe_heading(wrap_for_test(a))
    assert abs(p.net_turn - 2 * math.pi) < 1e-8


def wrap_for_test(a):

    return math.atan2(math.sin(a), math.cos(a))
