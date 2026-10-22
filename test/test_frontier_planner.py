import numpy as np
import pytest

from ail_tb4_maze.frontier_planner import choose_frontier


def test_frontier_goal_expires_when_lidar_finishes_observing_the_corridor():
    from ail_tb4_maze.frontier_planner import frontier_goal_active

    grid = corridor()
    goal = (6.85, 2.25)
    assert frontier_goal_active(grid, 0.1, (0, 0), goal)
    grid[20:26, 70:80] = 0
    grid[19:27, 80] = 100
    grid[19, 70:81] = 100
    grid[26, 70:81] = 100
    assert not frontier_goal_active(grid, 0.1, (0, 0), goal)


def test_unknown_behind_a_wall_does_not_keep_a_closed_goal_active():
    from ail_tb4_maze.frontier_planner import frontier_goal_active

    grid = np.full((30, 30), 100, dtype=np.int8)
    grid[10:20, 5:15] = 0
    grid[10:20, 16:20] = -1
    assert not frontier_goal_active(grid, 0.1, (0, 0), (1.45, 1.55))


def test_current_map_edge_remains_an_exploration_goal():
    from ail_tb4_maze.frontier_planner import frontier_goal_active

    grid = np.zeros((20, 30), dtype=np.int8)
    assert frontier_goal_active(grid, 0.1, (0, 0), (2.85, 1.05))
    assert not frontier_goal_active(grid, 0.1, (0, 0), (1.05, 1.05))
    assert not frontier_goal_active(grid, 0.1, (0, 0), None)


def test_direct_controller_redirects_before_reaching_a_visible_dead_end():
    import math

    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from scipy import ndimage

    from ail_tb4_maze.direct_navigation import DirectExplorer

    # The robot is still several metres from its old eastward goal. The map
    # now closes that whole corridor; a north branch remains unexplored.
    grid = np.full((80, 100), 100, dtype=np.int8)
    grid[30:50, 10:90] = 0
    grid[5:31, 30:50] = 0
    grid[:5, 30:50] = -1
    msg = OccupancyGrid()
    msg.info.width = 100
    msg.info.height = 80
    msg.info.resolution = 0.1
    msg.data = grid.ravel().tolist()
    rclpy.init(domain_id=119)
    node = DirectExplorer()
    try:
        node.map = msg
        node.pose = (4.05, 4.05)
        node.heading = 0.0
        node.target = (8.45, 4.05)
        node.map_clearance = ndimage.distance_transform_edt(grid < 50) * 0.1
        node.refresh_planning_map = lambda: None
        node.update_pose = lambda: True
        node.follower.set_route([node.pose, node.target], (*node.pose, node.heading))
        node.tick()
        assert node.observed_goal_replans == 1
        assert node.replan_reason == "frontier_observed"
        assert node.target[1] < 1.0
        assert math.dist(node.pose, (8.45, 4.05)) > 4.0
        assert all(y < 4.2 for x, y in node.follower.route)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def corridor():
    grid = np.full((60, 90), -1, dtype=np.int8)
    grid[20:26, 10:70] = 0
    grid[19, 9:70] = 100
    grid[26, 9:70] = 100
    grid[19:27, 9] = 100
    return grid


def test_narrow_corridor_frontier_is_reachable():
    target, frontiers = choose_frontier(corridor(), 0.1, (0, 0), (2.0, 2.25))
    assert target is not None and target[0] > 6
    assert frontiers


def test_wall_blocks_goal_in_an_unreachable_chamber():
    grid = corridor()
    grid[19:27, 40] = 100
    target, _ = choose_frontier(grid, 0.1, (0, 0), (2.0, 2.25))
    assert target is None


def test_blacklist_prevents_repeated_failed_goal():
    grid = corridor()
    target, _ = choose_frontier(grid, 0.1, (0, 0), (2.0, 2.25))
    assert target is not None
    replacement, _ = choose_frontier(grid, 0.1, (0, 0), (2.0, 2.25), [target])
    assert replacement is None


def test_unobserved_map_is_never_used_as_a_shortcut():
    grid = np.full((40, 40), -1, dtype=np.int8)
    target, _ = choose_frontier(grid, 0.1, (0, 0), (1.0, 1.0))
    assert target is None


def test_frontier_node_constructs_and_cleans_up():
    import rclpy

    from ail_tb4_maze.frontier_node import Explorer

    rclpy.init(domain_id=119)
    node = Explorer()
    assert node.goal_handle is None
    node.destroy_node()
    rclpy.shutdown()


def test_velocity_adapter_stops_when_commands_go_stale():
    import time
    from types import SimpleNamespace

    import rclpy

    from ail_tb4_maze.velocity_adapter import Adapter

    rclpy.init(domain_id=119)
    node = Adapter()
    pub = node.pub
    messages = []
    try:
        node.pub = SimpleNamespace(publish=messages.append)
        node.cmd.linear.x = 0.2
        node.last = time.monotonic()
        node.tick()
        node.last = time.monotonic() - 0.4
        node.tick()
        assert messages[0].twist.linear.x == 0.2
        assert messages[1].twist.linear.x == 0.0 and messages[1].twist.angular.z == 0.0
    finally:
        node.pub = pub
        node.destroy_node()
        rclpy.shutdown()


def test_late_cancelled_result_cannot_clear_a_new_goal():
    from concurrent.futures import Future
    from types import SimpleNamespace

    import rclpy

    from ail_tb4_maze.frontier_node import Explorer

    rclpy.init(domain_id=119)
    node = Explorer()
    try:
        active = object()
        node.sequence = 2
        node.goal_handle = active
        node.pending = True
        result = Future()
        result.set_result(SimpleNamespace(status=5))
        node.finished(result, 1, (1.0, 1.0))
        assert node.goal_handle is active and node.pending
        assert not node.blacklist
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_late_goal_acceptance_is_cancelled():
    from concurrent.futures import Future
    from types import SimpleNamespace

    import rclpy

    from ail_tb4_maze.frontier_node import Explorer

    rclpy.init(domain_id=119)
    node = Explorer()
    try:
        cancelled = []
        node.sequence = 2
        handle = SimpleNamespace(
            accepted=True, cancel_goal_async=lambda: cancelled.append(True)
        )
        result = Future()
        result.set_result(handle)
        node.accepted(result, 1, (1.0, 1.0))
        assert cancelled and node.goal_handle is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("current_x,extend", [(2.3, True), (1.7, False)])
def test_goal_handoff_only_extends_forward(current_x, extend):
    from concurrent.futures import Future
    from types import SimpleNamespace

    import rclpy
    from nav_msgs.msg import OccupancyGrid

    from ail_tb4_maze.frontier_node import Explorer

    rclpy.init(domain_id=119)
    node = Explorer()
    client = node.client
    tf = node.tf
    try:
        grid = corridor()
        m = OccupancyGrid()
        m.info.width = grid.shape[1]
        m.info.height = grid.shape[0]
        m.info.resolution = 0.1
        m.data = grid.ravel().tolist()
        node.map = m
        node.tf = SimpleNamespace(
            lookup_transform=lambda *args: SimpleNamespace(
                transform=SimpleNamespace(
                    translation=SimpleNamespace(x=2.0, y=2.25),
                    rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            )
        )
        sent = []

        def send(goal):
            sent.append(goal)
            return Future()

        node.client = SimpleNamespace(
            server_is_ready=lambda: True, send_goal_async=send
        )
        node.goal_handle = object()
        node.target = (current_x, 2.25)
        node.goal_time = node.get_clock().now().nanoseconds / 1e9
        node.tick()
        assert bool(sent) == extend
        if extend:
            assert sent[0].pose.pose.position.x > 6.0 and node.pending
        else:
            assert not node.pending
    finally:
        node.client = client
        node.tf = tf
        node.goal_handle = None
        node.destroy_node()
        rclpy.shutdown()


def test_failed_stack_startup_does_not_wait_forever():
    import rclpy

    from ail_tb4_maze.frontier_node import Explorer

    rclpy.init(domain_id=119)
    node = Explorer()
    try:
        node.started -= 61.0
        with pytest.raises(RuntimeError, match="did not become ready"):
            node.tick()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def room_with_map_hole():
    grid = np.full((50, 90), -1, dtype=np.int8)
    grid[10:40, 10:60] = 0
    grid[9, 9:61] = 100
    grid[40, 9:61] = 100
    grid[9:41, 9] = 100
    grid[24:26, 32:34] = -1
    return grid


def test_large_unknown_region_is_preferred_to_a_small_map_hole():
    target, _ = choose_frontier(room_with_map_hole(), 0.1, (0, 0), (2.0, 2.5))
    assert target is not None and target[0] > 5.5


def test_small_unknown_region_remains_a_fallback_if_it_is_the_only_option():
    grid = room_with_map_hole()
    grid[9:41, 60] = 100
    target, _ = choose_frontier(grid, 0.1, (0, 0), (2.0, 2.5))
    assert target is not None and 2.8 < target[0] < 3.6


def test_visited_frontier_loses_priority_without_being_forbidden():
    grid = np.full((50, 80), -1, dtype=np.int8)
    grid[10:40, 10:70] = 0
    grid[9, 9:71] = 100
    grid[40, 9:71] = 100
    target, _ = choose_frontier(grid, 0.1, (0, 0), (4.0, 2.5))
    assert target is not None
    x = target[0]
    visited = [(x, y) for y in np.arange(1.2, 3.9, 0.15)]
    replacement, _ = choose_frontier(grid, 0.1, (0, 0), (4.0, 2.5), visited=visited)
    assert replacement is not None and abs(replacement[0] - x) > 4.0
    fallback, _ = choose_frontier(
        corridor(), 0.1, (0, 0), (2.0, 2.25), visited=[(6.95, 2.25)]
    )
    assert fallback is not None


def test_free_map_edge_is_a_frontier_to_unobserved_space_beyond_map_bounds():
    target, _ = choose_frontier(np.zeros((30, 30), np.int8), 0.1, (0, 0), (1.5, 1.5))
    assert target is not None
    assert min(target[0], target[1], 3.0 - target[0], 3.0 - target[1]) < 0.11


def test_map_updates_can_invalidate_a_previously_safe_free_goal():
    from ail_tb4_maze.frontier_planner import goal_is_safe

    grid = np.zeros((30, 30), np.int8)
    grid[[0, -1], :] = 100
    grid[:, [0, -1]] = 100
    assert goal_is_safe(grid, 0.1, (0, 0), (1.05, 1.05))
    grid[10, 12] = 100
    assert not goal_is_safe(grid, 0.1, (0, 0), (1.05, 1.05))
    grid[10, 12] = 0
    grid[10, 10] = -1
    assert not goal_is_safe(grid, 0.1, (0, 0), (1.05, 1.05))


def test_close_frontier_is_not_discarded_when_no_distant_frontier_exists():
    target, _ = choose_frontier(corridor(), 0.1, (0, 0), (6.6, 2.25))
    assert (
        target is not None
        and 0.15 < np.linalg.norm(np.array(target) - (6.6, 2.25)) < 0.6
    )


def test_optional_route_stays_in_observed_free_cells():
    grid = corridor()
    target, _, route = choose_frontier(grid, 0.1, (0, 0), (2.0, 2.25), return_path=True)
    assert target is not None and route[-1] == target
    assert np.linalg.norm(np.array(route[0]) - (2.0, 2.25)) < 0.11
    assert all(grid[int(y / 0.1), int(x / 0.1)] == 0 for x, y in route)


def test_lidar_fusion_marks_a_wall_beyond_the_cropped_map_edge():
    from ail_tb4_maze.sensor_map import fuse_rays

    fused, origin = fuse_rays(
        np.zeros((10, 10), np.int8), 0.1, (0, 0), (0.5, 0.55), [(0.75, 0.0)], [True]
    )
    r = int((0.55 - origin[1]) / 0.1)
    c = int((1.25 - origin[0]) / 0.1)
    assert fused[r, c] == 100


def test_lidar_fusion_clears_obsolete_obstacles_and_preserves_current_endpoints():
    from ail_tb4_maze.sensor_map import fuse_rays

    grid = np.zeros((10, 10), np.int8)
    grid[5, 7] = 100
    fused, origin = fuse_rays(
        grid, 0.1, (0, 0), (0.5, 0.55), [(0.75, 0.0), (0.9, 0.0)], [True, False]
    )
    r = int((0.55 - origin[1]) / 0.1)
    assert fused[r, int((0.75 - origin[0]) / 0.1)] == 0
    assert fused[r, int((1.25 - origin[0]) / 0.1)] == 100


def test_explicit_clearance_fallback_reconnects_a_narrow_observed_corridor():
    grid = np.full((32, 100), 100, dtype=np.int8)
    grid[4:28, 2:30] = 0
    grid[11:21, 30:80] = 0
    grid[5:27, 70:85] = 0
    grid[5:27, 85:] = -1
    robot = (1.0, 0.8)
    assert choose_frontier(grid, 0.05, (0.0, 0.0), robot)[0] is None
    goal, _, route = choose_frontier(
        grid, 0.05, (0.0, 0.0), robot, return_path=True, clearance_margin=0.25
    )
    assert goal is not None and goal[0] > 3.8
    assert len(route) > 40
