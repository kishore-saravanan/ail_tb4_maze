"""Sensor-built mapping, frontier Dijkstra and checked waypoint motion."""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from tf2_ros import TransformException

from .frontier_node import Explorer
from .frontier_planner import choose_frontier, frontier_goal_active
from .path_follower import PathFollower, segment_clear, simplify_route


class DirectExplorer(Explorer):
    """Explore a sensor-built map with reachable frontier goals and live guards."""

    def __init__(self):
        super().__init__()
        self.follower = PathFollower()
        self.scan_wall = 0.0
        self.odom_wall = 0.0
        self.blocked_since = None
        self.motion_anchor = None
        self.motion_time = 0.0
        self.backup_until = 0.0
        self.motion_state = "waiting"
        self.replan = False
        self.algorithm = "Live-map frontier routing + observed-goal cancellation"
        self.route_revision = 0
        self.observed_goal_replans = 0
        self.replan_reason = "startup"
        self.commands = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.route_pub = self.create_publisher(Path, "/plan", 10)
        self.create_timer(0.05, self.control)

    def on_scan(self, msg):
        super().on_scan(msg)
        self.scan_wall = time.monotonic()

    def on_odom(self, msg):
        super().on_odom(msg)
        self.odom_wall = time.monotonic()

    def update_pose(self):
        """Refresh the SLAM-corrected pose used by planning and motion."""
        try:
            t = self.tf.lookup_transform("map", "base_link", Time()).transform
        except TransformException:

            return False
        q = t.rotation
        self.heading = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
        )
        self.pose = (t.translation.x, t.translation.y)

        return True

    def points(self):
        """Project the current LiDAR scan into the robot body frame."""
        if self.scan is None:

            return np.empty((0, 2))
        try:
            t = self.tf.lookup_transform(
                "base_link", self.scan.header.frame_id, Time()
            ).transform
        except TransformException:

            return np.empty((0, 2))
        s = self.scan
        q = t.rotation
        a = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
        values = np.asarray(s.ranges)
        ok = (np.isfinite(values) & (values >= s.range_min)) | np.isposinf(values)
        ranges = np.minimum(values[ok], min(8.0, s.range_max))
        x = np.cos(a[ok]) * ranges
        y = np.sin(a[ok]) * ranges

        return np.column_stack(
            (
                t.translation.x
                + (1 - 2 * (q.y * q.y + q.z * q.z)) * x
                + 2 * (q.x * q.y - q.w * q.z) * y,
                t.translation.y
                + 2 * (q.x * q.y + q.w * q.z) * x
                + (1 - 2 * (q.x * q.x + q.z * q.z)) * y,
            )
        )

    def publish(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.commands.publish(msg)
        self.command = (v, w)

    def navigation_report(self, state, frontiers=()):
        self.strategy_record = dict(
            route_revision=self.route_revision,
            observed_goal_replans=self.observed_goal_replans,
            replan_reason=self.replan_reason,
            speed_cap_mps=0.30,
        )
        self.report(state, frontiers)

    def publish_route(self, route):
        self.plan_points = route
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in route:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self.route_pub.publish(path)

    def tick(self):
        """Reconsider the target and route at the 2 Hz planning rate."""
        now = self.get_clock().now().nanoseconds / 1e9
        self.refresh_planning_map()
        if self.map is None or not self.update_pose():
            if time.monotonic() - self.started > 60.0:
                raise RuntimeError("SLAM did not become ready")
            self.navigation_report("waiting_for_slam")

            return
        if not self.visited or math.dist(self.pose, self.visited[-1]) > 0.3:
            self.visited.append(self.pose)
        m = self.map
        cells = np.asarray(m.data).reshape(m.info.height, m.info.width)
        origin = (m.info.origin.position.x, m.info.origin.position.y)
        if not self.follower.done and not self.replan:
            if not frontier_goal_active(cells, m.info.resolution, origin, self.target):
                self.replan = True
                self.observed_goal_replans += 1
                self.replan_reason = "frontier_observed"
            remaining = self.follower.route[self.follower.index :]
            previous = self.pose
            for p in remaining[:2]:
                if self.replan:
                    break
                if not segment_clear(
                    previous,
                    p,
                    cells,
                    m.info.resolution,
                    origin,
                    self.map_clearance,
                    margin=0.24,
                ):
                    self.replan = True
                    self.replan_reason = "route_changed"
                    break
                previous = p
            if not self.replan:
                self.navigation_report(self.motion_state)

                return
        elif self.follower.done:
            self.replan_reason = "route_finished"
        if now < self.backup_until:
            self.navigation_report(self.motion_state)

            return
        point, frontiers, route = choose_frontier(
            cells,
            m.info.resolution,
            origin,
            self.pose,
            self.blacklist,
            visited=self.visited,
            return_path=True,
        )
        if point is None:
            # A discretized SLAM corridor can lose connectivity at the preferred
            # 28 cm margin. Retry at 25 cm; live 23.5 cm swept-circle checks
            # still govern every command, independently of this planning map.
            point, frontiers, route = choose_frontier(
                cells,
                m.info.resolution,
                origin,
                self.pose,
                self.blacklist,
                visited=self.visited,
                return_path=True,
                clearance_margin=0.25,
            )
        if point is None:
            self.follower.set_route([], (*self.pose, self.heading))
            self.target = None
            self.publish_route([])
            self.navigation_report("no_reachable_frontier", frontiers)
            if self.blacklist:
                self.blacklist.pop(0)

            return
        self.target = point
        self.goal_time = now
        route = simplify_route(
            route, cells, m.info.resolution, origin, self.map_clearance
        )
        self.follower.set_route(route, (*self.pose, self.heading))
        self.replan = False
        self.blocked_since = None
        self.route_revision += 1
        self.publish_route(route)
        self.navigation_report("choosing_frontier", frontiers)

    def control(self):
        """Follow the route at 20 Hz, stopping when sensor data is stale."""
        now = self.get_clock().now().nanoseconds / 1e9
        if (
            time.monotonic() - self.scan_wall > 0.5
            or time.monotonic() - self.odom_wall > 0.5
            or not self.update_pose()
        ):
            self.publish(0.0, 0.0)

            return
        points = self.points()
        if not len(points):
            self.publish(0.0, 0.0)

            return
        if now < self.backup_until:
            if self.follower.safe(points, -0.1, 0.0):
                self.publish(-0.1, 0.0)
                self.motion_state = "recover_back"
            elif self.follower.safe(points, 0.0, 0.8):
                self.publish(0.0, 0.8)
                self.motion_state = "recover_turn"
            else:
                self.publish(0.0, 0.0)
                self.motion_state = "recover_stop"

            return
        v, w, state = self.follower.step((*self.pose, self.heading), points)
        self.motion_state = state
        if (
            self.motion_anchor is None
            or math.dist(self.pose, self.motion_anchor) > 0.05
        ):
            self.motion_anchor = self.pose
            self.motion_time = now
        if state == "blocked":
            if self.blocked_since is None:
                self.blocked_since = now
            if now - self.blocked_since > 1.0:
                self.replan = True
                self.replan_reason = "lidar_blocked"
        else:
            self.blocked_since = None
        if not self.follower.done and now - self.motion_time > 8.0:
            if self.target:
                self.blacklist.append(self.target)
                self.blacklist = self.blacklist[-12:]
            self.backup_until = now + 2.0
            self.replan = True
            self.replan_reason = "no_motion"
            self.motion_time = now
            self.follower.set_route([], (*self.pose, self.heading))
        self.publish(v, w)


def main():
    rclpy.init()
    node = DirectExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish(0.0, 0.0)
        if node.log:
            node.log.close()
        node.destroy_node()
        rclpy.try_shutdown()
