"""Explore an unknown SLAM map through Nav2; no simulator ground truth input."""

import base64
import copy
import json
import math
import time
import zlib
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from scipy import ndimage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .frontier_planner import choose_frontier, goal_is_safe
from .sensor_map import fuse_rays


class Explorer(Node):
    def __init__(self):
        super().__init__("maze_frontier_explorer")
        self.declare_parameter("recording_path", "")
        path = self.get_parameter("recording_path").value
        self.log = None
        if path:
            dest = Path(path)
            dest.parent.mkdir(parents=True, exist_ok=True)
            self.log = dest.open("w")
        self.algorithm = "SLAM + reachable frontier exploration + Nav2"
        self.sequence = 0
        self.started = time.monotonic()
        self.map = None
        self.pending = False
        self.goal_handle = None
        self.target = None
        self.blacklist = []
        self.goal_time = 0.0
        self.retry_time = 0.0
        self.pose = None
        self.raw_map = None
        self.planning_map_record = None
        self.map_clearance = None
        self.visited = []
        self.heading = None
        self.scan = None
        self.map_record = None
        self.local_map_record = None
        self.plan_points = []
        self.odom_pose = None
        self.command = (0.0, 0.0)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.pub = self.create_publisher(String, "/explorer/status", 10)
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            "/navigation/map",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self.on_local_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(NavPath, "/plan", self.on_plan, 10)
        self.create_subscription(
            Odometry, "/odom", self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(TwistStamped, "/cmd_vel", self.on_command, 10)
        self.create_timer(0.5, self.tick)

    @staticmethod
    def pack_grid(msg):
        return dict(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin=[msg.info.origin.position.x, msg.info.origin.position.y],
            frame=msg.header.frame_id,
            data_zlib_base64=base64.b64encode(
                zlib.compress(np.asarray(msg.data, dtype=np.int8).tobytes(), 1)
            ).decode(),
        )

    def on_map(self, msg):
        self.raw_map = msg
        if self.map is None:
            self.map = msg
            cells = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
            self.map_clearance = (
                ndimage.distance_transform_edt(cells < 50) * msg.info.resolution
            )
        if self.log:
            self.map_record = self.pack_grid(msg)

    def refresh_planning_map(self):
        if self.raw_map is None or self.scan is None:
            return
        try:
            t = self.tf.lookup_transform(
                "map", self.scan.header.frame_id, Time()
            ).transform
        except TransformException:
            return
        s = self.scan
        values = np.asarray(s.ranges)
        valid = (np.isfinite(values) & (values >= s.range_min)) | np.isposinf(values)
        if not valid.any():
            return
        angles = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
        q = t.rotation
        matrix = np.array(
            [
                [1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.w * q.z)],
                [2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z)],
            ]
        )
        direction = (
            np.column_stack((np.cos(angles[valid]), np.sin(angles[valid]))) @ matrix.T
        )
        length = np.minimum(values[valid], 3.0)
        hits = np.isfinite(values[valid]) & (
            values[valid] < min(3.0, s.range_max - 0.01)
        )
        raw = self.raw_map
        cells = np.asarray(raw.data).reshape(raw.info.height, raw.info.width)
        fused, origin = fuse_rays(
            cells,
            raw.info.resolution,
            (raw.info.origin.position.x, raw.info.origin.position.y),
            (t.translation.x, t.translation.y),
            direction * length[:, None],
            hits,
        )
        msg = OccupancyGrid()
        msg.header = copy.copy(raw.header)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = copy.deepcopy(raw.info)
        msg.info.width = fused.shape[1]
        msg.info.height = fused.shape[0]
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.data = fused.ravel().tolist()
        self.map = msg
        self.map_clearance = (
            ndimage.distance_transform_edt(fused < 50) * raw.info.resolution
        )
        self.map_pub.publish(msg)
        if self.log:
            self.planning_map_record = self.pack_grid(msg)

    def on_local_map(self, msg):
        if self.log:
            self.local_map_record = self.pack_grid(msg)

    def on_plan(self, msg):
        self.plan_points = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses[::2]
        ]

    def on_scan(self, msg):
        self.scan = msg

    def on_command(self, msg):
        self.command = (msg.twist.linear.x, msg.twist.angular.z)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_pose = (
            p.x,
            p.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        )

    def lidar_points(self):
        if self.scan is None:
            return []
        try:
            t = self.tf.lookup_transform(
                "base_link", self.scan.header.frame_id, Time()
            ).transform
        except TransformException:
            return []
        s = self.scan
        q = t.rotation
        angles = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
        values = np.asarray(s.ranges)
        valid = np.isfinite(values) & (values >= s.range_min) & (values < s.range_max)
        xx = np.cos(angles[valid]) * values[valid]
        yy = np.sin(angles[valid]) * values[valid]
        points = np.column_stack(
            (
                t.translation.x
                + (1 - 2 * (q.y * q.y + q.z * q.z)) * xx
                + 2 * (q.x * q.y - q.w * q.z) * yy,
                t.translation.y
                + 2 * (q.x * q.y + q.w * q.z) * xx
                + (1 - 2 * (q.x * q.x + q.z * q.z)) * yy,
            )
        )
        return points[::3].round(3).tolist()

    def report(self, state, frontiers=()):
        data = dict(
            sim_seconds=self.get_clock().now().nanoseconds / 1e9,
            state=state,
            goal=self.target,
            pose=self.pose,
            frontiers=list(frontiers),
            algorithm=self.algorithm,
            strategy=getattr(self, "strategy_record", None),
        )
        self.pub.publish(String(data=json.dumps(data)))
        if self.log:
            data.update(
                planning_map=self.planning_map_record,
                heading=self.heading,
                odom_pose=self.odom_pose,
                command=self.command,
                lidar_points=self.lidar_points(),
                map=self.map_record,
                local_map=self.local_map_record,
                plan=self.plan_points,
            )
            self.log.write(json.dumps(data) + "\n")
            self.log.flush()

    def finished(self, future, sequence, target):
        if sequence != self.sequence:
            return
        result = future.result()
        self.goal_handle = None
        self.pending = False
        if result.status != 4:
            self.blacklist.append(target)
        self.blacklist = self.blacklist[-20:]
        self.retry_time = self.get_clock().now().nanoseconds / 1e9 + 0.2

    def accepted(self, future, sequence, target):
        handle = future.result()
        if sequence != self.sequence:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        self.pending = False
        if not handle.accepted:
            self.blacklist.append(target)
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self.finished(f, sequence, target)
        )

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        self.refresh_planning_map()
        if self.map is None or not self.client.server_is_ready():
            if time.monotonic() - self.started > 60.0:
                raise RuntimeError(
                    "SLAM/Nav2 did not become ready within 60 seconds; "
                    "inspect controller.log"
                )
            self.report("waiting_for_slam_and_nav2")
            return
        try:
            t = self.tf.lookup_transform("map", "base_link", Time()).transform
        except TransformException:
            self.report("waiting_for_localization")
            return
        self.pose = (t.translation.x, t.translation.y)
        q = t.rotation
        self.heading = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
        )
        if not self.visited or math.dist(self.pose, self.visited[-1]) > 0.3:
            self.visited.append(self.pose)
        if self.pending:
            return
        if self.goal_handle:
            m = self.map
            cells = np.asarray(m.data).reshape(m.info.height, m.info.width)
            if not goal_is_safe(
                cells,
                m.info.resolution,
                (m.info.origin.position.x, m.info.origin.position.y),
                self.target,
                self.map_clearance,
            ):
                self.sequence += 1
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
                self.retry_time = now + 0.1
                self.report("goal_invalidated_by_map")
                return
            if now - self.goal_time > 40.0:
                self.sequence += 1
                self.goal_handle.cancel_goal_async()
                self.blacklist.append(self.target)
                self.goal_handle = None
                self.pending = False
                self.retry_time = now + 2
                self.report("goal_timeout")
                return
            if math.dist(self.pose, self.target) > 0.5:
                self.report("navigating")
                return
        if now < self.retry_time:
            return
        m = self.map
        if abs(m.info.origin.orientation.z) > 0.001:
            self.report("unsupported_map_rotation")
            return
        cells = np.asarray(m.data).reshape(m.info.height, m.info.width)
        point, frontiers = choose_frontier(
            cells,
            m.info.resolution,
            (m.info.origin.position.x, m.info.origin.position.y),
            self.pose,
            self.blacklist,
            visited=self.visited,
        )
        if point is None:
            if self.goal_handle:
                self.report("navigating")
                return
            self.report("no_reachable_frontier", frontiers)
            # A finite blacklist must not permanently forbid the remaining map.
            if self.blacklist:
                self.blacklist = self.blacklist[1:]
            self.retry_time = now + 2
            return
        if self.goal_handle:
            if math.dist(point, self.target) < 0.15:
                self.report("navigating")
                return
            old = (self.target[0] - self.pose[0], self.target[1] - self.pose[1])
            new = (point[0] - self.pose[0], point[1] - self.pose[1])
            # Only extend an ongoing approach forward. A different branch must
            # wait until this frontier has actually been reached and observed.
            if old[0] * new[0] + old[1] * new[1] < 0.7071 * math.hypot(
                *old
            ) * math.hypot(*new):
                self.report("navigating")
                return
        self.target = point
        self.goal_time = now
        self.pending = True
        self.sequence += 1
        sequence = self.sequence
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = point[0]
        goal.pose.pose.position.y = point[1]
        angle = math.atan2(point[1] - self.pose[1], point[0] - self.pose[0])
        goal.pose.pose.orientation.z = math.sin(angle / 2)
        goal.pose.pose.orientation.w = math.cos(angle / 2)
        self.client.send_goal_async(goal).add_done_callback(
            lambda f: self.accepted(f, sequence, point)
        )
        self.report("choosing_frontier", frontiers)


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.goal_handle:
            node.goal_handle.cancel_goal_async()
        if node.log:
            node.log.close()
        node.destroy_node()
        rclpy.try_shutdown()
