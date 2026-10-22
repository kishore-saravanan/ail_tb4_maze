"""Publish the travelled SLAM-pose trail for RViz, without simulator input."""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class NavigationTrail(Node):
    def __init__(self):
        super().__init__("navigation_trail")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(
            Path,
            "/travelled_path",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.poses = deque(maxlen=6000)
        self.last_time = None
        self.create_timer(0.1, self.tick)
        self.create_service(Trigger, "/display/clear_path", self.clear)

    def clear(self, request, response):
        self.poses.clear()
        response.success = True
        response.message = "Travelled path cleared"

        return response

    def tick(self):
        now = self.get_clock().now()
        seconds = now.nanoseconds / 1e9
        if self.last_time is not None and seconds < self.last_time - 0.5:
            self.poses.clear()
        self.last_time = seconds
        try:
            t = self.tf.lookup_transform("map", "base_link", Time()).transform
        except TransformException:

            return
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = now.to_msg()
        pose.pose.position.x = t.translation.x
        pose.pose.position.y = t.translation.y
        pose.pose.position.z = 0.025
        pose.pose.orientation = t.rotation
        distance = math.inf
        if self.poses:
            last = self.poses[-1].pose.position
            distance = math.hypot(last.x - t.translation.x, last.y - t.translation.y)
            if distance > 1.0:
                # A reset or a large map correction must not draw a diagonal
                # shortcut between two unrelated runs/localization epochs.
                self.poses.clear()
        if not self.poses or distance >= 0.04:
            self.poses.append(pose)
        path = Path()
        path.header = pose.header
        path.poses = list(self.poses)
        self.publisher.publish(path)


def main():
    rclpy.init()
    node = NavigationTrail()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
