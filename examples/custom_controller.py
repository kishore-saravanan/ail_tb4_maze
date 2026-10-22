#!/usr/bin/env python3
"""Minimal replacement-controller template; stays stopped until you add a policy.

Select this file in the setup GUI. The GUI supplies the navigation ROS domain
and simulation clock. See docs/EXTENDING.md for topics and mapping options.
"""

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class CustomController(Node):
    """Receive LiDAR and publish stamped velocity commands at 20 Hz."""

    def __init__(self):
        super().__init__("custom_maze_controller")
        self.declare_parameter("recording_path", "")
        self.scan = None
        self.last_scan = 0.0
        self.publisher = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_timer(0.05, self.tick)

    def on_scan(self, message):
        self.scan = message
        self.last_scan = time.monotonic()

    def decide_velocity(self, scan):
        """Return (linear m/s, angular rad/s); implement your policy here.

        Returning zero is intentional. A replacement policy must handle scan
        geometry, the robot's footprint and collision avoidance before moving.
        """
        return 0.0, 0.0

    def publish(self, linear, angular):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = float(linear)
        message.twist.angular.z = float(angular)
        self.publisher.publish(message)

    def tick(self):
        if self.scan is None or time.monotonic() - self.last_scan > 0.5:
            self.publish(0.0, 0.0)
            return
        self.publish(*self.decide_velocity(self.scan))


def main():
    rclpy.init()
    node = CustomController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
