"""Bridge Nav2 Twist commands to the challenge's stamped velocity interface."""

import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class Adapter(Node):
    def __init__(self):
        super().__init__("navigation_velocity_adapter")
        self.last = 0.0
        self.cmd = Twist()
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(Twist, "/navigation/cmd_vel", self.receive, 10)
        self.create_timer(0.05, self.tick)

    def receive(self, msg):
        self.cmd = msg
        self.last = time.monotonic()

    def tick(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        if time.monotonic() - self.last < 0.3:
            msg.twist = self.cmd
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(TwistStamped())
        node.destroy_node()
        rclpy.try_shutdown()
