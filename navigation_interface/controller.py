#!/usr/bin/python3
"""Implement compute_velocity() using permitted sensor observations only."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry

class Controller(Node):
    def __init__(self):
        super().__init__('maze_controller')
        self.scan = self.odom = self.imu = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', lambda m: setattr(self, 'imu', m), qos_profile_sensor_data)
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Placeholder running: implement compute_velocity(); default velocity is zero.')

    def compute_velocity(self):
        # Return forward speed (m/s), angular speed (rad/s).
        # Use permitted TF topics to interpret sensor axes when implementing a solver.
        return 0.0, 0.0

    def publish_velocity(self, forward, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(forward)
        msg.twist.angular.z = float(angular)
        self.publisher.publish(msg)

    def tick(self):
        self.publish_velocity(*self.compute_velocity())

def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_velocity(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
