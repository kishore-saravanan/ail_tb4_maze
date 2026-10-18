#!/usr/bin/python3
"""Reactive LiDAR wall following. No map, maze dimensions or exit coordinates."""
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener,TransformException
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from wall_follower import WallFollower,Decision


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class Controller(Node):
    def __init__(self):
        super().__init__('maze_wall_follower')
        self.declare_parameter('wall_side','right');self.declare_parameter('wall_distance',.45);self.declare_parameter('recording_path','')
        self.side=self.get_parameter('wall_side').value
        self.follower=WallFollower(self.get_parameter('wall_distance').value,self.side)
        self.anchor=None;self.progress_time=0.;self.recovery_until=0.;self.last_command=(0.,0.);self.odom_wall=0.
        self.scan=None;self.pose=None;self.scan_wall=0.;self.status_time=-math.inf;self.first_time=None
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.commands=self.create_publisher(TwistStamped,'/cmd_vel',10)
        self.status=self.create_publisher(String,'/wall_follower/status',10)
        self.recording=None;name=self.get_parameter('recording_path').value
        if name:
            path=Path(name).expanduser();path.parent.mkdir(parents=True,exist_ok=True);self.recording=path.open('w')
        self.create_timer(.05,self.tick)
        self.get_logger().info(f'Following the {self.side} wall at 0.45 m. No map or exit coordinates.')

    def on_scan(self,msg):self.scan=msg;self.scan_wall=time.monotonic()
    def on_odom(self,msg):
        p=msg.pose.pose.position;self.pose=(p.x,p.y,yaw(msg.pose.pose.orientation));self.odom_wall=time.monotonic()

    def publish(self,v,w):
        msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link'
        msg.twist.linear.x=float(v);msg.twist.angular.z=float(w);self.commands.publish(msg)

    def tick(self):
        now=self.get_clock().now().nanoseconds/1e9
        if self.scan is None or self.pose is None or time.monotonic()-self.odom_wall>.5 or time.monotonic()-self.scan_wall>.5:self.publish(0.,0.);return
        try:t=self.tf.lookup_transform('base_link',self.scan.header.frame_id,Time()).transform
        except TransformException:self.publish(0.,0.);return
        s=self.scan;q=t.rotation
        angles=s.angle_min+np.arange(len(s.ranges))*s.angle_increment
        values=np.asarray(s.ranges);valid=np.isfinite(values)&(values>=s.range_min)&(values<s.range_max)
        xx=np.cos(angles[valid])*values[valid];yy=np.sin(angles[valid])*values[valid]
        # Project the actual sensor rotation, including a reversed scan plane.
        points=np.column_stack((t.translation.x+(1-2*(q.y*q.y+q.z*q.z))*xx+2*(q.x*q.y-q.w*q.z)*yy,
                                t.translation.y+2*(q.x*q.y+q.w*q.z)*xx+(1-2*(q.x*q.x+q.z*q.z))*yy))
        decision=self.follower.step(points,self.pose[2],self.pose[:2])
        if self.anchor is None or math.dist(self.pose[:2],self.anchor[:2])>.04 or abs((self.pose[2]-self.anchor[2]+math.pi)%(2*math.pi)-math.pi)>.12:
            self.anchor=self.pose;self.progress_time=now
        if now-self.progress_time>4. and self.follower.acquisition=='done' and (abs(self.last_command[0])>.02 or abs(self.last_command[1])>.1):
            self.recovery_until=now+1.5;self.progress_time=now
            self.follower.corner=None;self.follower.right_armed=True
        if now<self.recovery_until:
            reverse=self.follower.safe_reverse(points)
            decision=Decision(reverse,0. if reverse else .7,'back_up' if reverse else 'recovery_turn',decision.front,decision.side,decision.wall_angle)
        self.last_command=(decision.forward,decision.angular)
        self.publish(*self.last_command)
        if self.first_time is None:self.first_time=now
        if now-self.status_time>=.25:
            def finite(x):return float(x) if math.isfinite(x) else None
            event=dict(sim_seconds=now,controller_seconds=now-self.first_time,state=decision.state,wall_side=self.side,
                       front_m=finite(decision.front),side_m=finite(decision.side),wall_angle_rad=decision.wall_angle,
                       command=[decision.forward,decision.angular],pose=self.pose,lidar_points=points[::4].round(3).tolist())
            self.status.publish(String(data=json.dumps(event,allow_nan=False)))
            if self.recording:self.recording.write(json.dumps(event,allow_nan=False)+'\n');self.recording.flush()
            self.status_time=now


def main():
    rclpy.init();node=Controller()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.publish(0.,0.)
        if node.recording:node.recording.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
