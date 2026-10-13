"""Allowlisted relay between infrastructure domain 61 and controller domain 62.
Logical separation only: not a security boundary against a same-user process.
"""
import math
import time
import os
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool

ALLOWED_EDGES = {('odom','base_link'), ('base_link','base_footprint'),
                 ('base_link','imu_link'), ('base_link','shell_link'),
                 ('shell_link','rplidar_link'),
                 ('rplidar_link','turtlebot4/rplidar_link/rplidar')}

def main():
    private=Context();public=Context()
    rclpy.init(context=private, domain_id=int(os.environ.get('MAZE_INFRA_DOMAIN','61')))
    rclpy.init(context=public, domain_id=int(os.environ.get('MAZE_NAV_DOMAIN','62')))
    source=Node('interface_source',context=private,parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    target=Node('navigation_interface',context=public,parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    executor=SingleThreadedExecutor(context=private)
    executor.add_node(source);executor.add_node(target)
    holders=[]
    def relay(cls, src, dst, qos, transform=None):
        pub=target.create_publisher(cls,dst,qos)
        def receive(msg):
            pub.publish(transform(msg) if transform else msg)
        holders.append(source.create_subscription(cls,src,receive,qos))
    relay(LaserScan,'/scan','/scan',qos_profile_sensor_data)
    origin=[None]
    def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def rebase(position,orientation):
        if origin[0] is None:return
        ox,oy,a=origin[0];dx=position.x-ox;dy=position.y-oy
        position.x=math.cos(a)*dx+math.sin(a)*dy;position.y=-math.sin(a)*dx+math.cos(a)*dy
        angle=yaw(orientation)-a;orientation.x=0.;orientation.y=0.;orientation.z=math.sin(angle/2);orientation.w=math.cos(angle/2)
    def local_odom(msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        if origin[0] is None:origin[0]=(p.x,p.y,yaw(q))
        rebase(p,q);return msg
    relay(Odometry,'/odom','/odom',qos_profile_sensor_data,local_odom)
    def imu_frame(msg):
        msg.header.frame_id='imu_link';return msg
    relay(Imu,'/imu','/imu',qos_profile_sensor_data,imu_frame)
    relay(Clock,'/clock','/clock',qos_profile_sensor_data)
    def filtered(msg):
        transforms=[]
        for t in msg.transforms:
            edge=(t.header.frame_id,t.child_frame_id)
            if edge not in ALLOWED_EDGES:continue
            if edge==('odom','base_link'):
                if origin[0] is None:continue
                rebase(t.transform.translation,t.transform.rotation)
            transforms.append(t)
        return TFMessage(transforms=transforms)
    relay(TFMessage,'/tf','/tf',100,filtered)
    static_edges={}
    def static_filtered(msg):
        for t in filtered(msg).transforms:
            static_edges[(t.header.frame_id,t.child_frame_id)]=t
        return TFMessage(transforms=list(static_edges.values()))
    relay(TFMessage,'/tf_static','/tf_static',QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL),static_filtered)
    pub=source.create_publisher(TwistStamped,'/diffdrive_controller/cmd_vel',10)
    last=[-math.inf]; active=[False]; enabled=[True]; trial_active=[False]
    def stop():
        msg=TwistStamped();msg.header.stamp=source.get_clock().now().to_msg();pub.publish(msg);active[0]=False
    def trial(msg):
        trial_active[0]=msg.data
        if not msg.data:stop()
    holders.append(source.create_subscription(Bool,'/privileged/trial_active',trial,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    def enable(req,res):
        enabled[0]=req.data
        if not req.data:stop()
        res.success=True;res.message='Interface enabled' if req.data else 'Commands inhibited';return res
    def reset(req,res):
        origin[0]=None;stop();res.success=True;res.message='Local wheel-odometry origin reset';return res
    holders.append(source.create_service(SetBool,'/interface/enable',enable))
    holders.append(source.create_service(Trigger,'/interface/reset',reset))
    def command(msg):
        if not enabled[0] or not trial_active[0]:return
        v=msg.twist.linear.x; w=msg.twist.angular.z
        if not math.isfinite(v) or not math.isfinite(w):return
        out=TwistStamped();out.header.stamp=source.get_clock().now().to_msg()
        out.twist.linear.x=max(-0.3,min(0.3,v));out.twist.angular.z=max(-1.5,min(1.5,w))
        pub.publish(out);last[0]=time.monotonic();active[0]=True
    holders.append(target.create_subscription(TwistStamped,'/cmd_vel',command,10))
    # The underlying wheel controller also has a 0.5 simulated-second timeout.
    def watchdog():
        if active[0] and time.monotonic()-last[0] > 0.5:
            msg=TwistStamped();msg.header.stamp=source.get_clock().now().to_msg();pub.publish(msg);active[0]=False
    holders.append(target.create_timer(0.05,watchdog,clock=rclpy.clock.Clock()))
    try:executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):pass
    finally:
        source.destroy_node();target.destroy_node();private.try_shutdown();public.try_shutdown()
