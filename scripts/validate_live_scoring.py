#!/usr/bin/python3
"""Privileged straight-line scoring checks in the live randomized scene."""
import sys,json,time,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from ail_tb4_maze.live_scene import transport,request,ros_service,reset_trial
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.physics_pb2 import Physics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan,Imu
from nav_msgs.msg import Odometry
rclpy.init(domain_id=62);node=Node('live_scoring_probe');data={}
for k,t,c in [('clock','/clock',Clock),('scan','/scan',LaserScan),('odom','/odom',Odometry),('imu','/imu',Imu)]:node.create_subscription(c,t,lambda m,k=k:data.__setitem__(k,m),qos_profile_sensor_data)
pub=node.create_publisher(TwistStamped,'/cmd_vel',10);gz=transport();records=[]
def spin(predicate,limit=20):
 deadline=time.monotonic()+limit
 while time.monotonic()<deadline and not predicate():rclpy.spin_once(node,timeout_sec=.03)
 assert predicate(),'Live scoring wait expired'
def now():
 c=data['clock'].clock;return c.sec+c.nanosec/1e9
def status():return json.loads((ROOT/'.runtime/results/status.json').read_text())
def set_pose(x,y):
 p=Pose(name='turtlebot4');p.position.x=x;p.position.y=y;p.position.z=.02;p.orientation.w=1
 request(gz,'/world/maze_escape/set_pose',p);ros_service('/interface/reset')
 begin=now();spin(lambda:now()-begin>1)
def drive(seconds):
 spin(lambda:status()['status']=='running')
 start=status()['elapsed_sim_seconds'];deadline=time.monotonic()+90
 while status()['elapsed_sim_seconds']-start<seconds and status()['status']=='running' and time.monotonic()<deadline:
  m=TwistStamped();m.twist.linear.x=.3;pub.publish(m);rclpy.spin_once(node,timeout_sec=.02);time.sleep(.025)
 pub.publish(TwistStamped())
try:
 spin(lambda:len(data)==4);assert len(data['scan'].ranges)==640;assert abs(data['imu'].linear_acceleration.z-9.8)<.5
 c=json.loads((ROOT/'state/challenge.json').read_text());half=c['maze_size_m']/2
 reset_trial(ROOT/'.runtime/state');set_pose(half-.6,half-c['maze_size_m']/c['grid_cells']/2)
 ros_service('/referee/start');drive(5);ros_service('/referee/abort');spin(lambda:status().get('result_file') is not None)
 res=json.loads(Path(status()['result_file']).read_text());assert res['collision_episodes']>=1,res;assert res['distance_meters']>.05;records.append({'case':'collision','result':res});print('Live collision scoring passed',flush=True)
 reset_trial(ROOT/'.runtime/state');exit=c['exit'];set_pose(half-.5,(exit['y_min']+exit['y_max'])/2)
 ros_service('/referee/start');drive(6);spin(lambda:status().get('result_file') is not None)
 res=json.loads(Path(status()['result_file']).read_text());assert res['status']=='success',res;records.append({'case':'success','result':res});print('Live exit scoring passed',flush=True)
 # Keep the real 300 simulated-second budget; accelerate only this test.
 reset_trial(ROOT/'.runtime/state');request(gz,'/world/maze_escape/set_physics',Physics(real_time_factor=10,max_step_size=.001))
 ros_service('/referee/start');spin(lambda:status()['status']=='timeout',600);spin(lambda:status().get('result_file') is not None)
 res=json.loads(Path(status()['result_file']).read_text());assert 300<=res['elapsed_sim_seconds']<300.1,res;records.append({'case':'timeout_300','result':res});print('Live full timeout scoring passed',flush=True)
 request(gz,'/world/maze_escape/set_physics',Physics(real_time_factor=1,max_step_size=.001));reset_trial(ROOT/'.runtime/state')
 (ROOT/'.runtime/results/live_scoring_validation.json').write_text(json.dumps({'passed':True,'cases':records,'sensor_topics_received':list(data)},indent=2))
finally:
 pub.publish(TwistStamped());node.destroy_node();rclpy.try_shutdown()
