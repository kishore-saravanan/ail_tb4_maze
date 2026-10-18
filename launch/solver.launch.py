"""Start the LiDAR wall follower in the sensor-side ROS domain."""
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ail_tb4_maze.paths import RESULTS

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('navigation_domain',default_value='62'),
        DeclareLaunchArgument('wall_side',default_value='right'),
        DeclareLaunchArgument('recording_path',default_value=str(RESULTS/f'wall_following_{time.time_ns()}.jsonl')),
        SetEnvironmentVariable('ROS_DOMAIN_ID',LaunchConfiguration('navigation_domain')),
        Node(package='ail_tb4_maze',executable='maze_solver',parameters=[{'use_sim_time':True,'wall_side':LaunchConfiguration('wall_side'),'recording_path':LaunchConfiguration('recording_path')}],output='screen')])
