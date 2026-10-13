"""Open the package's setup GUI; it owns the Gazebo/RViz environment lifecycle."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ail_tb4_maze.paths import RUNTIME_ROOT

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('runtime_dir',default_value=str(RUNTIME_ROOT),description='Directory for generated scenes, logs and scores'),
        SetEnvironmentVariable('AIL_TB4_MAZE_RUNTIME_DIR',LaunchConfiguration('runtime_dir')),
        SetEnvironmentVariable('ROS_DOMAIN_ID','61'),
        SetEnvironmentVariable('GZ_PARTITION','ai_legion_maze'),
        Node(package='ail_tb4_maze',executable='setup_gui',output='screen')])
