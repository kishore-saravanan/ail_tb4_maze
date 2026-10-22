"""SLAM and sensor-checked frontier waypoint exploration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = Path(share("ail_tb4_maze"))
    controller = Node(
        package="ail_tb4_maze",
        executable="direct_explorer",
        parameters=[
            {
                "use_sim_time": True,
                "recording_path": LaunchConfiguration("recording_path"),
            }
        ],
        output="screen",
    )

    def required_exit(event, context):
        return (
            []
            if context.is_shutdown
            else [EmitEvent(event=Shutdown(reason="Exploration controller exited"))]
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("recording_path", default_value=""),
            DeclareLaunchArgument("params_file", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(share("slam_toolbox")) / "launch/online_async_launch.py")
                ),
                launch_arguments={
                    "slam_params_file": str(pkg / "config/slam.yaml"),
                    "use_sim_time": "true",
                    "autostart": "true",
                }.items(),
            ),
            controller,
            RegisterEventHandler(
                OnProcessExit(target_action=controller, on_exit=required_exit)
            ),
        ]
    )
