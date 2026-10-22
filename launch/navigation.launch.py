"""SLAM Toolbox, Nav2 and frontier exploration in the current sensor domain."""

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
    params = LaunchConfiguration("params_file")
    common = {"use_sim_time": True}
    servers = [
        ("nav2_controller", "controller_server"),
        ("nav2_planner", "planner_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator"),
    ]
    actions = [
        DeclareLaunchArgument(
            "params_file", default_value=str(pkg / "config/navigation.yaml")
        ),
        DeclareLaunchArgument("recording_path", default_value=""),
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
    ]
    for package, executable in servers:
        actions.append(
            Node(
                package=package,
                executable=executable,
                name=executable,
                parameters=[params, common],
                remappings=[("cmd_vel", "/navigation/cmd_vel")],
                output="screen",
            )
        )
    actions.extend(
        [
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=[
                    dict(common, autostart=True, node_names=[s[1] for s in servers])
                ],
                output="screen",
            ),
            Node(
                package="ail_tb4_maze",
                executable="frontier_explorer",
                parameters=[
                    dict(common, recording_path=LaunchConfiguration("recording_path"))
                ],
                output="screen",
            ),
            Node(
                package="ail_tb4_maze",
                executable="velocity_adapter",
                parameters=[common],
                output="screen",
            ),
        ]
    )

    def required_exit(event, context):
        return (
            []
            if context.is_shutdown
            else [EmitEvent(event=Shutdown(reason="A required navigation node exited"))]
        )

    critical = [a for a in actions if isinstance(a, Node)]
    for node in critical:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(target_action=node, on_exit=required_exit)
            )
        )
    return LaunchDescription(actions)
