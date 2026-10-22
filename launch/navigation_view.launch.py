"""RViz robot geometry and travelled path, using only sensor-side TF."""

from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch_ros.actions import Node

from ail_tb4_maze.display_model import fixed_display_model


def generate_launch_description():
    description = (
        Path(share("turtlebot4_description")) / "urdf/standard/turtlebot4.urdf.xacro"
    )
    physical = xacro.process_file(
        str(description), mappings={"gazebo": "ignition", "namespace": ""}
    ).toxml()
    model = fixed_display_model(physical)
    common = {"use_sim_time": True}

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace="display",
                name="robot_state_publisher",
                parameters=[common, {"robot_description": model}],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="display_root",
                arguments=[
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "display_base_link",
                ],
                parameters=[common],
            ),
            Node(
                package="ail_tb4_maze",
                executable="navigation_view",
                parameters=[common],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    str(Path(share("ail_tb4_maze")) / "config/navigation.rviz"),
                ],
                parameters=[common],
                output="screen",
            ),
        ]
    )
