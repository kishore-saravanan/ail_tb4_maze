"""Official TurtleBot 4 standard model and wheel controller; custom minimal bridges."""

import os
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ail_tb4_maze.paths import RESULTS


def setup(context):
    pkg = Path(share("ail_tb4_maze"))
    description = Path(share("turtlebot4_description"))
    urdf = xacro.process_file(
        str(description / "urdf/standard/turtlebot4.urdf.xacro"),
        mappings={"gazebo": "ignition", "namespace": ""},
    ).toxml()
    robot = ET.fromstring(urdf)
    # Keep official robot geometry; simulate only sensors used by this challenge.
    for gazebo in robot.findall("gazebo"):
        for sensor in list(gazebo.findall("sensor")):
            if sensor.get("type") not in ("gpu_lidar", "imu", "contact"):
                gazebo.remove(sensor)
    # World owns these systems. Avoid duplicate rendering/contact engines.
    for gazebo in robot.findall("gazebo"):
        for plugin in list(gazebo.findall("plugin")):
            if plugin.get("name") in (
                "gz::sim::systems::Sensors",
                "gz::sim::systems::Contact",
            ):
                gazebo.remove(plugin)
    # Publish the robot model pose with frame metadata for the referee bridge.
    for plugin in robot.findall(".//plugin"):
        if plugin.get("name") == "gz::sim::systems::PosePublisher":
            ET.SubElement(plugin, "publish_model_pose").text = "true"
    urdf = ET.tostring(robot, encoding="unicode")
    headless = LaunchConfiguration("headless").perform(context) == "true"
    args = [
        "gz",
        "sim",
        "-r",
        "-v",
        "3",
        LaunchConfiguration("world_file").perform(context),
    ]
    if headless:
        args.extend(["-s", "--headless-rendering"])
    else:
        args.extend(["--gui-config", str(pkg / "config/gazebo.config")])
    actions = [ExecuteProcess(cmd=args, output="screen")]
    common = {"use_sim_time": True}
    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[common, {"robot_description": urdf}],
            remappings=[("joint_states", "/visual/joint_states")],
            output="screen",
        )
    )
    # Merge live encoder angles with defaults for passive suspension joints.
    # This completes the operator RobotModel display without exporting these TFs.
    actions.append(
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            namespace="visual",
            parameters=[common, {"source_list": ["/joint_states"]}],
            remappings=[("robot_description", "/robot_description")],
            output="screen",
        )
    )
    actions.append(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-world",
                "maze_escape",
                "-name",
                "turtlebot4",
                "-topic",
                "robot_description",
                "-x",
                LaunchConfiguration("x"),
                "-y",
                LaunchConfiguration("y"),
                "-z",
                "0.02",
                "-Y",
                LaunchConfiguration("yaw"),
            ],
            parameters=[common],
            output="screen",
        )
    )
    actions.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "diffdrive_controller",
                "--controller-manager-timeout",
                "60",
            ],
            parameters=[common],
            output="screen",
        )
    )
    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--frame-id",
                "rplidar_link",
                "--child-frame-id",
                "turtlebot4/rplidar_link/rplidar",
            ],
            parameters=[common],
        )
    )
    bridges = [
        ("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "/clock"),
        (
            "/world/maze_escape/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan",
            "sensor_msgs/msg/LaserScan",
            "gz.msgs.LaserScan",
            "/scan",
        ),
        (
            "/world/maze_escape/model/turtlebot4/link/imu_link/sensor/imu/imu",
            "sensor_msgs/msg/Imu",
            "gz.msgs.IMU",
            "/imu",
        ),
        (
            "/model/turtlebot4/pose",
            "tf2_msgs/msg/TFMessage",
            "gz.msgs.Pose_V",
            "/privileged/poses",
        ),
        (
            "/challenge/wall_contacts",
            "ros_gz_interfaces/msg/Contacts",
            "gz.msgs.Contacts",
            "/privileged/contacts",
        ),
    ]
    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[f"{gz}@{ros}[{typ}" for gz, ros, typ, dst in bridges],
            remappings=[(gz, dst) for gz, ros, typ, dst in bridges],
            parameters=[common],
            output="screen",
        )
    )
    actions.append(
        Node(
            package="ail_tb4_maze",
            executable="referee",
            parameters=[
                common,
                {
                    "config": LaunchConfiguration("config_file"),
                    "results_dir": LaunchConfiguration("results_dir"),
                    "time_limit": LaunchConfiguration("time_limit"),
                },
            ],
            output="screen",
        )
    )
    actions.append(Node(package="ail_tb4_maze", executable="gateway", output="screen"))
    if LaunchConfiguration("rviz").perform(context) == "true":
        actions.append(
            GroupAction(
                [
                    SetEnvironmentVariable(
                        "ROS_DOMAIN_ID", LaunchConfiguration("navigation_domain")
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            str(pkg / "launch/navigation_view.launch.py")
                        )
                    ),
                ]
            )
        )

    return actions


def generate_launch_description():
    resources = ":".join(
        [
            str(Path(share("turtlebot4_description")).parent),
            str(Path(share("irobot_create_description")).parent),
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("infrastructure_domain", default_value="61"),
            DeclareLaunchArgument("navigation_domain", default_value="62"),
            DeclareLaunchArgument("gazebo_partition", default_value="ai_legion_maze"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "world_file",
                default_value=str(
                    Path(share("ail_tb4_maze")) / "worlds/maze_escape.sdf"
                ),
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=str(
                    Path(share("ail_tb4_maze")) / "config/challenge.json"
                ),
            ),
            DeclareLaunchArgument("x", default_value="-8.5"),
            DeclareLaunchArgument("y", default_value="-8.5"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "time_limit",
                default_value="0.0",
                description="Simulation seconds; 0 disables the trial deadline",
            ),
            DeclareLaunchArgument("results_dir", default_value=str(RESULTS)),
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", LaunchConfiguration("infrastructure_domain")
            ),
            SetEnvironmentVariable(
                "MAZE_INFRA_DOMAIN", LaunchConfiguration("infrastructure_domain")
            ),
            SetEnvironmentVariable(
                "MAZE_NAV_DOMAIN", LaunchConfiguration("navigation_domain")
            ),
            SetEnvironmentVariable(
                "GZ_PARTITION", LaunchConfiguration("gazebo_partition")
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resources),
            OpaqueFunction(function=setup),
        ]
    )
