"""Run the navigation launch as part of the GUI-owned solver process group."""

import signal
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("maze_navigation_manager")
    node.declare_parameter("recording_path", "")
    path = node.get_parameter("recording_path").value
    node.declare_parameter("navigation_backend", "direct")
    backend = node.get_parameter("navigation_backend").value
    launch = {
        "direct": "direct_navigation.launch.py",
        "pledge": "pledge_navigation.launch.py",
        "nav2": "navigation.launch.py",
    }[backend]
    args = ["ros2", "launch", "ail_tb4_maze", launch, f"recording_path:={path}"]
    node.declare_parameter("navigation_params_file", "")
    params = node.get_parameter("navigation_params_file").value
    if params:
        args.append(f"params_file:={params}")
    process = subprocess.Popen(args)
    try:
        while rclpy.ok() and process.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if rclpy.ok():
            raise RuntimeError(
                "Navigation stack stopped unexpectedly; inspect controller.log"
            )
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        node.destroy_node()
        rclpy.try_shutdown()
