#!/usr/bin/python3
"""Full timeout check on separate ROS domains and Gazebo partition.
Leaves the user's live GUI and simulation untouched.
"""

import json
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.physics_pb2 import Physics
from gz.transport13 import Node as GzNode
from gz.transport13 import NodeOptions
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / ".runtime/results/isolated_gui_timeout"
OUT.mkdir(exist_ok=True)
for name in ["world.sdf", "challenge.json"]:
    shutil.copy2(ROOT / ".runtime/state" / name, OUT / name)
latest = OUT / "latest.json"
if latest.exists():
    latest.unlink()
c = json.loads((OUT / "challenge.json").read_text())
s = c["start"]
log = (OUT / "launch.log").open("w")
# Use direct launch to avoid overwriting the live setup window's launch PID file.
env = dict(os.environ, ROS_DOMAIN_ID="63")
proc = subprocess.Popen(
    [
        "ros2",
        "launch",
        "ail_tb4_maze",
        "challenge.launch.py",
        "headless:=false",
        "time_limit:=300.0",
        "rviz:=false",
        "infrastructure_domain:=63",
        "navigation_domain:=64",
        "gazebo_partition:=ai_legion_validation",
        f"world_file:={OUT}/world.sdf",
        f"config_file:={OUT}/challenge.json",
        f"results_dir:={OUT}",
        f"x:={s['x']}",
        f"y:={s['y']}",
    ],
    env=env,
    stdout=log,
    stderr=subprocess.STDOUT,
    start_new_session=True,
)
rclpy.init(domain_id=63)
n = Node("isolated_timeout_validation")
received = set()
for name, topic, cls in [
    ("clock", "/clock", Clock),
    ("scan", "/scan", LaserScan),
    ("imu", "/imu", Imu),
]:
    n.create_subscription(
        cls, topic, lambda m, name=name: received.add(name), qos_profile_sensor_data
    )


def spin(pred, seconds):
    end = time.monotonic() + seconds
    while not pred() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.03)
    assert pred(), "Timeout validation stalled; see " + str(OUT / "launch.log")


try:
    spin(lambda: len(received) == 3, 60)
    client = n.create_client(Trigger, "/referee/start")
    assert client.wait_for_service(timeout_sec=10)
    fut = client.call_async(Trigger.Request())
    spin(fut.done, 10)
    assert fut.result().success, fut.result().message
    options = NodeOptions()
    options.partition = "ai_legion_validation"
    gz = GzNode(options)
    ok, resp = gz.request(
        "/world/maze_escape/set_physics",
        Physics(real_time_factor=10, max_step_size=0.001),
        Physics,
        Boolean,
        5000,
    )
    assert ok and resp.data
    spin(latest.exists, 600)
    result = json.loads(latest.read_text())
    assert (
        result["status"] == "timeout" and 300 <= result["elapsed_sim_seconds"] < 300.1
    ), result
    (OUT / "validation.json").write_text(
        json.dumps(
            {
                "passed": True,
                "result": result,
                "separate_domains": [63, 64],
                "separate_partition": "ai_legion_validation",
                "live_gui_untouched": True,
            },
            indent=2,
        )
    )
    print(json.dumps(result), flush=True)
finally:
    n.destroy_node()
    rclpy.try_shutdown()
    os.kill(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=10)
    log.close()
