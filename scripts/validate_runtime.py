#!/usr/bin/python3
"""Privileged infrastructure tests. Fixed straight drives only; no maze solver."""

import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[1]
ENV = dict(os.environ, GZ_PARTITION="ai_legion_maze", ROS_DOMAIN_ID="61")


def gz(service, typ, req):
    p = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            service,
            "--reqtype",
            typ,
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "4000",
            "--req",
            req,
        ],
        env=ENV,
        text=True,
        capture_output=True,
        timeout=8,
    )
    assert p.returncode == 0 and "true" in p.stdout, (p.stdout, p.stderr)


def run_case(name, extra, drive=False, collision=False):
    output = ROOT / ".runtime/results" / name
    output.mkdir(exist_ok=True)
    latest = output / "latest.json"
    if latest.exists():
        latest.unlink()
    log = (output / "launch.log").open("w")
    proc = subprocess.Popen(
        [
            str(ROOT / "scripts/launch.sh"),
            "headless:=false",
            "time_limit:=300.0",
            "rviz:=false",
            f"results_dir:={output}",
            *extra,
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    cp = Context()
    cu = Context()
    rclpy.init(context=cp, domain_id=61)
    rclpy.init(context=cu, domain_id=62)
    private = Node("validation_operator", context=cp)
    public = Node("validation_controller_probe", context=cu)
    executor = SingleThreadedExecutor(context=cp)
    executor.add_node(private)
    executor.add_node(public)
    data = {}
    edges = set()
    wallcontacts = []
    gt = []
    odom = []

    def receive(k):
        def cb(m):
            data[k] = m

        return cb

    for key, topic, cls in [
        ("scan", "/scan", LaserScan),
        ("imu", "/imu", Imu),
        ("odom", "/odom", Odometry),
        ("clock", "/clock", Clock),
    ]:
        public.create_subscription(cls, topic, receive(key), qos_profile_sensor_data)
    public.create_subscription(
        Odometry,
        "/odom",
        lambda m: odom.append((m.pose.pose.position.x, m.pose.pose.position.y)),
        qos_profile_sensor_data,
    )
    for topic, qos in [
        ("/tf", 100),
        (
            "/tf_static",
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        ),
    ]:
        public.create_subscription(
            TFMessage,
            topic,
            lambda m: edges.update(
                (t.header.frame_id, t.child_frame_id) for t in m.transforms
            ),
            qos,
        )
    private.create_subscription(
        TFMessage,
        "/privileged/poses",
        lambda m: gt.extend(
            (
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            )
            for t in m.transforms
            if t.child_frame_id == "turtlebot4"
        ),
        qos_profile_sensor_data,
    )
    private.create_subscription(
        Contacts,
        "/privileged/contacts",
        lambda m: wallcontacts.extend(m.contacts),
        qos_profile_sensor_data,
    )
    cmd = public.create_publisher(TwistStamped, "/cmd_vel", 10)

    def spin_until(predicate, seconds=60):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), f"{name}: wait expired; see {output}/launch.log"

    def sim():
        m = data["clock"].clock
        return m.sec + m.nanosec / 1e9

    def call(path):
        client = private.create_client(Trigger, path)
        assert client.wait_for_service(timeout_sec=15)
        future = client.call_async(Trigger.Request())
        spin_until(future.done, 10)
        assert future.result().success, future.result().message

    try:
        required = {
            ("odom", "base_link"),
            ("base_link", "imu_link"),
            ("base_link", "shell_link"),
            ("shell_link", "rplidar_link"),
            ("rplidar_link", "turtlebot4/rplidar_link/rplidar"),
        }
        spin_until(lambda: len(data) == 4 and gt and required <= edges)
        # Allow the dropped official model to settle before checking initial contact.
        settled = sim()
        spin_until(lambda: sim() - settled >= 2)
        assert not wallcontacts, "Robot touches a wall at spawn"
        scan = data["scan"]
        assert (
            len(scan.ranges) == 640 and sum(math.isfinite(v) for v in scan.ranges) > 100
        )
        assert abs(data["imu"].linear_acceleration.z - 9.8) < 0.5
        assert (
            data["odom"].header.frame_id == "odom"
            and data["odom"].child_frame_id == "base_link"
        )
        topics = public.get_topic_names_and_types()
        services = public.get_service_names_and_types()
        allowed = {
            "/scan",
            "/odom",
            "/imu",
            "/cmd_vel",
            "/clock",
            "/tf",
            "/tf_static",
            "/rosout",
            "/parameter_events",
        }
        assert {t for t, _ in topics} <= allowed, topics
        assert not any(
            "referee" in t or "controller_manager" in t or "spawn" in t or "world" in t
            for t, _ in services
        )
        evidence = {
            "sensor_topics": [t for t, _ in topics],
            "scan_samples": len(scan.ranges),
            "scan_finite_samples": sum(math.isfinite(v) for v in scan.ranges),
            "imu_z_acceleration": data["imu"].linear_acceleration.z,
            "tf_edges": sorted(edges),
            "start_ground_truth": gt[-1],
            "start_odom": odom[-1],
            "spawn_wall_contacts": 0,
            "privileged_topics_visible_in_controller_domain": False,
            "services": services,
        }
        call("/referee/start")
        start = sim()
        first_odom = odom[-1]
        first_gt = gt[-1]
        if drive:
            while sim() - start < 8 and not latest.exists():
                msg = TwistStamped()
                msg.twist.linear.x = 0.3
                cmd.publish(msg)
                executor.spin_once(timeout_sec=0.02)
                time.sleep(0.025)
            cmd.publish(TwistStamped())
            stopped = sim()
            spin_until(lambda: sim() - stopped >= 1)
            evidence["odom_displacement_m"] = math.dist(first_odom, odom[-1])
            evidence["ground_truth_displacement_m"] = math.dist(
                first_gt[:2], gt[-1][:2]
            )
            if collision:
                assert wallcontacts, "Physical wall contact not observed"
                call("/referee/abort")
            else:
                spin_until(latest.exists, 40)
        else:
            # Check that pausing preserves simulated time, then accelerate this test.
            gz("/world/maze_escape/control", "gz.msgs.WorldControl", "pause: true")
            for _ in range(20):
                executor.spin_once(timeout_sec=0.05)
            paused = sim()
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            assert abs(sim() - paused) < 0.01
            evidence["paused_wall_seconds"] = 2
            evidence["paused_sim_seconds"] = sim() - paused
            gz(
                "/world/maze_escape/set_physics",
                "gz.msgs.Physics",
                "real_time_factor: 10.0 max_step_size: 0.001",
            )
            gz("/world/maze_escape/control", "gz.msgs.WorldControl", "pause: false")
            spin_until(latest.exists, 600)
        result = json.loads(latest.read_text())
        expected = "failure" if collision else ("success" if drive else "timeout")
        assert result["status"] == expected, result
        if collision:
            assert result["collision_episodes"] >= 1
        if not drive:
            assert 300 <= result["elapsed_sim_seconds"] < 300.1
        evidence["result"] = result
        evidence["passed"] = True
        (output / "validation.json").write_text(json.dumps(evidence, indent=2) + "\n")
        print(json.dumps({"case": name, "passed": True, "result": result}), flush=True)
    finally:
        public.destroy_node()
        private.destroy_node()
        executor.shutdown()
        cu.shutdown()
        cp.shutdown()
        os.kill(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=10)
        log.close()
        time.sleep(2)


if __name__ == "__main__":
    run_case("runtime_collision", [], drive=True, collision=True)
    run_case("runtime_success", ["x:=8.8", "y:=-2.0"], drive=True)
    run_case("runtime_timeout", [])
    print("All runtime validation cases passed.", flush=True)
