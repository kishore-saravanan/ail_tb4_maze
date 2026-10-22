#!/usr/bin/python3
"""Reproducible, ground-truth-scored trial; controller receives sensors only."""

import argparse
import fcntl
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.physics_pb2 import Physics
from gz.msgs10.world_control_pb2 import WorldControl
from gz.transport13 import Node as GzNode
from gz.transport13 import NodeOptions
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from ail_tb4_maze.paths import PACKAGE_ROOT, PACKAGE_SHARE, RESULTS
from ail_tb4_maze.world_factory import generate


def stop(proc):
    if proc is None or proc.poll() is not None:

        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=6)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--preset", choices=["official", "random"], default="official")
    p.add_argument("--size", type=int, default=20)
    p.add_argument("--cells", type=int, default=10)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--rtf", type=float, default=1.0, help="Simulation speed target")
    p.add_argument(
        "--time-limit",
        type=float,
        default=0.0,
        help="Trial deadline in simulated seconds; 0 means unlimited (default)",
    )
    p.add_argument(
        "--wall-timeout",
        type=float,
        default=0.0,
        help="Optional wall-clock watchdog in seconds; 0 disables it (default)",
    )
    p.add_argument("--backend", choices=["nav2", "direct", "pledge"], default="direct")
    p.add_argument(
        "--nav-params",
        default="",
        help="Alternative navigation YAML; default is the packaged configuration",
    )
    display = p.add_mutually_exclusive_group()
    display.add_argument(
        "--visible", dest="visible", action="store_true", help="Show Gazebo (default)"
    )
    display.add_argument(
        "--headless",
        dest="visible",
        action="store_false",
        help="Run without Gazebo GUI only when explicitly requested",
    )
    p.set_defaults(visible=True)
    p.add_argument(
        "--close-on-completion",
        action="store_true",
        help=(
            "Close visible simulation after scoring; "
            "default leaves it paused for inspection"
        ),
    )
    p.add_argument("--infra-domain", type=int, default=63)
    p.add_argument("--nav-domain", type=int, default=64)
    args = p.parse_args()
    if args.infra_domain == args.nav_domain:
        p.error("Infrastructure and navigation domains must differ")
    for name in ("time_limit", "wall_timeout"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) < 0:
            p.error(name.replace("_", "-") + " must be finite and nonnegative")
    RESULTS.mkdir(parents=True, exist_ok=True)
    domain_lock = (
        RESULTS / f"domains_{args.infra_domain}_{args.nav_domain}.lock"
    ).open("w")
    try:
        fcntl.flock(domain_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        p.error("This experiment domain pair is already in use")
    output = RESULTS / (
        "experiment_" + time.strftime("%Y%m%d_%H%M%S") + "_" + str(time.monotonic_ns())
    )
    output.mkdir(parents=True)
    nav_params = (
        Path(args.nav_params).resolve()
        if args.nav_params
        else PACKAGE_ROOT / "config/navigation.yaml"
    )
    code_sha256 = {
        name: hashlib.sha256((PACKAGE_ROOT / name).read_bytes()).hexdigest()
        for name in [
            "scripts/run_experiment.py",
            "ail_tb4_maze/scoring.py",
            "ail_tb4_maze/referee.py",
            "launch/challenge.launch.py",
            "launch/direct_navigation.launch.py",
            "navigation_interface/controller.py",
            "ail_tb4_maze/frontier_planner.py",
            "ail_tb4_maze/frontier_node.py",
            "ail_tb4_maze/navigation_manager.py",
            "ail_tb4_maze/velocity_adapter.py",
            "ail_tb4_maze/sensor_map.py",
            "ail_tb4_maze/direct_navigation.py",
            "ail_tb4_maze/path_follower.py",
            "ail_tb4_maze/pledge.py",
            "ail_tb4_maze/pledge_navigation.py",
            "ail_tb4_maze/dead_end.py",
            "config/navigation.yaml",
            "config/slam.yaml",
            "config/navigation.rviz",
            "ail_tb4_maze/navigation_view.py",
            "ail_tb4_maze/display_model.py",
            "launch/navigation_view.launch.py",
        ]
    }
    code_sha256["selected_navigation_parameters"] = hashlib.sha256(
        nav_params.read_bytes()
    ).hexdigest()
    shutil.copyfile(nav_params, output / "navigation.yaml")
    shutil.copyfile(PACKAGE_ROOT / "config/slam.yaml", output / "slam.yaml")
    scene = output / "scene"
    c = generate(PACKAGE_SHARE, scene, args.size, args.cells, args.seed, args.preset)
    partition = "maze_experiment_" + str(os.getpid())
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.infra_domain), GZ_PARTITION=partition)
    rclpy.init(domain_id=args.infra_domain)
    node = Node("experiment_operator")
    # Evaluation evidence stays on the infrastructure domain, outside the
    # navigation gateway. Never expose this trajectory to the solver.
    truth_log = (output / "operator_trajectory.jsonl").open("w")
    last_truth_time = [-float("inf")]

    def record_truth(msg):
        for transform in msg.transforms:
            if transform.child_frame_id != c["robot"]:
                continue
            stamp = transform.header.stamp
            now = stamp.sec + stamp.nanosec / 1e9
            if now - last_truth_time[0] < 0.5:

                return
            last_truth_time[0] = now
            p = transform.transform.translation
            truth_log.write(
                json.dumps(dict(sim_seconds=now, pose=[p.x, p.y, p.z])) + "\n"
            )
            truth_log.flush()
            break

    _truth_subscription = node.create_subscription(
        TFMessage, "/privileged/poses", record_truth, qos_profile_sensor_data
    )
    sim = solver = rviz = None

    def call(path):
        client = node.create_client(Trigger, path)
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError("Service unavailable: " + path)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or not future.result().success:
            raise RuntimeError("Service failed: " + path)

    def status():
        try:

            return json.loads((output / "status.json").read_text())
        except (OSError, ValueError):

            return {}

    try:
        # Distinct DDS domains AND partition prevent modifying an operator's GUI.
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        if any(
            name == "/referee/start" for name, _ in node.get_service_names_and_types()
        ):
            raise RuntimeError(
                "Infrastructure domain is already in use; choose another domain pair"
            )
        with (output / "simulation.log").open("w") as log:
            s = c["start"]
            sim = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "ail_tb4_maze",
                    "challenge.launch.py",
                    f"headless:={str(not args.visible).lower()}",
                    "rviz:=false",
                    f"time_limit:={args.time_limit}",
                    f"infrastructure_domain:={args.infra_domain}",
                    f"navigation_domain:={args.nav_domain}",
                    f"gazebo_partition:={partition}",
                    f"world_file:={scene}/world.sdf",
                    f"config_file:={scene}/challenge.json",
                    f"results_dir:={output}",
                    f"x:={s['x']}",
                    f"y:={s['y']}",
                ],
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            end = time.monotonic() + 75
            while time.monotonic() < end and not status().get("pose_ready"):
                if sim.poll() is not None:
                    raise RuntimeError("Simulator stopped; inspect simulation.log")
                time.sleep(0.2)
            if not status().get("pose_ready"):
                raise RuntimeError("Simulator pose never became ready")
            if args.visible:
                with (output / "rviz.log").open("w") as rviz_log:
                    rviz = subprocess.Popen(
                        ["ros2", "launch", "ail_tb4_maze", "navigation_view.launch.py"],
                        env=dict(env, ROS_DOMAIN_ID=str(args.nav_domain)),
                        stdout=rviz_log,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
            options = NodeOptions()
            options.partition = partition
            gz = GzNode(options)
            ok, response = gz.request(
                "/world/maze_escape/set_physics",
                Physics(real_time_factor=args.rtf, max_step_size=0.001),
                Physics,
                Boolean,
                7000,
            )
            if not ok or not response.data:
                raise RuntimeError("Could not set simulation speed")
            call("/referee/start")
            with (output / "controller.log").open("w") as control_log:
                solver = subprocess.Popen(
                    [
                        "/usr/bin/python3",
                        str(PACKAGE_ROOT / "navigation_interface/controller.py"),
                        "--ros-args",
                        "-p",
                        "use_sim_time:=true",
                        "-p",
                        f"recording_path:={output}/exploration.jsonl",
                        "-p",
                        f"navigation_params_file:={nav_params}",
                        "-p",
                        f"navigation_backend:={args.backend}",
                    ],
                    env=dict(env, ROS_DOMAIN_ID=str(args.nav_domain)),
                    stdout=control_log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                end = time.monotonic() + args.wall_timeout
                previous = -30
                while args.wall_timeout == 0 or time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    s = status()
                    if s.get("status") in ("success", "failure", "timeout"):
                        break
                    if solver.poll() is not None:
                        raise RuntimeError("Solver stopped; inspect controller.log")
                    if sim.poll() is not None:
                        raise RuntimeError("Simulator stopped; inspect simulation.log")
                    if s.get("elapsed_sim_seconds", 0) - previous >= 30:
                        print(
                            json.dumps(
                                {
                                    key: s.get(key)
                                    for key in [
                                        "status",
                                        "elapsed_sim_seconds",
                                        "collision_episodes",
                                        "distance_meters",
                                    ]
                                }
                            ),
                            flush=True,
                        )
                        previous = s.get("elapsed_sim_seconds", 0)
                    time.sleep(0.5)
                else:
                    raise RuntimeError("Trial exceeded wall-clock watchdog")
                report = dict(
                    code_sha256=code_sha256,
                    parameters=vars(args),
                    algorithm={
                        "direct": (
                            "Live-map frontier routing + observed-goal cancellation"
                        ),
                        "pledge": "Corridor-first Pledge + closed-corridor memory",
                        "nav2": "SLAM Toolbox + reachable frontiers + Nav2 A*",
                    }[args.backend],
                    navigation_parameters=nav_params.name,
                    controller_receives_preloaded_maze_or_exit=False,
                    result=s,
                    telemetry="exploration.jsonl",
                )
                (output / "experiment.json").write_text(
                    json.dumps(report, indent=2) + "\n"
                )
                print("RESULT " + json.dumps(report), flush=True)
                print("Evidence: " + str(output), flush=True)
                if args.visible and not args.close_on_completion:
                    ok, response = gz.request(
                        "/world/maze_escape/control",
                        WorldControl(pause=True),
                        WorldControl,
                        Boolean,
                        3000,
                    )
                    print(
                        "Trial finished. "
                        + (
                            "Simulation paused."
                            if ok and response.data
                            else (
                                "Pause request failed; "
                                "referee has disabled robot motion."
                            )
                        )
                        + " Keeping Gazebo and RViz open for inspection. "
                        "Close Gazebo or press Ctrl+C to finish.",
                        flush=True,
                    )
                    while sim.poll() is None:
                        rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        stop(solver)
        stop(rviz)
        stop(sim)
        truth_log.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
