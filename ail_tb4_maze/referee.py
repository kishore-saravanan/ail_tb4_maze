"""Privileged referee. Results are local files, never controller-domain topics."""

import json
import math
import os
import time
from pathlib import Path

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from .scoring import Scoring


class Referee(Node):
    def __init__(self):
        super().__init__("challenge_referee")
        self.declare_parameter("config", "")
        self.declare_parameter("results_dir", "")
        self.declare_parameter("time_limit", 0.0)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.c = json.loads(Path(self.get_parameter("config").value).read_text())
        self.c["time_limit"] = self.get_parameter("time_limit").value
        self.score = Scoring(self.c)
        self.pose = None
        self.pose_time = None
        self.last_wall_pose = time.monotonic()
        self.output = Path(self.get_parameter("results_dir").value)
        self.output.mkdir(parents=True, exist_ok=True)
        self.run_id = time.strftime("%Y%m%dT%H%M%S") + "-" + str(os.getpid())
        self.saved = False
        self.last_status_write = 0.0
        self.last_active = None
        self.active_publisher = self.create_publisher(
            Bool,
            "/privileged/trial_active",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            TFMessage, "/privileged/poses", self.on_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            Contacts, "/privileged/contacts", self.on_contacts, qos_profile_sensor_data
        )
        self.create_service(Trigger, "/referee/start", self.start)
        self.create_service(Trigger, "/referee/abort", self.abort)
        self.create_service(Trigger, "/referee/reset", self.reset)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def validate_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == "time_limit" and (
                not isinstance(parameter.value, (int, float))
                or not math.isfinite(parameter.value)
                or parameter.value < 0
            ):
                return SetParametersResult(
                    successful=False,
                    reason="Time limit must be nonnegative; 0 means unlimited",
                )
        return SetParametersResult(successful=True)

    def start(self, request, response):
        if self.pose is None or time.monotonic() - self.last_wall_pose > 2:
            response.success = False
            response.message = "No fresh ground-truth pose; wait for spawn."
        else:
            if self.score.status == "waiting":
                # Capture the selected limit once, before this attempt starts.
                self.c["time_limit"] = self.get_parameter("time_limit").value
            response.success = self.score.start(self.now(), self.pose)
            response.message = (
                "Started simulation-time trial"
                if response.success
                else "Restart launch for a fresh trial"
            )
        self.persist()
        return response

    def reset(self, request, response):
        if self.score.status == "running":
            self.score.finish("failure", "layout_or_trial_reset")
            self.persist()
        self.c = json.loads(Path(self.get_parameter("config").value).read_text())
        self.c["time_limit"] = self.get_parameter("time_limit").value
        self.score = Scoring(self.c)
        self.pose = None
        self.pose_time = None
        self.run_id = (
            time.strftime("%Y%m%dT%H%M%S")
            + "-"
            + str(os.getpid())
            + "-"
            + str(time.monotonic_ns())
        )
        self.saved = False
        self.last_status_write = 0.0
        self.persist()
        response.success = True
        response.message = "Fresh trial ready; waiting for pose"
        return response

    def abort(self, request, response):
        self.score.finish("failure", "operator_abort")
        self.persist()
        response.success = True
        response.message = "Aborted; scoring saved"
        return response

    def on_pose(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == self.c["robot"]:
                p = t.transform.translation
                stamp = t.header.stamp
                now = stamp.sec + stamp.nanosec / 1e9
                self.pose = (p.x, p.y, p.z)
                self.pose_time = now
                self.last_wall_pose = time.monotonic()
                self.score.update(now, *self.pose)
                self.persist()
                break

    def on_contacts(self, msg):
        for c in msg.contacts:
            names = (c.collision1.name, c.collision2.name)
            if any(self.c["robot"] in n for n in names) and any(
                "border" in n or "obstacles" in n for n in names
            ):
                # Aggregate multiple collision shapes of the same robot-wall link.
                wall = next(n for n in names if "border" in n or "obstacles" in n)
                self.score.contact(self.now(), "::".join(wall.split("::")[:2]))

    def tick(self):
        self.score.tick(self.now())
        if (
            self.score.status == "running"
            and self.pose_time is not None
            and self.now() - self.pose_time > 2
        ):
            self.score.finish("failure", "ground_truth_stream_lost")
        self.persist()

    def persist(self):
        active = self.score.status == "running"
        if active != self.last_active:
            self.active_publisher.publish(Bool(data=active))
            self.last_active = active
        if time.monotonic() - self.last_status_write >= 0.2:
            status = self.score.result()
            status.update(
                run_id=self.run_id,
                pose_ready=self.pose is not None,
                result_file=str(self.output / (self.run_id + ".json"))
                if self.saved
                else None,
            )
            tmp = self.output / "status.json.tmp"
            tmp.write_text(json.dumps(status))
            tmp.replace(self.output / "status.json")
            self.last_status_write = time.monotonic()
        if self.score.status in ("success", "timeout", "failure") and not self.saved:
            result = self.score.result()
            result["run_id"] = self.run_id
            result["world"] = self.c["world"]
            result["layout"] = {
                k: self.c.get(k)
                for k in [
                    "preset",
                    "seed",
                    "maze_size_m",
                    "grid_cells",
                    "layout_stats",
                    "clearance_validation",
                ]
            }
            import hashlib

            result["config_sha256"] = hashlib.sha256(
                json.dumps(self.c, sort_keys=True).encode()
            ).hexdigest()
            result["collision_source"] = "Gazebo wall contact sensors"
            result["distance_source"] = "sampled ground truth planar path"
            data = json.dumps(result, indent=2) + "\n"
            for filename in (self.run_id + ".json", "latest.json"):
                tmp = self.output / (filename + ".tmp")
                tmp.write_text(data)
                tmp.replace(self.output / filename)
            self.saved = True
            self.get_logger().info(data)


def main():
    rclpy.init()
    node = Referee()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node.score.status == "running":
            node.score.finish("failure", "infrastructure_shutdown")
            node.persist()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
