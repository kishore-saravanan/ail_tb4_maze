"""TB4 Pledge with observed dead-end avoidance and independent LiDAR safety."""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from visualization_msgs.msg import Marker, MarkerArray

from .dead_end import ClosedCorridors
from .direct_navigation import DirectExplorer
from .pledge import Pledge, PledgeMotion


class PledgeExplorer(DirectExplorer):
    def __init__(self):
        super().__init__()
        self.pledge = Pledge()
        self.motion = PledgeMotion(self.pledge)
        self.algorithm = "Corridor-first Pledge + closed-corridor memory"
        self.strategy_record = {}
        self.corridors = ClosedCorridors()
        self.skipped_corridors = set()
        self.proof_audit = {}
        self.virtual_wall = None
        self.proof_pose = None
        self.avoided_openings = 0
        self.had_proof = False
        self.safety_blocked = False
        self.debug_pub = self.create_publisher(MarkerArray, "/pledge/debug", 10)

    def tick(self):
        self.refresh_planning_map()
        self.virtual_wall = None
        self.proof_pose = None
        ready = self.map is not None and self.update_pose()
        if not ready:
            self.motion_state = "waiting_for_slam_pose"
            if time.monotonic() - self.started > 60.0:
                raise RuntimeError(
                    "SLAM pose did not become ready; inspect controller.log"
                )
        if ready:
            m = self.map
            self.proof_audit = {}
            self.virtual_wall = self.corridors.update(
                np.asarray(m.data).reshape(m.info.height, m.info.width),
                m.info.resolution,
                (m.info.origin.position.x, m.info.origin.position.y),
                (*self.pose, self.heading),
                audit=self.proof_audit,
            )
            self.proof_pose = (*self.pose, self.heading)
        proof = self.virtual_wall is not None
        if proof and not self.had_proof:
            self.avoided_openings += 1
        self.had_proof = proof
        if (
            proof
            and self.pledge.following
            and self.pledge.side(self.points(), 1) > 0.85
        ):
            self.skipped_corridors.add(self.corridors.selected_id)
        self.strategy_record = dict(
            following_wall=self.pledge.following,
            signed_turn_radians=round(self.pledge.net_turn, 4),
            signed_turn_quarters=round(self.pledge.net_turn / (math.pi / 2), 3),
            preferred_heading_radians=self.pledge.preferred,
            initial_corridor_selection=self.pledge.entry_phase,
            total_rotation_since_start=self.pledge.unwrapped
            - self.pledge.initial_heading,
            closed_opening_proven=proof,
            proof_events=self.avoided_openings,
            safety_blocked=self.safety_blocked,
            speed_cap_mps=0.30,
            recoveries=self.motion.recoveries,
            closed_corridors=self.corridors.describe(),
            selected_corridor=self.corridors.selected_id,
            skipped_corridors=len(self.skipped_corridors),
            invalidated_corridors=self.corridors.invalidated,
            proof_audit=self.proof_audit,
        )
        self.report(self.motion_state)
        self.publish_debug()

    def publish_debug(self):
        if self.pose is None:
            return
        text = Marker()
        text.header.frame_id = "map"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "pledge_status"
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x, text.pose.position.y = self.pose
        text.pose.position.y += 0.7
        text.pose.position.z = 0.5
        text.pose.orientation.w = 1.0
        text.scale.z = 0.24
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = (
            f"{self.motion_state}\n"
            f"Signed turns: {self.pledge.net_turn / (math.pi / 2):+.2f} quarters\n"
            f"Closed corridors: {len(self.corridors.records)} | "
            f"Skipped: {len(self.skipped_corridors)}"
        )
        line = Marker()
        line.header = text.header
        line.ns = "pledge"
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.DELETE
        line.pose.orientation.w = 1.0
        line.scale.x = 0.06
        line.color.r = line.color.a = 1.0
        if self.virtual_wall is not None and self.proof_pose is not None:
            line.action = Marker.ADD
            px, py, angle = self.proof_pose
            for x, y in self.virtual_wall[[0, -1]]:
                line.points.append(
                    Point(
                        x=px + math.cos(angle) * x - math.sin(angle) * y,
                        y=py + math.sin(angle) * x + math.cos(angle) * y,
                        z=0.15,
                    )
                )
        edges = Marker()
        edges.header = text.header
        edges.ns = "pledge"
        edges.id = 2
        edges.type = Marker.LINE_LIST
        edges.action = Marker.ADD
        edges.pose.orientation.w = 1.0
        edges.scale.x = 0.08
        edges.color.r, edges.color.g, edges.color.b, edges.color.a = (
            1.0,
            0.25,
            0.15,
            0.9,
        )
        for corridor in self.corridors.records:
            for x, y in (corridor["entry"], corridor["terminal"]):
                edges.points.append(Point(x=float(x), y=float(y), z=0.2))
        self.debug_pub.publish(MarkerArray(markers=[text, line, edges]))

    def control(self):
        if (
            time.monotonic() - self.scan_wall > 0.5
            or time.monotonic() - self.odom_wall > 0.5
            or self.odom_pose is None
            or not self.update_pose()
        ):
            self.publish(0.0, 0.0)
            return
        points = self.points()
        if len(points) < 12:
            self.publish(0.0, 0.0)
            return
        steering = points
        # Transform the most recent proof into the current body frame. It only
        # affects branch choice; real LiDAR always controls collision safety.
        if (
            self.virtual_wall is not None
            and self.proof_pose is not None
            and self.update_pose()
        ):
            px, py, angle = self.proof_pose
            co, si = math.cos(angle), math.sin(angle)
            virtual_world = self.virtual_wall @ np.array([[co, si], [-si, co]]) + [
                px,
                py,
            ]
            co, si = math.cos(self.heading), math.sin(self.heading)
            virtual_body = (virtual_world - self.pose) @ np.array([[co, -si], [si, co]])
            steering = np.concatenate((points, virtual_body))
        v, w = self.motion.step(
            points,
            (*self.pose, self.heading),
            self.get_clock().now().nanoseconds / 1e9,
            steering,
        )
        self.motion_state = self.pledge.state
        self.safety_blocked = self.motion.blocked
        self.publish(v, w)


def main():
    rclpy.init()
    node = PledgeExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish(0.0, 0.0)
        if node.log:
            node.log.close()
        node.destroy_node()
        rclpy.try_shutdown()
