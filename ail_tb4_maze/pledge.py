"""Sensor-only Pledge motion for orthogonal mazes, with actual signed turns.

Corridor spacing and maze extent are unknown. A side opening is detected from
LiDAR; the robot clears its wall end before turning and acquiring the next wall.
"""

import math

import numpy as np


def wrap(angle):

    return math.atan2(math.sin(angle), math.cos(angle))


class Pledge:
    def __init__(self):
        self.previous_yaw = None
        self.unwrapped = self.preferred = self.turn_origin = 0.0
        self.following = False
        self.net_turn = 0.0
        self.state = "waiting"
        self.turn_target = None
        self.egress_origin = None
        self.egress_heading = None
        self.advance_origin = None
        self.advance_distance = 0.38
        self.left_armed = False
        self.segment_heading = None
        self.initial_heading = 0.0
        self.initial_left_distance = None
        self.entry_choice_done = False
        self.entry_phase = False

    def observe_heading(self, yaw):
        if self.previous_yaw is None:
            self.preferred = self.unwrapped = self.turn_origin = yaw
            self.segment_heading = yaw
            self.initial_heading = yaw
        else:
            self.unwrapped += wrap(yaw - self.previous_yaw)
        self.previous_yaw = yaw
        self.net_turn = self.unwrapped - self.turn_origin if self.following else 0.0

    @staticmethod
    def front(points):
        ahead = points[(points[:, 0] > 0) & (np.abs(points[:, 1]) < 0.22)]

        return float(ahead[:, 0].min()) if len(ahead) else math.inf

    @staticmethod
    def side(points, sign):
        # A narrow lateral ray sector locates a wall end without looking far
        # into a different junction. A lower quantile lets a proven virtual
        # foreground wall occlude distant returns.
        selected = points[(points[:, 1] * sign > 0) & (np.abs(points[:, 0]) < 0.045)]

        return (
            float(np.quantile(selected[:, 1] * sign, 0.2))
            if len(selected) >= 2
            else math.inf
        )

    def start_turn(self, direction):
        target = self.segment_heading + direction * math.pi / 2
        self.turn_target = self.preferred + round(
            (target - self.preferred) / (math.pi / 2)
        ) * (math.pi / 2)
        self.advance_origin = None
        self.left_armed = False

    def reset_motion(self):
        if self.entry_phase:
            self.entry_phase = False
            self.entry_choice_done = True
        self.turn_target = self.advance_origin = self.egress_origin = None
        self.segment_heading = self.preferred + round(
            (self.unwrapped - self.preferred) / (math.pi / 2)
        ) * (math.pi / 2)
        self.left_armed = False

    def heading_command(self, heading):

        return float(np.clip(2.8 * (heading - self.unwrapped), -0.85, 0.85))

    def step(self, points, pose, steering_points=None):
        self.observe_heading(pose[2])
        points = np.asarray(points)
        if len(points) < 12:
            self.state = "sensor_stop"

            return 0.0, 0.0
        steering = points if steering_points is None else steering_points
        front = self.front(points)
        left = self.side(steering, 1)
        right = self.side(points, -1)
        if self.turn_target is not None:
            error = self.turn_target - self.unwrapped
            if abs(error) > 0.045:
                self.state = (
                    "committed_turn_left" if error > 0 else "committed_turn_right"
                )

                return 0.0, float(np.clip(2.8 * error, -0.85, 0.85))
            self.segment_heading = self.turn_target
            self.egress_heading = self.turn_target
            self.turn_target = None
            self.egress_origin = tuple(pose[:2])
        if self.egress_origin is not None:
            if math.dist(pose[:2], self.egress_origin) < 0.42 and front > 0.46:
                self.state = "clear_junction"

                return 0.20, self.heading_command(self.egress_heading)
            self.egress_origin = None
            if self.entry_phase:
                # Choose the initial preferred compass direction only after
                # entering the observed corridor. Wall-following has not begun;
                # its signed-turn ledger must never be reset this way later.
                self.preferred = self.segment_heading
                self.turn_origin = self.preferred
                self.net_turn = 0.0
                self.entry_phase = False
        if (
            self.advance_origin is not None
            and steering_points is not None
            and left < 0.70
            and self.side(points, 1) > 0.85
        ):
            # Mapping may finish proving the branch while we are clearing its
            # mouth. Cancel the pending turn before entering that dead end.
            self.advance_origin = None
            if self.entry_phase:
                self.entry_phase = False
                self.entry_choice_done = False
            self.left_armed = True
            self.state = "pass_proven_dead_end"

            return 0.27, self.heading_command(self.segment_heading)
        if self.advance_origin is not None:
            if (
                math.dist(pose[:2], self.advance_origin) < self.advance_distance
                and front > 0.32
            ):
                self.state = "advance_to_corner"

                return 0.18, self.heading_command(self.segment_heading)
            self.start_turn(1)
            self.state = "committed_turn_left"

            return 0.0, 0.85
        preferred_error = self.preferred - self.unwrapped
        if (
            self.following
            and abs(self.net_turn) < 0.18
            and abs(preferred_error) < 0.18
            and front > 0.75
        ):
            self.following = False
            self.net_turn = 0.0
        if not self.following:
            if not self.entry_choice_done:
                raw_left = self.side(points, 1)
                if (
                    self.initial_left_distance is None
                    and math.isfinite(raw_left)
                    and raw_left < 7.5
                ):
                    self.initial_left_distance = raw_left
                threshold = (
                    max(
                        self.initial_left_distance + 0.45,
                        self.initial_left_distance * 1.5,
                    )
                    if self.initial_left_distance is not None
                    else math.inf
                )
                if (
                    left > threshold
                    and front > 0.5
                    and abs(wrap(preferred_error)) < 0.12
                ):
                    self.entry_choice_done = True
                    self.entry_phase = True
                    self.advance_origin = tuple(pose[:2])
                    self.advance_distance = min(0.38, max(0.0, front - 0.34))
                    self.state = "select_initial_corridor"

                    return 0.18, self.heading_command(self.preferred)
            if front < 0.43 and abs(wrap(preferred_error)) < 0.18:
                self.entry_choice_done = True
                self.following = True
                self.turn_origin = self.preferred
                self.segment_heading = self.preferred
                self.start_turn(-1)
                self.state = "committed_turn_right"

                return 0.0, -0.85
            self.state = "preferred_direction"

            return 0.30 * max(
                0.0, 1.0 - abs(wrap(preferred_error)) / 0.4
            ), self.heading_command(self.preferred)
        if left < 0.70:
            self.left_armed = True
        if self.left_armed and left > 0.85:
            self.advance_origin = tuple(pose[:2])
            self.advance_distance = min(0.38, max(0.0, front - 0.34))
            self.left_armed = False
            self.state = "advance_to_corner"

            return 0.18, self.heading_command(self.segment_heading)
        if front < 0.43:
            self.start_turn(-1)
            self.state = "committed_turn_right"

            return 0.0, -0.85
        angular = self.heading_command(self.segment_heading)
        if 0.20 < left < 0.85:
            angular += float(np.clip(1.8 * (left - 0.38), -0.45, 0.45))
        if right < 0.30:
            angular += float(np.clip(2.0 * (0.32 - right), 0.0, 0.4))
        self.state = "follow_left_wall"

        return 0.27, float(np.clip(angular, -0.85, 0.85))


class PledgeMotion:
    """Live swept-body guard and bounded recovery; never resets Pledge turns."""

    def __init__(self, pledge):
        from .path_follower import PathFollower

        self.pledge = pledge
        self.guard = PathFollower()
        self.anchor = None
        self.progress_time = None
        self.recovery_until = 0.0
        self.blocked_since = None
        self.blocked = False
        self.recoveries = 0

    def step(self, points, pose, now, steering_points=None):
        self.pledge.observe_heading(pose[2])
        if self.anchor is None or math.dist(pose[:2], self.anchor) > 0.04:
            self.anchor = tuple(pose[:2])
            self.progress_time = now
        if now < self.recovery_until:
            self.pledge.state = "recover_back"
            if self.guard.safe(points, -0.10, 0.0):

                return -0.10, 0.0
            self.pledge.state = "recover_stop"

            return 0.0, 0.0
        v, w = self.pledge.step(points, pose, steering_points)
        self.blocked = False
        command = (0.0, 0.0)
        for scale in (1.0, 0.5, 0.25, 0.0):
            if self.guard.safe(points, v * scale, w):
                command = (v * scale, w)
                self.blocked = scale == 0.0 and v > 0.0
                break
        else:
            self.blocked = True
        if self.blocked:
            if self.blocked_since is None:
                self.blocked_since = now
        else:
            self.blocked_since = None
        if (
            self.blocked_since is not None and now - self.blocked_since > 1.5
        ) or now - self.progress_time > 8.0:
            self.recoveries += 1
            self.recovery_until = now + 1.2
            self.progress_time = now
            self.blocked_since = None
            self.pledge.reset_motion()
            self.pledge.state = "recover_back"

            return (-0.10, 0.0) if self.guard.safe(points, -0.10, 0.0) else (0.0, 0.0)

        return command
