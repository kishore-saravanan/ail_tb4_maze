"""Right-hand (or mirrored left-hand) LiDAR wall follower, independent of ROS."""

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Decision:
    forward: float
    angular: float
    state: str
    front: float
    side: float
    wall_angle: float


class WallFollower:
    def __init__(self, desired_distance=0.45, side="right"):
        if side not in ("right", "left"):
            raise ValueError("side must be right or left")
        self.distance = desired_distance
        self.sign = 1.0 if side == "right" else -1.0
        self.turning = False
        self.attached = False
        self.acquisition = None
        self.target_heading = None
        self.corner = None
        self.right_armed = True

    @staticmethod
    def sector(points, angle, width=8.0):
        bearing = np.arctan2(points[:, 1], points[:, 0])
        delta = np.arctan2(
            np.sin(bearing - math.radians(angle)), np.cos(bearing - math.radians(angle))
        )
        selected = points[np.abs(delta) < math.radians(width)]
        return (
            float(np.median(np.linalg.norm(selected, axis=1)))
            if len(selected) >= 2
            else math.inf
        )

    def step(self, scan_points, heading=None, position=None):
        points = np.asarray(scan_points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 12:
            return Decision(0.0, 0.0, "sensor_stop", 0.0, 0.0, 0.0)
        points = points[np.isfinite(points).all(axis=1)].copy()
        if len(points) < 12:
            return Decision(0.0, 0.0, "sensor_stop", 0.0, 0.0, 0.0)
        points[:, 1] *= self.sign
        side = self.sector(points, -90)
        diagonal = self.sector(points, -45)
        rear = self.sector(points, -135)
        ahead = points[(points[:, 0] > 0) & (np.abs(points[:, 1]) < 0.29)]
        front = float(ahead[:, 0].min()) if len(ahead) else math.inf
        phase = None
        if heading is not None:
            heading *= self.sign
            if self.acquisition is None:
                self.acquisition = "face"
                self.target_heading = heading - math.pi / 2
            if self.acquisition in ("face", "align"):
                error = (self.target_heading - heading + math.pi) % (
                    2 * math.pi
                ) - math.pi
                if abs(error) > 0.08:
                    phase = (
                        0.0,
                        float(np.clip(1.8 * error, -0.85, 0.85)),
                        "acquire_turn_right"
                        if self.acquisition == "face"
                        else "align_wall",
                    )
                elif self.acquisition == "face":
                    self.acquisition = "approach"
                else:
                    self.acquisition = "done"
                    self.attached = True
            if self.acquisition == "approach":
                if front > 0.55:
                    phase = (0.20, 0.0, "approach_wall")
                else:
                    self.acquisition = "align"
                    self.target_heading = heading + math.pi / 2
                    phase = (0.0, 0.85, "align_wall")
        corner_approach = (
            math.isfinite(side) and side < 1.0 and diagonal > max(1.2, side * 2.2)
        )
        opening = side > 1.0
        take_right = self.attached and opening and front > 0.36
        if take_right:
            self.turning = False
        if self.turning and front > 0.80:
            self.turning = False
        if front < (0.36 if corner_approach else 0.60) and not take_right:
            self.turning = True
        alpha = 0.0
        if side < 0.9:
            self.attached = True
            if self.corner is None:
                self.right_armed = True
        if position is not None and heading is not None and self.acquisition == "done":
            if self.corner is None and opening and self.right_armed:
                self.corner = dict(
                    phase="advance",
                    origin=tuple(position),
                    heading=heading,
                    target=heading - math.pi / 2,
                )
                self.right_armed = False
            if self.corner is not None:
                corner = self.corner
                if corner["phase"] == "advance":
                    if math.dist(position, corner["origin"]) < 0.32 and front > 0.40:
                        error = (corner["heading"] - heading + math.pi) % (
                            2 * math.pi
                        ) - math.pi
                        phase = (
                            0.12,
                            float(np.clip(2 * error, -0.5, 0.5)),
                            "advance_corner",
                        )
                    else:
                        corner["phase"] = "turn"
                if corner["phase"] == "turn":
                    error = (corner["target"] - heading + math.pi) % (
                        2 * math.pi
                    ) - math.pi
                    if abs(error) > 0.08:
                        phase = (
                            0.0,
                            float(np.clip(1.8 * error, -0.85, 0.85)),
                            "turn_right",
                        )
                    else:
                        self.corner = None
            if phase is not None:
                self.turning = False
        if phase is not None:
            speed, angular, state = phase
        elif self.turning:
            speed, angular, state = 0.0, 0.85, "turn_left"
        elif not self.attached:
            speed, angular, state = 0.24, -0.16, "seek_wall"
        elif take_right or side > 1.0 or not math.isfinite(side):
            # An opening on the followed side: round the corner into it.
            speed, angular, state = 0.17, -0.50, "turn_right"
        elif corner_approach and math.isfinite(rear) and rear < 2.5:
            theta = math.pi / 4
            alpha = math.atan2(side - rear * math.cos(theta), rear * math.sin(theta))
            perpendicular = side * math.cos(alpha)
            angular = float(
                np.clip(
                    -2.1 * alpha + 1.7 * (self.distance - perpendicular), -0.85, 0.85
                )
            )
            speed = 0.16
            state = "advance_corner"
        elif math.isfinite(diagonal) and diagonal < 2.5:
            theta = math.pi / 4
            alpha = math.atan2(
                diagonal * math.cos(theta) - side, diagonal * math.sin(theta)
            )
            perpendicular = side * math.cos(alpha)
            angular = float(
                np.clip(
                    -2.1 * alpha + 1.7 * (self.distance - perpendicular), -0.85, 0.85
                )
            )
            speed = 0.27 * max(0.35, 1.0 - abs(angular) * 0.65)
            state = "follow_wall"
        else:
            speed, angular, state = (
                0.16,
                float(np.clip(1.7 * (self.distance - side), -0.6, 0.6)),
                "follow_wall",
            )
        nearest = float(np.linalg.norm(points, axis=1).min())
        if nearest < 0.245:
            return Decision(0.0, 0.0, "obstacle_stop", front, side, alpha)
        # Predict the swept circular body, rather than checking a single ray.
        for candidate in [speed, speed * 0.5, 0.0]:
            t = np.linspace(0.0, 1.0, 13)
            if abs(angular) > 0.01:
                xx = candidate / angular * np.sin(angular * t)
                yy = candidate / angular * (1.0 - np.cos(angular * t))
            else:
                xx = candidate * t
                yy = np.zeros_like(t)
            separation = (points[:, 0, None] - xx) ** 2 + (points[:, 1, None] - yy) ** 2
            if float(separation.min()) > 0.285**2 or candidate == 0.0:
                return Decision(
                    candidate,
                    angular * self.sign,
                    state
                    if candidate or phase is not None
                    else ("turn_left" if angular > 0 else "turn_right"),
                    front,
                    side,
                    alpha,
                )
        return Decision(0.0, 0.0, "obstacle_stop", front, side, alpha)

    @staticmethod
    def safe_reverse(points):
        points = np.asarray(points, dtype=float)
        if len(points) < 12:
            return 0.0
        steps = np.linspace(0.0, 0.15, 16)
        separation = (points[:, 0, None] + steps) ** 2 + points[:, 1, None] ** 2
        return -0.10 if float(np.min(separation)) > 0.27**2 else 0.0
