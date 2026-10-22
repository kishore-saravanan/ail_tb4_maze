"""Deterministic waypoint tracking with live swept-circle LiDAR checks."""

import math

import numpy as np


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def segment_clear(a, b, grid, resolution, origin, clearance=None, margin=0.25):
    from scipy import ndimage

    if clearance is None:
        clearance = ndimage.distance_transform_edt(grid < 50) * resolution
    count = max(2, math.ceil(math.dist(a, b) / (resolution * 0.4)))
    p = np.linspace(a, b, count)
    ij = np.floor((p - np.asarray(origin)) / resolution).astype(int)
    cc, rr = ij.T
    if np.any((rr < 0) | (rr >= grid.shape[0]) | (cc < 0) | (cc >= grid.shape[1])):
        return False
    return bool(
        np.all(
            (grid[rr, cc] >= 0) & (grid[rr, cc] < 30) & (clearance[rr, cc] >= margin)
        )
    )


def simplify_route(route, grid, resolution, origin, clearance=None):
    if not route:
        return []
    result = [route[0]]
    i = 0
    while i < len(route) - 1:
        best = i + 1
        for j in range(i + 2, len(route)):
            if math.dist(route[i], route[j]) > 2.0:
                break
            if segment_clear(
                route[i], route[j], grid, resolution, origin, clearance, margin=0.28
            ):
                best = j
        result.append(route[best])
        i = best
    return result


class PathFollower:
    """Track map-frame waypoints with independent swept-body LiDAR checks."""

    def __init__(self):
        self.route = []
        self.index = 0
        self.turning = False

    def set_route(self, route, pose):
        self.route = list(route)
        self.index = 0
        self.turning = False
        self.advance(pose)

    def advance(self, pose):
        while self.index < len(self.route):
            distance = math.dist(pose[:2], self.route[self.index])
            if distance < 0.065:
                self.index += 1
                continue
            if distance < 0.25 and self.index + 1 < len(self.route):
                incoming = np.asarray(self.route[self.index]) - np.asarray(pose[:2])
                outgoing = np.asarray(self.route[self.index + 1]) - np.asarray(
                    self.route[self.index]
                )
                if np.dot(incoming, outgoing) > 0.985 * np.linalg.norm(
                    incoming
                ) * np.linalg.norm(outgoing):
                    self.index += 1
                    continue
            break

    @property
    def done(self):
        return self.index >= len(self.route)

    @staticmethod
    def safe(points, v, w, horizon=None, radius=0.235):
        if not len(points):
            return False
        points = np.asarray(points)
        if horizon is None:
            horizon = 0.25 + abs(v) / (2 * 0.6)
        times = np.linspace(0.05, max(0.05, horizon), 12)
        if abs(v) < 1e-5:
            return float(np.linalg.norm(points, axis=1).min()) > 0.205
        if abs(w) < 1e-4:
            centers = np.column_stack([v * times, np.zeros(len(times))])
        else:
            centers = np.column_stack(
                [v / w * np.sin(w * times), v / w * (1 - np.cos(w * times))]
            )
        minimum = np.linalg.norm(points[:, None, :] - centers[None, :, :], axis=2).min(
            axis=0
        )
        current = float(np.linalg.norm(points, axis=1).min())
        if current < radius and current > 0.205:
            return bool(
                minimum.min() >= current - 0.002 and minimum[-1] > current + 0.004
            )
        return bool(minimum.min() >= radius)

    def step(self, pose, points):
        self.advance(pose)
        if self.done:
            return 0.0, 0.0, "arrived"
        target = self.route[self.index]
        distance = math.dist(pose[:2], target)
        error = wrap(math.atan2(target[1] - pose[1], target[0] - pose[0]) - pose[2])
        if self.turning and abs(error) < 0.10:
            self.turning = False
        if abs(error) > 0.35:
            self.turning = True
        if self.turning:
            w = float(np.clip(3.2 * error, -1.3, 1.3))
            return (
                (0.0, w, "turn") if self.safe(points, 0.0, w) else (0.0, 0.0, "blocked")
            )
        v = min(0.30, math.sqrt(2 * 0.6 * max(0.005, distance - 0.04))) * max(
            0.4, 1.0 - abs(error)
        )
        w = float(np.clip(2.5 * error, -0.7, 0.7))
        for scale in [1.0, 0.5, 0.25, 0.125]:
            if self.safe(points, v * scale, w):
                return v * scale, w, "drive"
        if abs(w) > 0.05 and self.safe(points, 0.0, w):
            return 0.0, w, "align"
        return 0.0, 0.0, "blocked"
