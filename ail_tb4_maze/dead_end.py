"""Sensor-map proof that a doorway leads only to a closed robot-sized corridor.

Occupancy maps contain thin observed wall surfaces, not filled wall volumes.
Use configuration space: inflate observed walls by the robot clearance radius.
Unknown space outside that inflation remains a possible continuation and prevents
pruning. Tiny unseen pixels behind a known impassable wall are irrelevant.
"""

import math

import numpy as np
from scipy import ndimage

CONNECTIVITY = np.ones((3, 3), dtype=bool)


def closed_left_opening(cells, resolution, origin, pose, audit=None, details=None):
    cells = np.asarray(cells)
    h, w = cells.shape
    co, si = math.cos(pose[2]), math.sin(pose[2])
    clearance = ndimage.distance_transform_edt(cells < 50) * resolution
    blocked = clearance < 0.235
    observed_free = (cells >= 0) & (cells < 30)
    possible = ~blocked

    def reject(reason):
        if audit is not None:
            audit[reason] = audit.get(reason, 0) + 1

    def grid(x, y):
        wx = pose[0] + co * x - si * y
        wy = pose[1] + si * x + co * y

        return int(math.floor((wy - origin[1]) / resolution)), int(
            math.floor((wx - origin[0]) / resolution)
        )

    xs = np.arange(-0.9, 8.01, resolution * 0.5)
    for depth in (0.40, 0.50, 0.60, 0.70):
        coords = [grid(x, depth) for x in xs]
        inside = np.array([0 <= y < h and 0 <= x < w for y, x in coords])
        wall = np.array(
            [blocked[y, x] if ok else False for (y, x), ok in zip(coords, inside)]
        )
        free = np.array(
            [
                observed_free[y, x] and possible[y, x] if ok else False
                for (y, x), ok in zip(coords, inside)
            ]
        )
        labels, count = ndimage.label(free)
        for label in range(1, count + 1):
            indices = np.flatnonzero(labels == label)
            a, b = int(indices[0]), int(indices[-1])
            if a == 0 or b == len(xs) - 1 or not wall[a - 1] or not wall[b + 1]:
                reject("unbounded_doorway")
                continue
            low, high = xs[a], xs[b]
            # These are robot-centre widths after wall inflation, not raw
            # corridor widths. A 76 cm passage still has ~29 cm of safe width.
            if high - low < 0.10 or low > 0.35 or high < -0.05:
                reject("doorway_not_near_robot")
                continue
            cut = np.zeros((h, w), dtype=bool)
            for x in np.arange(xs[a - 1] - 0.08, xs[b + 1] + 0.08, resolution * 0.35):
                for y in np.arange(depth - 0.08, depth + 0.09, resolution * 0.35):
                    row, col = grid(x, y)
                    if 0 <= row < h and 0 <= col < w:
                        cut[row, col] = True
            walkable = observed_free & possible & ~cut
            components, _ = ndimage.label(walkable, CONNECTIVITY)
            sy, sx = grid((low + high) / 2, depth + 0.23)
            ry, rx = grid(0.0, 0.0)
            if not (0 <= sy < h and 0 <= sx < w and 0 <= ry < h and 0 <= rx < w):
                reject("seed_outside_map")
                continue
            component = int(components[sy, sx])
            if components[ry, rx] == 0:
                reject("parent_not_observed")
                continue
            if component == 0:
                reject("branch_not_observed")
                continue
            if components[ry, rx] == component:
                reject("second_entrance")
                continue
            region = components == component
            if (
                region[0].any()
                or region[-1].any()
                or region[:, 0].any()
                or region[:, -1].any()
            ):
                reject("map_edge")
                continue
            boundary = (
                ndimage.binary_dilation(region, CONNECTIVITY)
                & ~region
                & ~cut
                & possible
            )
            if np.any(boundary & ((cells < 0) | ((cells >= 30) & (cells < 50)))):
                reject("reachable_unknown")
                continue
            yy, xx = np.nonzero(region)
            world_x = origin[0] + (xx + 0.5) * resolution - pose[0]
            world_y = origin[1] + (yy + 0.5) * resolution - pose[1]
            along = co * world_x + si * world_y
            if along.min() < low - 0.20 or along.max() > high + 0.20:
                reject("branching_region")
                continue
            reject("closed_corridor")
            if details is not None:
                across = -si * world_x + co * world_y
                far = int(across.argmax())
                entry_local = ((low + high) / 2, depth)
                details.update(
                    entry=[
                        pose[0] + co * entry_local[0] - si * depth,
                        pose[1] + si * entry_local[0] + co * depth,
                    ],
                    terminal=[
                        float(origin[0] + (xx[far] + 0.5) * resolution),
                        float(origin[1] + (yy[far] + 0.5) * resolution),
                    ],
                    length_m=float(across.max() - depth),
                    centre_width_m=float(high - low),
                    observed_centre_cells=int(region.sum()),
                )
            # The virtual segment spans the physical mouth, rather than just
            # its eroded centre strip. It influences steering, never safety.

            return np.column_stack(
                (np.linspace(low - 0.24, high + 0.24, 100), np.full(100, depth))
            )

    return None


class ClosedCorridors:
    """One compressed edge per closed branch, revalidated against every map.

    Records use the sensor-built map frame. A new unknown continuation or changed
    wall geometry removes the record; an old observation is never a permanent
    virtual obstacle after the map changes.
    """

    def __init__(self):
        self.records = []
        self.sequence = 0
        self.invalidated = 0
        self.selected_id = None

    @staticmethod
    def to_world(points, pose):
        co, si = math.cos(pose[2]), math.sin(pose[2])

        return points @ np.array([[co, si], [-si, co]]) + pose[:2]

    @staticmethod
    def to_body(points, pose):
        co, si = math.cos(pose[2]), math.sin(pose[2])

        return (points - np.asarray(pose[:2])) @ np.array([[co, -si], [si, co]])

    def update(self, cells, resolution, origin, pose, audit=None):
        kept = []
        for record in self.records:
            details = {}
            portal = closed_left_opening(
                cells, resolution, origin, record["anchor"], details=details
            )
            if portal is None:
                self.invalidated += 1
                continue
            record.update(details)
            record["portal"] = self.to_world(portal, record["anchor"])
            kept.append(record)
        self.records = kept
        details = {}
        portal = closed_left_opening(
            cells, resolution, origin, pose, audit=audit, details=details
        )
        if portal is not None:
            entry = np.asarray(details["entry"])
            normal = np.array([-math.sin(pose[2]), math.cos(pose[2])])
            matching = next(
                (
                    r
                    for r in self.records
                    if np.linalg.norm(np.asarray(r["entry"]) - entry) < 0.5
                    and np.dot(normal, r["normal"]) > 0.9
                ),
                None,
            )
            if matching is None:
                self.sequence += 1
                self.records.append(
                    dict(
                        id=self.sequence,
                        anchor=tuple(pose),
                        normal=normal,
                        portal=self.to_world(portal, pose),
                        **details,
                    )
                )
        self.selected_id = None
        choices = []
        for record in self.records:
            body = self.to_body(record["portal"], pose)
            tangent = body[-1] - body[0]
            tangent /= np.linalg.norm(tangent)
            # A virtual left wall is applicable only beside this opening.
            # Keep all other records for future approaches and visualization.
            if (
                abs(tangent[0]) > 0.9
                and 0.1 < body[:, 1].mean() < 1.0
                and body[:, 0].min() < 0.45
                and body[:, 0].max() > -0.35
            ):
                choices.append(
                    (float(np.linalg.norm(body, axis=1).min()), record["id"], body)
                )
        if not choices:

            return None
        _, self.selected_id, portal = min(choices, key=lambda item: item[0])

        return portal

    def describe(self):

        return [
            {
                key: record[key]
                for key in ("id", "entry", "terminal", "length_m", "centre_width_m")
            }
            for record in self.records
        ]
