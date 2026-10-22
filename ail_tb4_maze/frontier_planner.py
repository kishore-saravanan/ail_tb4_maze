"""Reachable frontier selection on an observed SLAM occupancy grid."""

import math

import numpy as np
from scipy import ndimage, sparse
from scipy.sparse.csgraph import dijkstra
from scipy.spatial import cKDTree


def frontier_goal_active(grid, resolution, origin, goal, radius=0.30):
    """Whether the approach still reaches unobserved space near its old goal.

    A scan can finish observing a corridor before the robot gets to its goal.
    Only connected free cells near that goal count: unknown space behind a
    wall is not a reason to finish driving to an already observed dead end.
    Outside the current map is unknown, matching frontier selection below.
    """
    if goal is None:
        return False
    cells = np.asarray(grid)
    r = math.floor((goal[1] - origin[1]) / resolution)
    c = math.floor((goal[0] - origin[0]) / resolution)
    h, w = cells.shape
    if not (0 <= r < h and 0 <= c < w):
        return False
    extent = math.ceil(radius / resolution)
    r0, r1 = max(0, r - extent), min(h, r + extent + 1)
    c0, c1 = max(0, c - extent), min(w, c + extent + 1)
    rr, cc = np.mgrid[r0:r1, c0:c1]
    free = (cells[r0:r1, c0:c1] >= 0) & (cells[r0:r1, c0:c1] < 30)
    free &= ((rr - r) ** 2 + (cc - c) ** 2) * resolution**2 <= radius**2
    # Four-connectivity does not connect a frontier through a wall corner.
    labels, _ = ndimage.label(free)
    component = labels[r - r0, c - c0]
    if not component:
        return False
    adjacent = ndimage.binary_dilation(cells < 0, np.ones((3, 3)), border_value=1)
    return bool(np.any((labels == component) & adjacent[r0:r1, c0:c1]))


def choose_frontier(
    grid,
    resolution,
    origin,
    robot,
    blacklist=(),
    visited=(),
    return_path=False,
    clearance_margin=0.28,
):
    """Choose a reachable frontier using path cost, clearance and visit history.

    Coordinates are in the map frame. With return_path=True, return a target,
    candidate centers and a collision-cleared grid route. Unknown cells cannot
    be traversed; no reachable candidate is reported as a None target.
    """
    cells = np.asarray(grid)
    h, w = cells.shape
    occupied = cells >= 50
    free = (cells >= 0) & (cells < 30)
    clearance = ndimage.distance_transform_edt(~occupied) * resolution
    safe = free & (clearance >= clearance_margin)
    r = int((robot[1] - origin[1]) / resolution)
    c = int((robot[0] - origin[0]) / resolution)
    if not (0 <= r < h and 0 <= c < w):
        return (None, [], []) if return_path else (None, [])
    if not safe[r, c]:
        rows, cols = np.nonzero(safe)
        if not len(rows):
            return (None, [], []) if return_path else (None, [])
        distance = (rows - r) ** 2 + (cols - c) ** 2
        i = int(distance.argmin())
        if distance[i] * resolution**2 > 0.25**2:
            return (None, [], []) if return_path else (None, [])
        r, c = int(rows[i]), int(cols[i])
    unknown = cells < 0
    unknown_labels, _ = ndimage.label(unknown, np.ones((3, 3)))
    unknown_areas = np.bincount(unknown_labels.ravel()) * resolution**2
    unknown_areas[0] = 0.0
    border_labels = np.unique(
        np.concatenate(
            [
                unknown_labels[0],
                unknown_labels[-1],
                unknown_labels[:, 0],
                unknown_labels[:, -1],
            ]
        )
    )
    unknown_areas[border_labels[border_labels > 0]] = np.inf
    adjacent_area = ndimage.maximum_filter(
        unknown_areas[unknown_labels], size=3, mode="constant", cval=np.inf
    )
    frontier = safe & ndimage.binary_dilation(unknown, np.ones((3, 3)), border_value=1)
    history_points = np.asarray(visited) if len(visited) else np.empty((0, 2))
    history_points = history_points[
        np.linalg.norm(history_points - np.asarray(robot), axis=1) > 0.6
    ]
    history = cKDTree(history_points) if len(history_points) else None
    labels, count = ndimage.label(frontier, np.ones((3, 3)))
    if not count:
        return (None, [], []) if return_path else (None, [])
    sizes = np.bincount(labels.ravel())
    frontier &= sizes[labels] * resolution >= 0.15
    frontier &= labels > 0
    # Vectorized sparse graph, compiled Dijkstra: every candidate frontier is
    # evaluated in a single search, with no diagonal corner-cutting.
    idx = np.arange(h * w).reshape(h, w)
    sources = []
    targets = []
    weights = []
    penalty = 1.0 + 0.04 / np.maximum(clearance - 0.22, 0.05)
    for dr, dc in [(0, 1), (1, 0), (1, 1), (1, -1)]:
        ra = slice(0, h - dr)
        rb = slice(dr, h)
        ca = slice(max(0, -dc), min(w, w - dc))
        cb = slice(max(0, dc), min(w, w + dc))
        ok = safe[ra, ca] & safe[rb, cb]
        if dr and dc:
            ok &= safe[ra, cb] & safe[rb, ca]
        a = idx[ra, ca][ok]
        b = idx[rb, cb][ok]
        cost = (
            resolution
            * math.hypot(dr, dc)
            * (penalty[ra, ca][ok] + penalty[rb, cb][ok])
            / 2
        )
        sources.extend([a, b])
        targets.extend([b, a])
        weights.extend([cost, cost])
    if not sources:
        return (None, [], []) if return_path else (None, [])
    graph = sparse.csr_matrix(
        (np.concatenate(weights), (np.concatenate(sources), np.concatenate(targets))),
        shape=(h * w, h * w),
    )
    result = dijkstra(
        graph, directed=False, indices=r * w + c, return_predecessors=return_path
    )
    if return_path:
        distances, predecessors = result
    else:
        distances = result
    candidates = []
    centers = []
    for label in range(1, count + 1):
        rr, cc = np.nonzero(frontier & (labels == label))
        if not len(rr):
            continue
        travel = distances[rr * w + cc]
        finite = np.isfinite(travel)
        rr, cc, travel = rr[finite], cc[finite], travel[finite]
        if not len(rr):
            continue
        wx = origin[0] + (cc + 0.5) * resolution
        wy = origin[1] + (rr + 0.5) * resolution
        distance_from_robot = np.hypot(wx - robot[0], wy - robot[1])
        unblocked = np.ones(len(rr), bool)
        for bx, by in blacklist:
            unblocked &= np.hypot(wx - bx, wy - by) > 0.55
        allowed = unblocked & (distance_from_robot > 0.6)
        if not allowed.any():
            allowed = unblocked & (distance_from_robot > 0.15)
        if not allowed.any():
            continue
        # Select a reachable point near the frontier's center. Higher information
        # gain offsets travel modestly, rather than blindly chasing distant gaps.
        center_cost = np.hypot(rr - np.mean(rr), cc - np.mean(cc)) * resolution * 0.4
        # Tiny isolated unknown pockets are usually scan gaps inside already
        # bounded space. Keep them as fallbacks, rather than spending time there
        # while a substantial unexplored passage is reachable.
        noise_penalty = np.where(adjacent_area[rr, cc] < 0.25, 4.0, 0.0)
        revisit_penalty = np.zeros(len(rr))
        if history is not None:
            nearest, _ = history.query(np.column_stack((wx, wy)))
            revisit_penalty = np.maximum(0.0, 0.4 - nearest) * 10.0
        goal_margin = 0.08 / np.maximum(clearance[rr, cc] - 0.20, 0.05)
        values = np.where(
            allowed,
            travel + center_cost + noise_penalty + revisit_penalty + goal_margin,
            np.inf,
        )
        i = int(values.argmin())
        point = (float(wx[i]), float(wy[i]))
        centers.append(point)
        score = float(
            travel[i] + noise_penalty[i] + revisit_penalty[i] + goal_margin[i]
        ) - 0.6 * math.log1p(len(rr) * resolution)
        candidates.append((score, point, int(rr[i] * w + cc[i])))
    if not candidates:
        return (None, centers, []) if return_path else (None, centers)
    _, point, node = min(candidates)
    if not return_path:
        return point, centers
    route = [node]
    while node != r * w + c:
        node = int(predecessors[node])
        if node < 0:
            return None, centers, []
        route.append(node)
    route.reverse()
    path = [
        (
            origin[0] + (n % w + 0.5) * resolution,
            origin[1] + (n // w + 0.5) * resolution,
        )
        for n in route
    ]
    return point, centers, path


def goal_is_safe(grid, resolution, origin, point, clearance=None):
    cells = np.asarray(grid)
    r = math.floor((point[1] - origin[1]) / resolution)
    c = math.floor((point[0] - origin[0]) / resolution)
    if not (0 <= r < cells.shape[0] and 0 <= c < cells.shape[1]):
        return False
    if not 0 <= cells[r, c] < 30:
        return False
    if clearance is None:
        clearance = ndimage.distance_transform_edt(cells < 50) * resolution
    return bool(clearance[r, c] >= 0.27)
