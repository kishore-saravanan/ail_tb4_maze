"""Fuse current laser rays into a SLAM map; no simulator geometry is used."""

import numpy as np


def fuse_rays(grid, resolution, origin, sensor, rays, hits):
    cells = np.asarray(grid, dtype=np.int8)
    origin = np.asarray(origin, dtype=float)
    sensor = np.asarray(sensor, dtype=float)
    rays = np.asarray(rays, dtype=float)
    endpoints = sensor + rays
    h, w = cells.shape
    lo = (
        np.floor(
            (np.minimum(endpoints.min(axis=0), sensor) - origin) / resolution
        ).astype(int)
        - 2
    )
    hi = (
        np.floor(
            (np.maximum(endpoints.max(axis=0), sensor) - origin) / resolution
        ).astype(int)
        + 3
    )
    c0, r0 = np.minimum(lo, [0, 0])
    c1, r1 = np.maximum(hi, [w, h])
    out = np.full((r1 - r0, c1 - c0), -1, np.int8)
    out[-r0 : h - r0, -c0 : w - c0] = cells
    new_origin = origin + np.array([c0, r0]) * resolution
    length = np.linalg.norm(rays, axis=1)
    direction = rays / np.maximum(length[:, None], 1e-9)
    distance = np.arange(0, float(length.max()) + resolution / 2, resolution / 2)
    points = sensor + direction[:, None, :] * distance[None, :, None]
    indices = np.floor((points - new_origin) / resolution).astype(int)
    limit = length - np.where(hits, resolution * 0.6, 0.0)
    valid = distance[None, :] < limit[:, None]
    cc = indices[:, :, 0][valid]
    rr = indices[:, :, 1][valid]
    inside = (cc >= 0) & (cc < out.shape[1]) & (rr >= 0) & (rr < out.shape[0])
    out[rr[inside], cc[inside]] = 0
    # Mark all endpoints after clearing, so a crossing ray cannot erase a wall.
    endpoint_indices = np.floor(
        (endpoints[np.asarray(hits, dtype=bool)] - new_origin) / resolution
    ).astype(int)
    if len(endpoint_indices):
        out[endpoint_indices[:, 1], endpoint_indices[:, 0]] = 100
    return out, tuple(new_origin)
