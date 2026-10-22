#!/usr/bin/python3
"""Render sensor-only navigation evidence for diagnosing stalls."""

import argparse
import base64
import json
import math
import zlib
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle


def grid(record):
    return np.frombuffer(
        zlib.decompress(base64.b64decode(record["data_zlib_base64"])), np.int8
    ).reshape(record["height"], record["width"])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("telemetry")
    p.add_argument("--time", type=float, default=1e9)
    p.add_argument("--output", default="navigation_snapshot.png")
    args = p.parse_args()
    rows = [
        json.loads(line)
        for line in Path(args.telemetry).read_text().splitlines()
        if line.strip()
    ]
    rows = [r for r in rows if r.get("map") and r.get("pose")]
    r = min(rows, key=lambda x: abs(x["sim_seconds"] - args.time))
    fig, axes = plt.subplots(1, 2, figsize=(12, 7))
    map_pose = np.array(r["pose"])
    odom_pose = np.array(r["odom_pose"][:2])
    mh = r["heading"]
    oh = r["odom_pose"][2]
    plan = np.asarray(r.get("plan", []))
    scan = np.asarray(r.get("lidar_points", []))
    goal = np.asarray(r["goal"]) if r.get("goal") else None

    def rotate(points, angle):
        return points @ np.array(
            [[math.cos(angle), math.sin(angle)], [-math.sin(angle), math.cos(angle)]]
        )

    for ax, key, pose, heading in [
        (axes[0], "map", map_pose, mh),
        (axes[1], "local_map", odom_pose, oh),
    ]:
        m = r.get(key)
        if not m:
            continue
        a = grid(m)
        ox, oy = m["origin"]
        res = m["resolution"]
        if key == "map":
            colors = ListedColormap(["#34434d", "#edf1ed", "#986a52"])
            a = np.where(a < 0, 0, np.where(a >= 50, 2, 1))
            vmax = 2
        else:
            colors = plt.get_cmap("YlOrRd").copy()
            colors.set_under("#34434d")
            vmax = 100
        ax.imshow(
            a,
            origin="lower",
            extent=[ox, ox + m["width"] * res, oy, oy + m["height"] * res],
            cmap=colors,
            vmin=0,
            vmax=vmax,
            interpolation="nearest",
        )
        if len(scan):
            q = rotate(scan, heading) + pose
            ax.scatter(q[:, 0], q[:, 1], s=3, c="#ca2579", label="LiDAR returns")
        if len(plan):
            q = plan if key == "map" else rotate(plan - map_pose, oh - mh) + odom_pose
            ax.plot(q[:, 0], q[:, 1], c="#00a987", lw=2, label="Planned route")
        if goal is not None:
            q = goal if key == "map" else rotate(goal - map_pose, oh - mh) + odom_pose
            ax.scatter(*q, c="#1b49d6", marker="x", s=80, label="Goal")
        ax.add_patch(Circle(pose, 0.23, fill=False, color="#111", lw=2))
        ax.add_patch(Circle(pose, 0.255, fill=False, color="#111", ls=":"))
        ax.arrow(
            *pose,
            0.4 * math.cos(heading),
            0.4 * math.sin(heading),
            width=0.025,
            color="#111",
            zorder=5,
        )
        if key == "local_map":
            ax.set_xlim(pose[0] - 2, pose[0] + 2)
            ax.set_ylim(pose[1] - 2, pose[1] + 2)
        ax.set_aspect("equal")
        ax.set_title(key + " / " + m["frame"])
        ax.set_xlabel("metres")
        ax.legend(fontsize=8)
    fig.suptitle(
        f"{r['sim_seconds']:.1f} s · {r['state']} · "
        f"command v/w: {np.round(r['command'], 3)}\n"
        "Sensor-built map and live local costmap; no ground truth"
    )
    fig.tight_layout()
    fig.savefig(args.output, dpi=130)
    print(args.output)


if __name__ == "__main__":
    main()
