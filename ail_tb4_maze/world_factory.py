"""Privileged maze preparation. Generates layouts only, never robot commands."""

import json
import math
import random
import time
import xml.etree.ElementTree as E
from pathlib import Path


def box_mesh(path, sx, sy, sz, tile=1.0, texture=None, assets=None):
    """Six planar faces with world-sized UVs: bricks do not stretch on long walls."""
    if path.exists():
        return
    x, y, z = sx / 2, sy / 2, sz / 2
    faces = [
        ([(-x, -y, -z), (x, -y, -z), (x, -y, z), (-x, -y, z)], sx, sz),
        ([(x, y, -z), (-x, y, -z), (-x, y, z), (x, y, z)], sx, sz),
        ([(-x, y, -z), (-x, -y, -z), (-x, -y, z), (-x, y, z)], sy, sz),
        ([(x, -y, -z), (x, y, -z), (x, y, z), (x, -y, z)], sy, sz),
        ([(-x, -y, z), (x, -y, z), (x, y, z), (-x, y, z)], sx, sy),
        ([(-x, y, -z), (x, y, -z), (x, -y, -z), (-x, -y, -z)], sx, sy),
    ]
    vertices = []
    uv = []
    indices = []
    for f, (points, u, v) in enumerate(faces):
        vertices.extend(points)
        uv.extend([(0, 0), (u / tile, 0), (u / tile, v / tile), (0, v / tile)])
        for i in [0, 1, 2, 0, 2, 3]:
            indices.extend([f * 4 + i, f * 4 + i])
    normals = [(0, -1, 0), (0, 1, 0), (-1, 0, 0), (1, 0, 0), (0, 0, 1), (0, 0, -1)]
    lines = [
        "# UV-mapped, metre-scale Gazebo box",
        f"mtllib {path.with_suffix('.mtl').name}",
        "o surface",
        "usemtl surface",
    ]
    if texture and assets:
        path.with_suffix(".mtl").write_text(
            "newmtl surface\nKa 0.8 0.8 0.8\nKd 1 1 1\nKs 0 0 0\nNs 1\nmap_Kd "
            + str((assets / "textures" / f"{texture}_albedo.jpg").resolve())
            + "\n"
        )
    lines.extend("v " + " ".join(map(str, p)) for p in vertices)
    lines.extend("vt " + " ".join(map(str, p)) for p in uv)
    lines.extend("vn " + " ".join(map(str, p)) for p in normals)
    for f in range(6):
        for tri in [(0, 1, 2), (0, 2, 3)]:
            lines.append(
                "f " + " ".join(f"{f * 4 + i + 1}/{f * 4 + i + 1}/{f + 1}" for i in tri)
            )
    path.write_text("\n".join(lines) + "\n")


def material(visual, texture, assets):
    old = visual.find("material")
    if old is not None:
        visual.remove(old)
    m = E.SubElement(visual, "material")
    E.SubElement(m, "ambient").text = "0.8 0.8 0.8 1"
    E.SubElement(m, "diffuse").text = "1 1 1 1"
    p = E.SubElement(E.SubElement(m, "pbr"), "metal")
    for tag, channel in [
        ("albedo_map", "albedo"),
        ("normal_map", "normal"),
        ("roughness_map", "roughness"),
    ]:
        E.SubElement(p, tag).text = str(
            (assets / "textures" / f"{texture}_{channel}.jpg").resolve()
        )
    E.SubElement(p, "metalness").text = "0"
    E.SubElement(p, "roughness").text = ".85"


def wall(model, name, x, y, sx, sy, height=1.3):
    link = E.SubElement(model, "link", name=name)
    E.SubElement(link, "pose").text = f"{x} {y} {height / 2} 0 0 0"
    for typ in ["collision", "visual"]:
        item = E.SubElement(link, typ, name=name + "_" + typ)
        E.SubElement(
            E.SubElement(E.SubElement(item, "geometry"), "box"), "size"
        ).text = f"{sx} {sy} {height}"
    sensor = E.SubElement(link, "sensor", name="wall_contacts", type="contact")
    E.SubElement(sensor, "always_on").text = "true"
    E.SubElement(sensor, "update_rate").text = "50"
    contact = E.SubElement(sensor, "contact")
    E.SubElement(contact, "collision").text = name + "_collision"
    E.SubElement(contact, "topic").text = "/challenge/wall_contacts"
    return link


def generate(pkg, state, size=20.0, cells=10, seed=42, preset="random"):
    pkg = Path(pkg)
    state = Path(state)
    state.mkdir(parents=True, exist_ok=True)
    assets = pkg / "assets"
    meshes = state / "meshes"
    meshes.mkdir(exist_ok=True)
    if not 12 <= size <= 40:
        raise ValueError("Maze size must be 12–40 metres")
    if not 5 <= cells <= 20 or size / cells < 1:
        raise ValueError("Use a grid with at least 1 metre per cell")
    rng = random.Random(seed)
    tree = E.parse(pkg / "worlds/maze_escape.sdf")
    world = tree.getroot().find("world")
    c = json.loads((pkg / "config/challenge.json").read_text())
    half = size / 2
    c.update(
        bounds=[-half, half, -half, half],
        maze_size_m=size,
        grid_cells=cells,
        seed=seed,
        preset=preset,
    )
    boxes = []
    dead_ends = 0
    loops = 0
    if preset == "official":
        scale = size / 20
        for name in ["border", "obstacles"]:
            model = world.find(f"model[@name='{name}']")
            for link in model.findall("link"):
                pose = list(map(float, link.findtext("pose").split()))
                pose[0] *= scale
                pose[1] *= scale
                link.find("pose").text = " ".join(map(str, pose))
                for item in list(link.findall("collision")) + list(
                    link.findall("visual")
                ):
                    b = item.find("geometry/box/size")
                    sx, sy, sz = map(float, b.text.split())
                    sx *= scale
                    sy *= scale
                    # Brick border collisions match the visible surface.
                    if name == "border":
                        if sx < 0.1:
                            sx = 0.16
                        if sy < 0.1:
                            sy = 0.16
                    b.text = f"{sx} {sy} {sz}"
        c["start"] = {"x": -8.5 * scale, "y": -8.5 * scale, "z": 0.02, "yaw": 0.0}
        c["exit"] = {
            "x": (10.005 * scale + 0.08),
            "y_min": -3 * scale,
            "y_max": -1 * scale,
        }
    else:
        for name in ["border", "obstacles"]:
            world.remove(world.find(f"model[@name='{name}']"))
        border = E.SubElement(world, "model", name="border")
        E.SubElement(border, "static").text = "true"
        obstacles = E.SubElement(world, "model", name="obstacles")
        E.SubElement(obstacles, "static").text = "true"
        spacing = size / cells
        thickness = 0.24
        exit_row = rng.randrange(1, cells - 1)
        # DFS carving is a world generator, with no sensor input or robot navigation.
        for attempt in range(100):
            visited = {(0, 0)}
            stack = [(0, 0)]
            open_edges = set()
            while stack:
                x, y = stack[-1]
                neighbors = [
                    v
                    for v in [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
                    if 0 <= v[0] < cells and 0 <= v[1] < cells and v not in visited
                ]
                if neighbors:
                    v = rng.choice(neighbors)
                    open_edges.add(tuple(sorted(((x, y), v))))
                    visited.add(v)
                    stack.append(v)
                else:
                    stack.pop()
            closed = [
                tuple(sorted(((x, y), v)))
                for x in range(cells)
                for y in range(cells)
                for v in [(x + 1, y), (x, y + 1)]
                if v[0] < cells
                and v[1] < cells
                and tuple(sorted(((x, y), v))) not in open_edges
            ]
            rng.shuffle(closed)
            extra = max(1, round(cells * cells * 0.06))
            open_edges.update(closed[:extra])
            loops = extra
            degree = {(x, y): 0 for x in range(cells) for y in range(cells)}
            for a, b in open_edges:
                degree[a] += 1
                degree[b] += 1
            dead_ends = sum(d == 1 for d in degree.values())
            if dead_ends >= 3:
                break
        assert dead_ends >= 3
        gap0 = -half + exit_row * spacing
        gap1 = gap0 + spacing
        wall(border, "west", -half, 0, thickness, size + thickness)
        wall(border, "north", 0, half, size + thickness, thickness)
        wall(border, "south", 0, -half, size + thickness, thickness)
        wall(border, "east_lower", half, (-half + gap0) / 2, thickness, gap0 + half)
        wall(border, "east_upper", half, (gap1 + half) / 2, thickness, half - gap1)
        idx = 0
        for x in range(cells):
            for y in range(cells):
                for v in [(x + 1, y), (x, y + 1)]:
                    if (
                        v[0] >= cells
                        or v[1] >= cells
                        or tuple(sorted(((x, y), v))) in open_edges
                    ):
                        continue
                    idx += 1
                    if v[0] > x:
                        wall(
                            obstacles,
                            f"wall{idx}",
                            -half + (x + 1) * spacing,
                            -half + (y + 0.5) * spacing,
                            thickness,
                            spacing + thickness,
                        )
                    else:
                        wall(
                            obstacles,
                            f"wall{idx}",
                            -half + (x + 0.5) * spacing,
                            -half + (y + 1) * spacing,
                            spacing + thickness,
                            thickness,
                        )
        c["start"] = {
            "x": -half + spacing / 2,
            "y": -half + spacing / 2,
            "z": 0.02,
            "yaw": 0.0,
        }
        c["exit"] = {"x": half + thickness / 2, "y_min": gap0, "y_max": gap1}
        c["layout_stats"] = {
            "connected_cells": len(visited),
            "dead_ends": dead_ends,
            "independent_loops": loops,
            "cell_width_m": spacing,
            "corridor_width_m": spacing - thickness,
        }
    # Use UV-mapped textures for visuals and simple boxes for collisions.
    for model_name in ["border", "obstacles"]:
        model = world.find(f"model[@name='{model_name}']")
        model_pose = list(map(float, model.findtext("pose", "0 0 0 0 0 0").split()))
        for i, link in enumerate(model.findall("link")):
            dims = list(
                map(float, link.findtext("collision/geometry/box/size").split())
            )
            sx, sy, sz = dims
            pose = list(map(float, link.findtext("pose").split()))
            x = pose[0] + model_pose[0]
            y = pose[1] + model_pose[1]
            boxes.append([x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2])
            texture = "grass" if model_name == "obstacles" and i % 4 == 0 else "bricks"
            mesh = meshes / f"box_v2_{sx:.4f}_{sy:.4f}_{sz:.4f}_{texture}.obj"
            box_mesh(
                mesh,
                sx,
                sy,
                sz,
                tile=1 if texture == "bricks" else 0.7,
                texture=texture,
                assets=assets,
            )
            visual = link.find("visual")
            geometry = visual.find("geometry")
            geometry.clear()
            E.SubElement(E.SubElement(geometry, "mesh"), "uri").text = str(
                mesh.resolve()
            )
            material(visual, texture, assets)
            if texture == "grass":
                # Rounded clusters stay above LiDAR height and within wall footprint.
                length = max(sx, sy)
                width = min(sx, sy)
                r = min(0.38, width * 0.46)
                for j in range(max(1, int(length / max(0.15, r * 1.6)))):
                    v = E.SubElement(link, "visual", name=f"foliage_{j}")
                    axis = (j + 0.5) * length / max(
                        1, int(length / max(0.15, r * 1.6))
                    ) - length / 2
                    E.SubElement(v, "pose").text = (
                        f"{axis if sx >= sy else 0} {axis if sy > sx else 0} "
                        f"{sz / 2 - r * 0.45} 0 0 0"
                    )
                    E.SubElement(
                        E.SubElement(E.SubElement(v, "geometry"), "sphere"), "radius"
                    ).text = str(r)
                    material(v, "grass", assets)
    ground = world.find("model[@name='ground_plane']")
    link = ground.find("link")
    visual = link.find("visual")
    ground_mesh = meshes / "soil_ground_v2.obj"
    box_mesh(ground_mesh, 100, 100, 0.02, tile=2, texture="soil", assets=assets)
    visual.find("geometry").clear()
    E.SubElement(E.SubElement(visual.find("geometry"), "mesh"), "uri").text = str(
        ground_mesh.resolve()
    )
    E.SubElement(visual, "pose").text = "0 0 -0.011 0 0 0"
    material(visual, "soil", assets)
    # Natural daylight with soft shadows, keeping collision ground plane flat.
    world.find("scene/ambient").text = ".65 .65 .65 1"
    world.find("scene/background").text = ".68 .80 .92 1"
    for link in world.findall(".//link"):
        old = link.find("static")
        if old is not None:
            link.remove(old)
        for col in link.findall("collision"):
            old = col.find("material")
            if old is not None:
                col.remove(old)
    scene_id = str(time.monotonic_ns())
    c["scene_models"] = {}
    for name in ["border", "obstacles"]:
        model = world.find(f"model[@name='{name}']")
        new_name = name + "_" + scene_id
        model.set("name", new_name)
        c["scene_models"][name] = new_name
    c["wall_boxes"] = boxes
    validate(c)
    E.indent(tree)
    tree.write(state / "world.sdf", encoding="unicode", xml_declaration=True)
    (state / "challenge.json").write_text(json.dumps(c, indent=2) + "\n")
    return c


def validate(c):
    """Independent inflated-footprint flood-fill, does not produce driving commands."""
    from collections import deque

    lo, hi, bot, top = c["bounds"]
    resolution = 0.1
    radius = c["footprint_radius"] + 0.025
    boxes = c["wall_boxes"]
    bins = {}
    for box in boxes:
        a, b, d, e = box
        for i in range(math.floor(a - radius), math.floor(b + radius) + 1):
            for j in range(math.floor(d - radius), math.floor(e + radius) + 1):
                bins.setdefault((i, j), []).append(box)

    def clear(x, y):
        return all(
            math.hypot(max(a - x, 0, x - b), max(d - y, 0, y - e)) > radius
            for a, b, d, e in bins.get((math.floor(x), math.floor(y)), [])
        )

    nx = round((hi - lo + 1) / resolution)
    ny = round((top - bot) / resolution)

    def pt(i, j):
        return lo + i * resolution, bot + j * resolution

    def free(i, j):
        return 0 <= i <= nx and 0 <= j <= ny and clear(*pt(i, j))

    start = tuple(
        round((c["start"][k] - (lo if k == "x" else bot)) / resolution)
        for k in ["x", "y"]
    )
    assert free(*start), "Unsafe robot start"
    q = deque([start])
    seen = {start}
    exit = c["exit"]
    found = False
    while q:
        i, j = q.popleft()
        x, y = pt(i, j)
        if (
            x - radius > exit["x"]
            and exit["y_min"] + radius < y < exit["y_max"] - radius
        ):
            found = True
            break
        for n in [(i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)]:
            if n not in seen and free(*n):
                seen.add(n)
                q.append(n)
    if not found:
        raise ValueError("Maze clearance validation could not find a reachable exit")
    c["clearance_validation"] = {
        "passed": True,
        "grid_resolution_m": resolution,
        "radius_with_margin_m": radius,
        "visited_cells": len(seen),
    }
