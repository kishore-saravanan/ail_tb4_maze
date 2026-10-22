"""Operator-only live scene updates with Gazebo Transport, not shell payloads."""

import fcntl
import json
import time
import xml.etree.ElementTree as E
from pathlib import Path

import rclpy
from google.protobuf import text_format
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.world_control_pb2 import WorldControl
from gz.transport13 import Node as GzNode
from gz.transport13 import NodeOptions
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


def transport():
    options = NodeOptions()
    options.partition = "ai_legion_maze"
    return GzNode(options)


def request(node, service, message, response=Boolean):
    ok, result = node.request(service, message, type(message), response, 7000)
    if not ok or (response is Boolean and not result.data):
        raise RuntimeError("Gazebo did not accept " + service)
    return result


def gz(service, typ, text):
    cls = {
        "gz.msgs.WorldControl": WorldControl,
        "gz.msgs.Pose": Pose,
        "gz.msgs.Entity": Entity,
        "gz.msgs.EntityFactory": EntityFactory,
    }[typ]
    message = cls()
    text_format.Parse(text, message)
    return request(transport(), service, message)


def ros_service(path, kind=Trigger, data=None):
    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=61)
    node = Node("setup_operator_" + str(time.monotonic_ns()), context=context)
    executor = SingleThreadedExecutor(context=context)
    try:
        client = node.create_client(kind, path)
        if not client.wait_for_service(timeout_sec=8):
            raise RuntimeError("Environment service not ready: " + path)
        req = kind.Request()
        if data is not None:
            req.data = data
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, executor=executor, timeout_sec=8)
        if not future.done():
            raise RuntimeError("No response from " + path)
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return response.message
    finally:
        node.destroy_node()
        executor.shutdown()
        context.try_shutdown()


def update(draft, state=None):
    target = Path(state) if state else Path(draft)
    target.mkdir(parents=True, exist_ok=True)
    with (target / ".update.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        return _update(draft, target)


def _update(draft, state=None):
    draft = Path(draft)
    state = Path(state) if state else draft
    c = json.loads((draft / "challenge.json").read_text())
    world = E.parse(draft / "world.sdf").getroot().find("world")
    node = transport()
    prefix = "/world/maze_escape/"
    scene = request(node, prefix + "scene/info", Empty(), Scene)
    previous = [
        m.name for m in scene.model if m.name.startswith(("border", "obstacles"))
    ]
    # Preserve the prior scene while staging all of the replacement geometry.
    ros_service("/interface/enable", SetBool, False)
    ros_service("/referee/reset")
    created = []
    old_removed = False

    def pose(x, y, z):
        p = Pose(name="turtlebot4")
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.w = 1
        return p

    request(node, prefix + "control", WorldControl(pause=True))
    try:
        request(node, prefix + "set_pose", pose(45, 45, 0.4))
        # Clear DART contact caches before deleting static wall bodies; floor stays.
        request(node, prefix + "control", WorldControl(pause=False))
        time.sleep(0.15)
        request(node, prefix + "control", WorldControl(pause=True))
        for name in c["scene_models"].values():
            model = world.find(f"model[@name='{name}']")
            sdf = (
                '<sdf version="1.8">' + E.tostring(model, encoding="unicode") + "</sdf>"
            )
            request(
                node, prefix + "create", EntityFactory(sdf=sdf, allow_renaming=False)
            )
            created.append(name)
        for name in previous:
            request(node, prefix + "remove", Entity(name=name, type=Entity.MODEL))
        old_removed = True
        # Commit only after the complete new scene has been accepted.
        state.mkdir(parents=True, exist_ok=True)
        for filename in ["world.sdf", "challenge.json"]:
            content = (draft / filename).read_bytes()
            tmp = state / (filename + ".tmp")
            tmp.write_bytes(content)
            tmp.replace(state / filename)
        (state / "live_models.json").write_text(json.dumps(c["scene_models"]))
        start = c["start"]
        request(node, prefix + "set_pose", pose(start["x"], start["y"], 0.02))
        ros_service("/interface/reset")
        ros_service("/referee/reset")
    except Exception:
        if not old_removed:
            for name in created:
                try:
                    request(
                        node, prefix + "remove", Entity(name=name, type=Entity.MODEL)
                    )
                except Exception:
                    pass
        # Commands remain inhibited after any incomplete update.
        raise
    finally:
        request(node, prefix + "control", WorldControl(pause=False))
    ros_service("/interface/enable", SetBool, True)
    return "Live maze updated. Robot reset; timer ready."


def reset_trial(state):
    """Reset pose/score/local encoder origin without replacing geometry or Gazebo."""
    state = Path(state)
    c = json.loads((state / "challenge.json").read_text())
    s = c["start"]
    node = transport()
    prefix = "/world/maze_escape/"
    with (state / ".update.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        ros_service("/interface/enable", SetBool, False)
        ros_service("/referee/reset")
        request(node, prefix + "control", WorldControl(pause=True))
        try:
            p = Pose(name="turtlebot4")
            p.position.x = s["x"]
            p.position.y = s["y"]
            p.position.z = 0.02
            p.orientation.w = 1
            request(node, prefix + "set_pose", p)
            ros_service("/interface/reset")
            ros_service("/referee/reset")
        finally:
            request(node, prefix + "control", WorldControl(pause=False))
        ros_service("/interface/enable", SetBool, True)
    return "Robot reset. Timer ready; maze unchanged."
