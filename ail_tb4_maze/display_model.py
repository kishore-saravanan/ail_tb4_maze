"""Display-only TurtleBot 4 geometry, independent of simulator pose or joints."""

import copy
import xml.etree.ElementTree as ET


def fixed_display_model(description):
    source = ET.fromstring(description)
    robot = ET.Element("robot", name="turtlebot4_display")
    for material in source.findall("material"):
        robot.append(copy.deepcopy(material))
    for link in source.findall("link"):
        visual_link = ET.SubElement(
            robot, "link", name="display_" + link.attrib["name"]
        )
        for visual in link.findall("visual"):
            visual_link.append(copy.deepcopy(visual))
    for joint in source.findall("joint"):
        fixed = ET.SubElement(
            robot, "joint", name="display_" + joint.attrib["name"], type="fixed"
        )
        for tag in ("origin", "parent", "child"):
            element = joint.find(tag)
            if element is not None:
                element = copy.deepcopy(element)
                if tag in ("parent", "child"):
                    element.set("link", "display_" + element.attrib["link"])
                fixed.append(element)

    return ET.tostring(robot, encoding="unicode")
