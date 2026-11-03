"""Student projects: each ROS package with a project.json in its share
directory (built in /root/turtlebot_projects from the image, or a student's
own copy in ~/ros2_ws - that one wins). Used by status_page_server.py
(/api/projects for projects.html) and launch_control_node.py (start/stop).
"""
import glob
import json
import os
import shlex

ROOTS = ("/root/turtlebot_projects/install", "/root/ros2_ws/install")   # later wins


def projects():
    """id -> manifest (+ "source": "image" | "ros2_ws")."""
    out = {}
    for root in ROOTS:
        for path in sorted(glob.glob(os.path.join(root, "*", "share", "*", "project.json"))):
            try:
                with open(path) as f:
                    m = json.load(f)
                m["source"] = "ros2_ws" if root.startswith("/root/ros2_ws") else "image"
                out[m["id"]] = m
            except (OSError, ValueError, KeyError):
                continue
    return out


def find_launch(project_id, launch_id):
    m = projects().get(project_id)
    if not m:
        return None, None
    for l in m.get("launches", []):
        if l.get("id") == launch_id:
            return m, l
    return m, None


def shell_command(cmd):
    """bash -c with all workspaces sourced (launch_control itself only has
    /opt/ros + turtlebot3_ws)."""
    src = ("source /opt/ros/$ROS_DISTRO/setup.bash; source /root/turtlebot3_ws/install/setup.bash; "
           "[ -f /root/turtlebot_projects/install/setup.bash ] && source /root/turtlebot_projects/install/setup.bash; "
           "[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash; ")
    return ["bash", "-c", src + "exec " + " ".join(shlex.quote(c) for c in cmd)]
