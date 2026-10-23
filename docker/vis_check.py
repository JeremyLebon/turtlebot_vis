#!/usr/bin/env python3
"""`check`: is turtlebot-vis goed opgestart en praat hij met de robot?

Loopt vanzelf één keer bij de eerste shell in de container, en daarna met
`check`. Eerst zonder ROS (router bereikbaar? webpagina? Lichtblick?), dan
pas rclpy: zonder bereikbare Zenoh-router blijft rmw_zenoh hangen.

Belangrijkste test: komt er echt data binnen? Een Humble-laptop ziet de
topics van een Jazzy-robot (en omgekeerd) wel, maar krijgt geen berichten.
"""
import json
import os
import socket
import sys
import time
import urllib.request

OK, WARN, BAD = "\033[32m✔\033[0m", "\033[33m!\033[0m", "\033[31m✘\033[0m"
problems = 0


def line(sym, label, text):
    global problems
    if sym is BAD:
        problems += 1
    print("  %s %-14s %s" % (sym, label, text))


def port_open(host, port, timeout=1.0):
    t0 = time.time()
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return (time.time() - t0) * 1000
    except OSError:
        return None


def http_ok(url, timeout=2.0):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status == 200
    except OSError:
        return False


def ros_checks(sim):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    node = Node("vis_check")
    got = {}

    def keep(name):
        def cb(msg):
            got.setdefault(name, msg)
        return cb

    if sim:
        from rosgraph_msgs.msg import Clock
        node.create_subscription(Clock, "/clock", keep("clock"), 1)
    else:
        node.create_subscription(String, "/launch_status", keep("launch"), 1)
        node.create_subscription(String, "/system_info", keep("sys"), 1)
    want = {"clock"} if sim else {"launch", "sys"}
    end = time.time() + 6.0          # /system_info: every 2 s
    topics = []
    while time.time() < end and not (want <= set(got) and len(topics) > 5):
        rclpy.spin_once(node, timeout_sec=0.2)
        topics = node.get_topic_names_and_types()
    names = {t for t, _ in topics}
    node.destroy_node()
    rclpy.shutdown()

    if not topics:
        line(BAD, "Topics", "geen enkel topic zichtbaar (ROS_DOMAIN_ID=%s klopt?)" % os.environ.get("ROS_DOMAIN_ID"))
        return
    if not (want & set(got)):
        line(BAD, "Data", "%d topics zichtbaar, maar er komt GEEN data binnen - andere ROS-versie op robot en "
             "laptop (Humble <-> Jazzy)? Zie de versie op de webpagina van de robot." % len(topics))
        return
    if sim:
        line(OK, "Simulatie", "Gazebo draait (/clock), %d topics" % len(topics))
        return

    launch = json.loads(got["launch"].data) if "launch" in got else {}
    info = json.loads(got["sys"].data) if "sys" in got else {}
    version = info.get("image_version") or "?"
    line(OK, "Robot", "%s - Fenix %s" % (info.get("hostname", "?"), version.split(" ")[0]))
    robot_distro = "jazzy" if version.startswith("v1.") else "humble" if version.startswith("v0.") else None
    mine = os.environ.get("ROS_DISTRO", "?")
    if robot_distro and robot_distro != mine:
        line(BAD, "ROS-versie", "robot draait %s, deze laptop %s - die praten niet met elkaar (andere image "
             "nemen: turtlebot-vis:%s)" % (robot_distro, mine, "jazzy" if robot_distro == "jazzy" else "zenoh"))
    else:
        line(OK, "ROS-versie", "%s, zelfde als de robot" % mine)
    line(OK, "Data", "%d topics zichtbaar, berichten komen binnen" % len(topics))

    running = [k for k in ("bringup", "slam", "navigation", "camera", "joystick") if launch.get(k)]
    if not launch.get("bringup"):
        line(WARN, "Bringup", "staat uit - geen /scan, /odom of motoren. Start Bringup op de webpagina (Rijden)")
    else:
        missing = [t for t in ("/scan", "/odom", "/tf", "/cmd_vel") if t not in names]
        line(WARN if missing else OK, "Bringup",
             "draait (actief: %s)%s" % (", ".join(running),
                                         " - ontbreekt nog: " + ", ".join(missing) if missing else ""))


def main():
    nr = os.environ.get("TB_NR")
    sim = nr == "sim"
    print("Turtlebot-vis check (ROS 2 %s)" % os.environ.get("ROS_DISTRO", "?"))

    if not nr:
        line(BAD, "Robot", "nog geen robot gekozen - laptop op de wifi TB-AP-<nr>? robot aan? (of: robot <nr>)")
    else:
        host = "127.0.0.1" if sim else "10.0.%s.10" % nr
        ms = port_open(host, 7447)
        if ms is None:
            line(BAD, "Zenoh-router", "%s:7447 NIET bereikbaar - %s" % (
                host, "start de simulatie opnieuw: robot sim" if sim else
                "laptop op de wifi TB-AP-%s? robot aan (melodie gehoord)?" % nr))
        else:
            line(OK, "Zenoh-router", "%s:7447 bereikbaar (%.0f ms)%s" % (host, ms, "" if sim else ", robot %s" % nr))
            if not sim:
                line(OK if http_ok("http://%s:8080/" % host) else WARN, "Webpagina", "http://%s:8080" % host)
            try:
                ros_checks(sim)
            except Exception as exc:     # never a traceback for the student
                line(BAD, "ROS", "controle mislukt: %s" % exc)

    line(OK if port_open("127.0.0.1", 8080) is not None else WARN, "Lichtblick",
         "http://localhost:8080 (in de browser van Windows)")
    bridge = port_open("127.0.0.1", 8765) is not None
    line(OK if bridge else WARN, "Bridge",
         "foxglove_bridge ws://localhost:8765" + ("" if bridge else " - start pas als de robot bereikbaar is"))
    display = os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    line(OK if display and (os.path.exists("/tmp/.X11-unix") or os.path.exists("/mnt/wslg")) else WARN,
         "Display", "rviz2/rqt kunnen vensters openen" if display else
         "geen DISPLAY - rviz2/rqt openen geen venster (gebruik Lichtblick)")
    built = os.path.exists("/root/ros2_ws/install/setup.bash")
    line(OK if os.path.isdir("/root/ros2_ws") else WARN, "ros2_ws",
         "/root/ros2_ws" + (" (gebouwd)" if built else " (nog niet gebouwd: colcon build)"))
    print("  Alles in orde." if not problems else "  %d probleem/problemen - zie hierboven. Opnieuw: check" % problems)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
