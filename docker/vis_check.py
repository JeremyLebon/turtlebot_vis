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
import re
import shutil
import socket
import subprocess
import sys
import textwrap
import time
import urllib.request

OK, WARN, BAD, OFF = "\033[32m✔\033[0m", "\033[33m!\033[0m", "\033[31m✘\033[0m", "\033[2m·\033[0m"
BOLD, END = "\033[1m", "\033[0m"
# Regels per blok, pas aan het einde getoond (in deze volgorde): de controles
# lopen in een andere volgorde dan wat de student wil lezen.
BROWSER, CONN, ROBOT, LAPTOP = "Open in de browser", "Verbinding", "Robot", "Deze laptop"
sections = {BROWSER: [], CONN: [], ROBOT: [], LAPTOP: []}
count = {BAD: 0, WARN: 0}


def read_first(path):
    """First line of a version file in the image, or "?"."""
    try:
        with open(path) as f:
            return f.readline().strip() or "?"
    except OSError:
        return "?"


def line(sym, label, text, sec=CONN, more=()):
    """Eén rij in blok `sec`. Begint `text` met een adres (http://, ws://, host:poort),
    dan komt dat in de kolom Adres. `more`: extra uitleg in de kolom Info eronder."""
    if sym in count:
        count[sym] += 1
    first, _, rest = text.partition(" ")
    if "://" in first or re.match(r"^[\d.]+:\d+$", first):
        addr, info = first, rest
        if info.startswith("(") and info.endswith(")") and info.count("(") == 1:
            info = info[1:-1]           # "(1 ms)" -> "1 ms": de kolom maakt het al duidelijk
    else:
        addr, info = "", text
    sections[sec].append((sym, label, addr, [info] + list(more)))


def show():
    """Alle blokken als één tabel; Info loopt door op de volgende regel als ze te lang is."""
    blocks = [(t, rows) for t, rows in sections.items() if rows]
    rows = [r for _, rs in blocks for r in rs]
    w1 = max(len(r[1]) for r in rows)
    w2 = max([len(r[2]) for r in rows] + [5])
    cols = shutil.get_terminal_size((110, 24)).columns
    w3 = max(24, min(70, cols - (w1 + w2 + 1) - 13))
    widths = (1, w1, w2, w3)
    inner = sum(w + 2 for w in widths) + 3

    def rule(l, m, r):
        return l + m.join("─" * (w + 2) for w in widths) + r

    out = []
    for n, (title, rs) in enumerate(blocks):
        out.append("┌" + "─" * inner + "┐" if n == 0 else rule("├", "┴", "┤"))
        out.append("│ " + BOLD + title.ljust(inner - 2) + END + " │")
        out.append(rule("├", "┬", "┤"))
        for sym, label, addr, infos in rs:
            lines = [l for i in infos if i for l in textwrap.wrap(i, w3)] or [""]
            for k, l in enumerate(lines):
                c = (sym, label, addr) if k == 0 else (" ", "", "")
                out.append("│ %s │ %-*s │ %-*s │ %-*s │" % (c[0], w1, c[1], w2, c[2], w3, l))
    out.append(rule("└", "┴", "┘"))
    print("\n".join(out))


def port_open(host, port, timeout=1.0):
    t0 = time.time()
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return (time.time() - t0) * 1000
    except OSError:
        return None


def http_ok(url, timeout=4.0):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status == 200
    except OSError:
        return False


def bridge_count():
    """Aantal bridge-processen per soort (cmdline in /proc). Meer dan één is een
    restje van een vorige modus: houdt geheugen of de poort bezet."""
    kinds = {"rosbridge": "rosbridge_websocket --ros-args", "rosapi": "rosapi_node --ros-args",
             "foxglove_bridge": "lib/foxglove_bridge/foxglove_bridge"}
    n = dict.fromkeys(kinds, 0)
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            cmd = open("/proc/%s/cmdline" % pid, "rb").read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        for k, pat in kinds.items():
            n[k] += pat in cmd
    return n


def memory_check():
    """Geheugen van WSL (de container ziet dat van de hele VM) en afgesloten programma's.

    Zit WSL vol, dan sluit Linux het grootste programma af (`Killed` in de
    terminal). Dat staat in de kernellog (dmesg, de container is privileged);
    /proc/vmstat telt het ook zonder rechten.
    """
    try:
        mem = {k: int(v.split()[0]) for k, v in (l.split(":", 1) for l in open("/proc/meminfo"))}
        total, free = mem["MemTotal"] / 1048576, mem["MemAvailable"] / 1048576
    except (OSError, KeyError, ValueError):
        return
    try:
        kills = int(next(l.split()[1] for l in open("/proc/vmstat") if l.startswith("oom_kill ")))
    except (OSError, StopIteration, ValueError):
        kills = 0
    if not kills:
        line(OK if free >= 0.5 else WARN, "Geheugen", "WSL %.1f GB, %.1f GB vrij" % (total, free), LAPTOP,
             () if free >= 0.5 else ["weinig over: sluit programma's, of wsl --shutdown"])
        return
    killed = []
    try:
        uptime = time.clock_gettime(time.CLOCK_MONOTONIC)   # klok van dmesg (zonder slaapstand)
        log = subprocess.run(["dmesg"], capture_output=True, text=True, timeout=5).stdout
        for m in re.finditer(r"\[\s*([\d.]+)\].*Killed process \d+ \(([^)]*)\).*?anon-rss:(\d+)kB", log):
            killed.append("%s (%.1f GB, %.0f min geleden)" % (
                m.group(2), int(m.group(3)) / 1048576, (uptime - float(m.group(1))) / 60))
    except (OSError, subprocess.SubprocessError):
        pass
    line(WARN, "Geheugen", "WSL %.1f GB, %.1f GB vrij" % (total, free), LAPTOP, [
         "%d keer een programma afgesloten wegens geheugentekort%s" % (kills, ": " + ", ".join(killed[-3:]) if killed else ""),
         "Live per onderdeel: diagnose. Groeit je eigen node? Zie de GitBook, Setup laptop."])


def ros_checks(sim, html=False):
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
    start = time.time()
    while time.time() < end and not (want <= set(got) and len(topics) > 5):
        rclpy.spin_once(node, timeout_sec=0.2)
        topics = node.get_topic_names_and_types()
        # simulatie: na 1,5 s nog niemand die /clock publiceert = simulator draait niet
        if sim and time.time() - start > 1.5 and node.count_publishers("/clock") == 0:
            break
    names = {t for t, _ in topics}
    node.destroy_node()
    rclpy.shutdown()

    sim_url = "http://127.0.0.1:8080/sim/"
    if html:
        ver = "v" + read_first("/opt/lichtblick/sim/VERSION")
        if "clock" in got:
            line(OK, "Simulator", "%s %s, draait (%d topics)" % (sim_url, ver, len(topics)), BROWSER)
        else:
            line(WARN, "Simulator", "%s %s" % (sim_url, ver), BROWSER,
                 ["nog niet open: open de link in de browser (verbindt vanzelf)"])
        return
    if sim and "clock" not in got:
        line(WARN, "Gazebo", "draait nog niet (geen /clock)", CONN,
             ["start: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"])
        return
    if not topics:
        line(BAD, "Topics", "geen enkel topic zichtbaar (ROS_DOMAIN_ID=%s klopt?)" % os.environ.get("ROS_DOMAIN_ID"))
        return
    if not (want & set(got)):
        line(BAD, "Data", "%d topics zichtbaar, maar er komt GEEN data binnen" % len(topics), CONN,
             ["andere ROS-versie op robot en laptop (Humble <-> Jazzy)? Zie de versie op de Fenix-pagina."])
        return
    if sim:
        line(OK, "Gazebo", "draait (/clock), %d topics" % len(topics))
        return

    launch = json.loads(got["launch"].data) if "launch" in got else {}
    info = json.loads(got["sys"].data) if "sys" in got else {}
    version = info.get("image_version") or "?"
    line(OK, "Robot", "%s - Fenix %s" % (info.get("hostname", "?"), version.split(" ")[0]), ROBOT)
    robot_distro = "jazzy" if version.startswith("v1.") else "humble" if version.startswith("v0.") else None
    mine = os.environ.get("ROS_DISTRO", "?")
    if robot_distro and robot_distro != mine:
        line(BAD, "ROS-versie", "robot draait %s, deze laptop %s - die praten niet met elkaar" % (robot_distro, mine),
             ROBOT, ["andere image nemen: turtlebot-vis:%s" % ("jazzy" if robot_distro == "jazzy" else "zenoh")])
    else:
        line(OK, "ROS-versie", "%s, zelfde als de robot" % mine, ROBOT)
    line(OK, "Data", "%d topics zichtbaar, berichten komen binnen" % len(topics), ROBOT)

    running = [k for k in ("bringup", "slam", "navigation", "camera", "joystick") if launch.get(k)]
    if not launch.get("bringup"):
        line(WARN, "Bringup", "staat uit - geen /scan, /odom of motoren", ROBOT,
             ["start Bringup op de Fenix-pagina (Rijden)"])
    else:
        missing = [t for t in ("/scan", "/odom", "/tf", "/cmd_vel") if t not in names]
        line(WARN if missing else OK, "Bringup", "draait (actief: %s)" % ", ".join(running), ROBOT,
             ["ontbreekt nog: " + ", ".join(missing)] if missing else ())


def commands(nr, vr=False):
    """Wat de student daarna kan typen (zelfde als de banner van robot_select.sh)."""
    if nr == "htmlsim":
        return "%s  ·  robot (echte robot)  ·  diagnose  ·  check" % (
            "fenix stop (virtuele robot)" if vr else "fenix start (virtuele robot)")
    if nr == "sim":
        return "robot (echte robot)  ·  robot htmlsim  ·  diagnose  ·  check"
    return "robot <nr> (andere robot)  ·  robot sim / robot htmlsim  ·  diagnose  ·  check"


def main():
    nr = os.environ.get("TB_NR")
    html = nr == "htmlsim"
    sim = nr in ("sim", "htmlsim")
    vr_running = False
    mode = {"htmlsim": "HTML-simulator", "sim": "Gazebo-simulatie"}.get(nr, "TurtleBot %s" % nr if nr else "geen robot")
    print(BOLD + "Turtlebot-vis %s" % os.environ.get("TB_VIS_VERSION", "dev") + END
          + "  ·  ROS 2 %s  ·  %s" % (os.environ.get("ROS_DISTRO", "?"), mode))

    rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    if rmw != "rmw_zenoh_cpp":
        line(BAD, "Middleware", "RMW_IMPLEMENTATION=%s, moet rmw_zenoh_cpp zijn" % (rmw or "(niet gezet)"), CONN,
             ["zie docker-compose.yaml - zonder Zenoh ziet de container de robot niet"])
    if not nr:
        line(BAD, "Robot", "nog geen robot gekozen", CONN,
             ["laptop op de wifi TB-AP-<nr>? robot aan? (of: robot <nr>)"])
    else:
        host = "127.0.0.1" if sim else "10.0.%s.10" % nr
        ms = port_open(host, 7447)
        if ms is None:
            line(BAD, "Zenoh-router", "%s:7447 NIET bereikbaar" % host, CONN, [
                "start de simulatie opnieuw: robot %s" % nr if sim else
                "laptop op de wifi TB-AP-%s? robot aan (melodie gehoord)?" % nr])
        else:
            line(OK, "Zenoh-router", "%s:7447 (%.0f ms)" % (host, ms))
            if not sim:
                line(OK if http_ok("http://%s:8080/" % host) else WARN, "Fenix-pagina", "http://%s:8080" % host, BROWSER)
            if html:
                rb = port_open("127.0.0.1", 9090) is not None
                line(OK if rb else BAD, "rosbridge", "ws://127.0.0.1:9090 (voor de simulator)", CONN, () if rb else
                     ["luistert niet - zie /tmp/vis_services/rosbridge.log (of: robot %s)" % nr])
            try:
                ros_checks(sim, html)
            except Exception as exc:     # never a traceback for the student
                line(BAD, "ROS", "controle mislukt: %s" % exc)
            if html:
                port = os.environ.get("VR_PORT", "8090")
                vr_ver = read_first("/root/turtlebot3_ws/ROBOT_VERSION").split(" ")[0]
                if subprocess.run(["pgrep", "-f", "status_page_server.py"], capture_output=True).returncode != 0:
                    line(OFF, "Fenix-pagina", "virtuele robot niet gestart (back-up: fenix start)", BROWSER)
                else:
                    vr_running = True
                    vr = http_ok("http://127.0.0.1:%s/" % port)
                    line(OK if vr else BAD, "Fenix-pagina", "http://127.0.0.1:%s %s (virtuele robot)" % (port, vr_ver),
                         BROWSER, () if vr else ["antwoordt niet - zie /tmp/vis_services/vr_*.log (of: fenix stop; fenix start)"])

    line(OK if port_open("127.0.0.1", 8080) is not None else WARN, "Lichtblick", "http://127.0.0.1:8080", BROWSER)
    bridge = port_open("127.0.0.1", 8765) is not None
    line(OK if bridge else WARN, "Bridge", "ws://127.0.0.1:8765 voor Lichtblick (foxglove_bridge)", CONN,
         () if bridge else ["start pas als de robot bereikbaar is"])
    # dubbele bridges (restjes van een vorige modus): enkel dan een regel
    counts = bridge_count()
    extra = {k: v for k, v in counts.items() if v > 1}
    if extra:
        line(WARN, "Dubbel", ", ".join("%s %d keer" % kv for kv in extra.items()), CONN,
             ["restjes van een vorige modus, opruimen met: robot %s" % (nr or "")])
    display = os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    line(OK if display and (os.path.exists("/tmp/.X11-unix") or os.path.exists("/mnt/wslg")) else WARN,
         "Display", "rviz2/rqt kunnen vensters openen" if display else
         "geen DISPLAY - rviz2/rqt openen geen venster (gebruik Lichtblick)", LAPTOP)
    built = os.path.exists("/root/ros2_ws/install/setup.bash")
    line(OK if os.path.isdir("/root/ros2_ws") else WARN, "ros2_ws",
         "/root/ros2_ws" + (" (gebouwd)" if built else " (nog niet gebouwd: colcon build)"), LAPTOP)
    memory_check()

    print()
    show()
    if count[BAD]:
        print("%s %d probleem/problemen hierboven. Opnieuw: check" % (BAD, count[BAD]))
    elif count[WARN]:
        print("%s Alles werkt, %d opmerking(en) hierboven." % (WARN, count[WARN]))
    else:
        print("%s Alles in orde." % OK)
    print("Commando's: " + commands(nr, vr_running))
    return 1 if count[BAD] else 0


if __name__ == "__main__":
    sys.exit(main())
