"""Services of the robot container, for Status > Diensten (t79).

Shared by system_info_node.py (since when each one runs, in /system_info)
and status_page_server.py (teacher, after the PIN: last lines of the log,
restart). Patterns and start scripts as in services_supervisor.sh; the
[x]yz trick keeps pgrep/pkill from matching the shell that runs them.
"""
import os
import subprocess
import time

# name: (pgrep pattern, log file, start script or None)
SERVICES = {
    "zenoh": ("[r]mw_zenohd", "/tmp/rmw_zenohd.log", None),   # restart = every ROS connection gone
    "rosbridge": ("[l]ib/rosbridge_server/rosbridge_websocket", "/tmp/rosbridge.log", "rosbridge"),
    "webpagina": ("[s]tatus_page_server.py", "/tmp/status_page.log", "/usr/local/bin/status_page_start.sh"),
    "system_info": ("[s]ystem_info_node.py", "/tmp/system_info.log", "/usr/local/bin/status_page_start.sh"),
    "launch_control": ("[l]aunch_control_node.py", "/tmp/launch_control.log", "/usr/local/bin/launch_control_start.sh"),
    "twist_mux": ("[t]wist_mux --ros-args", "/tmp/twist_mux.log", "/usr/local/bin/twist_mux_start.sh"),
    "cmd_vel_guard": ("[c]md_vel_guard.py", "/tmp/cmd_vel_guard.log", "/usr/local/bin/twist_mux_start.sh"),
    "nav_web": ("[n]av_web_node.py", "/tmp/nav_web.log", "/usr/local/bin/nav_web_start.sh"),
    "camera": ("[c]amera_ros/camera_node", "/tmp/camera_node.log", None),     # on/off: Rijden / Beheer
    "terminal": ("[t]tyd -p 7681", "/tmp/ttyd.log", "/usr/local/bin/terminal_start.sh"),
}
# Launches (Rijden / Mappen / Navigatie start and stop them): only the log
LAUNCH_LOGS = {
    "bringup": "/tmp/launch_bringup.log",
    "slam": "/tmp/launch_slam.log",
    "navigation": "/tmp/launch_navigation.log",
    "joystick": "/tmp/launch_joystick.log",
    "supervisor": "/tmp/supervisor.log",
}


def _oldest_pid(pattern):
    try:
        out = subprocess.run(["pgrep", "-o", "-f", pattern], capture_output=True, text=True)
        return int(out.stdout.split()[0]) if out.returncode == 0 else None
    except (OSError, ValueError, IndexError):
        return None


def _start_time(pid):
    """Process start as epoch seconds (field 22 of /proc/<pid>/stat)."""
    with open("/proc/stat") as f:
        btime = next(int(l.split()[1]) for l in f if l.startswith("btime"))
    with open("/proc/%d/stat" % pid) as f:
        ticks = int(f.read().rsplit(")", 1)[1].split()[19])
    return btime + ticks / os.sysconf("SC_CLK_TCK")


def since():
    """{name: epoch the process started, or None when it doesn't run}."""
    out = {}
    for name, (pattern, _log, _script) in SERVICES.items():
        pid = _oldest_pid(pattern)
        try:
            out[name] = round(_start_time(pid)) if pid else None
        except (OSError, ValueError, StopIteration, IndexError):
            out[name] = None
    return out


def log_tail(name, lines=60):
    path = SERVICES[name][1] if name in SERVICES else LAUNCH_LOGS.get(name)
    if not path:
        return {"success": False, "message": "onbekende dienst: %s" % name}
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - 64 * 1024))
            text = f.read().decode("utf-8", "replace")
    except OSError:
        return {"success": True, "message": "%s: nog geen log" % path, "log": ""}
    return {"success": True, "message": path, "log": "\n".join(text.splitlines()[-lines:])}


def restart(name):
    if name not in SERVICES:
        return {"success": False, "message": "onbekende dienst: %s" % name}
    pattern, _log, script = SERVICES[name]
    if name == "zenoh":
        return {"success": False, "message": "Zenoh niet apart herstarten: alle ROS-verbindingen vallen weg - "
                                             "herstart de robot (knop rechtsboven)"}
    if not script:
        return {"success": False, "message": "de camera zet je aan/uit op Rijden (herstart = uit en weer aan)"}
    if name == "rosbridge":
        # as RosbridgeWatchdog in launch_control_node.py: it can ignore SIGTERM
        cmd = ("sleep 1; pkill -9 -f '[r]osbridge_websocket_launch.xml'; pkill -9 -f '%s'; sleep 1; "
               "ros2 launch rosbridge_server rosbridge_websocket_launch.xml websocket_ping_interval:=5.0 "
               "websocket_ping_timeout:=15.0 >> /tmp/rosbridge.log 2>&1" % pattern)
    else:
        # detached and after 1 s: the web page and launch_control can restart
        # themselves (the answer goes out first); the start script skips what
        # still runs, so twist_mux keeps running when only the guard restarts
        cmd = ("sleep 1; pkill -f '%s'; sleep 3; pkill -9 -f '%s'; sleep 1; %s > /dev/null 2>&1"
               % (pattern, pattern, script))
    # ROS environment as in services_start.sh (the caller may not have it)
    cmd = ("source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; source /root/turtlebot3_ws/install/setup.bash; "
           + cmd)
    subprocess.Popen(["bash", "-c", cmd], start_new_session=True,
                     stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return {"success": True, "message": "%s wordt herstart (enkele seconden)%s"
            % (name, " - deze pagina verbindt daarna opnieuw" if name in ("rosbridge", "webpagina") else "")}


if __name__ == "__main__":
    print(since(), time.time())
