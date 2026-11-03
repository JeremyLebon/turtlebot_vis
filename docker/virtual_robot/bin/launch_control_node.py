#!/usr/bin/env python3
# Whitelisted launch control for the status page (lesson 2 convenience -
# lesson 1 has students type these ros2 launch commands themselves in a
# real terminal). Exposes a fixed set of std_srvs/Trigger services - no
# arbitrary commands, just start/stop of specific known launch files.
import glob
import http.server
import json
import os
import queue
import re
import shutil
import signal
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from nav2_msgs.srv import LoadMap

FIXED_LAUNCHES = {
    "bringup": ["ros2", "launch", "turtlebot3_bringup", "robot.launch.py"],
    # Cartographer (as in the TurtleBot3 ROS2 e-manual), not Nav2's
    # slam_toolbox - see cartographer_headless.launch.py for why.
    "slam": ["ros2", "launch", "/root/turtlebot3_ws/launch/cartographer_headless.launch.py"],
    # Same command as camera_start.sh (auto-start at container start). The
    # on/off choice from the status page is remembered in CAMERA_STATE_FILE.
    "camera": ["ros2", "run", "camera_ros", "camera_node", "--ros-args",
               "--params-file", "/tmp/camera_params.yaml",   # camera_params.py
               "-r", "set_camera_info:=/camera/set_camera_info"],
    # F710 -> /cmd_vel_joy (hoogste prioriteit in twist_mux). Los van bringup,
    # niet onthouden: na een herstart staat de joystick uit.
    "joystick": ["ros2", "launch", "teleop_twist_joy", "teleop-launch.py",
                 "joy_vel:=cmd_vel_joy", "publish_stamped_twist:=true",
                 "config_filepath:=/root/turtlebot3_ws/joystick_f710.yaml"],
}

# Virtuele robot (turtlebot-vis, `robot virtueel`): dezelfde statuspagina en
# knoppen op een laptop, met de HTML-simulator als hardware. Bringup = enkel
# wachten op de simulator; SLAM en Navigatie op simulatietijd (/clock).
SIM = os.environ.get("FENIX_SIM") == "1"
if SIM:
    FIXED_LAUNCHES["bringup"] = ["ros2", "launch", "/root/turtlebot3_ws/launch/sim_bringup.launch.py"]

# Status is derived from the actual process list (pgrep), not from "did
# this node start it" bookkeeping - a robot driven after a student typed
# the launch command themselves in a terminal (lesson 1) should show up
# as running too, not just launches started through these buttons.
LAUNCH_PATTERNS = {
    "bringup": "turtlebot3_bringup robot.launch.py",
    # matches both the button's headless launch and a student-typed
    # `ros2 launch turtlebot3_cartographer cartographer.launch.py`
    "slam": "cartographer(_headless)?\\.launch\\.py",
    "navigation": "navigation2.launch.py.*map:=",
    # matches both the `ros2 run` wrapper and the camera_node binary,
    # whether started here or by camera_start.sh
    "camera": "camera_ros[/ ]camera_node",
    "joystick": "teleop-launch\\.py|lib/teleop_twist_joy/teleop_node|lib/joy/joy_node",
}

if SIM:
    LAUNCH_PATTERNS["bringup"] = "sim_bringup\\.launch\\.py"

# Processes a launch starts, for stopping: the launch process itself
# (LAUNCH_PATTERNS) plus its children, which sometimes outlive SIGINT/SIGTERM
# of the launch (2026-10-09: controller_server and planner_server kept
# running as orphans and clashed with the next Navigation start).
CHILD_PATTERNS = {
    "bringup": "lib/turtlebot3_node/turtlebot3_ros|lib/robot_state_publisher/robot_state_publisher"
               "|lib/ld08_driver/|lib/hls_lfcd_lds_driver/",
    "slam": "lib/cartographer_ros/|lib/nav2_map_server/map_saver_server|lifecycle_manager_slam",
    "navigation": "/opt/ros/%s/lib/nav2_" % os.environ.get("ROS_DISTRO", "jazzy"),
    "joystick": "lib/joy/joy_node|lib/teleop_twist_joy/teleop_node",
}
if SIM:
    CHILD_PATTERNS["bringup"] = "sim_bringup_node\\.py"

# Reboot/shutdown go through the host's logind over the system D-Bus socket
# (bind-mounted in docker-compose.yaml) - no sudo, no SSH to the host.
DBUS_SOCKET = "/run/dbus/system_bus_socket"
POWER_MESSAGES = {
    "Reboot": "robot herstart - pagina is binnen ~1 minuut terug",
    "PowerOff": "robot schakelt uit - wacht tot de groene led van de Pi niet meer "
                "knippert voor je de batterij loskoppelt",
}

# Software update: host unit turtlebot-update.service (turtlebot_setup:
# git pull + docker compose pull/up -d), started through systemd over the
# same D-Bus socket. Runs on the host, so recreating this container midway
# doesn't interrupt it. RestartUnit, not StartUnit: the unit has
# RemainAfterExit=yes (keeps the last result), so StartUnit would be a no-op
# after the first run.
UPDATE_UNIT = "turtlebot-update.service"
UPDATE_UNIT_PATH = "/org/freedesktop/systemd1/unit/turtlebot_2dupdate_2eservice"
# Back to the image from before the last update (turtlebot_setup
# rollback_turtlebot.sh, image :vorige) - works without internet
ROLLBACK_UNIT = "turtlebot-rollback.service"
ROLLBACK_UNIT_PATH = "/org/freedesktop/systemd1/unit/turtlebot_2drollback_2eservice"
# A fixed, tagged image version (Beheer > Versie kiezen, turtlebot_robot
# install_version.sh); template unit, one instance per version
INSTALL_UNIT = "turtlebot-install@%s.service"
VERSION_TAG_RE = re.compile(r"^v\d{1,3}\.\d{1,3}\.\d{1,3}$")
DOCKER_HUB_TAGS = "https://hub.docker.com/v2/repositories/nobel86/turtlebot-rpi5/tags?page_size=100"


# Camera on/off choice, read by camera_start.sh at every container start.
# /root/turtlebot_state is bind-mounted from the host (docker-compose.yaml).
CAMERA_STATE_FILE = "/root/turtlebot_state/camera_enabled"

# Saved maps (SLAM "Kaart opslaan" -> map_saver), in the host-mounted state
# dir so they survive updates/recreating the container.
MAP_DIR = "/root/turtlebot_state/maps"
# Map chosen on nav.html for the next Navigation start (name without .yaml)
NAV_OPTIONS_FILE = "/root/turtlebot_state/nav_options.json"
# Own Nav2 settings (nav.html "Geavanceerd", or edited by hand): applied on
# top of the generated parameters at every Navigation start. Flat per node:
#   controller_server:
#     MPPI.vx_max: 0.18
#   local_costmap/local_costmap:
#     inflation_layer.inflation_radius: 0.5
NAV2_OVERRIDES_FILE = "/root/turtlebot_state/nav2_overrides.yaml"
# TurtleBot3 Nav2 parameters + extra planners/controllers, written at every
# Navigation start (see write_nav2_params)
NAV2_PARAMS_OUT = "/tmp/nav2_params.yaml"
TB3_NAV2_PARAMS = ("/root/turtlebot3_ws/install/turtlebot3_navigation2/share/"
                   "turtlebot3_navigation2/param/burger.yaml")

# Extra global planners next to TurtleBot3's "GridBased" (NavFn, Dijkstra),
# all loaded at once so nav.html can pick one per goal (planner_id in the
# behavior tree, see nav_web_node.py) without restarting Nav2.
EXTRA_PLANNERS = {
    "AStar": {"plugin": "nav2_navfn_planner::NavfnPlanner", "use_astar": True,
              "tolerance": 0.5, "allow_unknown": True},
    "Smac2D": {"plugin": "nav2_smac_planner::SmacPlanner2D", "tolerance": 0.125,
               "downsample_costmap": False, "downsampling_factor": 1, "allow_unknown": True,
               "max_iterations": 1000000, "max_on_approach_iterations": 1000,
               "max_planning_time": 2.0, "cost_travel_multiplier": 2.0,
               "use_final_approach_orientation": False,
               "smoother": {"max_iterations": 1000, "w_smooth": 0.3, "w_data": 0.2,
                            "tolerance": 1.0e-10}},
    "ThetaStar": {"plugin": "nav2_theta_star_planner::ThetaStarPlanner", "how_many_corners": 8,
                  "w_euc_cost": 1.0, "w_traversal_cost": 2.0, "w_heuristic_cost": 1.0,
                  "use_final_approach_orientation": False, "allow_unknown": True},
}
# Extra local controllers next to TurtleBot3's "FollowPath" (DWB). Speeds
# within the burger's limits (0.21 m/s, see HW_MAX_LINEAR in cmd_vel_guard.py). MPPI is installed too but heavy on
# a Pi 5 next to everything else - not offered (yet).
EXTRA_CONTROLLERS = {
    "RPP": {"plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
            "desired_linear_vel": 0.18, "lookahead_dist": 0.4, "min_lookahead_dist": 0.25,
            "max_lookahead_dist": 0.8, "lookahead_time": 1.5,
            "rotate_to_heading_angular_vel": 1.2, "transform_tolerance": 0.2,
            "use_velocity_scaled_lookahead_dist": False, "min_approach_linear_velocity": 0.05,
            "approach_velocity_scaling_dist": 0.5, "use_collision_detection": True,
            "max_allowed_time_to_collision_up_to_carrot": 1.0,
            "use_regulated_linear_velocity_scaling": True,
            "use_cost_regulated_linear_velocity_scaling": False,
            "regulated_linear_scaling_min_radius": 0.9, "regulated_linear_scaling_min_speed": 0.1,
            "use_rotate_to_heading": True, "allow_reversing": False,
            "rotate_to_heading_min_angle": 0.785, "max_angular_accel": 3.2,
            "max_robot_pose_search_dist": 10.0, "use_interpolation": False},
    # MPPI (Nav2 docs example) scaled down for the Pi 5 and the Burger:
    # 1000 instead of 2000 sampled trajectories, a 3 s horizon of 30 x 0.1 s
    # steps (model_dt = 1 / controller_frequency 10 Hz), Burger limits
    # (0.21 m/s, 1.0 rad/s, no reversing).
    "MPPI": {"plugin": "nav2_mppi_controller::MPPIController",
             "time_steps": 30, "model_dt": 0.1, "batch_size": 1000, "iteration_count": 1,
             "vx_std": 0.2, "vy_std": 0.2, "wz_std": 0.4,
             "vx_max": 0.21, "vx_min": 0.0, "vy_max": 0.0, "wz_max": 1.0,
             "ax_max": 2.5, "ax_min": -2.5, "az_max": 3.2,
             "prune_distance": 1.7, "transform_tolerance": 0.2, "temperature": 0.3, "gamma": 0.015,
             "motion_model": "DiffDrive", "visualize": False, "reset_period": 1.0,
             "regenerate_noises": False,
             "TrajectoryVisualizer": {"trajectory_step": 5, "time_step": 3},
             "critics": ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic",
                         "PathAlignCritic", "PathFollowCritic", "PathAngleCritic",
                         "PreferForwardCritic"],
             "ConstraintCritic": {"enabled": True, "cost_power": 1, "cost_weight": 4.0},
             "GoalCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0,
                            "threshold_to_consider": 1.4},
             # 0.5 (Nav2 default): the robot turned to the goal heading
             # ~0.3 m before the goal and then could not reach it sideways
             # (no reversing) -> "Failed to make progress" (2026-10-09)
             "GoalAngleCritic": {"enabled": True, "cost_power": 1, "cost_weight": 3.0,
                                 "threshold_to_consider": 0.25},
             "PreferForwardCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0,
                                     "threshold_to_consider": 0.5},
             "CostCritic": {"enabled": True, "cost_power": 1, "cost_weight": 3.81,
                            "critical_cost": 300.0, "consider_footprint": False,
                            "collision_cost": 1000000.0, "near_goal_distance": 1.0,
                            "trajectory_point_step": 2},
             "PathAlignCritic": {"enabled": True, "cost_power": 1, "cost_weight": 14.0,
                                 "max_path_occupancy_ratio": 0.05, "trajectory_point_step": 4,
                                 "threshold_to_consider": 0.5, "offset_from_furthest": 20,
                                 "use_path_orientations": False},
             "PathFollowCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0,
                                  "offset_from_furthest": 5, "threshold_to_consider": 1.4},
             "PathAngleCritic": {"enabled": True, "cost_power": 1, "cost_weight": 2.0,
                                 "offset_from_furthest": 4, "threshold_to_consider": 0.5,
                                 "max_angle_to_furthest": 1.0, "mode": 0}},
}


def list_maps():
    return sorted(os.path.splitext(os.path.basename(p))[0]
                  for p in glob.glob(os.path.join(MAP_DIR, "*.yaml")))


# Files of one map: the map itself plus what belongs to it (zones and routes
# from nav_web_node, the original before the first edit in the map editor).
MAP_SUFFIXES = (".yaml", ".pgm", ".zones.json", ".routes.json", ".orig.pgm",
                ".zones.bak.json", ".edited.pgm")
MAP_NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,40}$")


def running_map():
    """Name of the map Navigation runs with, or None."""
    for pid in _pgrep_pids(LAUNCH_PATTERNS["navigation"]):
        try:
            with open("/proc/%d/cmdline" % pid, "rb") as f:
                args = f.read().split(b"\0")
        except OSError:
            continue
        for a in args:
            if a.startswith(b"map:="):
                return os.path.splitext(os.path.basename(a[5:].decode()))[0]
    return None


def map_files(name):
    return [(suf, os.path.join(MAP_DIR, name + suf)) for suf in MAP_SUFFIXES
            if os.path.exists(os.path.join(MAP_DIR, name + suf))]


def map_command(cmd):
    """rename / copy / delete a map with its zones and routes -> message."""
    kind, src = cmd.get("cmd"), str(cmd.get("name", ""))
    if src not in list_maps():
        return "kaart '%s' bestaat niet" % src
    if kind in ("rename", "delete") and src == running_map():
        return "kaart '%s' is in gebruik door Navigatie - stop Navigatie eerst" % src
    if kind == "delete":
        for _, path in map_files(src):
            os.remove(path)
        opts = nav_options()
        if opts.get("map") == src:
            opts.pop("map")
            save_nav_options(opts)
        return "kaart '%s' verwijderd" % src
    dst = str(cmd.get("to", "")).strip()
    if not MAP_NAME_RE.match(dst):
        return "ongeldige naam '%s' (enkel letters, cijfers, - en _)" % dst
    if dst in list_maps():
        return "kaart '%s' bestaat al" % dst
    for suf, path in map_files(src):
        target = os.path.join(MAP_DIR, dst + suf)
        if suf == ".yaml":   # the yaml names its image file
            with open(path) as f:
                text = f.read()
            text = re.sub(r"(?m)^image:.*$", "image: " + dst + ".pgm", text)
            with open(target, "w") as f:
                f.write(text)
            if kind == "rename":
                os.remove(path)
        elif kind == "rename":
            os.rename(path, target)
        else:
            shutil.copyfile(path, target)
    if kind == "rename":
        opts = nav_options()
        if opts.get("map") == src:
            opts["map"] = dst
            save_nav_options(opts)
        return "kaart '%s' hernoemd naar '%s'" % (src, dst)
    return "kaart '%s' gekopieerd naar '%s'" % (src, dst)


def nav_options():
    try:
        with open(NAV_OPTIONS_FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def nav2_overrides():
    import yaml
    try:
        with open(NAV2_OVERRIDES_FILE) as f:
            data = yaml.safe_load(f) or {}
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError, yaml.YAMLError):
        return {}


def save_nav2_overrides(data):
    import yaml
    os.makedirs(os.path.dirname(NAV2_OVERRIDES_FILE), exist_ok=True)
    with open(NAV2_OVERRIDES_FILE + ".tmp", "w") as f:
        f.write("# Eigen Nav2-instellingen (nav.html > Geavanceerd, of met de hand).\n"
                "# Toegepast bij elke start van Navigatie. Per node: parameter: waarde\n")
        yaml.safe_dump(data, f, sort_keys=True)
    os.replace(NAV2_OVERRIDES_FILE + ".tmp", NAV2_OVERRIDES_FILE)


def apply_nav2_overrides(params):
    """Put the overrides (node -> {"a.b.c": value}) into the Nav2 params dict."""
    for node, values in nav2_overrides().items():
        if not isinstance(values, dict) or node == "nav_web_node":
            continue   # nav_web_node reads its own section
        section = params
        for part in str(node).split("/"):
            section = section.setdefault(part, {})
        section = section.setdefault("ros__parameters", {})
        for name, value in values.items():
            target = section
            parts = str(name).split(".")
            for part in parts[:-1]:
                target = target.setdefault(part, {})
            target[parts[-1]] = value


def save_nav_options(opts):
    try:
        os.makedirs(os.path.dirname(NAV_OPTIONS_FILE), exist_ok=True)
        with open(NAV_OPTIONS_FILE, "w") as f:
            json.dump(opts, f)
    except OSError:
        pass


def selected_map():
    """Chosen map if it still exists, else "map", else the first one."""
    maps = list_maps()
    choice = nav_options().get("map")
    if choice in maps:
        return choice
    if "map" in maps:
        return "map"
    return maps[0] if maps else None


def write_nav2_params():
    import yaml
    with open(TB3_NAV2_PARAMS) as f:
        params = yaml.safe_load(f)
    if SIM:   # alle nodes op de klok van de simulator (/clock)
        for section in params.values():
            for sub in (section, *[v for v in section.values() if isinstance(v, dict)]):
                if isinstance(sub, dict) and "ros__parameters" in sub:
                    sub["ros__parameters"]["use_sim_time"] = True
    planner = params["planner_server"]["ros__parameters"]
    for name, cfg in EXTRA_PLANNERS.items():
        planner[name] = cfg
        if name not in planner["planner_plugins"]:
            planner["planner_plugins"].append(name)
    controller = params["controller_server"]["ros__parameters"]
    # DWB: 0.21 instead of TB3's 0.22 - at 0.22 the XL430 motors refuse the
    # command and the wheels stop (t78, cmd_vel_guard caps it anyway)
    controller["FollowPath"]["max_vel_x"] = 0.21
    controller["FollowPath"]["max_speed_xy"] = 0.21
    for name, cfg in EXTRA_CONTROLLERS.items():
        controller[name] = cfg
        if name not in controller["controller_plugins"]:
            controller["controller_plugins"].append(name)
    # A goal with another planner/controller loads another behavior tree,
    # whose BT nodes create new action/service clients. Over Zenoh their
    # first request takes longer than Nav2's default 20 ms acknowledge
    # timeout: the first goal after a switch failed at once, and its
    # timed-out recovery requests (spin, backup, wait) still ran afterwards
    # (2026-10-09, turtlebot09). Under Jazzy 1 s was still too short: every
    # second goal from nav.html failed with "Timed out while waiting for action
    # server to acknowledge goal request for compute_path_to_pose" (2026-10-28,
    # turtlebot09). Only an upper limit, a normal goal is not slower.
    params["bt_navigator"]["ros__parameters"]["default_server_timeout"] = 5000
    # Map: only via the launch's map:= argument. Jazzy's localization_launch
    # passes it as a "/**" wildcard parameter, and TurtleBot3's
    # "map_server: yaml_filename: map.yaml" names the node, so it wins over
    # the wildcard: map_server tried the bare map.yaml and Nav2 did not come
    # up (2026-10-28, turtlebot06 and 09).
    params.get("map_server", {}).get("ros__parameters", {}).pop("yaml_filename", None)
    # Goal tolerance "normaal" (10 cm, ~9 deg) instead of TB3's 25 cm / 14 deg -
    # also for Simple Commander. nav.html sets it per goal (nav_web_node).
    # Jazzy's TurtleBot3 parameters call the goal checker "goal_checker",
    # Humble's "general_goal_checker"; keep the Humble name, used by
    # nav_web_node (GOAL_CHECKER) and in saved nav2_overrides.yaml files.
    if "general_goal_checker" not in controller:
        old = controller["goal_checker_plugins"][0]
        controller["general_goal_checker"] = controller.pop(old)
        controller["goal_checker_plugins"] = ["general_goal_checker"]
    checker = controller["general_goal_checker"]
    checker["xy_goal_tolerance"] = 0.10
    checker["yaw_goal_tolerance"] = 0.15
    # collision_monitor (new in Jazzy): TurtleBot3 sets source_timeout 0.2 s
    # for /scan, exactly one scan period of the 5 Hz lidar - nearly every
    # scan counted as "too old" and the monitor kept stopping the robot, Nav2
    # goals failed (2026-10-22, Jazzy simulation). 1 s = 5 scans.
    monitor = params.get("collision_monitor", {}).get("ros__parameters", {})
    for source in monitor.get("observation_sources", []):
        monitor[source]["source_timeout"] = max(monitor[source].get("source_timeout", 0.0), 1.0)
    # Zones (nav.html, nav_web_node publishes the masks): no-go zones and
    # virtual walls in both costmaps, preferred zones only for the planner.
    keepout = {"plugin": "nav2_costmap_2d::KeepoutFilter", "enabled": True,
               "filter_info_topic": "/keepout_filter_info"}
    lanes = {"plugin": "nav2_costmap_2d::KeepoutFilter", "enabled": True,
             "filter_info_topic": "/lanes_filter_info"}
    gc = params["global_costmap"]["global_costmap"]["ros__parameters"]
    gc["filters"] = ["keepout_filter", "lanes_filter", "speed_filter"]
    gc["keepout_filter"], gc["lanes_filter"] = keepout, lanes
    # speed zones: SpeedFilter publishes /speed_limit (percent of max speed),
    # controller_server applies it to DWB / RPP / MPPI
    gc["speed_filter"] = {"plugin": "nav2_costmap_2d::SpeedFilter", "enabled": True,
                          "filter_info_topic": "/speed_filter_info",
                          "speed_limit_topic": "/speed_limit"}
    params["controller_server"]["ros__parameters"]["speed_limit_topic"] = "/speed_limit"
    lc = params["local_costmap"]["local_costmap"]["ros__parameters"]
    lc["filters"] = ["keepout_filter"]
    lc["keepout_filter"] = dict(keepout)
    apply_nav2_overrides(params)
    with open(NAV2_PARAMS_OUT, "w") as f:
        yaml.safe_dump(params, f, sort_keys=False)
    return NAV2_PARAMS_OUT


def camera_sensor():
    """Sensor name of the attached CSI camera, or None (camera_present.sh)."""
    out = subprocess.run(["/usr/local/bin/camera_present.sh"], capture_output=True, text=True)
    return out.stdout.strip() if out.returncode == 0 else None


# /dev/input is bind-mounted from the host (docker-compose.yaml), so a
# joystick receiver plugged in after the container started shows up too.
JOYSTICK_DEV = "/dev/input/js0"


def joystick_name():
    """Name of the attached joystick (e.g. "Logitech Gamepad F710"), or None."""
    if not os.path.exists(JOYSTICK_DEV):
        return None
    try:
        with open("/sys/class/input/js0/device/name") as f:
            return f.read().strip() or "joystick"
    except OSError:
        return "joystick"


# USB power (usb_power.sh): the Pi 5 can only switch its 4 USB ports
# together, and the LDS-02 spins whenever it has USB power (rated 1000 h).
# In "auto" mode the power goes off when nothing has used the robot's USB
# devices for USB_IDLE_OFF seconds, and back on when a launch needs them.
USB_POWER = "/usr/local/bin/usb_power"
USB_AUTO_FILE = "/root/turtlebot_state/usb_auto"
USB_IDLE_OFF = 120
USB_LAUNCHES = ("bringup", "slam", "navigation", "joystick")


def usb_auto():
    try:
        with open(USB_AUTO_FILE) as f:
            return f.read().strip() != "false"
    except OSError:
        return True


def set_usb_auto(enabled):
    try:
        with open(USB_AUTO_FILE, "w") as f:
            f.write("true" if enabled else "false")
    except OSError:
        pass


def usb_power(action):
    """'on' (waits for OpenCR + lidar), 'off', or 'status' -> "on"/"off"."""
    if SIM:
        return "on"   # geen USB-hub: de simulator is altijd "aan"
    args = [USB_POWER, action] + (["--wait"] if action == "on" else [])
    out = subprocess.run(args, capture_output=True, text=True, timeout=30)
    return out.stdout.strip()


def usb_devices_open():
    """True if any process in the container has the OpenCR, the lidar or a
    joystick open (also a driver a student started from a terminal)."""
    for fd_dir in glob.glob("/proc/[0-9]*/fd"):
        try:
            for fd in os.listdir(fd_dir):
                try:
                    target = os.readlink(os.path.join(fd_dir, fd))
                except OSError:
                    continue
                if target in ("/dev/ttyACM0", "/dev/ttyUSB0") or target.startswith("/dev/input/"):
                    return True
        except OSError:
            continue
    return False


def camera_enabled():
    try:
        with open(CAMERA_STATE_FILE) as f:
            return f.read().strip() == "true"
    except OSError:
        return os.environ.get("ENABLE_CAMERA", "false") == "true"


def set_camera_enabled(enabled):
    try:
        os.makedirs(os.path.dirname(CAMERA_STATE_FILE), exist_ok=True)
        with open(CAMERA_STATE_FILE, "w") as f:
            f.write("true" if enabled else "false")
    except OSError:
        pass


def _busctl(*args):
    return subprocess.run(["busctl", "--json=short"] + list(args),
                          capture_output=True, text=True, timeout=5)


# Desktop (HDMI-scherm) on the host: off by default since 2026-10-10
# (turtlebot_setup setup_turtlebot.sh reads the same file at every boot/update
# and sets the default target). Switched live via systemd on the host.
DESKTOP_FILE = "/root/turtlebot_state/desktop"
DM_UNIT = "lightdm.service"
DM_UNIT_PATH = "/org/freedesktop/systemd1/unit/lightdm_2eservice"


def desktop_wanted():
    try:
        with open(DESKTOP_FILE) as f:
            return f.read().strip() == "true"
    except OSError:
        return False


def desktop_state():
    """{"wanted": bool, "active": lightdm ActiveState or "unknown"}"""
    out = _busctl("get-property", "org.freedesktop.systemd1", DM_UNIT_PATH,
                  "org.freedesktop.systemd1.Unit", "ActiveState")
    try:
        active = json.loads(out.stdout)["data"] if out.returncode == 0 else "unknown"
    except (ValueError, KeyError):
        active = "unknown"
    return {"wanted": desktop_wanted(), "active": active}


def set_desktop(enabled):
    """Remember the choice, set the host's default target and start/stop the
    display manager now (no reboot needed). Returns (ok, message)."""
    with open(DESKTOP_FILE, "w") as f:
        f.write("true" if enabled else "false")
    target = "graphical.target" if enabled else "multi-user.target"
    out = _busctl("call", "org.freedesktop.systemd1", "/org/freedesktop/systemd1",
                  "org.freedesktop.systemd1.Manager", "SetDefaultTarget", "sb", target, "true")
    if out.returncode != 0:
        return False, "kon de opstartmodus niet zetten: " + out.stderr.strip()
    out = _busctl("call", "org.freedesktop.systemd1", "/org/freedesktop/systemd1",
                  "org.freedesktop.systemd1.Manager", "StartUnit" if enabled else "StopUnit", "ss",
                  DM_UNIT, "replace")
    if out.returncode != 0:
        return False, ("opstartmodus %s ingesteld, maar %s %s lukte niet (desktop niet geïnstalleerd?): %s"
                       % (target, DM_UNIT, "starten" if enabled else "stoppen", out.stderr.strip()))
    return True, ("desktop gestart - ook na een herstart" if enabled
                  else "desktop gestopt - ook na een herstart")


UPDATE_STATUS_FILE = "/root/turtlebot_state/update_status.txt"


def _update_message():
    """Last step / error written by update_turtlebot.sh on the host."""
    try:
        with open(UPDATE_STATUS_FILE) as f:
            return f.read().strip() or None
    except OSError:
        return None


def _previous_image():
    """Version of the host's :vorige image (setup_turtlebot.sh writes it to
    host_info.json after every update/rollback/boot), or None."""
    try:
        with open("/root/turtlebot_state/host_info.json") as f:
            return json.load(f).get("image_previous") or None
    except (OSError, ValueError):
        return None


ADMIN_PIN_FILE = "/root/turtlebot_state/admin_pin"
ADMIN_HASH_RE = re.compile(r"^pbkdf2_sha256:\d{4,7}:[0-9a-f]{32}:[0-9a-f]{64}$")


def set_admin_pin_hash(h):
    """Teacher PIN for the whole fleet from the fleet monitor: only the PBKDF2
    hash travels (fenix_admin.py format), never the PIN itself."""
    if not ADMIN_HASH_RE.match(h):
        return False, "ongeldige PIN-hash"
    tmp = ADMIN_PIN_FILE + ".tmp"
    with open(tmp, "w") as f:
        f.write(h + "\n")
    os.chmod(tmp, 0o600)
    os.replace(tmp, ADMIN_PIN_FILE)
    return True, "docenten-PIN ingesteld"


def _internet_ok():
    """Quick check before starting an update (same idea as the script's own
    check), so the page says "no internet" at once instead of after a timeout."""
    for host in ("1.1.1.1", "8.8.8.8"):
        try:
            with socket.create_connection((host, 443), timeout=3):
                return True
        except OSError:
            pass
    return False


def _update_status():
    """{"state": activating/active/failed/inactive/unknown, "result": ...,
    "finished": epoch seconds of the last finished run or None}"""
    try:
        out = _busctl("get-property", "org.freedesktop.systemd1", UPDATE_UNIT_PATH,
                      "org.freedesktop.systemd1.Unit", "ActiveState")
        state = json.loads(out.stdout)["data"] if out.returncode == 0 else "unknown"
        out = _busctl("get-property", "org.freedesktop.systemd1", UPDATE_UNIT_PATH,
                      "org.freedesktop.systemd1.Service", "Result", "ExecMainExitTimestamp")
        lines = out.stdout.splitlines() if out.returncode == 0 else []
        result = json.loads(lines[0])["data"] if len(lines) > 0 else None
        exit_usec = json.loads(lines[1])["data"] if len(lines) > 1 else 0
        out = _busctl("get-property", "org.freedesktop.systemd1", ROLLBACK_UNIT_PATH,
                      "org.freedesktop.systemd1.Unit", "ActiveState")
        rollback = json.loads(out.stdout)["data"] if out.returncode == 0 else "unknown"
        return {"state": state, "result": result,
                "finished": exit_usec / 1e6 if exit_usec else None,
                "message": _update_message(), "rollback": rollback,
                "install": "activating" if _install_active() else "inactive"}
    except Exception:
        return {"state": "unknown", "result": None, "finished": None, "rollback": "unknown"}


def _install_active():
    """True while a turtlebot-install@<version> unit runs."""
    out = _busctl("call", "org.freedesktop.systemd1", "/org/freedesktop/systemd1",
                  "org.freedesktop.systemd1.Manager", "ListUnitsByPatterns", "asas",
                  "1", "activating", "1", "turtlebot-install@*")
    try:
        return out.returncode == 0 and len(json.loads(out.stdout)["data"][0]) > 0
    except (ValueError, KeyError, IndexError):
        return False


def list_versions():
    """Tagged image versions on Docker Hub (vX.Y.Z), newest first. In the
    http thread of AdminCommands, not the executor (can take seconds)."""
    try:
        with urllib.request.urlopen(DOCKER_HUB_TAGS, timeout=8) as r:
            tags = json.loads(r.read()).get("results", [])
    except (OSError, ValueError) as exc:
        return {"success": False, "message": "Docker Hub niet bereikbaar (internet via de AP?): %s" % exc}
    versions = [{"tag": t["name"], "date": (t.get("last_updated") or "")[:10]}
                for t in tags if VERSION_TAG_RE.match(t.get("name", ""))]
    versions.sort(key=lambda v: [int(x) for x in v["tag"][1:].split(".")], reverse=True)
    return {"success": True, "message": "%d versies" % len(versions), "versions": versions[:20],
            "current": os.environ.get("TURTLEBOT_IMAGE_VERSION")}


def _pgrep_pids(pattern):
    try:
        out = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True)
        return [int(p) for p in out.stdout.split()]
    except Exception:
        return []


# Fleet monitor (turtlebot_setup/monitor/monitor.py on the teacher's laptop):
# the robot asks the monitor for commands (long-poll on <base>/poll), because
# the monitor can't reach the robot behind its AP's NAT. Only this fixed list,
# executed like the status page buttons; results go back to <base>/result.
MONITOR_URL = os.environ.get("MONITOR_URL", "").strip()
MONITOR_BASE = MONITOR_URL[:-len("/push")] if MONITOR_URL.endswith("/push") else ""
MONITOR_LAUNCHES = ("bringup", "slam", "navigation", "camera", "joystick")
# system.html: commands from the monitor allowed on this robot (default yes)
MONITOR_CONTROL_FILE = "/root/turtlebot_state/monitor_control"
# AdminCommands: Beheer actions from status_page_server.py (after the PIN)
ADMIN_PORT = 8091


# Updatekanaal (turtlebot_robot setup_turtlebot.sh): "jazzy" = tag :jazzy bij
# Software bijwerken, anders :raspios-zenoh. Gekozen in Beheer > Software.
IMAGE_CHANNEL_FILE = "/root/turtlebot_state/image_channel"


def image_channel():
    try:
        with open(IMAGE_CHANNEL_FILE) as f:
            return "jazzy" if f.read().strip() == "jazzy" else "humble"
    except OSError:
        return "humble"


def monitor_control():
    try:
        with open(MONITOR_CONTROL_FILE) as f:
            return f.read().strip() != "false"
    except OSError:
        return True
MONITOR_COMMANDS = (["start:" + n for n in MONITOR_LAUNCHES] + ["stop:" + n for n in MONITOR_LAUNCHES] +
                    ["stop_all", "beep", "update", "reboot", "shutdown", "set_pin",
                     # map library on the monitor (map_bundle.py), "arg" = map name
                     "map_upload", "map_download"])


class RosbridgeWatchdog:
    """Restarts rosbridge when it no longer passes data on.

    2026-10-09: during an overload (SLAM + two Nav2 stacks, load 8) the Zenoh
    session of rosbridge got stuck ("Didn't receive DeclareFinal ... Timeout");
    it still accepted the page's connections and subscriptions but delivered
    nothing, and ignored SIGTERM. Every 30 s: connect to localhost:9090,
    subscribe to /launch_status (this node publishes it every second) and wait
    5 s for a message. Twice nothing (or no connection) -> kill -9 + restart.
    """
    PERIOD, WAIT, GRACE = 30.0, 5.0, 60.0

    def __init__(self, log):
        self.log = log
        self.fails = 0
        self.restarts = 0
        self.last_restart = None
        threading.Thread(target=self._loop, daemon=True).start()

    @staticmethod
    def _probe():
        """True if a message arrives through rosbridge within WAIT seconds."""
        import base64
        import struct
        s = socket.create_connection(("127.0.0.1", 9090), timeout=5)
        try:
            key = base64.b64encode(os.urandom(16)).decode()
            s.sendall(("GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                       "Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n" % key).encode())
            data = b""
            while b"\r\n\r\n" not in data:
                chunk = s.recv(1024)
                if not chunk:
                    return False
                data += chunk
            if b" 101 " not in data.split(b"\r\n", 1)[0]:
                return False
            data = data.split(b"\r\n\r\n", 1)[1]
            body = json.dumps({"op": "subscribe", "topic": "/launch_status",
                               "type": "std_msgs/msg/String", "id": "watchdog"}).encode()
            mask = os.urandom(4)
            header = bytes([0x81]) + (bytes([0x80 | len(body)]) if len(body) < 126
                                      else bytes([0x80 | 126]) + struct.pack(">H", len(body)))
            s.sendall(header + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(body)))
            s.settimeout(0.5)
            end = time.time() + RosbridgeWatchdog.WAIT
            while time.time() < end:
                if b"/launch_status" in data and b'"op": "publish"' in data.replace(b'"op":"publish"', b'"op": "publish"'):
                    return True
                try:
                    chunk = s.recv(65536)
                    if not chunk:
                        return False
                    data += chunk
                except socket.timeout:
                    pass
            return False
        finally:
            s.close()

    def _restart(self):
        for pat in ("rosbridge_websocket_launch.xml", "lib/rosbridge_server/rosbridge_websocket"):
            for pid in _pgrep_pids(pat):
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        time.sleep(1)
        with open("/tmp/rosbridge.log", "a") as f:
            subprocess.Popen(["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml",
                              "websocket_ping_interval:=5.0", "websocket_ping_timeout:=15.0"],   # as status_page_start.sh
                             preexec_fn=os.setsid, stdout=f, stderr=subprocess.STDOUT)
        self.restarts += 1
        self.last_restart = time.time()

    def _loop(self):
        time.sleep(self.GRACE)   # container start: let rosbridge come up first
        while True:
            try:
                ok = self._probe()
            except OSError:
                ok = False
            self.fails = 0 if ok else self.fails + 1
            if self.fails >= 2:
                self.log("rosbridge geeft geen gegevens meer door - herstarten (waakhond)")
                try:
                    self._restart()
                except Exception as exc:
                    self.log("rosbridge herstarten mislukt: %s" % exc)
                self.fails = 0
                time.sleep(self.GRACE)
                continue
            time.sleep(self.PERIOD)


class AdminCommands:
    """Beheer actions (docent, PIN), only on 127.0.0.1:ADMIN_PORT.

    status_page_server.py checks the PIN token and forwards the action here
    (POST {"action", "args"} -> {"success", "message"}). Before 0.21.0 these
    were ROS services/topics (/system/update, /system/rollback, ...): the
    PIN only hid the page, every student with ROS on their laptop could call
    them directly (rosbridge or Zenoh). Not reachable from outside: rosbridge
    and Zenoh only carry ROS, and students have no shell on the robot.
    Run in the executor thread (take() from a timer), like MonitorCommands.
    """
    TIMEOUT = 20.0

    def __init__(self, port):
        self.q = queue.Queue()
        cmds = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_POST(self):
                try:
                    n = int(self.headers.get("Content-Length", 0))
                    body = json.loads(self.rfile.read(min(n, 100000)) or b"{}")
                    item = {"action": str(body.get("action", "")), "args": body.get("args") or {},
                            "done": threading.Event(), "result": None}
                except ValueError:
                    item = None
                if item and item["action"] == "list_versions":
                    out = list_versions()
                elif item:
                    cmds.q.put(item)
                    if item["done"].wait(cmds.TIMEOUT):
                        out = item["result"]
                    else:
                        out = {"success": False, "message": "launch_control antwoordt niet (time-out)"}
                else:
                    out = {"success": False, "message": "ongeldige opdracht"}
                data = json.dumps(out).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *args):
                pass

        srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()

    def take(self):
        items = []
        while True:
            try:
                items.append(self.q.get_nowait())
            except queue.Empty:
                return items


class MonitorCommands:
    """Long-poll thread: queues commands from the monitor for the node."""

    def __init__(self, base, robot):
        self.base, self.robot = base, robot
        self.queue = []
        self.lock = threading.Lock()
        self.last = None          # {"cmd", "message", "ok", "t"} for the status page
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        url = self.base + "/poll?" + urllib.parse.urlencode({"robot": self.robot})
        while True:
            try:
                with urllib.request.urlopen(url, timeout=30) as r:
                    cmds = json.loads(r.read() or b"{}").get("cmds", [])
                with self.lock:
                    self.queue.extend(c for c in cmds if c.get("cmd") in MONITOR_COMMANDS)
            except Exception:
                time.sleep(10)    # monitor not there (laptop off): try again later

    def take(self):
        with self.lock:
            cmds, self.queue = self.queue, []
        return cmds

    def report(self, cmd_id, cmd, ok, message):
        self.last = {"cmd": cmd, "ok": ok, "message": message, "t": time.time()}
        body = json.dumps({"robot": self.robot, "id": cmd_id, "cmd": cmd, "ok": ok,
                           "message": message}).encode()

        def post():
            try:
                req = urllib.request.Request(self.base + "/result", data=body,
                                             headers={"Content-Type": "application/json"})
                urllib.request.urlopen(req, timeout=3).read()
            except Exception:
                pass
        threading.Thread(target=post, daemon=True).start()


# ---- "Aanpassingen": everything that differs from the standard set-up ----
# Shown as an icon on every page (common.js) and in the fleet monitor, so a
# teacher sees at a glance what students changed. Nav2 parameters that can be
# changed live (nav.html "Geavanceerd" > Toepassen, ros2 param set, rqt):
# compared every 10 s with the values Navigation was started with.
LIVE_PARAMS = {
    "controller_server": ["FollowPath.max_vel_x", "FollowPath.max_vel_theta", "RPP.desired_linear_vel",
                          "RPP.lookahead_dist", "MPPI.vx_max", "MPPI.wz_max", "MPPI.PathFollowCritic.cost_weight",
                          "MPPI.PathAlignCritic.cost_weight", "MPPI.GoalCritic.cost_weight",
                          "MPPI.CostCritic.cost_weight"],
    "local_costmap/local_costmap": ["inflation_layer.inflation_radius", "inflation_layer.cost_scaling_factor"],
    "global_costmap/global_costmap": ["inflation_layer.inflation_radius", "inflation_layer.cost_scaling_factor"],
}


def expected_param(node, name):
    """Value in the Nav2 parameters Navigation was started with, or None."""
    import yaml
    try:
        with open(NAV2_PARAMS_OUT) as f:
            section = yaml.safe_load(f)
        for part in node.split("/"):
            section = section[part]
        section = section["ros__parameters"]
        for part in name.split("."):
            section = section[part]
        return section
    except (OSError, KeyError, TypeError):
        return None


def zones_count(map_name):
    try:
        with open(os.path.join(MAP_DIR, map_name + ".zones.json")) as f:
            return len(json.load(f).get("zones", []))
    except (OSError, ValueError):
        return 0


class LaunchControlNode(Node):
    def __init__(self):
        super().__init__("launch_control_node")
        for name in ("bringup", "slam", "navigation", "camera", "joystick"):
            self.create_service(Trigger, "/launch/" + name, self._make_start_cb(name))
            self.create_service(Trigger, "/launch/stop_" + name, self._make_stop_one_cb(name))
        self.create_service(Trigger, "/launch/stop_all", self._stop_all_cb)
        self.create_service(Trigger, "/system/reboot", self._make_power_cb("Reboot"))
        self.create_service(Trigger, "/system/shutdown", self._make_power_cb("PowerOff"))
        # Beheer (update, rollback, desktop, USB, monitor, camera): not as ROS
        # services/topics, only via status_page_server.py after the PIN (AdminCommands)
        self.admin = AdminCommands(ADMIN_PORT)
        self.create_timer(0.2, self._admin_tick)

        # Power off soon after boot (once the "on" melody played), not after
        # the full idle time - the lidar then runs ~1 min per boot.
        self.usb_state = usb_power("status")
        self.usb_last_use = time.monotonic() - USB_IDLE_OFF + 60
        self.create_timer(5.0, self._usb_idle_check)

        self.monitor = MonitorCommands(MONITOR_BASE, socket.gethostname()) if MONITOR_BASE else None
        self.rb_watchdog = RosbridgeWatchdog(lambda m: self.get_logger().warn(m))
        if self.monitor:
            # in the executor thread, so commands run like the service callbacks
            self.create_timer(0.3, self._monitor_tick)
        # projects.html: {"project", "launch", "action": "start"|"stop"}
        self.create_subscription(String, "/launch/project_cmd", self._on_project_cmd, 10)
        self.project_msg = None

        # Aanpassingen: live Nav2 values (checked every 10 s), reset requests
        self.live_drift = {}          # "node|name" -> (live, expected)
        self.param_clients = {n: (self.create_client(GetParameters, "/%s/get_parameters" % n),
                                  self.create_client(SetParameters, "/%s/set_parameters" % n))
                              for n in LIVE_PARAMS}
        self.create_timer(10.0, self._check_live_params)
        self.create_subscription(String, "/system/reset", self._on_reset, 10)
        self.nav_cmd_pub = self.create_publisher(String, "/web/nav_cmd", 10)
        self.loadmap_client = self.create_client(LoadMap, "/map_server/load_map")

        # nav.html: {"map": "<name>"} - map for the next Navigation start
        self.create_subscription(String, "/launch/nav_options", self._on_nav_options, 10)
        # nav.html "Geavanceerd": {"overrides": {node: {name: value}}} replaces the file
        self.create_subscription(String, "/launch/nav2_overrides", self._on_nav2_overrides, 10)
        # nav.html map management: {"cmd": "rename"|"copy"|"delete", "name", "to"}
        self.create_subscription(String, "/launch/map_cmd", self._on_map_cmd, 10)
        self.map_msg = None   # {"text", "t"} of the last map command

        self.status_pub = self.create_publisher(String, "/launch_status", 10)
        self.create_timer(1.0, self._publish_status)

        self.nodes_pub = self.create_publisher(String, "/active_nodes", 10)
        self.create_timer(2.0, self._publish_active_nodes)

        self.update_status = None
        self.desktop = None
        self.update_pub = self.create_publisher(String, "/system/update_status", 10)
        self.create_timer(2.0, self._publish_update_status)

        self.get_logger().info("launch_control_node ready: /launch/{bringup,slam,navigation,camera,joystick,stop_all}, /system/{reboot,shutdown,update}")

    # ---- aanpassingen -------------------------------------------------------
    def _check_live_params(self):
        if not self._is_running("navigation"):
            self.live_drift = {}
            return
        for node, names in LIVE_PARAMS.items():
            get_cli = self.param_clients[node][0]
            if not get_cli.service_is_ready():
                continue
            req = GetParameters.Request()
            req.names = names
            get_cli.call_async(req).add_done_callback(lambda f, node=node, names=names: self._on_live(f, node, names))

    def _on_live(self, future, node, names):
        try:
            values = future.result().values
        except Exception:
            return
        for name, v in zip(names, values):
            live = v.double_value if v.type == ParameterType.PARAMETER_DOUBLE else (
                v.integer_value if v.type == ParameterType.PARAMETER_INTEGER else None)
            exp = expected_param(node, name)
            key = node + "|" + name
            if live is not None and isinstance(exp, (int, float)) and abs(float(live) - float(exp)) > 1e-6:
                self.live_drift[key] = (live, exp)
            else:
                self.live_drift.pop(key, None)

    def changes(self):
        """List of {"id", "text"} - everything that differs from the standard."""
        out = []
        ov = nav2_overrides()
        if ov:
            items = ["%s=%s" % (k, v) for vals in ov.values() if isinstance(vals, dict) for k, v in vals.items()]
            out.append({"id": "nav2_overrides", "text": "Bewaarde Nav2-instellingen: " + ", ".join(items)})
        for key, (live, exp) in sorted(self.live_drift.items()):
            node, name = key.split("|")
            out.append({"id": "live:" + key,
                        "text": "Nav2 live gewijzigd (niet bewaard): %s = %g (gestart met %g)" % (name, live, exp)})
        for m in list_maps():
            if os.path.exists(os.path.join(MAP_DIR, m + ".orig.pgm")):
                out.append({"id": "map_edit:" + m, "text": "Kaart '%s' is bewerkt (origineel bewaard)" % m})
            n = zones_count(m)
            if n:
                out.append({"id": "zones:" + m, "text": "Kaart '%s' heeft %d zone(s)" % (m, n)})
        if not usb_auto():
            out.append({"id": "usb_auto", "text": "USB-spaarstand staat uit (lidar draait altijd)"})
        if not monitor_control():
            out.append({"id": "monitor_control", "text": "Opdrachten van de monitor staan uit"})
        if desktop_wanted():
            out.append({"id": "desktop", "text": "Desktop (HDMI-scherm) staat aan"})
        try:
            import camera_params
            if not camera_params.is_default():
                cs = camera_params.settings()
                out.append({"id": "camera_settings", "text": "Camera-instellingen: %s, %d fps, kwaliteit %d"
                            % (cs["resolution"], cs["fps"], cs["jpeg_quality"])})
        except Exception:
            pass
        default_cam = os.environ.get("ENABLE_CAMERA", "false") == "true"
        if camera_enabled() != default_cam:
            out.append({"id": "camera", "text": "Camera staat %s (robotconfiguratie: %s)"
                        % ("aan" if camera_enabled() else "uit", "aan" if default_cam else "uit")})
        return out

    def _on_reset(self, msg):
        try:
            what = json.loads(msg.data)["what"]
        except (ValueError, KeyError, TypeError):
            return
        # "all" = settings only: map edits and zones are someone's work, not a
        # setting - those only one by one (2026-10-09: "all" wiped a map edit
        # and 4 zones that were meant to stay)
        ids = ([c["id"] for c in self.changes() if not c["id"].startswith(("map_edit:", "zones:"))]
               if what == "all" else [what])
        for cid in ids:
            try:
                self._reset_one(cid)
                self.get_logger().info("aanpassing hersteld: %s" % cid)
            except Exception as exc:
                self.get_logger().warn("herstellen %s mislukt: %s" % (cid, exc))

    def _reset_one(self, cid):
        if cid == "nav2_overrides":
            save_nav2_overrides({})
        elif cid.startswith("live:"):
            node, name = cid[5:].split("|")
            exp = expected_param(node, name)
            set_cli = self.param_clients[node][1]
            if exp is not None and set_cli.service_is_ready():
                req = SetParameters.Request()
                val = (ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(exp))
                       if isinstance(exp, int) and not isinstance(exp, bool) else
                       ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(exp)))
                req.parameters = [Parameter(name=name, value=val)]
                set_cli.call_async(req)
                self.live_drift.pop(node + "|" + name, None)
        elif cid.startswith("map_edit:"):
            m = cid[9:]
            orig = os.path.join(MAP_DIR, m + ".orig.pgm")
            pgm = os.path.join(MAP_DIR, m + ".pgm")
            shutil.copyfile(pgm, os.path.join(MAP_DIR, m + ".edited.pgm"))   # backup of the edit
            shutil.copyfile(orig, pgm)
            os.remove(orig)
            if m == running_map() and self.loadmap_client.service_is_ready():
                req = LoadMap.Request()
                req.map_url = os.path.join(MAP_DIR, m + ".yaml")
                self.loadmap_client.call_async(req)   # map_server: original again
        elif cid.startswith("zones:"):
            m = cid[6:]
            zf = os.path.join(MAP_DIR, m + ".zones.json")
            if os.path.exists(zf):
                shutil.copyfile(zf, os.path.join(MAP_DIR, m + ".zones.bak.json"))   # backup
            if m == running_map():
                self.nav_cmd_pub.publish(String(data=json.dumps({"cmd": "zones", "zones": []})))
            else:
                with open(os.path.join(MAP_DIR, m + ".zones.json"), "w") as f:
                    json.dump({"zones": []}, f)
        elif cid == "camera_settings":
            self._on_camera_settings(String(data="{}"))
        elif cid == "usb_auto":
            set_usb_auto(True)
        elif cid == "desktop":
            set_desktop(False)
        elif cid == "monitor_control":
            with open(MONITOR_CONTROL_FILE, "w") as f:
                f.write("true")
        elif cid == "camera":
            default_cam = os.environ.get("ENABLE_CAMERA", "false") == "true"
            if default_cam:
                self._make_start_cb("camera")(None, Trigger.Response())
            else:
                self._make_stop_one_cb("camera")(None, Trigger.Response())

    # ---- student projects (projects_lib.py, project.json) -------------------
    def _project_running(self):
        import projects_lib
        out = {}
        for pid, m in projects_lib.projects().items():
            for l in m.get("launches", []):
                pat = l.get("pattern")
                out[pid + ":" + l["id"]] = bool(pat and _pgrep_pids(pat))
        return out

    def _exclusive_project_running(self, name):
        """Title of a running project launch that excludes launch <name>, or None."""
        import projects_lib
        for m in projects_lib.projects().values():
            for l in m.get("launches", []):
                if name in l.get("exclusive_with", []) and l.get("pattern") and _pgrep_pids(l["pattern"]):
                    return m.get("title", m["id"])
        return None

    def _stop_projects(self, only=None):
        """SIGINT, then SIGKILL after 3 s, for the project launches (all, or one pattern)."""
        import projects_lib
        pats = [only] if only else [l.get("stop_pattern") or l.get("pattern") for m in projects_lib.projects().values()
                                    for l in m.get("launches", []) if l.get("stop_pattern") or l.get("pattern")]
        stopped = False
        for sig, wait in ((signal.SIGINT, 3.0), (signal.SIGKILL, 0.5)):
            pids = [p for pat in pats for p in _pgrep_pids(pat) if p != os.getpid()]
            if not pids:
                break
            stopped = True
            for pid in pids:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.time() + wait
            while time.time() < deadline and any(_pgrep_pids(pat) for pat in pats):
                time.sleep(0.2)
        return stopped

    def _on_project_cmd(self, msg):
        import projects_lib
        try:
            c = json.loads(msg.data)
            m, l = projects_lib.find_launch(str(c["project"]), str(c["launch"]))
        except (ValueError, KeyError, TypeError):
            return
        if not l:
            self.project_msg = {"text": "onbekend project of knop", "t": time.time()}
            return
        label = "%s - %s" % (m.get("title", m["id"]), l.get("label", l["id"]))
        if c.get("action") == "stop":
            self._stop_projects(l.get("stop_pattern") or l.get("pattern"))
            text = label + ": gestopt"
        elif l.get("pattern") and _pgrep_pids(l["pattern"]):
            text = label + ": draait al"
        elif any(self._is_running(n) for n in l.get("exclusive_with", [])):
            text = label + ": kan niet - stop eerst " + " en ".join(
                n for n in l.get("exclusive_with", []) if self._is_running(n)) + " (Navigatie-pagina)"
        else:
            if l.get("needs_bringup") and not self._is_running("bringup"):
                self._make_start_cb("bringup")(None, Trigger.Response())
                time.sleep(4)
            if l.get("nav2_params"):
                # same Nav2 parameters as Navigation (zone filters, extra
                # planners/controllers, saved overrides) -> /tmp/nav2_params.yaml
                try:
                    write_nav2_params()
                except Exception as exc:
                    self.get_logger().warn("Nav2-parameters schrijven mislukt: %s" % exc)
            if l.get("needs_camera") and not self._is_running("camera"):
                self._make_start_cb("camera")(None, Trigger.Response())
            log = "/tmp/project_%s_%s.log" % (m["id"], l["id"])
            with open(log, "w") as f:
                subprocess.Popen(projects_lib.shell_command(l["cmd"]), preexec_fn=os.setsid,
                                 stdout=f, stderr=subprocess.STDOUT)
            text = label + ": gestart (log " + log + ")"
        self.project_msg = {"text": text, "t": time.time()}
        self.get_logger().info(text)

    def _admin_tick(self):
        for item in self.admin.take():
            action, args = item["action"], item["args"]
            try:
                item["result"] = self._admin_action(action, args)
            except Exception as exc:
                item["result"] = {"success": False, "message": "fout: %s" % exc}
            self.get_logger().info("Beheer: %s %s -> %s" % (action, json.dumps(args)[:200],
                                                             item["result"]["message"][:120]))
            item["done"].set()

    def _admin_action(self, action, args):
        trigger = {"update": self._update_cb, "rollback": self._rollback_cb,
                   "usb_on": self._usb_on_cb, "usb_off": self._usb_off_cb}
        if action in trigger:
            r = trigger[action](None, Trigger.Response())
            return {"success": r.success, "message": r.message}
        if action == "install_version":
            return self._install_version(str(args.get("version", "")))
        if action == "desktop":
            req = SetBool.Request()
            req.data = bool(args.get("on"))
            r = self._desktop_cb(req, SetBool.Response())
            return {"success": r.success, "message": r.message}
        # system.html: {"auto": bool} - USB power off by itself when unused
        if action == "usb_options":
            self._on_usb_options(String(data=json.dumps(args)))
            return {"success": True, "message": "USB-spaarstand " + ("aan" if usb_auto() else "uit")}
        if action == "monitor_options":
            self._on_monitor_options(String(data=json.dumps(args)))
            return {"success": True, "message": "monitor-opdrachten " + ("toegelaten" if monitor_control() else "uit")}
        # {"resolution", "fps", "jpeg_quality"} or {} = standard
        if action == "camera_settings":
            self._on_camera_settings(String(data=json.dumps(args)))
            return {"success": True, "message": "camera-instellingen toegepast"}
        return {"success": False, "message": "onbekende opdracht: %s" % action}

    def _install_version(self, tag):
        if not VERSION_TAG_RE.match(tag):
            return {"success": False, "message": "ongeldige versie: %s" % tag}
        if not os.path.exists(DBUS_SOCKET):
            return {"success": False, "message": "systeembus niet beschikbaar in de container"}
        st = _update_status()
        if st["state"] == "activating" or st.get("rollback") == "activating" or _install_active():
            return {"success": False, "message": "er is al een update, terugschakeling of installatie bezig"}
        out = _busctl("call", "org.freedesktop.systemd1", "/org/freedesktop/systemd1",
                      "org.freedesktop.systemd1.Manager", "StartUnit", "ss",
                      INSTALL_UNIT % tag, "replace")
        if out.returncode != 0:
            if "not found" in out.stderr.lower() or "no such file" in out.stderr.lower():
                return {"success": False, "message": "deze robot kent Versie kiezen nog niet - eerst één keer "
                                                     "Software bijwerken (nieuwe setup-scripts)"}
            return {"success": False, "message": "starten mislukt: %s" % out.stderr.strip()[:200]}
        return {"success": True, "message": "installatie van %s gestart - de container herstart, de pagina is "
                                            "~1 minuut weg (zonder deze versie op de robot: eerst downloaden)" % tag}

    def _on_camera_settings(self, msg):
        import camera_params
        try:
            camera_params.save(json.loads(msg.data))
        except (ValueError, TypeError, OSError) as exc:
            self.get_logger().warn("camera-instellingen bewaren mislukt: %s" % exc)
            return
        # restart the camera so the new settings take effect
        if self._is_running("camera"):
            self._terminate(["camera"])
            self._make_start_cb("camera")(None, Trigger.Response())
        self.get_logger().info("camera-instellingen: %s" % camera_params.settings())

    def _on_monitor_options(self, msg):
        try:
            allow = bool(json.loads(msg.data)["control"])
        except (ValueError, KeyError, TypeError):
            return
        try:
            with open(MONITOR_CONTROL_FILE, "w") as f:
                f.write("true" if allow else "false")
        except OSError:
            pass

    def _monitor_tick(self):
        cmds = self.monitor.take()
        if cmds and not monitor_control():
            for c in cmds:
                self.monitor.report(c.get("id"), c["cmd"], False,
                                    "opdrachten van de monitor staan uit op deze robot (Systeem-pagina)")
            return
        for c in cmds:
            cmd = c["cmd"]
            self.get_logger().info("opdracht van de monitor: %s" % cmd)
            resp = Trigger.Response()
            try:
                if cmd.startswith("start:"):
                    resp = self._make_start_cb(cmd[6:])(None, resp)
                elif cmd.startswith("stop:"):
                    resp = self._make_stop_one_cb(cmd[5:])(None, resp)
                elif cmd == "stop_all":
                    resp = self._stop_all_cb(None, resp)
                elif cmd == "update":
                    resp = self._update_cb(None, resp)
                elif cmd in ("reboot", "shutdown"):
                    resp = self._make_power_cb("Reboot" if cmd == "reboot" else "PowerOff")(None, resp)
                elif cmd == "set_pin":
                    resp.success, resp.message = set_admin_pin_hash(str(c.get("arg", "")))
                elif cmd in ("map_upload", "map_download"):
                    resp.success, resp.message = self._monitor_map(cmd, str(c.get("arg", "")))
                elif cmd == "beep":
                    subprocess.Popen(["/usr/local/bin/beep.py", "on"], start_new_session=True,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    resp.success, resp.message = True, "piep"
            except Exception as exc:
                resp.success, resp.message = False, str(exc)
            self.monitor.report(c.get("id"), cmd, bool(resp.success), resp.message)

    def _monitor_map(self, cmd, name):
        """Map library on the monitor: upload a map bundle / download + import one."""
        import map_bundle
        if not map_bundle.NAME_RE.match(name):
            return False, "ongeldige kaartnaam"
        url = self.monitor.base + "/maps/" + urllib.parse.quote(name)
        if cmd == "map_upload":
            body = json.dumps(map_bundle.export_bundle(name)).encode()
            req = urllib.request.Request(url, data=body, headers={"Content-Type": "application/json"})
            urllib.request.urlopen(req, timeout=15).read()
            return True, "kaart '%s' naar de bibliotheek gestuurd" % name
        try:
            with urllib.request.urlopen(url, timeout=15) as r:
                bundle = json.loads(r.read())
        except urllib.error.HTTPError as exc:
            return False, ("kaart '%s' staat niet in de bibliotheek" % name if exc.code == 404 else str(exc))
        return True, map_bundle.import_bundle(bundle, name)

    def _on_map_cmd(self, msg):
        try:
            text = map_command(json.loads(msg.data))
        except (ValueError, OSError, AttributeError) as exc:
            text = "kaartbeheer mislukt: %s" % exc
        self.map_msg = {"text": text, "t": time.time()}
        self.get_logger().info(text)

    def _on_nav2_overrides(self, msg):
        try:
            data = json.loads(msg.data).get("overrides")
        except (ValueError, AttributeError):
            return
        if not isinstance(data, dict):
            return
        clean = {}
        for node, values in data.items():
            if isinstance(values, dict):
                vals = {str(k): v for k, v in values.items()
                        if isinstance(v, (bool, int, float, str))}
                if vals:
                    clean[str(node)] = vals
        try:
            save_nav2_overrides(clean)
        except OSError as exc:
            self.get_logger().warn("nav2_overrides bewaren mislukt: %s" % exc)

    def _on_nav_options(self, msg):
        try:
            new = json.loads(msg.data)
        except ValueError:
            return
        opts = nav_options()
        if "map" in new and new["map"] in list_maps():
            opts["map"] = new["map"]
        save_nav_options(opts)

    # ---- USB power ---------------------------------------------------------
    def _usb_in_use(self):
        return any(self._is_running(n) for n in USB_LAUNCHES) or usb_devices_open()

    def _usb_on(self):
        if self.usb_state != "on":
            usb_power("on")
            self.usb_state = "on"
        self.usb_last_use = time.monotonic()

    def _usb_idle_check(self):
        now = time.monotonic()
        if self.usb_state == "on" and self._usb_in_use():
            self.usb_last_use = now
            return
        if self.usb_state == "on" and usb_auto() and now - self.usb_last_use > USB_IDLE_OFF:
            # status again first: a student may have switched it with usb_power
            if usb_power("status") == "on":
                usb_power("off")
                self.get_logger().info("USB-stroom uit (%d s niet gebruikt)" % USB_IDLE_OFF)
            self.usb_state = "off"
        elif self.usb_state == "off" and os.path.exists("/sys/class/tty/ttyUSB0"):
            self.usb_state = "on"   # switched on outside this node (usb_power on)
            self.usb_last_use = now

    def _usb_on_cb(self, request, response):
        self._usb_on()
        response.success = True
        response.message = ("USB-stroom aan" + (" - gaat vanzelf weer uit na %d s zonder gebruik"
                                                % USB_IDLE_OFF if usb_auto() else ""))
        return response

    def _usb_off_cb(self, request, response):
        if self._usb_in_use():
            response.success = False
            response.message = "USB is in gebruik (bringup, SLAM, Navigatie of joystick) - stop die eerst"
            return response
        usb_power("off")
        self.usb_state = "off"
        response.success = True
        response.message = "USB-stroom uit - lidar staat stil"
        return response

    def _on_usb_options(self, msg):
        try:
            new = json.loads(msg.data)
        except ValueError:
            return
        if "auto" in new:
            set_usb_auto(bool(new["auto"]))
            self.usb_last_use = time.monotonic()

    def _is_running(self, name):
        return len(_pgrep_pids(LAUNCH_PATTERNS[name])) > 0

    def _publish_status(self):
        status = {name: self._is_running(name) for name in LAUNCH_PATTERNS}
        # sensor detection is a few sysfs reads, cheap enough at 1 Hz
        status["camera_sensor"] = camera_sensor()
        status["camera_enabled"] = camera_enabled()
        status["joystick_device"] = joystick_name()
        # turtlebot3_node itself (bringup can run without it: it dies when
        # the OpenCR stops answering, e.g. motor power cut by low battery)
        status["robot_node"] = SIM or bool(_pgrep_pids("lib/turtlebot3_node/turtlebot3_ros"))
        status["usb_power"] = self.usb_state
        status["usb_auto"] = usb_auto()
        status["usb_off_in"] = (max(0, round(USB_IDLE_OFF - (time.monotonic() - self.usb_last_use)))
                                if self.usb_state == "on" and usb_auto() else None)
        status["maps"] = list_maps()
        status["map_selected"] = selected_map()
        status["nav2_overrides"] = nav2_overrides()
        status["map_running"] = running_map()
        status["map_msg"] = self.map_msg
        status["monitor_cmd"] = self.monitor.last if self.monitor else None
        # for the fleet monitor's "update all" queue (knows when a robot is done)
        status["update"] = self.update_status
        status["desktop"] = self.desktop
        status["monitor_control"] = monitor_control()
        status["image_channel"] = image_channel()
        status["rosbridge_watchdog"] = {"restarts": self.rb_watchdog.restarts,
                                        "last": self.rb_watchdog.last_restart}
        try:
            status["projects_running"] = self._project_running()
        except Exception:
            status["projects_running"] = {}
        status["project_msg"] = self.project_msg
        status["changes"] = self.changes()
        try:
            import camera_params
            cs = camera_params.settings()
            status["camera_settings"] = dict(cs, mbit=camera_params.mbit_estimate(cs),
                                             default=camera_params.defaults(),
                                             calibrated=camera_params.calibrated(cs["resolution"]))
        except Exception:
            status["camera_settings"] = None
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def _publish_active_nodes(self):
        try:
            names = sorted(
                (ns.rstrip("/") + "/" + name) if ns not in ("", "/") else "/" + name
                for name, ns in self.get_node_names_and_namespaces()
            )
        except Exception:
            names = []
        msg = String()
        msg.data = json.dumps(names)
        self.nodes_pub.publish(msg)

    def _publish_update_status(self):
        self.update_status = _update_status()
        self.desktop = desktop_state()
        msg = String()
        msg.data = json.dumps(self.update_status)
        self.update_pub.publish(msg)

    def _update_cb(self, request, response):
        if not os.path.exists(DBUS_SOCKET):
            response.success = False
            response.message = ("systeembus niet beschikbaar in de container - mount "
                                + DBUS_SOCKET + " in docker-compose.yaml")
            return response
        st = _update_status()
        if st["state"] == "activating" or st.get("rollback") == "activating":
            response.success = False
            response.message = "update is al bezig"
            return response
        if not _internet_ok():
            response.success = False
            response.message = ("geen internet via de AP (WAN-poort van de AP niet verbonden?) - "
                                "update niet mogelijk")
            return response
        out = _busctl("call", "org.freedesktop.systemd1", "/org/freedesktop/systemd1",
                      "org.freedesktop.systemd1.Manager", "RestartUnit", "ss",
                      UPDATE_UNIT, "replace")
        response.success = out.returncode == 0
        response.message = (
            "update gestart - is er een nieuwe image, dan wordt de container "
            "herstart en is de pagina ~1 minuut weg"
            if response.success else
            "kon " + UPDATE_UNIT + " niet starten: " + out.stderr.strip())
        return response

    def _desktop_cb(self, request, response):
        if not os.path.exists(DBUS_SOCKET):
            response.success, response.message = False, "systeembus niet beschikbaar in de container"
            return response
        response.success, response.message = set_desktop(bool(request.data))
        self.desktop = desktop_state()
        return response

    def _rollback_cb(self, request, response):
        if not os.path.exists(DBUS_SOCKET):
            response.success = False
            response.message = "systeembus niet beschikbaar in de container"
            return response
        st = _update_status()
        if st["state"] == "activating" or st.get("rollback") == "activating":
            response.success = False
            response.message = "er is al een update of terugschakeling bezig"
            return response
        prev = _previous_image()
        if not prev:
            response.success = False
            response.message = ("geen vorige versie bewaard - die wordt bewaard bij de volgende update "
                                "(als er minstens 5 GB vrij is)")
            return response
        out = _busctl("call", "org.freedesktop.systemd1", "/org/freedesktop/systemd1",
                      "org.freedesktop.systemd1.Manager", "RestartUnit", "ss",
                      ROLLBACK_UNIT, "replace")
        response.success = out.returncode == 0
        response.message = (
            "terugschakelen naar " + prev + " gestart - de container herstart, de pagina is ~1 minuut weg"
            if response.success else
            "kon " + ROLLBACK_UNIT + " niet starten (setup nog niet bijgewerkt?): " + out.stderr.strip())
        return response

    def _make_start_cb(self, name):
        def cb(request, response):
            if name == "camera":
                # Remember the choice even if it can't start now: it then
                # starts by itself after a reboot with a camera attached.
                set_camera_enabled(True)
                if camera_sensor() is None:
                    response.success = False
                    response.message = ("geen camera gevonden - 'aan' onthouden, de camera start "
                                        "vanzelf na een herstart met een camera aangesloten")
                    return response
            if name in USB_LAUNCHES:
                was_off = self.usb_state != "on"
                self._usb_on()
                if name == "joystick" and was_off:
                    # After power-on the F710 receiver first enumerates in
                    # DirectInput mode ("RumblePad 2") and a few seconds later
                    # re-enumerates in XInput mode, which joystick_f710.yaml uses.
                    for _ in range(48):
                        jn = joystick_name()
                        if jn and "RumblePad" not in jn:
                            break
                        time.sleep(0.25)
                    time.sleep(0.5)
            if name == "joystick" and joystick_name() is None:
                response.success = False
                response.message = ("geen joystick gevonden - steek de USB-ontvanger van de F710 "
                                    "in de Pi en zet de controller aan")
                return response
            switched_from = None
            if self._is_running(name):
                # Mappen > Openen with another map while Navigation runs:
                # restart Navigation with the chosen map (t82: it answered
                # "draait al" and kept the old map). Not map_server/load_map:
                # the launch's map:= argument is what running_map() - and so
                # zones, routes and nav_web_node - go by.
                running = running_map() if name == "navigation" else None
                if running and selected_map() and selected_map() != running:
                    self._terminate(["navigation"])
                    switched_from = running
                else:
                    response.success = False
                    response.message = name + " draait al"
                    return response
            # a project with its own SLAM/Nav2 (explore_demo) excludes these:
            # two Nav2 stacks both drive the robot (2026-10-09: robot kept driving)
            busy = self._exclusive_project_running(name)
            if busy:
                response.success = False
                response.message = ("%s kan niet: project '%s' draait (eigen SLAM/Nav2) - stop dat eerst "
                                    "(tab Projecten of Stop alles)" % (name, busy))
                return response
            if name == "camera":
                import camera_params
                camera_params.write()   # settings from system.html
            if name == "navigation":
                os.makedirs(MAP_DIR, exist_ok=True)
                map_name = selected_map()
                map_yaml = os.environ.get("NAV_MAP_YAML") or (
                    os.path.join(MAP_DIR, map_name + ".yaml") if map_name else "")
                if not os.path.isfile(map_yaml):
                    response.success = False
                    response.message = "geen kaart gevonden - maak eerst een kaart met SLAM en sla ze op"
                    return response
                try:
                    params_file = write_nav2_params()
                except Exception as exc:
                    response.success = False
                    response.message = "Nav2-parameters schrijven mislukt: " + str(exc)
                    return response
                # use_composition:=False - every Nav2 node its own process. With
                # composition, AMCL's map->odom TF stopped reaching the
                # controller in the same nav2_container after the first
                # message. The root cause turned out to be Zenoh peer mode
                # (see ZENOH_CONFIG_OVERRIDE in the Dockerfile); only the
                # separate-processes + client-mode combination is tested
                # (2026-10-09, turtlebot09), so keep both.
                cmd = ["ros2", "launch", "turtlebot3_navigation2", "navigation2.launch.py",
                       "map:=" + map_yaml, "params_file:=" + params_file,
                       "use_composition:=False"] + (["use_sim_time:=True"] if SIM else [])
            else:
                cmd = FIXED_LAUNCHES[name]

            # SLAM and Navigation need the robot (bringup): start it first, so
            # nav.html can go straight to step 2 or 3.
            started_bringup = False
            if name in ("slam", "navigation") and not self._is_running("bringup"):
                with open("/tmp/launch_bringup.log", "w") as log_file:
                    subprocess.Popen(FIXED_LAUNCHES["bringup"], preexec_fn=os.setsid,
                                     stdout=log_file, stderr=subprocess.STDOUT)
                started_bringup = True
                time.sleep(2 if SIM else 6)   # OpenCR + lidar + robot_state_publisher up

            try:
                log_path = "/tmp/launch_" + name + ".log"
                with open(log_path, "w") as log_file:
                    proc = subprocess.Popen(
                        cmd, preexec_fn=os.setsid, stdout=log_file, stderr=subprocess.STDOUT,
                    )
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                return response

            if name == "joystick":
                time.sleep(2)
                if proc.poll() is not None:
                    response.success = False
                    response.message = "joystick stopte meteen (log: " + log_path + ")"
                    return response

            if name == "camera":
                # Without a camera attached camera_node exits within a second
                # ("no cameras available") - report that instead of "gestart".
                time.sleep(3)
                if proc.poll() is not None:
                    response.success = False
                    response.message = ("camera_node stopte meteen - camera niet aangesloten? "
                                        "(log: " + log_path + ")")
                    return response

            response.success = True
            if switched_from:
                response.message = ("Navigatie herstart met kaart %s (was %s) - zet de beginpositie"
                                    % (map_name, switched_from))
            else:
                response.message = (("bringup + " if started_bringup else "") + name +
                                    " gestart (pid " + str(proc.pid) + ")" +
                                    (" - kaart: " + map_name if name == "navigation" else ""))
            return response
        return cb

    def _make_stop_one_cb(self, name):
        def cb(request, response):
            if name == "camera":
                set_camera_enabled(False)
            children = CHILD_PATTERNS.get(name)
            if not _pgrep_pids(LAUNCH_PATTERNS[name]) and not (children and _pgrep_pids(children)):
                response.success = name == "camera"
                response.message = "camera uitgeschakeld (onthouden)" if name == "camera" else name + " draait niet"
                return response
            self._terminate([name])
            response.success = True
            response.message = name + " gestopt"
            return response
        return cb

    def _stop_all_cb(self, request, response):
        stopped = self._stop_all_launches()
        response.success = True
        response.message = "gestopt: " + (", ".join(sorted(set(stopped))) if stopped else "niets liep")
        return response

    def _stop_all_launches(self, include_camera=False):
        # "Stop alles" is about the robot moving - the camera keeps running
        # unless the Pi is going down (reboot/shutdown).
        names = [n for n in LAUNCH_PATTERNS if include_camera or n != "camera"]
        # student projects too (drive_to_tag drives the robot!)
        projects_stopped = self._stop_projects()
        stopped = self._terminate(names)
        return stopped + (["projecten"] if projects_stopped else [])

    def _terminate(self, names):
        """SIGINT to the launches (clean shutdown), then SIGTERM, then SIGKILL
        to whatever is left of them and their child processes."""
        def pids():
            found = {}
            for n in names:
                for pat in (LAUNCH_PATTERNS[n], CHILD_PATTERNS.get(n)):
                    if pat:
                        for pid in _pgrep_pids(pat):
                            found[pid] = n
            found.pop(os.getpid(), None)
            return found

        def send(sig, targets):
            for pid in targets:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass

        first = pids()
        stopped = sorted(set(first.values()))
        launch_pids = [p for n in names for p in _pgrep_pids(LAUNCH_PATTERNS[n])]
        send(signal.SIGINT, launch_pids)
        for sig, wait in ((signal.SIGTERM, 5.0), (signal.SIGKILL, 3.0)):
            deadline = time.time() + wait
            while pids() and time.time() < deadline:
                time.sleep(0.25)
            left = pids()
            if not left:
                break
            send(sig, left)
        return stopped

    def _make_power_cb(self, method):
        def cb(request, response):
            if not os.path.exists(DBUS_SOCKET):
                response.success = False
                response.message = ("systeembus niet beschikbaar in de container - mount "
                                    + DBUS_SOCKET + " in docker-compose.yaml")
                return response
            # Motors off before the Pi goes down, then the OpenCR's "off"
            # melody (port is free once bringup is stopped), then ask the
            # host's logind. Detached with a short delay so this service
            # response still reaches the web page first.
            self._stop_all_launches(include_camera=True)
            subprocess.Popen(
                ["bash", "-c",
                 "sleep 2; " + USB_POWER + " on --wait; /usr/local/bin/beep.py off; sleep 1; "
                 "busctl call org.freedesktop.login1 /org/freedesktop/login1 "
                 "org.freedesktop.login1.Manager " + method + " b false"],
                start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            response.success = True
            response.message = POWER_MESSAGES[method]
            return response
        return cb


def main():
    rclpy.init()
    node = LaunchControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
