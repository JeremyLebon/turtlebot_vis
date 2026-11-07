#!/usr/bin/env python3
# Nav2 for the status page (nav.html), via rosbridge:
#   /web/nav_cmd     std_msgs/String (JSON) in:
#                      {"cmd": "initialpose", "x", "y", "yaw"}
#                      {"cmd": "goal", "x", "y", "yaw", "planner", "controller", "tolerance"}
#                      {"cmd": "route", "poses": [{"x", "y", "yaw"}, ...], "loop": bool,
#                       "planner", "controller", "tolerance", "via_tolerance"}
#                    tolerance: "ruim" | "normaal" | "precies" | {"xy": m, "yaw": rad};
#                    via_tolerance (route): for the points before the last one
#                    (all points of a loop), null = same as tolerance
#                      {"cmd": "cancel"}
#                      {"cmd": "zones", "zones": [{"type": "nogo"|"wall"|"lane"|"speed",
#                                                  "points": [[x, y], ...],
#                                                  "percent": 1-100 (speed)}, ...]}
#                        -> saved for the running map, masks republished
#                      {"cmd": "route_save", "name", "poses", "loop"} / {"cmd": "route_delete", "name"}
#                        -> routes per map (state/maps/<map>.routes.json)
#                      {"cmd": "map_edit", "width", "height", "data": base64 bytes per
#                       cell (0 free, 100 occupied, 255 unknown), row 0 = bottom}
#                      {"cmd": "map_restore"} -> back to the map as it was saved
#                        -> map editor: rewrites <map>.pgm of the running map (first
#                           time: <map>.orig.pgm backup) and reloads map_server
#                      {"cmd": "relocalize"} -> AMCL global localization + spin 360°
#   /web/nav_status  std_msgs/String (JSON) out, 2 Hz; also "loc" (AMCL
#                    uncertainty + quality, t49) and "pose_suggestion" (last
#                    good pose on this map while AMCL has none yet, t51)
#   /web/robot_pose  geometry_msgs/PoseStamped out (frame map), 5 Hz - only
#                    while someone subscribes (the TF listener costs CPU)
#   /web/zones       std_msgs/String (JSON) out: {"map": name, "zones": [...]},
#                    on change and every 5 s
#   /web/map_odom    geometry_msgs/PoseStamped out: map->odom (AMCL), 5 Hz,
#                    likewise only while subscribed - nav.html places the
#                    local costmap (odom frame, rolling window) with it
#
# The Nav2 action (navigate_to_pose) runs here instead of in the browser:
# rosbridge's action support is limited, and this way a page reload doesn't
# lose the running goal. A route is a series of navigate_to_pose goals (not
# follow_waypoints), so the chosen planner/controller applies to it too.
# Planner/controller per goal: a copy of Nav2's default behavior tree with
# planner_id/controller_id replaced (the plugins are all loaded by
# launch_control_node's Nav2 parameters). Students use Nav2 Simple Commander
# from their own code; a goal from either side preempts the other.
import base64
import glob
import json
import re
import shutil
import math
import os
import time

import numpy as np

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.srv import LoadMap
from nav_msgs.srv import GetMap
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from turtlebot3_msgs.srv import Sound
from tf2_ros import Buffer, TransformListener

DEFAULT_BT = ("/opt/ros/" + os.environ.get("ROS_DISTRO", "jazzy") +
              "/share/nav2_bt_navigator/behavior_trees/"
              "navigate_to_pose_w_replanning_and_recovery.xml")
PLANNERS = ("GridBased", "AStar", "Smac2D", "ThetaStar")
CONTROLLERS = ("FollowPath", "RPP", "MPPI")

# Goal tolerance: set on Nav2's goal checker (controller_server, dynamic
# parameters) just before each goal. One checker, not one per preset: with
# several, a goal without goal_checker_id (Nav2's default behavior tree, so
# also the students' Simple Commander goals) is refused. The default after a
# Navigation start is "normaal" (launch_control_node's Nav2 parameters).
GOAL_CHECKER = "general_goal_checker"
TOLERANCES = {"ruim": (0.25, 0.25), "normaal": (0.10, 0.15), "precies": (0.05, 0.08)}


# Action per route point (nav.html route bar), done on arrival before the
# robot goes on to the next point:
#   wait:<s>  wait s seconds        beep   OpenCR melody (turtlebot3_node /sound)
#   photo     camera picture -> state/photos (needs Camera running)
#   spin      turn 360 degrees (Nav2 behavior_server)
#   hold      stop until "Verder" on the page
ACTION_RE = re.compile(r"^(wait:\d{1,3}|beep|photo|spin|hold)$")
PHOTO_DIR = "/root/turtlebot_state/photos"
PHOTO_KEEP = 300         # newest photos kept (~50 kB each)
CAMERA_TOPIC = "/camera/image_raw/compressed"


def clean_pose(p):
    pose = {"x": float(p["x"]), "y": float(p["y"]), "yaw": float(p.get("yaw", 0.0))}
    if ACTION_RE.match(str(p.get("action") or "")):
        pose["action"] = p["action"]
    if p.get("free"):
        pose["free"] = True   # clicked without dragging: no heading of its own
    return pose


def parse_tolerance(t):
    """(xy m, yaw rad) from a preset name or {"xy", "yaw"}, None if not given."""
    if t is None:
        return None
    if isinstance(t, str):
        return TOLERANCES.get(t)
    xy = min(max(float(t["xy"]), 0.02), 1.0)
    yaw = min(max(float(t["yaw"]), 0.02), math.pi)
    return (xy, yaw)


# Zones (Nav2 costmap filters, KeepoutFilter): masks the size of the map.
# Mask value 100 = lethal (the robot never goes there), -1 = this mask leaves
# the cell alone, values in between = extra cost. Two filters, configured in
# launch_control_node's Nav2 parameters:
#   keepout_filter (global + local costmap): no-go zones and virtual walls
#   lanes_filter   (global costmap only):    outside preferred zones a medium
#                                            cost, so the planner prefers them
#   speed_filter   (global costmap, SpeedFilter): in a speed zone the
#                  controller's max speed is limited to <percent> % (mask
#                  value = percent, 0 = no limit), via /speed_limit
# ROS parameter lane_outside_cost (0-99, default LANE_OUTSIDE): the extra
# cost outside preferred zones - nav.html "Geavanceerd", saved like the Nav2
# settings (state/nav2_overrides.yaml, section nav_web_node).
MAP_DIR = "/root/turtlebot_state/maps"
KEEPOUT_INFO, KEEPOUT_MASK = "/keepout_filter_info", "/keepout_filter_mask"
LANES_INFO, LANES_MASK = "/lanes_filter_info", "/lanes_filter_mask"
SPEED_INFO, SPEED_MASK = "/speed_filter_info", "/speed_filter_mask"
OVERRIDES_FILE = "/root/turtlebot_state/nav2_overrides.yaml"
LANE_OUTSIDE = 40         # mask value outside preferred zones (cost ~100 of 254)
WALL_HALF_WIDTH = 0.05    # virtual wall: 10 cm wide


# Zones without a saved map: during SLAM / exploration (explore_demo) the map
# is still being made, so the zones belong to this "session" - copied to the
# map's own name when it is saved (nav.html "Kaart opslaan" -> zones_adopt).
SESSION = "_verkenning"


# AMCL quality from its covariance (standard deviation): good = position to
# ~15 cm and heading to ~9°, doubt up to 0.5 m / ~35°, else lost. Right after
# setting the initial pose it is "doubt" (0.5 m spread, like rviz) until the
# robot has driven a bit (AMCL only updates after update_min_d = 0.25 m).
LOC_GOOD = (0.15, 0.15)
LOC_DOUBT = (0.5, 0.6)


def loc_from_amcl(msg):
    """{"x", "y", "yaw", "sx", "sy", "syaw", "a", "b", "angle", "quality"}:
    a/b/angle = standard deviations along the ellipse axes (nav.html draws 2x)."""
    c = msg.pose.covariance
    cxx, cxy, cyy, cyaw = c[0], c[1], c[7], c[35]
    mid, half = (cxx + cyy) / 2, math.hypot((cxx - cyy) / 2, cxy)
    a, b = math.sqrt(max(mid + half, 0.0)), math.sqrt(max(mid - half, 0.0))
    syaw = math.sqrt(max(cyaw, 0.0))
    if a <= LOC_GOOD[0] and syaw <= LOC_GOOD[1]:
        quality = "good"
    elif a <= LOC_DOUBT[0] and syaw <= LOC_DOUBT[1]:
        quality = "doubt"
    else:
        quality = "lost"
    q = msg.pose.pose.orientation
    return {"x": round(msg.pose.pose.position.x, 3), "y": round(msg.pose.pose.position.y, 3),
            "yaw": round(2 * math.atan2(q.z, q.w), 3),
            "sx": round(math.sqrt(max(cxx, 0.0)), 3), "sy": round(math.sqrt(max(cyy, 0.0)), 3),
            "syaw": round(syaw, 3), "a": round(a, 3), "b": round(b, 3),
            "angle": round(0.5 * math.atan2(2 * cxy, cxx - cyy), 3), "quality": quality}


def lastpose_file(map_name):
    return os.path.join(MAP_DIR, map_name + ".lastpose.json")


def load_lastpose(map_name):
    try:
        with open(lastpose_file(map_name)) as f:
            p = json.load(f)
        return {k: float(p[k]) for k in ("x", "y", "yaw")} | {"t": float(p.get("t", 0))}
    except (OSError, ValueError, KeyError, TypeError):
        return None


def running_map():
    """Name of the map Navigation runs with (from its command line), SESSION
    while SLAM runs without Navigation (mapping / exploring), or None."""
    slam = False
    for cmdline in glob.glob("/proc/[0-9]*/cmdline"):
        try:
            with open(cmdline, "rb") as f:
                args = f.read().split(b"\0")
        except OSError:
            continue
        if any(b"navigation2.launch.py" in a for a in args):
            for a in args:
                if a.startswith(b"map:="):
                    return os.path.splitext(os.path.basename(a[5:].decode()))[0]
        if any(a.endswith(b"cartographer_node") for a in args):
            slam = True
    return SESSION if slam else None


def explore_running():
    """True while explore_lite (projects/explore_demo) runs."""
    for cmdline in glob.glob("/proc/[0-9]*/cmdline"):
        try:
            with open(cmdline, "rb") as f:
                if b"lib/explore_lite/explore" in f.read():
                    return True
        except OSError:
            continue
    return False


def zones_grid(zones, res=0.05, margin=1.0):
    """Mask geometry around the zones (no saved map yet): Nav2's costmap
    filters work in world coordinates, the mask need not match the map."""
    pts = [p for z in zones for p in z.get("points", [])]
    info = MapMetaData()
    info.resolution = res
    if not pts:
        info.width = info.height = 1
        return info
    x0, y0 = min(p[0] for p in pts) - margin, min(p[1] for p in pts) - margin
    x1, y1 = max(p[0] for p in pts) + margin, max(p[1] for p in pts) + margin
    info.width = int(math.ceil((x1 - x0) / res))
    info.height = int(math.ceil((y1 - y0) / res))
    info.origin.position.x = x0
    info.origin.position.y = y0
    info.origin.orientation.w = 1.0
    return info


def routes_file(map_name):
    return os.path.join(MAP_DIR, map_name + ".routes.json")


def load_routes(map_name):
    try:
        with open(routes_file(map_name)) as f:
            return json.load(f).get("routes", {})
    except (OSError, ValueError):
        return {}


def write_json(path, data):
    with open(path + ".tmp", "w") as f:
        json.dump(data, f)
    os.replace(path + ".tmp", path)


def map_pgm(map_name):
    """Path of the map's image, from its yaml."""
    yaml_path = os.path.join(MAP_DIR, map_name + ".yaml")
    with open(yaml_path) as f:
        m = re.search(r"(?m)^image:\s*(\S+)", f.read())
    img = m.group(1) if m else map_name + ".pgm"
    return yaml_path, img if os.path.isabs(img) else os.path.join(MAP_DIR, img)


def write_pgm(path, width, height, cells):
    """Trinary map image like map_saver: occupied 0, free 254, unknown 205;
    cells row 0 = bottom of the map, the image starts at the top."""
    grid = np.frombuffer(cells, dtype=np.uint8).reshape(height, width)[::-1]
    img = np.full(grid.shape, 205, dtype=np.uint8)
    img[grid == 0] = 254
    img[grid == 100] = 0
    with open(path + ".tmp", "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (width, height))
        f.write(img.tobytes())
    os.replace(path + ".tmp", path)


def zones_file(map_name):
    return os.path.join(MAP_DIR, map_name + ".zones.json")


def load_zones(map_name):
    try:
        with open(zones_file(map_name)) as f:
            return json.load(f).get("zones", [])
    except (OSError, ValueError):
        return []


def rasterize(info, zones, lane_outside=None):
    """(keepout, lanes, speed) masks as int8 arrays [row, col] for this map."""
    if lane_outside is None:
        lane_outside = LANE_OUTSIDE
    w, h, res = info.width, info.height, info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    xs = ox + (np.arange(w) + 0.5) * res
    ys = oy + (np.arange(h) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)          # cell centres, row = y
    keepout = np.full((h, w), -1, dtype=np.int8)
    lanes = np.full((h, w), -1, dtype=np.int8)
    speed = np.zeros((h, w), dtype=np.int8)       # 0 = no speed limit
    if any(z.get("type") == "lane" for z in zones):
        lanes[:] = lane_outside
    for z in zones:
        if z.get("type") == "area":
            continue   # exploration area: used by explore_demo's map_threshold, not by Nav2
        pts = np.array(z.get("points") or [], dtype=float)
        if z.get("type") == "wall" and len(pts) >= 2:
            inside = np.zeros((h, w), dtype=bool)
            for (x1, y1), (x2, y2) in zip(pts[:-1], pts[1:]):
                dx, dy = x2 - x1, y2 - y1
                L2 = dx * dx + dy * dy or 1e-9
                t = np.clip(((gx - x1) * dx + (gy - y1) * dy) / L2, 0, 1)
                d = np.hypot(gx - (x1 + t * dx), gy - (y1 + t * dy))
                inside |= d <= max(WALL_HALF_WIDTH, res * 0.75)
            keepout[inside] = 100
        elif len(pts) >= 3:
            inside = np.zeros((h, w), dtype=bool)   # even-odd ray casting
            for (x1, y1), (x2, y2) in zip(pts, np.roll(pts, -1, axis=0)):
                cross = (y1 > gy) != (y2 > gy)
                with np.errstate(divide="ignore", invalid="ignore"):
                    xint = x1 + (gy - y1) * (x2 - x1) / (y2 - y1)
                inside ^= cross & (gx < xint)
            if z.get("type") == "lane":
                lanes[inside] = 0
            elif z.get("type") == "speed":
                speed[inside] = int(min(max(z.get("percent", 50), 1), 100))
            else:
                keepout[inside] = 100
    return keepout, lanes, speed


MAP_FRAME = "map"
BASE_FRAME = "base_footprint"
ODOM_FRAME = "odom"

STATE_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_ABORTED: "aborted",
    GoalStatus.STATUS_CANCELED: "canceled",
}


def behavior_tree(planner, controller):
    """Path of a behavior tree using this planner/controller ("" = Nav2's default)."""
    if planner not in PLANNERS:
        planner = "GridBased"
    if controller not in CONTROLLERS:
        controller = "FollowPath"
    if (planner, controller) == ("GridBased", "FollowPath"):
        return ""
    path = "/tmp/bt_%s_%s.xml" % (planner, controller)
    with open(DEFAULT_BT) as f:
        xml = f.read()
    # Humble: planner_id="GridBased" on ComputePathToPose. Jazzy: a
    # PlannerSelector/ControllerSelector with default_planner="GridBased" -
    # without these the tree stayed Nav2's default and every goal ran with
    # NavFn + DWB whatever nav.html chose (2026-10-28, Jazzy audit).
    for attr, old, new in (("planner_id", "GridBased", planner), ("default_planner", "GridBased", planner),
                           ("controller_id", "FollowPath", controller),
                           ("default_controller", "FollowPath", controller)):
        xml = xml.replace('%s="%s"' % (attr, old), '%s="%s"' % (attr, new))
    with open(path, "w") as f:
        f.write(xml)
    return path


def make_pose(stamp, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = MAP_FRAME
    p.header.stamp = stamp
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(float(yaw) / 2)
    p.pose.orientation.w = math.cos(float(yaw) / 2)
    return p


class NavWebNode(Node):
    def __init__(self):
        super().__init__("nav_web_node")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.status_pub = self.create_publisher(String, "/web/nav_status", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/web/robot_pose", 10)
        self.map_odom_pub = self.create_publisher(PoseStamped, "/web/map_odom", 10)
        self.create_subscription(String, "/web/nav_cmd", self._on_cmd, 10)
        # Exploring (explore_lite) sends its own goals to the same Nav2: a goal
        # or route from the page pauses it first, else the two keep replacing
        # each other's goal (2026-10-09). "explore_resume" starts it again.
        self.explore_pub = self.create_publisher(Bool, "/explore/resume", 10)
        self.exploring = False      # explore_lite runs (checked every 5 s)
        self.explore_paused = False
        self.pending_timer = None   # goal/route waiting for the pause to take effect
        self.spin_client = ActionClient(self, Spin, "spin")
        self.sound_client = self.create_client(Sound, "/sound")
        self.action_timer = None    # wait / photo timeout of a route point action
        self.photo_sub = None
        self.paused_goal = None     # goal paused from the page (cmd "pause")
        self.last_photo = None

        # zones: filter info + masks latched (transient local), like map_server
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.filter_pubs = {
            "keepout": (self.create_publisher(CostmapFilterInfo, KEEPOUT_INFO, latched),
                        self.create_publisher(OccupancyGrid, KEEPOUT_MASK, latched), KEEPOUT_MASK),
            "lanes": (self.create_publisher(CostmapFilterInfo, LANES_INFO, latched),
                      self.create_publisher(OccupancyGrid, LANES_MASK, latched), LANES_MASK),
            "speed": (self.create_publisher(CostmapFilterInfo, SPEED_INFO, latched),
                      self.create_publisher(OccupancyGrid, SPEED_MASK, latched), SPEED_MASK),
        }
        # Localization (t49/t51): AMCL's pose with covariance, latched like AMCL publishes it
        self.loc = None             # loc_from_amcl() of the last /amcl_pose, None = no pose yet
        self.loc_map = None         # map the loc belongs to
        self.lastpose_saved = (None, 0.0)   # (pose, time) of the last write
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, latched)
        self.reloc_client = self.create_client(Empty, "/reinitialize_global_localization")
        self.declare_parameter("lane_outside_cost", self._saved_lane_cost())
        self.add_on_set_parameters_callback(self._on_params)
        self.zones_pub = self.create_publisher(String, "/web/zones", 10)
        self.getmap_client = self.create_client(GetMap, "/map_server/map")
        self.loadmap_client = self.create_client(LoadMap, "/map_server/load_map")
        self.zone_map = None      # map name the masks were made for
        self.last_tick_map = running_map()   # no wipe for a run already going at start
        self.zones = []
        self.zones_busy = False
        self.create_timer(5.0, self._zones_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.param_client = self.create_client(SetParameters, "/controller_server/set_parameters")
        self.tolerance = None   # last (xy, yaw) set on the goal checker
        self.cmd_seq = 0        # +1 per command from the page

        self.goal_handle = None
        self.status = {"state": "idle", "mode": None, "message": ""}
        self.route = None  # {"poses": [...], "loop": bool, "lap": int, "index": int, "bt": str}

        # TF listener only while a page shows the robot pose
        self.tf_buffer = None
        self.tf_listener = None
        self.last_pose_sub = 0.0
        # wall clock: on simulation time (virtual robot) the timer would stop
        # after a jump back in time (simulator page reloaded)
        self.last_ros_t = None
        self.create_timer(0.2, self._publish_pose, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(0.5, self._publish_status, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info("nav_web_node ready: /web/nav_cmd -> Nav2, /web/nav_status, /web/robot_pose")

    # ---- robot pose ------------------------------------------------------
    def _publish_pose(self):
        now = time.monotonic()
        if self.pose_pub.get_subscription_count() > 0 or self.map_odom_pub.get_subscription_count() > 0:
            self.last_pose_sub = now
            if self.tf_listener is None:
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
        elif self.tf_listener is not None and now - self.last_pose_sub > 10:
            self.tf_listener.unregister()
            self.tf_listener = None
            self.tf_buffer = None
        if self.tf_buffer is None:
            return
        # Buffer() without a node does not see the /clock: after the simulator
        # restarts at t=0 it kept the old, newer transforms -> frozen robot arrow
        ros_t = self.get_clock().now().nanoseconds
        if self.last_ros_t is not None and ros_t < self.last_ros_t - 1e9:
            self.get_logger().warn("tijd sprong terug (simulator herstart) - TF-buffer gewist")
            self.tf_buffer.clear()
        self.last_ros_t = ros_t
        for child, pub in ((BASE_FRAME, self.pose_pub), (ODOM_FRAME, self.map_odom_pub)):
            if pub.get_subscription_count() == 0:
                continue
            try:
                t = self.tf_buffer.lookup_transform(MAP_FRAME, child, Time())
            except Exception:
                continue  # no map frame yet (no SLAM/AMCL), or no bringup
            p = PoseStamped()
            p.header = t.header
            p.pose.position.x = t.transform.translation.x
            p.pose.position.y = t.transform.translation.y
            p.pose.orientation = t.transform.rotation
            pub.publish(p)

    # ---- commands ----------------------------------------------------------
    def _on_cmd(self, msg):
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            return
        kind = cmd.get("cmd")
        if kind in ("goal", "route", "cancel", "explore_resume", "pause", "resume"):
            self.cmd_seq += 1
            if self.pending_timer is not None:
                self.pending_timer.cancel()
                self.pending_timer = None
        if kind in ("goal", "route") and not self.explore_paused and explore_running():
            # pause exploring; explore_lite then cancels *all* Nav2 goals, so
            # send ours only after that (1.5 s)
            self.exploring = self.explore_paused = True
            self.explore_pub.publish(Bool(data=False))
            self._set_status("sending", kind, "verkennen gepauzeerd - %s volgt" % ("doel" if kind == "goal" else "route"))

            def later():
                self.pending_timer.cancel()
                self.pending_timer = None
                self._run_cmd(kind, cmd)
            self.pending_timer = self.create_timer(1.5, later)
            return
        self._run_cmd(kind, cmd)

    def _run_cmd(self, kind, cmd):
        try:
            if kind == "initialpose":
                self._set_initial_pose(cmd)
            elif kind == "relocalize":
                self._relocalize()
            elif kind == "explore_resume":
                self.route = None
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self.explore_paused = False
                self.explore_pub.publish(Bool(data=True))
                self._set_status("idle", None, "verkennen hervat")
            elif kind == "pause":
                self._pause()
            elif kind == "resume":
                self._resume()
            elif kind == "goal":
                self.route = None
                self._stop_action()
                self.paused_goal = None
                self.last_goal = {"x": float(cmd["x"]), "y": float(cmd["y"]), "yaw": float(cmd["yaw"]),
                                  "bt": behavior_tree(cmd.get("planner"), cmd.get("controller")),
                                  "tol": parse_tolerance(cmd.get("tolerance"))}
                self._send_goal(cmd, behavior_tree(cmd.get("planner"), cmd.get("controller")), "goal",
                                parse_tolerance(cmd.get("tolerance")))
            elif kind == "route":
                poses = cmd.get("poses") or []
                if not poses:
                    return
                tol = parse_tolerance(cmd.get("tolerance"))
                self._stop_action()
                self.paused_goal = None
                poses = [clean_pose(p) for p in poses][:100]
                self.route = {"poses": poses, "loop": bool(cmd.get("loop")), "lap": 1, "index": 0,
                              "bt": behavior_tree(cmd.get("planner"), cmd.get("controller")),
                              "tol": tol, "via_tol": parse_tolerance(cmd.get("via_tolerance")) or tol}
                self._send_route_point()
            elif kind == "cancel":
                self.route = None
                self._stop_action()
                if self.paused_goal is not None and self.goal_handle is None:
                    self._set_status("canceled", None, "geannuleerd")
                self.paused_goal = None
                self._cancel()
            elif kind == "zones":
                self._set_zones(cmd.get("zones") or [], cmd.get("map"))
            elif kind in ("route_save", "route_delete"):
                self._route_file_cmd(kind, cmd)
            elif kind == "zones_adopt":
                # nav.html saved the map of this exploration as <map>: its zones go along
                target = str(cmd.get("map", ""))
                if re.match(r"^[A-Za-z0-9_-]{1,40}$", target) and load_zones(SESSION) \
                        and not os.path.exists(zones_file(target)):
                    shutil.copyfile(zones_file(SESSION), zones_file(target))
                    self.status["message"] = "zones van de verkenning bewaard bij kaart " + target
            elif kind in ("map_edit", "map_restore"):
                self._map_edit(kind, cmd)
        except (KeyError, TypeError, ValueError, OSError) as exc:
            self._set_status("error", None, "ongeldig commando: " + str(exc))

    def _set_initial_pose(self, cmd):
        m = PoseWithCovarianceStamped()
        p = make_pose(self.get_clock().now().to_msg(), cmd["x"], cmd["y"], cmd["yaw"])
        m.header = p.header
        m.pose.pose = p.pose
        # same spread as rviz's "2D Pose Estimate"
        m.pose.covariance[0] = 0.25
        m.pose.covariance[7] = 0.25
        m.pose.covariance[35] = 0.06853891945200942
        self.initialpose_pub.publish(m)
        self.status["message"] = "beginpositie gezet"

    # ---- localization (t49/t51) -------------------------------------------
    def _on_amcl_pose(self, msg):
        name = self.last_tick_map
        if not name or name == SESSION:
            return
        self.loc = loc_from_amcl(msg)
        self.loc_map = name
        # Last good pose per map: suggested as initial pose at the next start
        if self.loc["quality"] != "good":
            return
        prev, t = self.lastpose_saved
        moved = prev is None or math.hypot(prev["x"] - self.loc["x"], prev["y"] - self.loc["y"]) > 0.05 \
            or abs(prev["yaw"] - self.loc["yaw"]) > 0.1
        if moved and time.time() - t > 3:
            pose = {"x": self.loc["x"], "y": self.loc["y"], "yaw": self.loc["yaw"], "t": round(time.time())}
            try:
                tmp = lastpose_file(name) + ".tmp"
                with open(tmp, "w") as f:
                    json.dump(pose, f)
                os.replace(tmp, lastpose_file(name))
                self.lastpose_saved = (pose, time.time())
            except OSError:
                pass

    def _relocalize(self):
        """AMCL forgets where it is (particles over the whole map), then the
        robot turns 360° in place so the lidar sees everything around it."""
        if not self.reloc_client.service_is_ready():
            self._set_status("unavailable", None, "AMCL draait niet - start eerst Navigatie")
            return
        self.route = None
        self._stop_action()
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.reloc_client.call_async(Empty.Request())
        self.loc = None
        if not self.spin_client.server_is_ready():
            self._set_status("idle", None, "opnieuw lokaliseren: rij zelf een stukje rond (spin niet beschikbaar)")
            return
        goal = Spin.Goal()
        goal.target_yaw = 2 * math.pi
        goal.time_allowance.sec = 30
        self.spin_client.send_goal_async(goal)
        self._set_status("relocalizing", None, "opnieuw lokaliseren: robot draait ter plaatse")

    def _send_goal(self, pose, bt, mode, tol=None):
        if not self.nav_client.server_is_ready():
            self._set_status("unavailable", mode, "Nav2 draait niet - start eerst Navigatie")
            self.route = None
            return
        # always set (not only when changed): a restarted Nav2 is back at its default
        if tol and self.param_client.service_is_ready():
            req = SetParameters.Request()
            for name, value in zip(("xy_goal_tolerance", "yaw_goal_tolerance"), tol):
                req.parameters.append(Parameter(
                    name=GOAL_CHECKER + "." + name,
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))))
            seq = self.cmd_seq
            future = self.param_client.call_async(req)
            future.add_done_callback(lambda f: self._on_tolerance_set(f, tol, pose, bt, mode, seq))
            return
        self._send_goal_now(pose, bt, mode)

    def _on_tolerance_set(self, future, tol, pose, bt, mode, seq):
        try:
            ok = all(r.successful for r in future.result().results)
        except Exception:
            ok = False
        if ok:
            self.tolerance = tol
        else:
            self.get_logger().warn("doeltolerantie zetten mislukt - vorige waarde blijft")
        if seq != self.cmd_seq or (mode == "route" and (self.route is None or self.route.get("paused"))):
            return  # cancelled or replaced by a newer command meanwhile
        self._send_goal_now(pose, bt, mode)

    def _send_goal_now(self, pose, bt, mode):
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(self.get_clock().now().to_msg(), pose["x"], pose["y"], pose["yaw"])
        goal.behavior_tree = bt
        if mode == "goal":
            self._set_status("sending", "goal", "doel verzonden")
        future = self.nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)
        future.add_done_callback(lambda f: self._on_accepted(f, mode))

    def _send_route_point(self):
        r = self.route
        if r["index"] == 0:
            self._set_status("sending", "route", "route verzonden")
        self.status.update({"waypoint": r["index"], "waypoints": len(r["poses"]), "lap": r["lap"]})
        last = r["index"] == len(r["poses"]) - 1 and not r["loop"]
        pose = dict(r["poses"][r["index"]])
        # A point without a chosen heading faces the direction the robot comes
        # from (previous point; in a loop the first point from the last one).
        # The page's guess for point 1 (from where the robot stood) is wrong
        # from lap 2 on: the robot then had to turn ~130 deg at the point, and
        # MPPI (GoalAngleCritic) stalled ~0.3 m short -> recoveries (2026-10-09).
        prev = r["index"] - 1 if r["index"] > 0 else (len(r["poses"]) - 1 if r["lap"] > 1 else None)
        if pose.get("free") and prev is not None and prev != r["index"]:
            p0 = r["poses"][prev]
            pose["yaw"] = math.atan2(pose["y"] - p0["y"], pose["x"] - p0["x"])
        self._send_goal(pose, r["bt"], "route", r["tol"] if last else r["via_tol"])

    def _cancel(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.status["message"] = "annuleren..."
        else:
            self._set_status("idle", None, "niets om te annuleren")

    # ---- action callbacks --------------------------------------------------
    def _on_accepted(self, future, mode):
        handle = future.result()
        if not handle.accepted:
            self.route = None
            self._set_status("rejected", mode, "Nav2 weigerde het doel")
            return
        if mode == "route" and (self.route is None or self.route.get("paused")):
            handle.cancel_goal_async()  # route was cancelled while this point was being sent
            return
        self.goal_handle = handle
        if mode == "route":
            self.status.update({"state": "navigating", "mode": "route",
                                "message": "onderweg naar punt %d" % (self.route["index"] + 1)})
        else:
            self._set_status("navigating", mode, "onderweg")
        handle.get_result_async().add_done_callback(lambda f, h=handle: self._on_result(f, h, mode))

    def _on_nav_feedback(self, msg):
        fb = msg.feedback
        eta = fb.estimated_time_remaining.sec + fb.estimated_time_remaining.nanosec * 1e-9
        self.status.update({
            "distance_remaining": round(fb.distance_remaining, 2),
            "eta_s": round(eta, 1),
            "recoveries": fb.number_of_recoveries,
            "elapsed_s": fb.navigation_time.sec,
        })

    def _on_result(self, future, handle, mode):
        if handle is not self.goal_handle:
            return  # preempted by a newer goal from this page
        self.goal_handle = None
        res = future.result()
        state = STATE_NAMES.get(res.status, "unknown")
        message = {"succeeded": "aangekomen", "aborted": "mislukt (Nav2 gaf op)",
                   "canceled": "geannuleerd"}.get(state, state)
        if state == "canceled" and (self.paused_goal is not None or (
                mode == "route" and self.route and self.route.get("paused"))):
            return   # paused from the page: status already says so
        if mode == "route" and self.route:
            r = self.route
            if state == "succeeded":
                r["arrived"] = True
                action = r["poses"][r["index"]].get("action")
                if action:
                    self._start_action(action)
                else:
                    self._next_route_point()
                return
            else:
                message = "route gestopt bij punt %d: %s" % (r["index"] + 1, message)
            self.route = None
        self._set_status(state, mode, message)

    def _next_route_point(self):
        r = self.route
        if r is None:
            return
        r["arrived"] = False
        r["index"] += 1
        if r["index"] >= len(r["poses"]):
            if not r["loop"]:
                self.route = None
                self._set_status("succeeded", "route", "route klaar")
                return
            r["index"] = 0
            r["lap"] += 1
        self._send_route_point()

    # ---- pause / resume ----------------------------------------------------
    def _pause(self):
        r = self.route
        if r is not None:
            if r.get("paused"):
                return
            r["paused"] = True
            self._stop_action()
            self.status.update({"state": "paused", "mode": "route",
                                "message": "route gepauzeerd bij punt %d - Verder om door te rijden" % (r["index"] + 1)})
        elif self.goal_handle is not None and getattr(self, "last_goal", None):
            self.paused_goal = self.last_goal
            self._set_status("paused", "goal", "doel gepauzeerd - Verder om door te rijden")
        else:
            self.status["message"] = "niets om te pauzeren"
            return
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def _resume(self):
        r = self.route
        if r is not None and r.get("paused"):
            r["paused"] = False
            if r.get("arrived"):      # paused during (or as: hold) the point's action
                self._next_route_point()
            else:
                self._send_route_point()
        elif self.paused_goal is not None:
            g, self.paused_goal = self.paused_goal, None
            self._send_goal(g, g["bt"], "goal", g["tol"])
        else:
            self.status["message"] = "niets om te hervatten"

    # ---- actions per route point -------------------------------------------
    def _stop_action(self):
        if self.action_timer is not None:
            self.action_timer.cancel()
            self.action_timer = None
        if self.photo_sub is not None:
            self.destroy_subscription(self.photo_sub)
            self.photo_sub = None

    def _action_done(self, r, message=None):
        """Action of the current point finished: on to the next point (if the
        route was not cancelled, replaced or paused meanwhile)."""
        self._stop_action()
        if message:
            self.status["message"] = message
        if self.route is r and not r.get("paused"):
            self._next_route_point()

    def _after(self, seconds, fn):
        self._stop_action()

        def once():
            self.action_timer.cancel()
            self.action_timer = None
            fn()
        self.action_timer = self.create_timer(seconds, once)

    def _start_action(self, action):
        r = self.route
        n = r["index"] + 1
        self.status.update({"state": "action", "mode": "route"})
        self.get_logger().info("route: punt %d bereikt, actie %s" % (n, action))
        if action.startswith("wait:"):
            sec = max(1, int(action[5:]))
            self.status["message"] = "punt %d: %d s wachten" % (n, sec)
            self._after(sec, lambda: self._action_done(r))
        elif action == "beep":
            self.status["message"] = "punt %d: piep" % n
            if self.sound_client.service_is_ready():
                # 1 = "on" (rising scale, ~2.5 s); button1/2 (4/5) are silent in the OpenCR firmware
                self.sound_client.call_async(Sound.Request(value=1))
            self._after(2.5, lambda: self._action_done(r))
        elif action == "photo":
            self.status["message"] = "punt %d: foto..." % n
            self._after(3.0, lambda: self._action_done(r, "punt %d: geen camerabeeld - start Camera (Systeem)" % n))
            self.photo_sub = self.create_subscription(
                CompressedImage, CAMERA_TOPIC, lambda m: self._on_photo(m, r, n), 1)
        elif action == "spin":
            if not self.spin_client.server_is_ready():
                self._action_done(r, "punt %d: draaien niet beschikbaar" % n)
                return
            self.status["message"] = "punt %d: 360 graden draaien" % n
            goal = Spin.Goal()
            goal.target_yaw = 2 * math.pi
            goal.time_allowance.sec = 30
            self.spin_client.send_goal_async(goal).add_done_callback(lambda f: self._on_spin_accepted(f, r))
        elif action == "hold":
            r["paused"] = True
            self.status.update({"state": "paused", "message": "punt %d: wacht - druk Verder" % n})
        else:
            self._next_route_point()

    def _on_spin_accepted(self, future, r):
        handle = future.result()
        if not handle.accepted:
            self._action_done(r)
            return
        handle.get_result_async().add_done_callback(lambda f: self._action_done(r))

    def _on_photo(self, msg, r, n):
        if self.photo_sub is None:
            return
        self._stop_action()
        try:
            os.makedirs(PHOTO_DIR, exist_ok=True)
            ext = "png" if "png" in msg.format else "jpg"
            name = "%s_%s_punt%d.%s" % (running_map() or "kaart", time.strftime("%Y%m%d-%H%M%S"), n, ext)
            with open(os.path.join(PHOTO_DIR, name), "wb") as f:
                f.write(bytes(msg.data))
            photos = sorted(glob.glob(os.path.join(PHOTO_DIR, "*.*")), key=os.path.getmtime)
            for old in photos[:-PHOTO_KEEP]:   # only the robot's own route photos live here
                os.remove(old)
            self.last_photo = name
            self._action_done(r, "punt %d: foto %s" % (n, name))
        except OSError as exc:
            self._action_done(r, "punt %d: foto bewaren mislukt: %s" % (n, exc))

    # ---- zones ---------------------------------------------------------------
    def _saved_lane_cost(self):
        try:
            import yaml
            with open(OVERRIDES_FILE) as f:
                v = (yaml.safe_load(f) or {}).get("nav_web_node", {}).get("lane_outside_cost")
            return int(min(max(int(v), 0), 99)) if v is not None else LANE_OUTSIDE
        except Exception:
            return LANE_OUTSIDE

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "lane_outside_cost":
                if not isinstance(p.value, int) or not 0 <= p.value <= 99:
                    return SetParametersResult(successful=False, reason="geheel getal 0-99")
                self.zone_map = None      # rebuild the masks with the new cost
        return SetParametersResult(successful=True)

    def _target_map(self, wanted, what):
        """(map name, loaded) for a zones/map edit: nav.html sends the map
        name (t83) - a saved map Navigation doesn't run with is edited in
        its files only (loaded = False). Without a name: the running map."""
        running = running_map()
        if wanted and wanted != running:
            wanted = str(wanted)
            if wanted == SESSION or not re.match(r"^[A-Za-z0-9_-]{1,40}$", wanted) \
                    or not os.path.exists(os.path.join(MAP_DIR, wanted + ".yaml")):
                self._set_status("error", None, "%s: kaart '%s' bestaat niet" % (what, wanted))
                return None, False
            return wanted, False
        if not running:
            self._set_status("error", None, "%s: kies een kaart (of start Navigatie)" % what)
            return None, False
        return running, True

    def _set_zones(self, zones, wanted=None):
        name, loaded = self._target_map(wanted, "zones")
        if not name:
            return
        clean = []
        for z in zones:
            if z.get("type") in ("nogo", "wall", "lane", "speed", "area"):   # area: explore_demo only
                pts = [[float(p[0]), float(p[1])] for p in z.get("points", [])][:200]
                if len(pts) >= (2 if z["type"] == "wall" else 3):
                    zone = {"type": z["type"], "points": pts}
                    if z["type"] == "speed":
                        zone["percent"] = int(min(max(float(z.get("percent", 50)), 1), 100))
                    clean.append(zone)
        os.makedirs(MAP_DIR, exist_ok=True)
        # the previous zones stay as <map>.zones.bak.json (one step back)
        if os.path.exists(zones_file(name)) and load_zones(name) != clean:
            shutil.copyfile(zones_file(name), os.path.join(MAP_DIR, name + ".zones.bak.json"))
        with open(zones_file(name) + ".tmp", "w") as f:
            json.dump({"zones": clean}, f)
        os.replace(zones_file(name) + ".tmp", zones_file(name))
        if not loaded:
            self.status["message"] = "zones bewaard voor kaart %s (%d) - gelden zodra Navigatie met deze kaart start" \
                % (name, len(clean))
            return
        self.zones = clean
        self.zone_map = None          # rebuild + publish the masks
        self._zones_tick()
        self.status["message"] = "zones bewaard voor kaart %s (%d)" % (name, len(clean))

    def _zones_tick(self):
        name = running_map()
        self.exploring = explore_running()
        if not self.exploring:
            self.explore_paused = False   # a new exploration starts unpaused
        # Zones of an exploration are NOT wiped when a new run starts (tried
        # 2026-10-09: it silently removed zones the user had just drawn). The
        # map frame of a new run starts where the robot stands: if it started
        # elsewhere, the zones show up in the wrong place on nav.html.
        self.last_tick_map = name
        if name == SESSION and name != self.zone_map:
            zones = load_zones(name)
            self._publish_mask_grid(zones_grid(zones), name, zones)
        elif name and name != SESSION and name != self.zone_map and not self.zones_busy \
                and self.getmap_client.service_is_ready():
            self.zones_busy = True
            zones = load_zones(name)
            self.getmap_client.call_async(GetMap.Request()).add_done_callback(
                lambda f: self._publish_masks(f, name, zones))
        msg = String()
        msg.data = json.dumps({
            "map": name,
            "zones": self.zones if name == self.zone_map else load_zones(name) if name else [],
            "routes": load_routes(name) if name else {},
            "map_edited": bool(name) and os.path.exists(os.path.join(MAP_DIR, name + ".orig.pgm")),
        })
        self.zones_pub.publish(msg)

    def _route_file_cmd(self, kind, cmd):
        name = running_map()
        if not name:
            self._set_status("error", None, "routes: start eerst Navigatie met de kaart")
            return
        route_name = str(cmd.get("name", "")).strip()[:40]
        if not route_name:
            return
        routes = load_routes(name)
        if kind == "route_save":
            poses = [clean_pose(p) for p in (cmd.get("poses") or [])][:100]
            if not poses:
                return
            routes[route_name] = {"poses": poses, "loop": bool(cmd.get("loop"))}
            self.status["message"] = "route '%s' bewaard (%d punten)" % (route_name, len(poses))
        else:
            routes.pop(route_name, None)
            self.status["message"] = "route '%s' verwijderd" % route_name
        write_json(routes_file(name), {"routes": routes})
        self._zones_tick()

    def _map_edit(self, kind, cmd):
        name, loaded = self._target_map(cmd.get("map"), "kaart bewerken")
        if not name:
            return
        if name == SESSION:
            self._set_status("error", None, "kaart bewerken: sla de kaart eerst op")
            return
        if loaded and not self.loadmap_client.service_is_ready():
            self._set_status("error", None, "kaart bewerken: map_server antwoordt niet - wacht tot Navigatie klaar is")
            return
        yaml_path, pgm = map_pgm(name)
        orig = os.path.join(MAP_DIR, name + ".orig.pgm")
        if kind == "map_restore":
            if not os.path.exists(orig):
                self.status["message"] = "deze kaart is nooit bewerkt"
                return
            shutil.copyfile(orig, pgm)
        else:
            w, h = int(cmd["width"]), int(cmd["height"])
            cells = base64.b64decode(cmd["data"])
            if len(cells) != w * h:
                raise ValueError("kaartgrootte klopt niet")
            if not os.path.exists(orig):
                shutil.copyfile(pgm, orig)   # first edit: keep the original
            write_pgm(pgm, w, h, cells)
            # 205 (unknown) must load as unknown: free_thresh below 0.196
            with open(yaml_path) as f:
                text = f.read()
            new = re.sub(r"(?m)^free_thresh:.*$", "free_thresh: 0.196", text)
            if new != text:
                with open(yaml_path, "w") as f:
                    f.write(new)
        if not loaded:
            self.status["message"] = "kaart %s %s" % (name, "teruggezet" if kind == "map_restore" else "bewaard")
            return
        req = LoadMap.Request()
        req.map_url = yaml_path
        self.loadmap_client.call_async(req).add_done_callback(
            lambda f: self._on_map_loaded(f, name, kind))

    def _on_map_loaded(self, future, name, kind):
        try:
            ok = future.result().result == 0     # RESULT_SUCCESS
        except Exception:
            ok = False
        what = "teruggezet" if kind == "map_restore" else "bewaard"
        self.status["message"] = ("kaart %s %s en opnieuw geladen" % (name, what) if ok
                                  else "kaart %s %s, maar herladen mislukt - herstart Navigatie" % (name, what))
        self.zone_map = None          # masks again (same size, but to be sure)
        self._zones_tick()

    def _publish_masks(self, future, name, zones):
        self.zones_busy = False
        try:
            grid = future.result().map
        except Exception as exc:
            self.get_logger().warn("zones: kaart ophalen mislukt: %s" % exc)
            return
        self._publish_mask_grid(grid.info, name, zones)

    def _publish_mask_grid(self, info_meta, name, zones):
        class G:
            info = info_meta
        grid = G()
        keepout, lanes, speed = rasterize(grid.info, zones,
                                          self.get_parameter("lane_outside_cost").value)
        stamp = self.get_clock().now().to_msg()
        for key, mask in (("keepout", keepout), ("lanes", lanes), ("speed", speed)):
            info_pub, mask_pub, mask_topic = self.filter_pubs[key]
            m = OccupancyGrid()
            m.header.frame_id = MAP_FRAME
            m.header.stamp = stamp
            m.info = grid.info
            m.data = mask.flatten().tolist()
            mask_pub.publish(m)
            info = CostmapFilterInfo()
            info.header.frame_id = MAP_FRAME
            info.header.stamp = stamp
            info.type = 1 if key == "speed" else 0   # SPEED_FILTER_PERCENT / KEEPOUT
            info.filter_mask_topic = mask_topic
            info.base = 0.0
            info.multiplier = 1.0
            info_pub.publish(info)
        self.zones = zones
        self.zone_map = name
        self.get_logger().info("zones: %d voor kaart %s gepubliceerd" % (len(zones), name))

    # ---- status ------------------------------------------------------------
    def _set_status(self, state, mode, message):
        keep = {k: self.status[k] for k in ("waypoint", "waypoints", "lap") if k in self.status} \
            if mode == "route" else {}
        self.status = {"state": state, "mode": mode, "message": message, "t": time.time()}
        self.status.update(keep)

    def _publish_status(self):
        s = dict(self.status)
        s["nav2_ready"] = self.nav_client.server_is_ready()
        s["route_loop"] = bool(self.route and self.route["loop"])
        s["last_photo"] = self.last_photo
        s["explore"] = ("paused" if self.explore_paused else "running") if self.exploring else None
        # AMCL gone or another map: forget the old quality
        if not s["nav2_ready"] or self.loc_map != self.last_tick_map:
            self.loc = None
        s["loc"] = self.loc
        name = self.last_tick_map
        s["pose_suggestion"] = load_lastpose(name) if s["nav2_ready"] and self.loc is None \
            and name and name != SESSION else None
        if self.tolerance:
            s["tolerance"] = {"xy": self.tolerance[0], "yaw": self.tolerance[1]}
        msg = String()
        msg.data = json.dumps(s)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = NavWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
