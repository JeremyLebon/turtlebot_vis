#!/usr/bin/env python3
"""Simulated external sensors and camera (session 4, until the real ones arrive).

Started and stopped with the "Sensoren" button on the status page
(launch_control_node: /launch/sensors). The values depend on where the robot
is: a few hotspots on the map (warm, humid, gas leak, dark, noisy), chosen per
robot from the free cells of the map, so every group measures something else.
Each sensor follows its target with a time constant (a gas sensor needs a few
seconds at a waypoint) and gets noise, like a real one.

Topics (the real sensors should use the same names later):
  /sensors/temperature   sensor_msgs/Temperature        degrees C
  /sensors/humidity      sensor_msgs/RelativeHumidity   0.0 - 1.0 (ROS convention)
  /sensors/pressure      sensor_msgs/FluidPressure      Pa
  /sensors/illuminance   sensor_msgs/Illuminance        lux
  /sensors/gas/co2       std_msgs/Float32               ppm
  /sensors/gas/tvoc      std_msgs/Float32               ppb
  /sensors/gas/co        std_msgs/Float32               ppm
  /sensors/sound         std_msgs/Float32               dB(A)
  /sensors/summary       std_msgs/String                JSON of all values (status page)
Camera, only when no real camera_node runs (camera_present.sh / camera button):
  /camera/image_raw/compressed, /camera/camera_info - a view rendered from the
  map and the robot pose (walls, floor tiles, a coloured marker at every hotspot).

Hotspots by hand: /root/turtlebot_state/sim_sensors.json
  {"hotspots": [{"type": "gas", "x": 1.0, "y": -0.5, "r": 0.6}, ...]}  (map frame)
"""
import json
import math
import os
import re
import socket
import subprocess
import time

import numpy as np
import cv2
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import (CameraInfo, CompressedImage, FluidPressure, Illuminance,
                             RelativeHumidity, Temperature)
from std_msgs.msg import Float32, String

HOTSPOT_FILE = "/root/turtlebot_state/sim_sensors.json"
RATE_HZ = 2.0
CAM_HZ = 5.0
CAM_W, CAM_H = 320, 240
CAM_FOV = math.radians(62)       # Pi camera module v2
CAM_Z = 0.18                     # camera height above the floor (m)
WALL_Z = 0.40                    # walls in the image (the map has no heights)
RAY_MAX, RAY_STEP = 5.0, 0.03
JPEG_QUALITY = 40

# effect of one hotspot at its centre; g = gaussian with radius r (m)
HOTSPOT_TYPES = {
    "warm":    {"temperature": 9.0, "humidity": -12.0, "colour": (40, 90, 230)},     # BGR
    "vochtig": {"humidity": 30.0, "temperature": -1.5, "colour": (220, 140, 40)},
    "gas":     {"co2": 1800.0, "tvoc": 900.0, "co": 35.0, "colour": (60, 200, 200)},
    "donker":  {"illuminance": -0.95, "colour": (40, 40, 40)},                       # factor
    "lawaai":  {"sound": 35.0, "colour": (180, 60, 170)},
}
BASE = {"temperature": 21.0, "humidity": 45.0, "pressure": 101325.0, "illuminance": 350.0,
        "co2": 450.0, "tvoc": 80.0, "co": 0.5, "sound": 38.0}
TAU = {"temperature": 4.0, "humidity": 3.0, "pressure": 1.0, "illuminance": 0.3,
       "co2": 2.5, "tvoc": 2.0, "co": 2.0, "sound": 0.2}
NOISE = {"temperature": 0.08, "humidity": 0.4, "pressure": 4.0, "illuminance": 0.03,
         "co2": 12.0, "tvoc": 8.0, "co": 0.3, "sound": 1.2}       # illuminance: relative


def robot_nr():
    m = re.search(r"(\d+)$", socket.gethostname())
    return int(m.group(1)) if m else 0


def real_camera_running():
    return subprocess.run(["pgrep", "-f", "camera_ros[/ ]camera_node"],
                          capture_output=True).returncode == 0


class SimSensors(Node):
    def __init__(self):
        super().__init__("sim_sensors")
        self.rng = np.random.default_rng()
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.grid = None            # (occ array, resolution, origin x, origin y)
        self.hotspots = []
        self.hotspot_frame = None
        self.manual = self._load_manual()
        self.values = None
        self.last_t = None
        self.pose = None            # (x, y, yaw, frame)
        self.out = None             # last published values (camera overlay)

        # map_server latches its map, Cartographer publishes it again and again
        for durability in (DurabilityPolicy.TRANSIENT_LOCAL, DurabilityPolicy.VOLATILE):
            qos = QoSProfile(depth=1, durability=durability, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, "/map", self._on_map, qos)

        self.pubs = {
            "temperature": self.create_publisher(Temperature, "/sensors/temperature", 10),
            "humidity": self.create_publisher(RelativeHumidity, "/sensors/humidity", 10),
            "pressure": self.create_publisher(FluidPressure, "/sensors/pressure", 10),
            "illuminance": self.create_publisher(Illuminance, "/sensors/illuminance", 10),
            "co2": self.create_publisher(Float32, "/sensors/gas/co2", 10),
            "tvoc": self.create_publisher(Float32, "/sensors/gas/tvoc", 10),
            "co": self.create_publisher(Float32, "/sensors/gas/co", 10),
            "sound": self.create_publisher(Float32, "/sensors/sound", 10),
        }
        self.summary_pub = self.create_publisher(String, "/sensors/summary", 10)
        self.img_pub = self.create_publisher(CompressedImage, "/camera/image_raw/compressed", 1)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 1)
        self.real_cam = real_camera_running()
        self.create_timer(1.0 / RATE_HZ, self._tick_sensors)
        self.create_timer(1.0 / CAM_HZ, self._tick_camera)
        self.create_timer(2.0, self._check_camera)
        self.get_logger().info("sim_sensors ready (robot %d, hotspots %s)"
                               % (robot_nr(), "by hand" if self.manual else "from the map"))

    # ---- map, pose, hotspots ---------------------------------------------
    def _load_manual(self):
        try:
            with open(HOTSPOT_FILE) as f:
                hs = json.load(f).get("hotspots", [])
            return [h for h in hs if h.get("type") in HOTSPOT_TYPES] or None
        except (OSError, ValueError, AttributeError):
            return None

    def _on_map(self, msg):
        info = msg.info
        occ = np.array(msg.data, dtype=np.int8).reshape(info.height, info.width)
        self.grid = (occ, info.resolution, info.origin.position.x, info.origin.position.y)
        if not self.manual and self.hotspot_frame == "map" and all(self._free(h["x"], h["y"]) for h in self.hotspots):
            return   # keep them while the map grows (SLAM)
        self._make_hotspots("map")

    def _free(self, x, y):
        occ, res, ox, oy = self.grid
        i, j = int((y - oy) / res), int((x - ox) / res)
        return 0 <= i < occ.shape[0] and 0 <= j < occ.shape[1] and occ[i, j] == 0

    def _make_hotspots(self, frame):
        self.hotspot_frame = frame
        if self.manual:
            self.hotspots = [dict(h, r=float(h.get("r", 0.6))) for h in self.manual]
            return
        types = list(HOTSPOT_TYPES)
        if frame == "map" and self.grid is not None:
            occ, res, ox, oy = self.grid
            # same robot + same map -> same hotspots after a restart
            rng = np.random.default_rng(robot_nr() * 100003 + occ.shape[0] * 7 + occ.shape[1])
            # free cells at least 0.25 m from a wall or unknown space
            bad = occ != 0
            k = max(1, int(0.25 / res))
            grown = bad.copy()
            for di in range(-k, k + 1):
                for dj in range(-k, k + 1):
                    grown |= np.roll(np.roll(bad, di, 0), dj, 1)
            cells = np.argwhere(~grown)
            if len(cells) == 0:
                cells = np.argwhere(occ == 0)
            pts = []
            for _ in range(400):
                if len(pts) == len(types) or len(cells) == 0:
                    break
                i, j = cells[rng.integers(len(cells))]
                p = (ox + (j + 0.5) * res, oy + (i + 0.5) * res)
                if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 1.0 for q in pts):
                    pts.append(p)
        else:
            # no map yet: around the start position (odom)
            rng = np.random.default_rng(robot_nr())
            pts = [(1.6 * math.cos(a), 1.6 * math.sin(a))
                   for a in np.linspace(0, 2 * math.pi, len(types), endpoint=False) + rng.uniform(0, 1)]
        rng.shuffle(types)
        self.hotspots = [{"type": t, "x": round(p[0], 2), "y": round(p[1], 2), "r": round(float(rng.uniform(0.5, 0.8)), 2)}
                         for t, p in zip(types, pts)]
        self.get_logger().info("hotspots (%s): %s" % (frame, json.dumps(self.hotspots)))

    def _update_pose(self):
        for frame in ("map", "odom"):
            try:
                t = self.tf.lookup_transform(frame, "base_footprint", Time())
            except tf2_ros.TransformException:
                continue
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.pose = (t.transform.translation.x, t.transform.translation.y, yaw, frame)
            if not self.manual and self.hotspot_frame != frame and not (frame == "map" and self.grid is None):
                self._make_hotspots(frame)
            return True
        return False

    # ---- sensors ----------------------------------------------------------
    def _targets(self, x, y):
        tgt = dict(BASE)
        t = time.time()
        tgt["temperature"] += 0.4 * math.sin(t / 900.0)      # slow drift in the room
        tgt["pressure"] += 60.0 * math.sin(t / 3600.0)
        dark = 1.0
        for h in self.hotspots:
            g = math.exp(-((x - h["x"]) ** 2 + (y - h["y"]) ** 2) / (2 * h["r"] ** 2))
            for k, v in HOTSPOT_TYPES[h["type"]].items():
                if k == "illuminance":
                    dark *= 1 + v * g
                elif k == "sound":
                    # sound levels add up logarithmically
                    tgt["sound"] = 10 * math.log10(10 ** (tgt["sound"] / 10) + 10 ** ((BASE["sound"] + v * g) / 10) - 10 ** (BASE["sound"] / 10))
                elif k != "colour":
                    tgt[k] += v * g
        tgt["illuminance"] *= dark
        tgt["humidity"] = min(100.0, max(0.0, tgt["humidity"]))
        return tgt

    def _tick_sensors(self):
        if not self._update_pose():
            return   # no odom yet: bringup not running
        x, y = self.pose[:2]
        tgt = self._targets(x, y)
        now = time.monotonic()
        if self.values is None:
            self.values = dict(tgt)
        else:
            dt = now - self.last_t
            for k in tgt:
                self.values[k] += (tgt[k] - self.values[k]) * (1 - math.exp(-dt / TAU[k]))
        self.last_t = now
        out = {}
        for k, v in self.values.items():
            n = self.rng.normal(0, NOISE[k])
            out[k] = max(0.0, v * (1 + n) if k == "illuminance" else v + n)
        stamp = self.get_clock().now().to_msg()

        def stamped(msg, field, value, variance):
            msg.header.stamp = stamp
            msg.header.frame_id = "base_link"
            setattr(msg, field, value)
            msg.variance = variance
            return msg
        self.pubs["temperature"].publish(stamped(Temperature(), "temperature", out["temperature"], NOISE["temperature"] ** 2))
        self.pubs["humidity"].publish(stamped(RelativeHumidity(), "relative_humidity", out["humidity"] / 100.0,
                                              (NOISE["humidity"] / 100.0) ** 2))
        self.pubs["pressure"].publish(stamped(FluidPressure(), "fluid_pressure", out["pressure"], NOISE["pressure"] ** 2))
        self.pubs["illuminance"].publish(stamped(Illuminance(), "illuminance", out["illuminance"],
                                                 (NOISE["illuminance"] * out["illuminance"]) ** 2))
        for k in ("co2", "tvoc", "co", "sound"):
            self.pubs[k].publish(Float32(data=float(out[k])))
        summary = {k: round(v, 2) for k, v in out.items()}
        summary.update(frame=self.pose[3], x=round(x, 2), y=round(y, 2), sim=True, camera=not self.real_cam)
        self.summary_pub.publish(String(data=json.dumps(summary)))
        self.out = out

    # ---- camera -----------------------------------------------------------
    def _check_camera(self):
        self.real_cam = real_camera_running()

    def _tick_camera(self):
        if self.real_cam or self.pose is None:
            return
        if self.img_pub.get_subscription_count() == 0 and self.info_pub.get_subscription_count() == 0:
            return   # nobody watching: no CPU
        img = self._render()
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            return
        stamp = self.get_clock().now().to_msg()
        msg = CompressedImage(format="jpeg", data=jpg.tobytes())
        msg.header.stamp, msg.header.frame_id = stamp, "camera"
        self.img_pub.publish(msg)
        f = (CAM_W / 2) / math.tan(CAM_FOV / 2)
        info = CameraInfo(width=CAM_W, height=CAM_H, distortion_model="plumb_bob", d=[0.0] * 5,
                          k=[f, 0.0, CAM_W / 2, 0.0, f, CAM_H / 2, 0.0, 0.0, 1.0],
                          r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                          p=[f, 0.0, CAM_W / 2, 0.0, 0.0, f, CAM_H / 2, 0.0, 0.0, 0.0, 1.0, 0.0])
        info.header.stamp, info.header.frame_id = stamp, "camera"
        self.info_pub.publish(info)

    def _render(self):
        """First-person view from the occupancy grid: floor tiles with a
        coloured disc at every hotspot, walls by ray marching."""
        W, H = CAM_W, CAM_H
        x, y, th, _ = self.pose
        f = (W / 2) / math.tan(CAM_FOV / 2)
        hz = H / 2
        c, s = math.cos(th), math.sin(th)
        u = (W / 2 - np.arange(W) - 0.5) / f                 # left of the axis = +

        img = np.empty((H, W, 3), np.float32)
        # above the horizon: dark wall of the room
        img[: int(hz)] = np.linspace(55, 35, int(hz))[:, None, None] * np.array([1.0, 1.0, 1.1])

        # floor: forward distance per row, world position per pixel
        rows = np.arange(int(hz), H)
        z = f * CAM_Z / (rows + 0.5 - hz)
        Z = z[:, None]
        px = x + Z * c - Z * u[None, :] * s
        py = y + Z * s + Z * u[None, :] * c
        tile = ((np.floor(px / 0.5) + np.floor(py / 0.5)) % 2)[..., None]
        floor = np.where(tile > 0, np.array([150, 160, 165], np.float32), np.array([125, 135, 140], np.float32))
        for h in self.hotspots:
            if self.hotspot_frame != self.pose[3]:
                break
            d2 = (px - h["x"]) ** 2 + (py - h["y"]) ** 2
            floor[d2 < 0.18 ** 2] = HOTSPOT_TYPES[h["type"]]["colour"]
        fog = np.clip(1.0 - Z / 6.0, 0.35, 1.0)[..., None]
        img[int(hz):] = floor * fog

        # walls: march every column's ray through the grid
        if self.grid is not None and self.pose[3] == "map":
            occ, res, ox, oy = self.grid
            ang = th + np.arctan(u)
            t = np.arange(RAY_STEP, RAY_MAX, RAY_STEP)
            gx = ((x + np.cos(ang)[:, None] * t - ox) / res).astype(np.int32)
            gy = ((y + np.sin(ang)[:, None] * t - oy) / res).astype(np.int32)
            inside = (gx >= 0) & (gx < occ.shape[1]) & (gy >= 0) & (gy < occ.shape[0])
            hit = np.zeros_like(inside)
            hit[inside] = occ[gy[inside], gx[inside]] > 50
            any_hit = hit.any(axis=1)
            dist = t[hit.argmax(axis=1)] * np.cos(ang - th)          # forward distance
            dist = np.maximum(dist, 0.05)
            top = hz - f * (WALL_Z - CAM_Z) / dist
            bottom = hz + f * CAM_Z / dist
            R = np.arange(H)[:, None]
            mask = (R >= top[None, :]) & (R <= bottom[None, :]) & any_hit[None, :]
            shade = np.clip(1.0 - dist / 6.0, 0.25, 1.0)
            # a vertical seam every 0.25 m along the wall, so turning is visible
            hx, hy = x + np.cos(ang) * dist / np.cos(ang - th), y + np.sin(ang) * dist / np.cos(ang - th)
            seam = ((np.abs(hx / 0.25 - np.round(hx / 0.25)) < 0.04) | (np.abs(hy / 0.25 - np.round(hy / 0.25)) < 0.04))
            wall = np.array([175, 195, 210], np.float32)[None, :] * (shade * np.where(seam, 0.8, 1.0))[:, None]
            img[mask] = np.broadcast_to(wall[None, :, :], (H, W, 3))[mask]

        img = np.clip(img, 0, 255).astype(np.uint8)
        cv2.putText(img, "SIM %.2f %.2f" % (x, y), (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        o = self.out
        if o:
            cv2.putText(img, "%.1f C  %.0f %%  %.0f ppm  %.0f lx" % (o["temperature"], o["humidity"], o["co2"], o["illuminance"]),
                        (6, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        return img


def main():
    rclpy.init()
    node = SimSensors()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
