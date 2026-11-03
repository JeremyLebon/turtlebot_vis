#!/usr/bin/env python3
"""Camera settings from system.html (resolution, fps, JPEG quality).

The image's camera_params.yaml holds the defaults; the choice from the page is
kept in state/camera_settings.json (survives updates) and merged into
/tmp/camera_params.yaml, which camera_node is started with (camera_start.sh
at container start, launch_control_node for the page buttons).

    python3 camera_params.py    # writes /tmp/camera_params.yaml, prints its path
"""
import json
import os

import yaml

BASE = "/root/turtlebot3_ws/camera_params.yaml"
OUT = "/tmp/camera_params.yaml"
STATE = "/root/turtlebot_state/camera_settings.json"
# Intrinsic calibration (camera_calibration > Commit -> /camera/set_camera_info),
# per resolution - a calibration only holds for the resolution it was made at.
CALIB_DIR = "/root/turtlebot_state/camera_info"


def calib_file(resolution):
    return os.path.join(CALIB_DIR, "camera_%s.yaml" % resolution)


def calibrated(resolution=None):
    return os.path.exists(calib_file(resolution or settings()["resolution"]))

# choices on the page (the IMX708 / Camera Module 3 is 16:9; the ISP scales)
RESOLUTIONS = {"640x480": (640, 480), "800x600": (800, 600), "1280x720": (1280, 720),
               "1536x864": (1536, 864)}
FPS = (5, 10, 15, 20, 30)


def defaults():
    with open(BASE) as f:
        p = yaml.safe_load(f)["/camera"]["ros__parameters"]
    return {"resolution": "%dx%d" % (p["width"], p["height"]),
            "fps": round(1e6 / p["FrameDurationLimits"][0]),
            "jpeg_quality": p["jpeg_quality"]}


def settings():
    s = defaults()
    try:
        with open(STATE) as f:
            saved = json.load(f)
        if saved.get("resolution") in RESOLUTIONS:
            s["resolution"] = saved["resolution"]
        if saved.get("fps") in FPS:
            s["fps"] = saved["fps"]
        if isinstance(saved.get("jpeg_quality"), int) and 20 <= saved["jpeg_quality"] <= 95:
            s["jpeg_quality"] = saved["jpeg_quality"]
    except (OSError, ValueError):
        pass
    return s


def save(new):
    """Keep only valid values that differ from the defaults; {} = defaults."""
    d = defaults()
    keep = {k: v for k, v in new.items() if k in d and v != d[k]}
    if keep:
        with open(STATE, "w") as f:
            json.dump(keep, f)
    elif os.path.exists(STATE):
        os.remove(STATE)


def is_default():
    return settings() == defaults()


def mbit_estimate(s):
    """Rough Mbit/s of the compressed stream: measured 3.4 Mbit/s at 640x480,
    quality 60, 15 fps and 6.7 Mbit/s at 30 fps (turtlebot09, 2026-10-09;
    depends on what the camera sees)."""
    w, h = RESOLUTIONS[s["resolution"]]
    q = s["jpeg_quality"]
    qf = 0.7 if q <= 40 else 1.0 if q <= 60 else 1.5 if q <= 80 else 2.4
    return round(3.4 * (w * h / 307200.0) * (s["fps"] / 15.0) * qf, 1)


def write():
    s = settings()
    with open(BASE) as f:
        p = yaml.safe_load(f)
    r = p["/camera"]["ros__parameters"]
    r["width"], r["height"] = RESOLUTIONS[s["resolution"]]
    us = int(round(1e6 / s["fps"]))
    r["FrameDurationLimits"] = [us, us]
    r["jpeg_quality"] = s["jpeg_quality"]
    os.makedirs(CALIB_DIR, exist_ok=True)   # camera_info_manager writes the calibration here
    r["camera_info_url"] = "file://" + calib_file(s["resolution"])
    with open(OUT, "w") as f:
        yaml.safe_dump(p, f, sort_keys=False)
    return OUT


if __name__ == "__main__":
    try:
        print(write())
    except Exception:
        print(BASE)   # never keep the camera from starting
