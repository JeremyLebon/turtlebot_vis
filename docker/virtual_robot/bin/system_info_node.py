#!/usr/bin/env python3
"""Publishes basic host system info (IP, MAC, CPU load, memory, wifi
signal, uptime, Pi health, SD card, Zenoh clients)
as a JSON-encoded std_msgs/String on /system_info, for the status page.
Not exposed via any standard ROS2 topic, hence this small helper node.
Stdlib-only (no psutil) - keeps the image lean."""

import array
import collections
import fcntl
import glob
import json
import os
import socket
import re
import struct
import subprocess
import threading
import time
import urllib.request

import fenix_services
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import String

IFACE = os.environ.get("STATUS_PAGE_IFACE", "wlan0")

# Robot config env vars worth showing on the status page (see
# turtlebot_setup/setup_turtlebot.sh / docker-compose.yaml for where
# these come from). Deliberately a fixed allowlist, not all of os.environ.
CONFIG_ENV_VARS = [
    "ROS_DOMAIN_ID",
    "RMW_IMPLEMENTATION",
    "LDS_MODEL",
    "TURTLEBOT3_MODEL",
    "TURTLEBOT3_NAME",
    "TURTLEBOT_NR",
    "ENABLE_CAMERA",
    "ZENOH_CONFIG_OVERRIDE",
]

CPU_EMA_ALPHA = 0.35  # smoothing factor for per-core CPU% (higher = more responsive/jumpy)


def get_ip(iface):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ip = socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', iface[:15].encode())
        )[20:24])
        s.close()
        return ip
    except Exception:
        return None


def get_mac(iface):
    try:
        with open(f"/sys/class/net/{iface}/address") as f:
            return f.read().strip().upper()
    except Exception:
        return None


def get_mem_info():
    """Returns (percent_used, total_mb, used_mb) - /proc/meminfo values are in kB."""
    try:
        info = {}
        with open("/proc/meminfo") as f:
            for line in f:
                key, val = line.split(":")
                info[key.strip()] = int(val.strip().split()[0])
        total_kb = info.get("MemTotal", 0)
        avail_kb = info.get("MemAvailable", 0)
        if total_kb <= 0:
            return None, None, None
        used_kb = total_kb - avail_kb
        percent = round(used_kb / total_kb * 100, 1)
        return percent, round(total_kb / 1024, 0), round(used_kb / 1024, 0)
    except Exception:
        return None, None, None


def read_cpu_times():
    """Returns {'cpu0': (total, idle), 'cpu1': (...), ...} from /proc/stat."""
    times = {}
    try:
        with open("/proc/stat") as f:
            for line in f:
                if not line.startswith("cpu") or line.startswith("cpu "):
                    continue
                parts = line.split()
                label = parts[0]
                fields = [int(x) for x in parts[1:]]
                # user nice system idle iowait irq softirq steal ...
                idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
                total = sum(fields)
                times[label] = (total, idle)
    except Exception:
        pass
    return times


def get_uptime_seconds():
    try:
        with open("/proc/uptime") as f:
            return float(f.read().split()[0])
    except Exception:
        return None


def read_net_bytes(iface):
    """Returns (rx_bytes, tx_bytes) for iface from /proc/net/dev, or (None, None)."""
    try:
        with open("/proc/net/dev") as f:
            for line in f:
                if ":" not in line:
                    continue
                name, rest = line.split(":", 1)
                if name.strip() != iface:
                    continue
                fields = rest.split()
                # rx: bytes packets errs drop ... | tx: bytes packets errs drop ...
                return int(fields[0]), int(fields[8])
    except Exception:
        pass
    return None, None


def get_wifi(iface):
    """Returns (signal_dbm, link_quality_percent) for iface from
    /proc/net/wireless, or (None, None) if not a wifi interface / not
    associated. Link quality is reported out of 70 by most drivers
    (incl. the RPi5's brcmfmac)."""
    try:
        with open("/proc/net/wireless") as f:
            for line in f:
                if ":" not in line:
                    continue
                name, rest = line.split(":", 1)
                if name.strip() != iface:
                    continue
                fields = rest.split()
                # status  link  level  noise ...
                quality = float(fields[1].rstrip("."))
                level = float(fields[2].rstrip("."))
                return int(level), round(min(quality / 70 * 100, 100))
    except Exception:
        pass
    return None, None


NET_AVG_S = 10         # s: wifi traffic averaged over this window (net_*_kbps_avg)
SIOCGIWFREQ = 0x8B05   # wireless extensions: current frequency of the interface


def wifi_band(freq_mhz):
    """(band, channel) for a wifi frequency in MHz."""
    if freq_mhz is None:
        return None, None
    if freq_mhz < 3000:
        return "2.4 GHz", 14 if freq_mhz == 2484 else (freq_mhz - 2407) // 5
    if freq_mhz < 5950:
        return "5 GHz", (freq_mhz - 5000) // 5
    return "6 GHz", (freq_mhz - 5950) // 5


# nl80211 channel width enum -> MHz (80P80 counted as 160)
NL80211_WIDTH_MHZ = {0: 20, 1: 20, 2: 40, 3: 80, 4: 160, 5: 160, 6: 5, 7: 10, 13: 320}


def _nl_attrs(data):
    attrs, i = {}, 0
    while i + 4 <= len(data):
        ln, typ = struct.unpack_from("HH", data, i)
        if ln < 4:
            break
        attrs[typ & 0x3FFF] = data[i + 4:i + ln]
        i += (ln + 3) & ~3
    return attrs


def _genl(sock, family, cmd, attrs, seq):
    payload = b"".join(struct.pack("HH", 4 + len(v), t) + v + b"\0" * (-len(v) % 4) for t, v in attrs)
    msg = struct.pack("BBH", cmd, 1, 0) + payload
    sock.send(struct.pack("IHHII", 16 + len(msg), family, 1, seq, 0) + msg)   # NLM_F_REQUEST
    data = sock.recv(65536)
    ln, typ = struct.unpack_from("IH", data, 0)
    if typ == 2:            # NLMSG_ERROR
        raise OSError("netlink error %d" % struct.unpack_from("i", data, 16)[0])
    return _nl_attrs(data[20:ln])


def get_wifi_channel_info(iface):
    """(freq_mhz, width_mhz, center_mhz) of iface via nl80211 (generic
    netlink, no `iw` needed), or (None, None, None)."""
    try:
        s = socket.socket(socket.AF_NETLINK, socket.SOCK_RAW, 16)   # NETLINK_GENERIC
        try:
            s.settimeout(1)
            s.bind((0, 0))
            fam = _genl(s, 0x10, 3, [(2, b"nl80211\0")], 1)          # CTRL_CMD_GETFAMILY
            fid = struct.unpack("H", fam[1][:2])[0]
            a = _genl(s, fid, 5, [(3, struct.pack("I", socket.if_nametoindex(iface)))], 2)  # GET_INTERFACE
        finally:
            s.close()
        freq = struct.unpack("I", a[38][:4])[0] if 38 in a else None
        width = NL80211_WIDTH_MHZ.get(struct.unpack("I", a[159][:4])[0]) if 159 in a else None
        center = struct.unpack("I", a[160][:4])[0] if 160 in a else None
        return freq, width, center
    except (OSError, KeyError, struct.error):
        return None, None, None


def get_wifi_freq(iface):
    """Frequency in MHz the wifi interface uses now, or None: the kernel's
    wireless ioctl (cfg80211's wext compat, no extra tools), else `iw`."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # struct iwreq: ifname[16] + struct iw_freq {s32 m; s16 e; u8 i; u8 flags}
            res = fcntl.ioctl(s.fileno(), SIOCGIWFREQ, struct.pack("16s16x", iface.encode()[:15]))
        finally:
            s.close()
        m, e = struct.unpack_from("ih", res, 16)
        if e > 0 or m >= 1000:      # e == 0 and m < 1000 is a channel number, not a frequency
            return int(round(m * 10 ** e / 1e6))
    except OSError:
        pass
    try:
        out = subprocess.run(["iw", "dev", iface, "link"], capture_output=True, text=True, timeout=2).stdout
        m = re.search(r"freq:\s*(\d+)", out)
        return int(float(m.group(1))) if m else None
    except (OSError, subprocess.SubprocessError):
        return None


# --- Pi health -------------------------------------------------------------
# vcgencmd isn't in this (Ubuntu) image, but it is only a thin wrapper around
# the VideoCore mailbox: the same "gencmd" request over /dev/vcio (available
# because the container is privileged) gives get_throttled and, on a Pi 5,
# the PMIC's 5V input voltage.
IOCTL_MBOX_PROPERTY = 0xC0086400  # _IOWR(100, 0, char *) on 64-bit
GENCMD_TAG = 0x00030080
GENCMD_BUFLEN = 1024


def gencmd(cmd):
    """Runs a vcgencmd command via the mailbox, returns its text or None."""
    try:
        words = 6 + GENCMD_BUFLEN // 4 + 1
        raw = bytearray(words * 4)
        struct.pack_into("6I", raw, 0, words * 4, 0, GENCMD_TAG, GENCMD_BUFLEN, 0, 0)
        req = cmd.encode() + b"\0"
        raw[24:24 + len(req)] = req
        buf = array.array("I", bytes(raw))
        with open("/dev/vcio", "rb") as f:
            fcntl.ioctl(f, IOCTL_MBOX_PROPERTY, buf, True)
        return buf.tobytes()[24:24 + GENCMD_BUFLEN].split(b"\0", 1)[0].decode(errors="replace")
    except Exception:
        return None


def get_throttled():
    """get_throttled bitmask (bit 0 undervoltage now, 1 freq capped, 2
    throttled, 3 soft temp limit; bits 16-19 = same, occurred since boot)."""
    out = gencmd("get_throttled")  # "throttled=0x50000"
    try:
        return int(out.split("=")[1], 16)
    except Exception:
        return None


def get_ext5v():
    out = gencmd("pmic_read_adc EXT5V_V")  # "EXT5V_V volt(24)=5.06922000V" (Pi 5)
    try:
        return round(float(out.split("=")[1].rstrip("V")), 2)
    except Exception:
        return None


# Virtuele robot (turtlebot-vis, `robot virtueel`): geen Pi en geen OpenCR - de
# temperatuur van de laptop is geen Pi-waarschuwing, de simulator speelt OpenCR en lidar.
SIM = os.environ.get("FENIX_SIM") == "1"


def get_cpu_temp():
    if SIM:
        return None
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return round(int(f.read()) / 1000, 1)
    except Exception:
        return None


def get_fan():
    """Pi 5 fan header (Active Cooler): {"rpm", "pwm_percent", "level",
    "max_level"} or None if there is no pwm-fan. The firmware switches it
    on at ~50 °C (level 1) and steps up at ~60/67.5/75 °C. The hwmon index
    changes between boots, so look it up by name."""
    for name_file in glob.glob("/sys/class/hwmon/hwmon*/name"):
        try:
            with open(name_file) as f:
                if f.read().strip() != "pwmfan":
                    continue
            base = os.path.dirname(name_file)
            with open(base + "/fan1_input") as f:
                rpm = int(f.read())
            with open(base + "/pwm1") as f:
                pwm = int(f.read())
            fan = {"rpm": rpm, "pwm_percent": round(pwm / 255 * 100), "level": None, "max_level": None}
            for cd in glob.glob("/sys/class/thermal/cooling_device*"):
                with open(cd + "/type") as f:
                    if f.read().strip() != "pwm-fan":
                        continue
                with open(cd + "/cur_state") as f:
                    fan["level"] = int(f.read())
                with open(cd + "/max_state") as f:
                    fan["max_level"] = int(f.read())
            return fan
        except Exception:
            continue
    return None


def get_disk():
    """(total_gb, free_gb) of the SD card. The container's / is an overlay on
    the host's root filesystem, so this is the card's free space."""
    try:
        st = os.statvfs("/")
        return (round(st.f_blocks * st.f_frsize / 1e9, 1),
                round(st.f_bavail * st.f_frsize / 1e9, 1))
    except Exception:
        return None, None


ZENOH_PORT = 7447


def get_zenoh():
    """(router_listening, [client IPs]) from /proc/net/tcp - the container
    uses host networking, so these are the robot's own sockets. Clients are
    established connections to port 7447 from another host (students'
    laptops); local ROS nodes connect over 127.0.0.1 and are left out."""
    listening = False
    clients = set()
    try:
        with open("/proc/net/tcp") as f:
            next(f)
            for line in f:
                fields = line.split()
                local, remote, state = fields[1], fields[2], fields[3]
                if int(local.split(":")[1], 16) != ZENOH_PORT:
                    continue
                if state == "0A":  # LISTEN
                    listening = True
                elif state == "01":  # ESTABLISHED
                    ip = socket.inet_ntoa(struct.pack("<I", int(remote.split(":")[0], 16)))
                    if not ip.startswith("127."):
                        clients.add(ip)
    except Exception:
        pass
    return listening, sorted(clients)


# Open web pages: every Fenix page keeps one rosbridge websocket (common.js),
# so established connections to 9090 from another host = open pages; the
# devices are the different IPs on any of the web ports (the status page
# itself, rosbridge, the web terminal).
WEB_PORTS = {"pages": 9090, "http": 8080, "terminal": 7681}


def get_web_clients():
    conns = {k: 0 for k in WEB_PORTS}
    ips = set()
    port_key = {p: k for k, p in WEB_PORTS.items()}
    for path in ("/proc/net/tcp", "/proc/net/tcp6"):
        try:
            with open(path) as f:
                next(f)
                for line in f:
                    fields = line.split()
                    local, remote, state = fields[1], fields[2], fields[3]
                    k = port_key.get(int(local.rsplit(":", 1)[1], 16))
                    if not k or state != "01":   # ESTABLISHED
                        continue
                    addr = remote.rsplit(":", 1)[0]
                    if len(addr) == 8:
                        ip = socket.inet_ntoa(struct.pack("<I", int(addr, 16)))
                    elif addr.startswith("0000000000000000FFFF0000"):   # IPv4-mapped IPv6
                        ip = socket.inet_ntoa(struct.pack("<I", int(addr[24:], 16)))
                    else:
                        ip = addr
                    if ip.startswith("127.") or ip == "00000000000000000000000001000000":
                        continue
                    conns[k] += 1
                    ips.add(ip)
        except Exception:
            pass
    return {"pages": conns["pages"], "devices": len(ips), "terminal": conns["terminal"]}


# --- Diagnostics -------------------------------------------------------------
STATE_DIR = "/root/turtlebot_state"  # bind-mounted from ~/turtlebot_setup/state
SERVICE_PORTS = {"zenoh": 7447, "rosbridge": 9090, "webpagina": 8080, "terminal": 7681}
# Errors on the SD card (mmc0 / mmcblk0; mmc1 is the wifi chip) or the
# filesystem - an early warning for a card that is wearing out.
SD_ERROR_RE = re.compile(r"(mmc0|mmcblk0).*(error|timeout|timed out|fail|retry)|EXT4-fs error|I/O error",
                         re.IGNORECASE)


def get_listening_ports():
    ports = set()
    for path in ("/proc/net/tcp", "/proc/net/tcp6"):
        try:
            with open(path) as f:
                next(f)
                for line in f:
                    fields = line.split()
                    if fields[3] == "0A":  # LISTEN
                        ports.add(int(fields[1].rsplit(":", 1)[1], 16))
        except Exception:
            pass
    return ports


def get_services():
    ports = get_listening_ports()
    services = {name: port in ports for name, port in SERVICE_PORTS.items()}
    # Zonder wachtwoord start de webterminal bewust niet (terminal_start.sh): geen fout
    if not os.environ.get("TERMINAL_PASSWORD") and not services.get("terminal"):
        services["terminal"] = None
    try:
        out = subprocess.run(["pgrep", "-f", "launch_control_node.py"], capture_output=True)
        services["launch_control"] = out.returncode == 0
    except Exception:
        services["launch_control"] = None
    try:
        out = subprocess.run(["pgrep", "-f", "nav_web_node.py"], capture_output=True)
        services["nav_web"] = out.returncode == 0
    except Exception:
        services["nav_web"] = None
    # Zonder twist_mux en cmd_vel_guard rijdt de robot niet (turtlebot3_node luistert op /cmd_vel_out)
    try:
        out = subprocess.run(["pgrep", "-f", "twist_mux --ros-args"], capture_output=True)
        services["twist_mux"] = out.returncode == 0
    except Exception:
        services["twist_mux"] = None
    try:
        out = subprocess.run(["pgrep", "-f", "cmd_vel_guard.py"], capture_output=True)
        services["cmd_vel_guard"] = out.returncode == 0
    except Exception:
        services["cmd_vel_guard"] = None
    return services


# /rosout ERROR/FATAL for the robot alerts on every page (t35); known
# harmless errors left out: no calibration file yet at every camera start;
# the zone filters (KeepoutFilter...) of Nav2 at 5 Hz as long as the initial
# pose is not set yet (no "map" frame from AMCL, e.g. right after Openen);
# SmacPlanner2D at every Navigation start: it passes a fixed cost 0.0 to its
# collision checker ("for 2D cost at inscribed isn't relevant") and then
# complains about the inflation layer - only matters for non-circular
# footprints, the TurtleBot uses robot_radius
ROSOUT_IGNORE = re.compile(r"Unable to open camera calibration file"
                           r'|to mask frame \(map\) with error: "map" passed to lookupTransform argument target_frame does not exist'
                           r"|^Inflation layer either not found or inflation is not set sufficiently for optimized non-circular")
# rcl_interfaces/Log levels (the Python constants Log.ERROR/FATAL are bytes)
LOG_ERROR, LOG_FATAL = 40, 50


def get_supervisor():
    """Restarts by services_supervisor.sh since the container start:
    {name: {"restarts": n, "last": epoch}}."""
    try:
        with open("/tmp/supervisor.json") as f:
            return json.load(f)
    except Exception:
        return None


def get_container():
    """Container start (epoch) from PID 1's start time, plus the start log
    written by services_start.sh."""
    info = {"started": None, "starts_24h": None}
    try:
        with open("/proc/stat") as f:
            btime = next(int(l.split()[1]) for l in f if l.startswith("btime"))
        with open("/proc/1/stat") as f:
            start_ticks = int(f.read().rsplit(")", 1)[1].split()[19])
        info["started"] = btime + start_ticks / os.sysconf("SC_CLK_TCK")
    except Exception:
        pass
    try:
        with open(os.path.join(STATE_DIR, "container_starts")) as f:
            starts = [int(l) for l in f if l.strip().isdigit()]
        info["starts_24h"] = sum(1 for t in starts if t > time.time() - 86400)
    except Exception:
        pass
    return info


def get_host_info():
    try:
        with open(os.path.join(STATE_DIR, "host_info.json")) as f:
            return json.load(f)
    except Exception:
        return None


def _busctl_prop(service, path, iface, prop):
    out = subprocess.run(["busctl", "--json=short", "get-property", service, path, iface, prop],
                         capture_output=True, text=True, timeout=5)
    return json.loads(out.stdout)["data"] if out.returncode == 0 else None


def get_ntp():
    """NTP status from the host's timedated/timesyncd over the mounted
    system D-Bus."""
    try:
        synced = _busctl_prop("org.freedesktop.timedate1", "/org/freedesktop/timedate1",
                              "org.freedesktop.timedate1", "NTPSynchronized")
        server = _busctl_prop("org.freedesktop.timesync1", "/org/freedesktop/timesync1",
                              "org.freedesktop.timesync1.Manager", "ServerName")
        return {"synced": synced, "server": server or None}
    except Exception:
        return {"synced": None, "server": None}


def get_gateway(iface):
    try:
        with open("/proc/net/route") as f:
            next(f)
            for line in f:
                fields = line.split()
                if fields[0] == iface and fields[1] == "00000000":
                    return socket.inet_ntoa(struct.pack("<I", int(fields[2], 16)))
    except Exception:
        pass
    return None


def ping_gateway(iface):
    """5 pings to the access point: loss %, avg rtt, jitter (mdev) in ms.
    High jitter was the symptom of last year's shared-wifi problem."""
    gw = get_gateway(iface)
    if not gw:
        return None
    try:
        out = subprocess.run(["ping", "-c", "5", "-i", "0.2", "-W", "1", "-q", gw],
                             capture_output=True, text=True, timeout=10).stdout
        loss = re.search(r"([\d.]+)% packet loss", out)
        rtt = re.search(r"= ([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+) ms", out)
        return {
            "gateway": gw,
            "loss_percent": float(loss.group(1)) if loss else None,
            "avg_ms": float(rtt.group(2)) if rtt else None,
            "jitter_ms": float(rtt.group(4)) if rtt else None,
        }
    except Exception:
        return {"gateway": gw, "loss_percent": None, "avg_ms": None, "jitter_ms": None}


def get_sd_errors():
    try:
        out = subprocess.run(["dmesg"], capture_output=True, text=True, timeout=5).stdout
        errors = [l for l in out.splitlines() if SD_ERROR_RE.search(l)]
        return {"count": len(errors), "last": errors[-1][:200] if errors else None}
    except Exception:
        return {"count": None, "last": None}


# --- Update check ------------------------------------------------------------
# Is there something new for the "Software bijwerken" button? Compared with
# what runs on this robot (host_info.json):
#  - setup repo: latest commit of the branch on GitHub (API, plain sha)
#  - Docker image: digest of the tag on Docker Hub (registry HEAD request,
#    doesn't count against the pull rate limit). Deliberately not the latest
#    turtlebot_docker commit: a commit that wasn't built yet can't be pulled.
UPDATE_CHECK_INTERVAL = 1800  # s
MANIFEST_ACCEPT = ", ".join([
    "application/vnd.oci.image.index.v1+json",
    "application/vnd.docker.distribution.manifest.list.v2+json",
    "application/vnd.oci.image.manifest.v1+json",
    "application/vnd.docker.distribution.manifest.v2+json",
])


def _http(url, headers=None, method="GET"):
    req = urllib.request.Request(url, headers=headers or {}, method=method)
    return urllib.request.urlopen(req, timeout=10)


def latest_setup_commit(repo_url, branch):
    # https://github.com/<owner>/<repo> -> API
    path = repo_url.split("github.com/", 1)[1]
    with _http("https://api.github.com/repos/" + path + "/commits/" + branch,
               {"Accept": "application/vnd.github.sha", "User-Agent": "turtlebot-status"}) as r:
        return r.read().decode().strip()


def latest_image_digest(image_ref):
    # nobel86/turtlebot-rpi5:raspios-zenoh (Docker Hub)
    repo, tag = image_ref.rsplit(":", 1)
    with _http("https://auth.docker.io/token?service=registry.docker.io&scope=repository:"
               + repo + ":pull") as r:
        token = json.load(r)["token"]
    with _http("https://registry-1.docker.io/v2/" + repo + "/manifests/" + tag,
               {"Authorization": "Bearer " + token, "Accept": MANIFEST_ACCEPT}, method="HEAD") as r:
        return r.headers.get("Docker-Content-Digest")


def check_latest(host):
    """{"setup": sha, "image": digest, "checked": epoch, "error": str|None}"""
    latest = {"setup": None, "image": None, "checked": time.time(), "error": None}
    errors = []
    try:
        if host.get("setup_repo_url") and host.get("setup_branch"):
            latest["setup"] = latest_setup_commit(host["setup_repo_url"], host["setup_branch"])
    except Exception as exc:
        errors.append("GitHub: " + str(exc)[:80])
    try:
        if host.get("image_ref"):
            latest["image"] = latest_image_digest(host["image_ref"])
    except Exception as exc:
        errors.append("Docker Hub: " + str(exc)[:80])
    latest["error"] = "; ".join(errors) or None
    return latest


def update_status(host, latest):
    """Compares the cached latest versions with what this robot runs now
    (recomputed on every publish, so it clears right after an update)."""
    if not latest:
        return None
    host = host or {}
    def behind(current, newest):
        return None if not current or not newest else current != newest
    return {
        "setup_behind": behind(host.get("setup_commit_full"), latest.get("setup")),
        "image_behind": behind(host.get("image_digest"), latest.get("image")),
        "checked": latest.get("checked"),
        "error": latest.get("error"),
    }


def get_internet():
    """Internet via the AP's WAN uplink: TCP connect to a public IP (needs no
    DNS) plus a DNS lookup, so "no uplink" and "uplink but no DNS" differ.
    {"online": bool, "ip": bool, "dns": bool, "latency_ms": int|None, "checked": epoch}"""
    res = {"online": False, "ip": False, "dns": False, "latency_ms": None, "checked": time.time()}
    for host in ("1.1.1.1", "8.8.8.8"):
        t0 = time.monotonic()
        try:
            with socket.create_connection((host, 443), timeout=3):
                res["ip"] = True
                res["latency_ms"] = round((time.monotonic() - t0) * 1000)
                break
        except OSError:
            pass
    try:
        socket.getaddrinfo("github.com", 443, type=socket.SOCK_STREAM)
        res["dns"] = True
    except OSError:
        pass
    res["online"] = res["ip"] and res["dns"]
    return res


# Lidar running hours. The LDS-02 has no motor on/off command: it spins as
# long as it has USB power, also without bringup (checked 2026-10-09: data
# on /dev/ttyUSB0 with no driver running). So count the time the lidar's
# USB serial device is present (in /sys: /dev in the container is a copy
# from container start and keeps the node after a USB power-off, see
# usb_power.sh). Stored in the host-mounted state dir (one
# small write per minute), survives updates.
LIDAR_HOURS_FILE = os.path.join(STATE_DIR, "lidar_hours.json")
LIDAR_SYS = "/sys/class/tty/ttyUSB0"


class LidarHours:
    def __init__(self):
        self.data = {"seconds": 0.0, "since": time.strftime("%Y-%m-%d")}
        try:
            with open(LIDAR_HOURS_FILE) as f:
                self.data.update(json.load(f))
        except (OSError, ValueError):
            pass
        self.last = time.monotonic()
        self.last_save = 0.0

    def tick(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        if os.path.exists(LIDAR_SYS) and dt < 120:
            self.data["seconds"] += dt
        if now - self.last_save > 60:
            self.last_save = now
            try:
                with open(LIDAR_HOURS_FILE + ".tmp", "w") as f:
                    json.dump(self.data, f)
                os.replace(LIDAR_HOURS_FILE + ".tmp", LIDAR_HOURS_FILE)
            except OSError:
                pass
        return {"hours": round(self.data["seconds"] / 3600, 2), "since": self.data["since"],
                "spinning": os.path.exists(LIDAR_SYS)}


def usb_power_state():
    """"on"/"off" as last set by usb_power.sh, None if never switched."""
    try:
        with open(os.path.join(STATE_DIR, "usb_power")) as f:
            return f.read().strip() or None
    except OSError:
        return None


def get_devices():
    return {
        "opencr": SIM or os.path.exists("/sys/class/tty/ttyACM0"),
        "lidar_usb": SIM or os.path.exists(LIDAR_SYS),
        "usb_power": usb_power_state(),
    }


# Fleet monitor (turtlebot_setup/monitor/monitor.py on the teacher's laptop):
# every robot pushes its status there, because the laptop on the switch can't
# reach the robots behind their AP's NAT. Address from MONITOR_URL
# (turtlebot_setup/monitor.conf -> .env -> docker-compose); empty = off.
MONITOR_URL = os.environ.get("MONITOR_URL", "").strip()
MONITOR_PERIOD = 5.0


class MonitorPush:
    """Posts the robot's status to MONITOR_URL in a background thread; never
    blocks the node. After a failure (laptop not there) it waits 30 s."""

    def __init__(self, url):
        self.url = url
        self.next = 0.0
        self.busy = False
        self.ok = None          # last result, for the status page

    def maybe_push(self, payload):
        now = time.monotonic()
        if not self.url or self.busy or now < self.next:
            return
        self.busy = True
        threading.Thread(target=self._post, args=(payload,), daemon=True).start()

    def _post(self, payload):
        try:
            req = urllib.request.Request(self.url, data=json.dumps(payload).encode(),
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=2) as r:
                r.read()
            self.ok = True
            self.next = time.monotonic() + MONITOR_PERIOD
        except Exception:
            self.ok = False
            self.next = time.monotonic() + 30.0
        finally:
            self.busy = False


class SystemInfoNode(Node):
    def __init__(self):
        super().__init__("system_info")
        self.pub = self.create_publisher(String, "/system_info", 10)
        self.hostname = "virtuele-robot" if SIM else socket.gethostname()
        self.prev_cpu_times = read_cpu_times()
        self.cpu_ema = {}   # label -> smoothed percent
        self.cpu_min = {}   # label -> min percent seen
        self.cpu_max = {}   # label -> max percent seen
        self.prev_net_bytes = read_net_bytes(IFACE)
        self.prev_net_time = time.monotonic()
        # (time, rx_bytes, tx_bytes) of the last NET_AVG_S seconds: average
        # traffic, steadier than the rate between two publishes (2 s)
        self.net_samples = collections.deque()
        # Slower diagnostics (ping takes ~1 s, dmesg/D-Bus calls) in a
        # background thread so the 2 s publish timer never blocks.
        self.slow = {}
        self.lidar_hours = LidarHours()
        threading.Thread(target=self._slow_loop, daemon=True).start()
        self.timer = self.create_timer(2.0, self.publish_info)

        # Robot alerts (common.js): last ERROR/FATAL lines from /rosout. Here
        # and not in every page: the pages then don't get every log line.
        self.rosout_errors = collections.deque(maxlen=10)
        self.create_subscription(Log, "/rosout", self._on_rosout, 50)

        # for the fleet monitor: what runs (launch_control) and the battery
        self.monitor = MonitorPush(MONITOR_URL)
        self.launch_status = {}
        self.battery = None
        if MONITOR_URL:
            from sensor_msgs.msg import BatteryState
            self.create_subscription(String, "/launch_status", self._on_launch_status, 10)
            self.create_subscription(BatteryState, "/battery_state", self._on_battery, 10)

    def _on_launch_status(self, msg):
        try:
            self.launch_status = json.loads(msg.data)
            self.launch_status["_t"] = time.time()
        except ValueError:
            pass

    def _on_rosout(self, msg):
        if msg.level < LOG_ERROR or ROSOUT_IGNORE.search(msg.msg):
            return
        self.rosout_errors.append({"t": round(msg.stamp.sec + msg.stamp.nanosec / 1e9, 3), "node": msg.name,
                                   "level": "FATAL" if msg.level >= LOG_FATAL else "ERROR", "msg": msg.msg[:300]})

    def _on_battery(self, msg):
        self.battery = {"v": round(msg.voltage, 2), "t": time.time()}

    def _monitor_payload(self, data):
        keep = ("bringup", "slam", "navigation", "camera", "joystick", "robot_node",
                "usb_power", "joystick_device", "camera_sensor", "map_running", "changes", "maps", "update")
        ls = self.launch_status if time.time() - self.launch_status.get("_t", 0) < 10 else {}
        bat = self.battery if self.battery and time.time() - self.battery["t"] < 10 else None
        status = {k: v for k, v in data.items() if k not in ("host", "config")}
        host = data.get("host") or {}
        return {"robot": self.hostname, "nr": os.environ.get("TURTLEBOT_NR"),
                "sent": time.time(), "status": status,
                "launch": {k: ls.get(k) for k in keep if k in ls},
                "battery_v": bat["v"] if bat else None,
                "golden": host.get("golden") if isinstance(host, dict) else None,
                "setup_commit": host.get("setup_commit") if isinstance(host, dict) else None}

    def _slow_loop(self):
        n = 0
        last_check = 0
        while True:
            self.slow["ping"] = ping_gateway(IFACE)
            self.slow["internet"] = get_internet()
            self.slow["lidar"] = self.lidar_hours.tick()
            if n % 2 == 0:  # every ~30 s
                self.slow["ntp"] = get_ntp()
                self.slow["sd_errors"] = get_sd_errors()
                self.slow["host"] = get_host_info()
            # back online after an offline check: check for updates right away
            latest = self.slow.get("latest")
            if latest and latest["error"] and self.slow["internet"]["online"]:
                last_check = 0
            if self.slow.get("host") and time.time() - last_check > UPDATE_CHECK_INTERVAL:
                self.slow["latest"] = check_latest(self.slow["host"])
                # on failure (no internet) retry sooner than 30 min
                last_check = time.time() - (UPDATE_CHECK_INTERVAL - 300 if self.slow["latest"]["error"] else 0)
            n += 1
            time.sleep(15)

    def net_kbps_avg(self):
        """Average (rx_kbps, tx_kbps) over the last NET_AVG_S seconds, or (None, None)."""
        now = time.monotonic()
        rx, tx = read_net_bytes(IFACE)
        if rx is None:
            return None, None
        self.net_samples.append((now, rx, tx))
        while len(self.net_samples) > 2 and now - self.net_samples[1][0] >= NET_AVG_S:
            self.net_samples.popleft()
        t0, rx0, tx0 = self.net_samples[0]
        dt = now - t0
        if dt <= 0 or rx < rx0 or tx < tx0:     # first sample, or counters reset
            return None, None
        return round((rx - rx0) * 8 / 1000 / dt, 1), round((tx - tx0) * 8 / 1000 / dt, 1)

    def net_kbps(self):
        """Returns (rx_kbps, tx_kbps) since the last call, or (None, None)."""
        now_bytes = read_net_bytes(IFACE)
        now_time = time.monotonic()
        rx_prev, tx_prev = self.prev_net_bytes
        rx_now, tx_now = now_bytes
        dt = now_time - self.prev_net_time
        self.prev_net_bytes = now_bytes
        self.prev_net_time = now_time
        if rx_prev is None or rx_now is None or dt <= 0:
            return None, None
        rx_kbps = (rx_now - rx_prev) * 8 / 1000 / dt
        tx_kbps = (tx_now - tx_prev) * 8 / 1000 / dt
        return round(rx_kbps, 1), round(tx_kbps, 1)

    def per_cpu_percent(self):
        now = read_cpu_times()
        raw = {}
        for label, (total, idle) in now.items():
            prev = self.prev_cpu_times.get(label)
            if prev is None:
                continue
            prev_total, prev_idle = prev
            dt = total - prev_total
            di = idle - prev_idle
            raw[label] = (dt - di) / dt * 100 if dt > 0 else 0.0
        self.prev_cpu_times = now

        cur, mn, mx = [], [], []
        # cpu0, cpu1, ... in order, not dict insertion order (which follows /proc/stat)
        for label, value in sorted(raw.items(), key=lambda kv: int(kv[0][3:])):
            prev_ema = self.cpu_ema.get(label, value)
            ema = CPU_EMA_ALPHA * value + (1 - CPU_EMA_ALPHA) * prev_ema
            self.cpu_ema[label] = ema
            self.cpu_min[label] = min(self.cpu_min.get(label, ema), ema)
            self.cpu_max[label] = max(self.cpu_max.get(label, ema), ema)
            cur.append(round(ema, 1))
            mn.append(round(self.cpu_min[label], 1))
            mx.append(round(self.cpu_max[label], 1))
        return cur, mn, mx

    def get_config_env(self):
        return {k: os.environ.get(k) for k in CONFIG_ENV_VARS}

    def publish_info(self):
        load1, load5, load15 = os.getloadavg()
        mem_percent, mem_total_mb, mem_used_mb = get_mem_info()
        cpu_cur, cpu_min, cpu_max = self.per_cpu_percent()
        net_rx_kbps, net_tx_kbps = self.net_kbps()
        net_rx_avg, net_tx_avg = self.net_kbps_avg()
        wifi_dbm, wifi_quality = get_wifi(IFACE)
        wifi_freq, wifi_width, wifi_center = get_wifi_channel_info(IFACE)
        wifi_freq = wifi_freq or get_wifi_freq(IFACE)
        wifi_band_name, wifi_channel = wifi_band(wifi_freq)
        disk_total_gb, disk_free_gb = get_disk()
        zenoh_router, zenoh_clients = get_zenoh()
        data = {
            "hostname": self.hostname,
            "ip": get_ip(IFACE),
            "mac": get_mac(IFACE),
            "cpu_load_1min": round(load1, 2),
            "cpu_load_5min": round(load5, 2),
            "cpu_load_15min": round(load15, 2),
            "cpu_count": os.cpu_count(),
            "cpu_percent_per_core": cpu_cur,
            "cpu_percent_per_core_min": cpu_min,
            "cpu_percent_per_core_max": cpu_max,
            "mem_percent": mem_percent,
            "mem_total_mb": mem_total_mb,
            "mem_used_mb": mem_used_mb,
            "net_rx_kbps": net_rx_kbps,
            "net_tx_kbps": net_tx_kbps,
            "net_rx_kbps_avg": net_rx_avg,     # average over NET_AVG_S
            "net_tx_kbps_avg": net_tx_avg,
            "net_avg_s": NET_AVG_S,
            "wifi_signal_dbm": wifi_dbm,
            "wifi_quality_percent": wifi_quality,
            "wifi_freq_mhz": wifi_freq,
            "wifi_band": wifi_band_name,
            "wifi_channel": wifi_channel,
            "wifi_width_mhz": wifi_width,
            "wifi_center_mhz": wifi_center,
            "uptime_seconds": get_uptime_seconds(),
            "cpu_temp_c": get_cpu_temp(),
            "throttled": get_throttled(),
            "ext5v_v": get_ext5v(),
            "fan": get_fan(),
            "time_unix": time.time(),
            "ntp": self.slow.get("ntp"),
            "ping": self.slow.get("ping"),
            "internet": self.slow.get("internet"),
            "lidar": self.slow.get("lidar"),
            "sd_errors": self.slow.get("sd_errors"),
            "host": self.slow.get("host"),
            "updates": update_status(self.slow.get("host"), self.slow.get("latest")),
            "image_commit": os.environ.get("TURTLEBOT_IMAGE_COMMIT") or None,
            "image_subject": os.environ.get("TURTLEBOT_IMAGE_SUBJECT") or None,
            "services": get_services(),
            "supervisor": get_supervisor(),
            "services_since": fenix_services.since(),   # Status > Diensten (t79)
            "rosout_errors": list(self.rosout_errors),
            "container": get_container(),
            "disk_total_gb": disk_total_gb,
            "disk_free_gb": disk_free_gb,
            "zenoh_router": zenoh_router,
            "zenoh_clients": zenoh_clients,
            "web_clients": get_web_clients(),
            "devices": get_devices(),
            # build arg IMAGE_VERSION (datum + git-commit), zie Dockerfile
            "image_version": os.environ.get("TURTLEBOT_IMAGE_VERSION"),
            "config": self.get_config_env(),
        }
        data["monitor"] = {"url": MONITOR_URL or None, "ok": self.monitor.ok}
        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)
        self.monitor.maybe_push(self._monitor_payload(data))


def main():
    rclpy.init()
    node = SystemInfoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
