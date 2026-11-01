#!/usr/bin/env python3
"""`diagnose`: wie gebruikt het geheugen en de schijf in WSL en de container?

    diagnose          live overzicht, elke 2 s (Ctrl+C stopt)
    diagnose --log    één regel per 30 s naar ~/ros2_ws/.diagnose/geheugen.log
                      (start vanzelf met de container, zie vis_start.sh)

Zoals `docker stats`, maar per onderdeel: VS Code, Pylance, de bridges, de
simulatie en je eigen nodes. De container ziet ook het geheugen van heel WSL
(/proc/meminfo). Loopt WSL vast, dan staat in het log wat er vlak daarvoor
groeide: na `wsl --shutdown` en opnieuw opstarten terug te lezen, ook vanuit
Windows (\\\\wsl$\\Ubuntu\\...\\turtlebot_vis\\ros2_ws\\.diagnose).

Geheugen per proces = PSS (gedeelde bibliotheken eerlijk verdeeld, niet dubbel
geteld zoals RSS bij de vele node-processen van VS Code).
"""
import os
import re
import sys
import time

LOGDIR = "/root/ros2_ws/.diagnose"
LOGFILE = LOGDIR + "/geheugen.log"
LOG_MAX = 1_000_000          # bytes, daarna geheugen.log.1 (1 MB ~ 4 dagen)

# (naam, patroon op de commandoregel); de eerste die past, telt
GROUPS = [
    ("Pylance", r"vscode-pylance|server\.bundle\.js"),
    ("VS Code", r"\.vscode-server|vscode-remote-containers"),
    ("Bridges", r"rosbridge|foxglove_bridge|rosapi"),
    ("Simulatie", r"zenohd|gz |gzserver|gazebo|ruby .*gz|sim_teleop_relay|robot_state_publisher|nav2|slam_toolbox"),
    ("Lichtblick-server", r"^python3 - \d+"),
    ("Eigen nodes", r"ros2_ws|ros2 run|ros2 launch|^python3 \S+\.py"),
]


def meminfo():
    m = {}
    for line in open("/proc/meminfo"):
        k, v = line.split(":", 1)
        m[k] = int(v.split()[0]) / 1024        # MB
    return m


def cgroup():
    try:
        st = dict(l.split() for l in open("/sys/fs/cgroup/memory.stat"))
        return int(st["anon"]) / 1048576, int(st["file"]) / 1048576
    except (OSError, KeyError, ValueError):
        return None, None


def oom_kills():
    try:
        return int(next(l.split()[1] for l in open("/proc/vmstat") if l.startswith("oom_kill ")))
    except (OSError, StopIteration, ValueError):
        return 0


def processes():
    """[(pid, naam, groep, pss_MB, geschreven_bytes)]"""
    out = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="replace").strip()
            if not cmd:
                continue
            pss = 0
            for line in open(f"/proc/{pid}/smaps_rollup"):
                if line.startswith("Pss:"):
                    pss = int(line.split()[1]) / 1024
                    break
            wb = 0
            for line in open(f"/proc/{pid}/io"):
                if line.startswith("write_bytes:"):
                    wb = int(line.split()[1])
        except (OSError, ValueError):
            continue
        group = next((g for g, pat in GROUPS if re.search(pat, cmd)), "Overig")
        name = os.path.basename(cmd.split()[0])
        if name in ("python3", "node", "bash", "sh") and len(cmd.split()) > 1:
            arg = next((a for a in cmd.split()[1:] if not a.startswith("-")), "")
            name += " " + os.path.basename(arg)[:28]
        out.append((int(pid), name, group, pss, wb))
    return out


def snapshot(prev_io, dt):
    """Sommen per groep; schrijven in MB/s t.o.v. de vorige meting (per pid)."""
    procs = processes()
    groups = {g: [0.0, 0.0] for g, _ in GROUPS}
    groups["Overig"] = [0.0, 0.0]
    io = {}
    for pid, name, g, pss, wb in procs:
        io[pid] = wb
        rate = max(0, wb - prev_io.get(pid, wb)) / 1048576 / dt if dt else 0
        groups[g][0] += pss
        groups[g][1] += rate
    return procs, groups, io


def live():
    prev_io, t0 = {}, None
    try:
        while True:
            t = time.time()
            procs, groups, prev_io = snapshot(prev_io, (t - t0) if t0 else 0)
            t0 = t
            m, (anon, cache) = meminfo(), cgroup()
            used = m["MemTotal"] - m["MemAvailable"]
            swap = m["SwapTotal"] - m["SwapFree"]
            out = ["\033[H\033[J" + "Diagnose  %s   (Ctrl+C stopt)" % time.strftime("%H:%M:%S"), ""]
            warn = used > 0.85 * m["MemTotal"] or swap > 200
            cache = m["MemTotal"] - m["MemFree"] - used    # bestandscache: vrij te maken, Windows ziet het wel
            out.append("  WSL          %.1f GB programma's + %.1f GB cache = %.1f GB van %.1f GB (vmmemWSL in Taakbeheer)%s" % (
                used / 1024, cache / 1024, (used + cache) / 1024, m["MemTotal"] / 1024,
                "   \033[33m<- bijna vol\033[0m" if warn else ""))
            out.append("               swap %.0f MB. Cache teruggeven aan Windows: sudo sh -c 'sync; echo 3 > /proc/sys/vm/drop_caches' (in WSL)" % swap)
            if anon is not None:
                out.append("  Container    %4.0f MB programma's, %.0f MB bestandscache" % (anon, cache))
            k = oom_kills()
            if k:
                out.append("  \033[33m%d keer een programma afgesloten wegens geheugentekort (sudo dmesg | grep -i killed)\033[0m" % k)
            out += ["", "  %-18s %9s %12s" % ("onderdeel", "geheugen", "schrijven")]
            for g, (pss, rate) in sorted(groups.items(), key=lambda x: -x[1][0]):
                if pss >= 1:
                    out.append("  %-18s %6.0f MB %8.1f MB/s" % (g, pss, rate))
            out += ["", "  grootste processen:"]
            for pid, name, g, pss, wb in sorted(procs, key=lambda p: -p[3])[:6]:
                out.append("  %7d  %-34s %6.0f MB  (%s)" % (pid, name[:34], pss, g))
            print("\n".join(out), flush=True)
            time.sleep(2)
    except KeyboardInterrupt:
        print()


def log(interval=int(os.environ.get("DIAG_INTERVAL", 30))):
    os.makedirs(LOGDIR, exist_ok=True)
    head = "tijd_UTC wsl_gebruikt_MB wsl_totaal_MB swap_MB container_MB " + \
        " ".join(g.replace(" ", "_").replace("-", "_") + "_MB" for g, _ in GROUPS) + \
        " overig_MB schrijven_MB/s oom_kills grootste"
    prev_io, t0 = {}, None
    while True:
        t = time.time()
        procs, groups, prev_io = snapshot(prev_io, (t - t0) if t0 else 0)
        t0 = t
        m, (anon, _) = meminfo(), cgroup()
        top = max(procs, key=lambda p: p[3], default=None)
        line = "%s %d %d %d %d %s %d %.1f %d %s" % (
            time.strftime("%Y-%m-%d_%H:%M:%S"), m["MemTotal"] - m["MemAvailable"], m["MemTotal"],
            m["SwapTotal"] - m["SwapFree"], anon or 0,
            " ".join("%d" % groups[g][0] for g, _ in GROUPS), groups["Overig"][0],
            sum(r for _, r in groups.values()), oom_kills(),
            "%s=%dMB" % (top[1].replace(" ", "_"), top[3]) if top else "-")
        try:
            if os.path.exists(LOGFILE) and os.path.getsize(LOGFILE) > LOG_MAX:
                os.replace(LOGFILE, LOGFILE + ".1")
            new = not os.path.exists(LOGFILE)
            with open(LOGFILE, "a") as f:
                if new:
                    f.write(head + "\n")
                f.write(line + "\n")
        except OSError:
            pass
        time.sleep(interval)


if __name__ == "__main__":
    if "--log" in sys.argv:
        log()
    elif "-h" in sys.argv or "--help" in sys.argv:
        print(__doc__)
    else:
        live()
