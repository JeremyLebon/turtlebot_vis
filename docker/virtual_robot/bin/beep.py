#!/usr/bin/env python3
# Play one of the OpenCR's built-in melodies, e.g. as an audible "all
# services are up" signal at container start or before a shutdown/reboot.
#
#   beep.py on | off | low_battery | error | button1 | button2
#
# button1 / button2 are silent: the OpenCR firmware (turtlebot3_sensor.cpp
# makeMelody) has no melody for them (checked 2026-10-09).
#
# Without bringup the OpenCR's serial port is free, so we write the `sound`
# control-table item (address 50) directly with one Dynamixel protocol 2.0
# WRITE packet - no pyserial/dynamixel_sdk needed. While bringup runs,
# turtlebot3_node owns the port, so we go through its /sound service instead.
import os
import subprocess
import sys
import termios
import time
import tty

SOUNDS = {"off": 0, "on": 1, "low_battery": 2, "error": 3, "button1": 4, "button2": 5}

OPENCR_PORT = "/dev/ttyACM0"
OPENCR_ID = 200          # turtlebot3_node opencr.id
ADDR_SOUND = 50          # control_table.hpp: sound = {50, RAM, 1, READ_WRITE}
INST_WRITE = 0x03


def _crc16(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def _write_packet(value):
    params = [ADDR_SOUND & 0xFF, ADDR_SOUND >> 8, value]
    length = len(params) + 3  # instruction + 2 CRC bytes
    packet = [0xFF, 0xFF, 0xFD, 0x00, OPENCR_ID, length & 0xFF, length >> 8, INST_WRITE] + params
    crc = _crc16(packet)
    return bytes(packet + [crc & 0xFF, crc >> 8])


def beep_serial(value):
    fd = os.open(OPENCR_PORT, os.O_RDWR | os.O_NOCTTY)
    try:
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = termios.B1000000
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        os.write(fd, _write_packet(value))
        termios.tcdrain(fd)
        time.sleep(0.1)  # let the OpenCR answer before the port closes
    finally:
        os.close(fd)


def bringup_running():
    return subprocess.run(["pgrep", "-f", "turtlebot3_ros"], capture_output=True).returncode == 0


def beep_service(value):
    subprocess.run(
        ["ros2", "service", "call", "/sound", "turtlebot3_msgs/srv/Sound", "{value: %d}" % value],
        capture_output=True, timeout=15,
    )


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "on"
    if name not in SOUNDS:
        sys.exit("gebruik: beep.py " + " | ".join(SOUNDS))
    if bringup_running():
        beep_service(SOUNDS[name])
    else:
        if not os.path.exists("/sys/class/tty/ttyACM0"):
            # USB power off (power saving, usb_power.sh): on for the melody;
            # launch_control_node switches it off again when unused
            subprocess.run(["/usr/local/bin/usb_power", "on", "--wait"], timeout=30)
        beep_serial(SOUNDS[name])


if __name__ == "__main__":
    main()
