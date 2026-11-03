#!/bin/bash
# Prints the CSI camera sensor name and exits 0 if one is attached, else
# exits 1. A sensor shows up as a v4l-subdev named after the chip (imx708,
# imx219, imx477, imx296, imx500, ov5647, ...). The Pi only detects cameras
# at boot, so plugging one in needs a reboot. (`cam -l` would be the libcamera
# way, but it segfaults in this image.)
for f in /sys/class/video4linux/v4l-subdev*/name; do
    [ -r "$f" ] || continue
    if grep -qiE '^(imx|ov|ar|gc|os|vd)[0-9]' "$f"; then
        cat "$f"
        exit 0
    fi
done
exit 1
