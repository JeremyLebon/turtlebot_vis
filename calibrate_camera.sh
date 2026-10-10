#!/bin/bash
# Intrinsieke camerakalibratie van een TurtleBot (Raspberry Pi Camera Module 3)
# met een geprint dambord - draait in de turtlebot-vis-container op de laptop
# (verbonden met de robot zoals altijd: .env ROBOT_ZENOH_IP / ROS_DOMAIN_ID).
#
#   ./calibrate_camera.sh [binnenhoeken] [vakje_in_m]
#   ./calibrate_camera.sh 8x6 0.025        # standaard: 9x7 vakjes = 8x6 binnenhoeken, 25 mm
#
# Werkwijze: houd het dambord voor de camera en beweeg het (links/rechts,
# boven/onder, dichtbij/ver, schuin) tot X, Y, Size en Skew groen zijn ->
# CALIBRATE (even wachten) -> COMMIT. De robot bewaart de kalibratie zelf
# (state/camera_info/camera_<resolutie>.yaml) en gebruikt ze meteen; de
# Systeem-pagina toont "Intrinsiek gekalibreerd".
#
# Het ruwe beeld (~110 Mbit/s) gaat niet over de wifi: het gecomprimeerde
# beeld wordt hier op de laptop terug uitgepakt (image_transport republish).
# Een kalibratie geldt voor 1 resolutie: na het wijzigen van de resolutie op
# de Systeem-pagina opnieuw kalibreren.
set -e
SIZE=${1:-8x6}
SQUARE=${2:-0.025}
C=turtlebot-vis

if ! docker ps --format '{{.Names}}' | grep -qx "$C"; then
  echo "De container $C draait niet - eerst: docker compose up -d" >&2
  exit 1
fi
echo "Dambord: $SIZE binnenhoeken, vakjes van $SQUARE m"
docker exec "$C" bash -c '
  source /opt/ros/humble/setup.bash
  ros2 run image_transport republish compressed raw --ros-args \
      -r in/compressed:=/camera/image_raw/compressed -r out:=/calib/image_raw &
  REP=$!
  trap "kill $REP 2>/dev/null" EXIT
  sleep 2
  ros2 run camera_calibration cameracalibrator --size '"$SIZE"' --square '"$SQUARE"' \
      --ros-args -r image:=/calib/image_raw -p camera:=/camera
'
