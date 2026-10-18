#!/bin/bash
# Entrypoint van turtlebot-vis: start twee achtergronddiensten en daarna het
# gewone commando (CMD, standaard `bash`).
#
#   Lichtblick (web-versie van Foxglove Studio)  http://localhost:8080
#   foxglove_bridge                              ws://localhost:8765
#
# De student surft vanuit Windows naar http://localhost:8080 (WSL2 stuurt
# localhost door; de container draait met network_mode: host). Lichtblick
# draait in de browser - geen WSLg/OpenGL in Docker nodig, het alternatief
# voor rviz2 als dat venster niet zichtbaar wordt. De bridge haalt de topics
# via Zenoh bij de robot, net als rviz2: geen extra belasting op de robot.
# Logs: /tmp/vis_services/.

LOG=/tmp/vis_services
mkdir -p "$LOG"

python3 -m http.server "${LICHTBLICK_PORT:-8080}" --bind 0.0.0.0 \
    --directory /opt/lichtblick > "$LOG/lichtblick.log" 2>&1 &

# In een lus: valt de bridge weg (robot uit, Zenoh-router herstart), dan
# komt ze vanzelf terug.
(
    source /opt/ros/${ROS_DISTRO}/setup.bash
    # turtlebot3_description: de bridge levert de meshes (package://...)
    # van het robotmodel aan Lichtblick.
    source /root/turtlebot3_ws/install/setup.bash
    while true; do
        ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
            port:="${FOXGLOVE_BRIDGE_PORT:-8765}"
        sleep 3
    done
) > "$LOG/foxglove_bridge.log" 2>&1 &

exec "$@"
