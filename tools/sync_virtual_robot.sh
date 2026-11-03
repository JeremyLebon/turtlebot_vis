#!/bin/bash
# Neemt de robotsoftware (statuspagina, knoppen, zones) over uit turtlebot_docker
# voor de virtuele robot (`robot htmlsim`, zie docker/virtual_robot_start.sh).
# Zelfde bestanden als op de echte robot; enkel FENIX_SIM=1 maakt het verschil
# (bringup = de HTML-simulator, simulatietijd). Opnieuw draaien na een wijziging
# aan de robot, dan de image opnieuw bouwen.
#
#   tools/sync_virtual_robot.sh [pad naar turtlebot_docker]
set -e
SRC=${1:-$(dirname "$0")/../../turtlebot_docker}
DST=$(dirname "$0")/../docker/virtual_robot
[ -f "$SRC/docker/launch_control_node.py" ] || { echo "turtlebot_docker niet gevonden: $SRC" >&2; exit 1; }
rm -rf "$DST"; mkdir -p "$DST/bin" "$DST/launch" "$DST/turtlebot3"
for f in launch_control_node.py nav_web_node.py system_info_node.py cmd_vel_guard.py status_page_server.py \
         map_bundle.py fenix_services.py fenix_admin.py projects_lib.py camera_params.py beep.py \
         sim_bringup_node.py camera_present.sh; do
    cp "$SRC/docker/$f" "$DST/bin/"
done
cp "$SRC/docker/cartographer_headless.launch.py" "$SRC/docker/sim_bringup.launch.py" "$DST/launch/"
cp "$SRC/docker/twist_mux.yaml" "$SRC/docker/joystick_f710.yaml" "$DST/"
cp -r "$SRC/status_page" "$DST/status_page"
# TurtleBot3-fork van de robot: Nav2 zonder rviz2, Cartographer-instellingen van de robot
for p in turtlebot3_navigation2 turtlebot3_cartographer; do
    cp -r "$SRC/turtlebot3/$p" "$DST/turtlebot3/$p"
done
git -C "$SRC" log -1 --format='turtlebot_docker %h (%cs) %s' > "$DST/BRON"
echo "overgenomen: $(cat "$DST/BRON")"
