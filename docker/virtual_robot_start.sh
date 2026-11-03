#!/bin/bash
# Virtuele robot (`robot virtueel`): de software van de echte TurtleBot - statuspagina
# (http://localhost:8090), knoppen Bringup/SLAM/Navigatie, kaarten, zones, twist_mux en
# cmd_vel_guard - op de laptop, met de HTML-simulator (http://localhost:8080/sim/) als
# hardware. Back-up voor als de robots niet werken. De bestanden komen uit turtlebot_docker
# (tools/sync_virtual_robot.sh); FENIX_SIM=1 zet bringup op de simulator en SLAM/Nav2 op
# simulatietijd.
#
#   virtual_robot_start.sh start | stop
# Gestart en gestopt door vis_start.sh bij het wisselen van modus. Logs: /tmp/vis_services/vr_*.log
# Kaarten en instellingen: ~/ros2_ws/virtuele_robot (blijft bewaard).

LOG=/tmp/vis_services
PROCS="twist_mux --ros-args|cmd_vel_guard\.py|status_page_server\.py|system_info_node\.py|launch_control_node\.py|nav_web_node\.py"
# wat de knoppen starten (SLAM, Navigatie, bringup) gaat mee uit bij het stoppen
LAUNCHED="sim_bringup|cartographer_headless\.launch\.py|lib/cartographer_ros/|map_saver_server|lifecycle_manager_slam|navigation2\.launch\.py|/opt/ros/jazzy/lib/nav2_"

stop() {
    pkill -TERM -f "$PROCS|$LAUNCHED" 2>/dev/null || return 0
    for _ in 1 2 3 4 5 6; do pgrep -f "$PROCS|$LAUNCHED" > /dev/null || return 0; sleep 0.5; done
    pkill -KILL -f "$PROCS|$LAUNCHED" 2>/dev/null; true
}

start() {
    mkdir -p "$LOG" /root/ros2_ws/virtuele_robot/maps
    export FENIX_SIM=1 TURTLEBOT3_MODEL=burger
    source /root/turtlebot3_ws/install/setup.bash
    pgrep -f "[t]wist_mux --ros-args" > /dev/null || nohup ros2 run twist_mux twist_mux --ros-args \
        --params-file /root/turtlebot3_ws/twist_mux.yaml -r cmd_vel_out:=cmd_vel_mux > "$LOG/vr_twist_mux.log" 2>&1 &
    pgrep -f "[c]md_vel_guard.py" > /dev/null || nohup python3 /usr/local/bin/cmd_vel_guard.py --ros-args \
        --params-file /root/turtlebot3_ws/twist_mux.yaml > "$LOG/vr_cmd_vel_guard.log" 2>&1 &
    pgrep -f "[s]tatus_page_server.py" > /dev/null || nohup python3 /usr/local/bin/status_page_server.py \
        "${VR_PORT:-8090}" /root/turtlebot3_ws/status_page > "$LOG/vr_status_page.log" 2>&1 &
    pgrep -f "[s]ystem_info_node.py" > /dev/null || nohup python3 /usr/local/bin/system_info_node.py > "$LOG/vr_system_info.log" 2>&1 &
    pgrep -f "[l]aunch_control_node.py" > /dev/null || nohup python3 /usr/local/bin/launch_control_node.py > "$LOG/vr_launch_control.log" 2>&1 &
    # simulatietijd: de poses (beginpositie, doel) krijgen dan de klok van de simulator
    pgrep -f "[n]av_web_node.py" > /dev/null || nohup python3 /usr/local/bin/nav_web_node.py --ros-args \
        -p use_sim_time:=true > "$LOG/vr_nav_web.log" 2>&1 &
    true
}

case $1 in
    start) start ;;
    stop) stop ;;
    *) echo "gebruik: $0 start|stop" >&2; exit 2 ;;
esac
