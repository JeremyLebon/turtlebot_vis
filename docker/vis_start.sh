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
#
# Welke robot: automatisch, zie robot_select.sh. Met `robot sim` de
# simulatie: eigen Zenoh-router, de bridge op simulatietijd en
# sim_teleop_relay.py voor het Teleop-paneel. `robot htmlsim`: hetzelfde plus
# rosbridge (ws://localhost:9090) voor de HTML-simulator op /sim/.
# Logs: /tmp/vis_services/.

LOG=/tmp/vis_services
mkdir -p "$LOG"

# Een ros2-daemon (poort 11511 + ROS_DOMAIN_ID, gedeeld via network_mode: host)
# onthoudt de omgeving van bij zijn start: dan blijft `ros2 topic list` leeg.
# Stoppen; het volgende ros2-commando start een verse.
(source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash && ros2 daemon stop) >/dev/null 2>&1

# Zoals python3 -m http.server, maar met Cache-Control: no-cache: na een nieuwe
# image moet de browser de nieuwe index.html laden (layout-reset, Lichtblick-update).
python3 - "${LICHTBLICK_PORT:-8080}" > "$LOG/lichtblick.log" 2>&1 <<'PY' &
import functools, http.server, sys
class NoCache(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()
http.server.ThreadingHTTPServer(("0.0.0.0", int(sys.argv[1])),
    functools.partial(NoCache, directory="/opt/lichtblick")).serve_forever()
PY

(
    source /opt/ros/${ROS_DISTRO}/setup.bash
    # turtlebot3_description (apt): de bridge levert de meshes
    # (package://...) van het robotmodel aan Lichtblick.
    source /root/robot_select.sh

    while true; do
        # Robot zoeken. Zonder bereikbare router blijft rmw_zenoh hangen vóór
        # de bridge poort 8765 opent (Lichtblick: "connection failed"), dus
        # eerst wachten, met een duidelijke melding in het log.
        n=0
        until NR=$(robot_detect 2>&1) && [[ $NR =~ ^([0-9]+|sim|htmlsim)$ ]] \
              && { [[ ! $NR =~ ^(sim|htmlsim)$ ]] || sim_router_start; } && robot_reachable "$NR"; do
            if [ $((n % 10)) -eq 0 ]; then
                if [[ $NR =~ ^([0-9]+|sim|htmlsim)$ ]]; then
                    echo "$(date +%T) wacht op robot $NR: $(robot_host "$NR"):7447 niet bereikbaar (robot aan? laptop op TB-AP-$NR?)"
                else
                    echo "$(date +%T) wacht op de robot: ${NR:-geen TurtleBot bereikbaar (laptop op de wifi TB-AP-<nr>? robot aan?)}"
                fi
            fi
            n=$((n + 1)); sleep 3
        done
        robot_write_env "$NR"
        source "$ROBOT_ENV"
        SIM=false RELAY= ROSBRIDGE=
        if [ "$NR" = sim ] || [ "$NR" = htmlsim ]; then
            SIM=true
            echo "$(date +%T) $NR - foxglove_bridge start (simulatietijd)"
            python3 /root/sim_teleop_relay.py >> "$LOG/sim_teleop_relay.log" 2>&1 &
            RELAY=$!
            if [ "$NR" = htmlsim ]; then
                # HTML-simulator (http://localhost:8080/sim/) praat via rosbridge
                echo "$(date +%T) rosbridge start op ws://localhost:9090"
                ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 \
                    >> "$LOG/rosbridge.log" 2>&1 &
                ROSBRIDGE=$!
            fi
        else
            echo "$(date +%T) TurtleBot $NR (10.0.$NR.10) - foxglove_bridge start"
        fi

        # use_sim_time in de simulatie: Gazebo stempelt /tf, /scan, /odom
        # met zijn eigen klok (/clock)
        ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
            port:="${FOXGLOVE_BRIDGE_PORT:-8765}" use_sim_time:=$SIM &
        BRIDGE=$!
        # Robot 15 s onbereikbaar (andere wifi, robot uit)? Bridge stoppen en
        # opnieuw zoeken. `robot` in een shell stopt de bridge ook.
        miss=0
        while kill -0 $BRIDGE 2>/dev/null; do
            sleep 5
            if robot_reachable "$NR"; then miss=0; else miss=$((miss + 1)); fi
            if [ $miss -ge 3 ]; then
                echo "$(date +%T) robot $NR niet meer bereikbaar - opnieuw zoeken"
                kill $BRIDGE; break
            fi
        done
        wait $BRIDGE
        [ -n "$RELAY" ] && kill $RELAY 2>/dev/null
        [ -n "$ROSBRIDGE" ] && kill -INT $ROSBRIDGE 2>/dev/null
        sleep 2
    done
) > "$LOG/foxglove_bridge.log" 2>&1 &

exec "$@"
