# Robotkeuze voor turtlebot-vis: met welke TurtleBot praat deze container
# (ROS_DOMAIN_ID = robotnummer, Zenoh-router op 10.0.<nr>.10:7447).
#
# Gesourcet door vis_start.sh (foxglove_bridge) en door elke interactieve
# shell (.bashrc). Niemand hoeft .env aan te passen: de access points zijn
# van elkaar afgeschermd, dus vanaf de wifi TB-AP-<nr> is alleen robot <nr>
# bereikbaar - die wordt gekozen.
#
# Vast nummer i.p.v. zoeken: `robot <nr>` in de container, of ROBOT_NR=<nr>
# in .env (nodig als er meer dan één robot bereikbaar is).
#
# Simulatie (Gazebo, thuis of zonder robot): `robot sim`, of ROBOT_NR=sim in
# .env. De container start dan zijn eigen Zenoh-router, enkel op localhost
# (andere laptops op dezelfde wifi zien je simulatie niet).

ROBOT_ENV=/tmp/robot.env      # gekozen robot (TB_NR, ROS_DOMAIN_ID, Zenoh);
                              # elke shell leest dit in. ROBOT_NR (.env) is
                              # enkel de vaste keuze, niet de gekozen robot.
ROBOT_FIXED=/tmp/robot_nr     # vast nummer, gezet met `robot <nr>`
ROBOT_MAX=12                  # zoek 10.0.1.10 .. 10.0.12.10

robot_host() {
    case $1 in sim|htmlsim) echo 127.0.0.1 ;; *) echo "10.0.$1.10" ;; esac
}

robot_reachable() {
    timeout 1 bash -c "exec 3<>/dev/tcp/$(robot_host "$1")/7447" 2>/dev/null
}

# Zenoh-router voor de simulatie (rmw_zenohd), enkel op localhost. Gestart
# door `robot sim` en door vis_start.sh (ROBOT_NR=sim, herstart container).
sim_router_start() {
    pgrep -f "[r]mw_zenohd" > /dev/null && return 0
    mkdir -p /tmp/vis_services
    ( export ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false'
      source /opt/ros/${ROS_DISTRO}/setup.bash
      nohup ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/vis_services/zenohd.log 2>&1 & )
    local i
    for i in $(seq 1 40); do robot_reachable sim && return 0; sleep 0.25; done
    echo "Zenoh-router voor de simulatie start niet, zie /tmp/vis_services/zenohd.log" >&2
    return 1
}

# Nummers van alle robots waarvan de Zenoh-router antwoordt (parallel, ~1 s).
robot_probe() {
    local n
    for n in $(seq 1 "$ROBOT_MAX"); do
        ( robot_reachable "$n" && echo "$n" ) &
    done
    wait
}

# Eén robotnummer (exitcode 0), of niets (exitcode 1: geen of meerdere
# robots bereikbaar).
robot_detect() {
    if [ -s "$ROBOT_FIXED" ]; then cat "$ROBOT_FIXED"; return 0; fi
    if [ -n "$ROBOT_NR" ]; then echo "$ROBOT_NR"; return 0; fi
    local found count
    found=$(robot_probe | sort -n | xargs)
    count=$(wc -w <<< "$found")
    if [ "$count" -eq 1 ]; then echo "$found"; return 0; fi
    [ "$count" -gt 1 ] && echo "Meerdere robots bereikbaar ($found): kies met robot <nr>" >&2
    return 1
}

# Schrijf ROBOT_ENV, enkel als de keuze verandert (shells reageren op de
# wijziging, zie _robot_refresh).
robot_write_env() {
    local new domain=$1
    case $1 in sim|htmlsim) domain=0 ;; esac
    new="export TB_NR=$1
export ROS_DOMAIN_ID=$domain
export ZENOH_CONFIG_OVERRIDE='mode=\"client\";connect/endpoints=[\"tcp/$(robot_host "$1"):7447\"]'"
    [ "$(cat "$ROBOT_ENV" 2>/dev/null)" = "$new" ] || echo "$new" > "$ROBOT_ENV"
    # zelfde keuze voor VS Code (Run/F5 leest geen .bashrc): env-bestand zonder 'export'
    [ -f /root/.ros_base.env ] && { cat /root/.ros_base.env; echo "$new" | sed 's/^export //'; } > /root/.ros.env 2>/dev/null
}

# --- enkel voor interactieve shells ---------------------------------------

# `robot`       opnieuw zoeken
# `robot <nr>`  vast robotnummer
# `robot sim`   simulatie (Gazebo in deze container)
# `robot htmlsim` HTML-simulator in de browser (http://localhost:8080/sim/, rosbridge)
robot() {
    if [ -n "$1" ]; then
        [[ $1 =~ ^([0-9]+|sim|htmlsim)$ ]] || { echo "gebruik: robot [nummer|sim|htmlsim]"; return 1; }
        echo "$1" > "$ROBOT_FIXED"
        [[ $1 =~ ^(sim|htmlsim)$ ]] && { sim_router_start || return 1; }
    else
        rm -f "$ROBOT_FIXED"
    fi
    local nr
    nr=$(robot_detect)
    if [ -z "$nr" ]; then
        echo "Geen TurtleBot gevonden. Laptop op de wifi TB-AP-<nr>? Robot aan?"
        return 1
    fi
    robot_reachable "$nr" || echo "LET OP: robot $nr is (nog) niet bereikbaar op $(robot_host "$nr"):7447."
    robot_write_env "$nr"
    _robot_refresh
    # foxglove_bridge (Lichtblick) en de ros2-daemon herstarten met de nieuwe robot.
    pkill -f "foxglove_bridge_launch.xml" 2>/dev/null
    [ "${TB_DAEMON_RESET:-0}" = 1 ] && ros2 daemon stop >/dev/null 2>&1   # tijdelijk uit (WSL: ~70 s)
    return 0
}

# `check`: draait alles en praat de container met de robot? (vis_check.py)
# Ook vanzelf bij de eerste shell na het starten van de container.
check() {
    _robot_refresh > /dev/null
    timeout -s KILL 20 python3 /root/vis_check.py
}

_robot_banner() {
    if [ "$TB_NR" = htmlsim ]; then
        echo "HTML-simulator  (ROS_DOMAIN_ID=0, Zenoh-router en rosbridge in deze container)"
        echo "Open in de browser: http://localhost:8080/sim/ (verbindt vanzelf)   Lichtblick: http://localhost:8080   Echte robot: robot"
    elif [ "$TB_NR" = sim ]; then
        echo "Simulatie  (ROS_DOMAIN_ID=0, Zenoh-router in deze container)"
        echo "Gazebo starten: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
        echo "Lichtblick in de browser: http://localhost:8080   Echte robot: robot"
    else
        echo "TurtleBot $TB_NR  (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, Zenoh-router 10.0.$TB_NR.10)"
        echo "Lichtblick in de browser: http://localhost:8080   Andere robot: robot <nr>   Simulatie: robot sim / robot htmlsim   Controle: check"
    fi
}

# Voor elke prompt: is de robotkeuze gewijzigd (wifi gewisseld, `robot`),
# dan deze shell meteen aanpassen. Draaiende programma's (rviz2, ...) niet:
# die opnieuw starten.
_robot_seen=""
_robot_refresh() {
    local cur
    cur=$(cat "$ROBOT_ENV" 2>/dev/null)
    [ -z "$cur" ] || [ "$cur" = "$_robot_seen" ] && return
    source "$ROBOT_ENV"
    # andere robot: de ros2-daemon kent nog de oude (lege `ros2 topic list`)
    [ -n "$_robot_seen" ] && { echo "Robotkeuze gewijzigd:"; [ "${TB_DAEMON_RESET:-0}" = 1 ] && ros2 daemon stop >/dev/null 2>&1; }
    _robot_seen=$cur
    _robot_banner
}
