#!/bin/bash
# turtlebot-vis starten: container op de achtergrond, `check` en een shell.
#
#   ./start.sh             starten, check, shell in de container
#   ./start.sh --no-shell  enkel starten en check
#   ./start.sh --pull      eerst de nieuwste image ophalen (internet nodig)
#   ./start.sh --htmlsim   HTML-simulator: http://127.0.0.1:8080/sim/ (thuis, huiswerk)
#   ./start.sh --fenix     HTML-simulator + virtuele robot: Fenix-pagina op http://127.0.0.1:8090
#                          (SLAM, Navigatie; back-up als de echte robots niet werken)
#   ./start.sh --sim       Gazebo-simulatie in de container
#
# Met de laptop op de wifi van de robot (TB-AP-<nr>). Verlaat de shell met
# `exit`: de container blijft draaien (Lichtblick, bridge); opnieuw binnen
# met `docker exec -it turtlebot-vis bash`, stoppen met `docker compose down`.
set -e
cd "$(dirname "$0")"

OPEN_SHELL=true PULL=false MODE= FENIX=false
for arg in "$@"; do
    case $arg in
        --no-shell) OPEN_SHELL=false ;;
        --pull) PULL=true ;;
        --htmlsim) MODE=htmlsim ;;
        --fenix) MODE=htmlsim FENIX=true ;;
        --sim) MODE=sim ;;
        -h|--help) sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "onbekende optie: $arg (zie ./start.sh --help)"; exit 1 ;;
    esac
done

# Onder WSL: geheugenlimiet in %USERPROFILE%\.wslconfig? (anders voorstel)
./wsl_check.sh || true

export XAUTH=${XAUTH:-}     # anders waarschuwt docker compose (enkel nodig onder Linux met X11)
if $PULL; then docker compose pull; fi
# Stil als de image er al is; de eerste keer (image ophalen, ~5 GB) wel de voortgang tonen.
if docker image inspect "$(docker compose config --images | head -1)" > /dev/null 2>&1; then
    out=$(docker compose up -d 2>&1) || { echo "$out"; exit 1; }
else
    docker compose up -d
fi

# -t enkel met een terminal (anders faalt docker exec, bv. in een script)
TTY=-i; [ -t 0 ] && [ -t 1 ] && TTY=-it
# De eerste shell doet de check vanzelf (.bashrc); hier één keer uitdrukkelijk,
# dus die automatische check uitzetten om hem niet dubbel te tonen.
docker exec turtlebot-vis touch /tmp/vis_checked
if [ -n "$MODE" ]; then
    docker exec turtlebot-vis bash -ic "robot $MODE" > /dev/null 2>&1 || true
    if [ "$MODE" = htmlsim ]; then
        echo "HTML-simulator starten (rosbridge)..."
        docker exec turtlebot-vis bash -c 'for i in $(seq 1 40); do (exec 3<>/dev/tcp/127.0.0.1/9090) 2>/dev/null && exit 0; sleep 0.5; done' || true
    fi
    if $FENIX; then
        echo "Virtuele robot starten..."
        docker exec turtlebot-vis bash -ic "fenix start" > /dev/null 2>&1 || true
    fi
fi
# VIS_QUIET: de shell toont de banner (robot, links) niet, check toont hetzelfde
docker exec -e VIS_QUIET=1 $TTY turtlebot-vis bash -ic check || true

if $OPEN_SHELL; then
    echo
    exec docker exec -e VIS_QUIET=1 $TTY turtlebot-vis bash
fi
