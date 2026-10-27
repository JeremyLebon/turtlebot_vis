#!/bin/bash
# turtlebot-vis starten: container op de achtergrond, `check` en een shell.
#
#   ./start.sh             starten, check, shell in de container
#   ./start.sh --no-shell  enkel starten en check
#   ./start.sh --pull      eerst de nieuwste image ophalen (internet nodig)
#   ./start.sh --htmlsim   HTML-simulator: http://localhost:8080/sim/ (thuis, huiswerk)
#   ./start.sh --sim       Gazebo-simulatie in de container
#
# Met de laptop op de wifi van de robot (TB-AP-<nr>). Verlaat de shell met
# `exit`: de container blijft draaien (Lichtblick, bridge); opnieuw binnen
# met `docker exec -it turtlebot-vis bash`, stoppen met `docker compose down`.
set -e
cd "$(dirname "$0")"

OPEN_SHELL=true PULL=false MODE=
for arg in "$@"; do
    case $arg in
        --no-shell) OPEN_SHELL=false ;;
        --pull) PULL=true ;;
        --htmlsim) MODE=htmlsim ;;
        --sim) MODE=sim ;;
        -h|--help) sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "onbekende optie: $arg (zie ./start.sh --help)"; exit 1 ;;
    esac
done

if $PULL; then docker compose pull; fi
docker compose up -d

# -t enkel met een terminal (anders faalt docker exec, bv. in een script)
TTY=-i; [ -t 0 ] && [ -t 1 ] && TTY=-it
# De eerste shell doet de check vanzelf (.bashrc); hier één keer uitdrukkelijk,
# dus die automatische check uitzetten om hem niet dubbel te tonen.
docker exec turtlebot-vis touch /tmp/vis_checked
T0=$SECONDS
if [ -n "$MODE" ]; then
    echo "Simulatiemodus instellen (robot $MODE)..."
    docker exec turtlebot-vis bash -ic "robot $MODE" > /dev/null 2>&1 || true
    echo "  klaar na $((SECONDS - T0)) s"
    if [ "$MODE" = htmlsim ]; then
        echo "HTML-simulator starten (rosbridge)..."
        docker exec turtlebot-vis bash -c 'for i in $(seq 1 40); do (exec 3<>/dev/tcp/127.0.0.1/9090) 2>/dev/null && exit 0; sleep 0.5; done' || true
    fi
fi
echo "Check uitvoeren (tot 20 s)..."
docker exec $TTY turtlebot-vis bash -ic check || true
echo "  start + check: $((SECONDS - T0)) s"

if $OPEN_SHELL; then
    echo
    exec docker exec $TTY turtlebot-vis bash
fi
