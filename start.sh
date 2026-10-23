#!/bin/bash
# turtlebot-vis starten: container op de achtergrond, `check` en een shell.
#
#   ./start.sh             starten, check, shell in de container
#   ./start.sh --no-shell  enkel starten en check
#   ./start.sh --pull      eerst de nieuwste image ophalen (internet nodig)
#
# Met de laptop op de wifi van de robot (TB-AP-<nr>). Verlaat de shell met
# `exit`: de container blijft draaien (Lichtblick, bridge); opnieuw binnen
# met `docker exec -it turtlebot-vis bash`, stoppen met `docker compose down`.
set -e
cd "$(dirname "$0")"

OPEN_SHELL=true PULL=false
for arg in "$@"; do
    case $arg in
        --no-shell) OPEN_SHELL=false ;;
        --pull) PULL=true ;;
        -h|--help) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
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
docker exec $TTY turtlebot-vis bash -ic check || true

if $OPEN_SHELL; then
    echo
    exec docker exec $TTY turtlebot-vis bash
fi
