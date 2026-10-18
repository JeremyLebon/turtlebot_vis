#!/bin/bash
# Start turtlebot-vis voor de juiste robot en open een shell in de container.
#
#   ./start.sh        robotnummer automatisch (wifi TB-AP-<nr>), anders vragen
#   ./start.sh 5      robot 5
#
# Zet ROS_DOMAIN_ID=<nr> en ROBOT_ZENOH_IP=10.0.<nr>.10 voor docker compose
# (omgevingsvariabelen gaan voor .env), zodat niemand .env hoeft te bewerken.
# Een andere robot dan de draaiende container -> compose maakt de container
# opnieuw aan met de nieuwe waarden.

cd "$(dirname "$0")" || exit 1

detect_robot() {
    # WSL: wifinaam van Windows (TB-AP-05 -> 5).
    if command -v netsh.exe >/dev/null 2>&1; then
        local ssid
        ssid=$(netsh.exe wlan show interfaces 2>/dev/null | tr -d '\r' \
               | grep -m1 -E '^\s*SSID\s*:' | sed 's/.*:\s*//')
        if [[ $ssid =~ ^TB-AP-0*([1-9][0-9]*)$ ]]; then
            echo "${BASH_REMATCH[1]}"; return
        fi
    fi
    # Linux, of WSL in mirrored-netwerkmodus: eigen adres 10.0.<nr>.x.
    local nr
    nr=$(ip -4 -o addr show 2>/dev/null | grep -oE 'inet 10\.0\.[0-9]+\.' \
         | head -1 | cut -d. -f3)
    [ -n "$nr" ] && [ "$nr" -ge 1 ] && echo "$nr"
}

NR=$1
if [ -z "$NR" ]; then
    NR=$(detect_robot)
    [ -n "$NR" ] && echo "Robot $NR gevonden via de wifi (TB-AP-$(printf %02d "$NR"))."
fi
while ! [[ $NR =~ ^[1-9][0-9]?$ ]]; do
    read -rp "Robotnummer (1-9)? " NR
done

export ROS_DOMAIN_ID=$NR
export ROBOT_ZENOH_IP=10.0.$NR.10
echo "TurtleBot $NR: ROS_DOMAIN_ID=$ROS_DOMAIN_ID, Zenoh-router $ROBOT_ZENOH_IP:7447"

if ! timeout 3 bash -c "exec 3<>/dev/tcp/$ROBOT_ZENOH_IP/7447" 2>/dev/null; then
    echo
    echo "LET OP: robot $NR niet bereikbaar op $ROBOT_ZENOH_IP:7447."
    echo "  - Staat de robot aan (en is de software gestart)?"
    echo "  - Zit je laptop op de wifi TB-AP-$(printf %02d "$NR")?"
    echo "De container start toch; hij verbindt zodra de robot bereikbaar is."
    echo
fi

docker compose up -d || exit 1
echo
echo "Lichtblick (visualisatie in de browser): http://localhost:8080"
echo "Extra terminal in de container: docker exec -it turtlebot-vis bash"
echo
exec docker exec -it turtlebot-vis bash
