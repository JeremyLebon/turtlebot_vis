#!/bin/bash
# turtlebot-vis starten: container op de achtergrond, `check` en een shell.
#
#   ./start.sh             starten, check, shell in de container
#   ./start.sh --no-shell  enkel starten en check
#   ./start.sh --pull      eerst de nieuwste scripts (git pull) en image ophalen (internet nodig)
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
ARGS=("$@")
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

# Uitvoer in blokken: Scripts (git), Image en container (docker), Onderdelen, Controle.
B= E= OK=✔ WARN=!
if [ -t 1 ]; then B=$'\e[1m' E=$'\e[0m' OK=$'\e[32m✔\e[0m' WARN=$'\e[33m!\e[0m'; fi
hdr() { printf '\n%s%s%s\n' "$B" "$1" "$E"; }

# Versie van de scripts: VERSION (zelfde nummer als de image) + commit en datum van deze map
printf '%sTurtlebot-vis start.sh%s  ·  scripts v%s (%s)\n' "$B" "$E" "$(cat VERSION 2>/dev/null || echo '?')" \
    "$(git log -1 --format='%h, %cd' --date=format:'%d-%m-%Y' 2>/dev/null || echo 'geen git')"

# --pull: eerst de scripts bijwerken (git pull), dan de image. Is start.sh zelf
# veranderd, dan meteen de nieuwe versie starten (TB_START_PULLED: maar één keer).
if $PULL && [ -z "${TB_START_PULLED:-}" ] && [ -d .git ]; then
    hdr "Scripts (git pull)"
    old=$(git rev-parse HEAD)
    if msg=$(git pull --ff-only 2>&1); then
        if [ "$(git rev-parse HEAD)" = "$old" ]; then
            echo "  $OK al de nieuwste versie"
        else
            echo "  $OK $(git rev-list --count "$old"..HEAD) nieuwe wijziging(en), $(git rev-parse --short "$old") -> $(git rev-parse --short HEAD):"
            git log --format='      - %s' "$old"..HEAD | cut -c1-100 | head -8
            git diff --quiet "$old" HEAD -- start.sh || exec env TB_START_PULLED=1 ./start.sh "${ARGS[@]}"
        fi
    else
        echo "  $WARN niet bijgewerkt (git pull lukt niet), verder met de huidige scripts:"
        echo "$msg" | tail -n 6 | sed 's/^/      /'
        echo "    Eigen wijzigingen in deze map? Zie de GitBook (git checkout .env, git stash)."
    fi
fi

# Onder WSL: geheugenlimiet in %USERPROFILE%\.wslconfig? (anders voorstel)
./wsl_check.sh || true

hdr "Image en container (docker)"
export XAUTH=${XAUTH:-}     # anders waarschuwt docker compose (enkel nodig onder Linux met X11)
IMG=$(docker compose config --images | head -1)
img_id() { docker image inspect -f '{{.Id}}' "$IMG" 2>/dev/null; }
was_running=$(docker ps -q -f name='^turtlebot-vis$')
before=$(img_id)
if $PULL; then docker compose pull; fi
# Stil als de image er al is; de eerste keer (image ophalen, ~5 GB) wel de voortgang tonen.
if [ -n "$(img_id)" ]; then
    out=$(docker compose up -d 2>&1) || { echo "$out"; exit 1; }
else
    docker compose up -d
fi
if $PULL; then
    [ "$(img_id)" = "$before" ] && echo "  $OK al de nieuwste image" || echo "  $OK nieuwe image binnengehaald"
fi
if [ -n "$was_running" ] && [ "$(img_id)" = "$before" ]; then state="draaide al"; else state="gestart"; fi
echo "  $OK container turtlebot-vis $state"

# -t enkel met een terminal (anders faalt docker exec, bv. in een script)
TTY=-i; [ -t 0 ] && [ -t 1 ] && TTY=-it
# De eerste shell doet de check vanzelf (.bashrc); hier één keer uitdrukkelijk,
# dus die automatische check uitzetten om hem niet dubbel te tonen.
docker exec turtlebot-vis touch /tmp/vis_checked
if [ -n "$MODE" ]; then
    hdr "Onderdelen starten"
    docker exec turtlebot-vis bash -ic "robot $MODE" > /dev/null 2>&1 || true
    if [ "$MODE" = htmlsim ]; then
        if docker exec turtlebot-vis bash -c 'for i in $(seq 1 40); do (exec 3<>/dev/tcp/127.0.0.1/9090) 2>/dev/null && exit 0; sleep 0.5; done'; then
            echo "  $OK HTML-simulator (rosbridge)"
        else
            echo "  $WARN HTML-simulator: rosbridge start niet (zie de controle hieronder)"
        fi
    else
        echo "  $OK Gazebo-simulatie (Zenoh-router in de container)"
    fi
    if $FENIX; then
        docker exec turtlebot-vis bash -ic "fenix start" > /dev/null 2>&1 || true
        if docker exec turtlebot-vis bash -c '(exec 3<>/dev/tcp/127.0.0.1/8090) 2>/dev/null'; then
            echo "  $OK virtuele robot (Fenix-pagina)"
        else
            echo "  $WARN virtuele robot start niet (zie de controle hieronder)"
        fi
    fi
fi
hdr "Controle (check)"
# VIS_QUIET: de shell toont de banner (robot, links) niet, check toont hetzelfde
docker exec -e VIS_QUIET=1 $TTY turtlebot-vis bash -ic check || true

if $OPEN_SHELL; then
    echo
    exec docker exec -e VIS_QUIET=1 $TTY turtlebot-vis bash
fi
