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
        -h|--help) sed -n '2,/^set/{/^#/p}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "onbekende optie: $arg (zie ./start.sh --help)"; exit 1 ;;
    esac
done

# Elke stap wordt een rij in de tabel van `check` (blok Opstarten, VIS_START_ROWS);
# een oudere image kent dat niet: dan als lijst. Tijdens het werk één regel voortgang.
ROWS=
row() { ROWS+="$1"$'\t'"$2"$'\t'"$3"$'\n'; }     # ok|warn|bad, stap, tekst
more() { ROWS+="+"$'\t\t'"$1"$'\n'; }             # uitleg bij de vorige rij
busy() { [ -t 1 ] && printf '\r\e[K%s' "$1"; }    # voortgang, wordt overschreven
done_busy() { [ -t 1 ] && printf '\r\e[K'; }

VER=$(cat VERSION 2>/dev/null || echo '?')
here() { git log -1 --format='%h, %cd' --date=format:'%d-%m-%Y' 2>/dev/null || echo 'geen git'; }

# --pull: eerst de scripts bijwerken (git pull), dan de image. Is start.sh zelf
# veranderd, dan meteen de nieuwe versie starten (TB_START_PULLED: maar één keer).
if $PULL && [ -z "${TB_START_PULLED:-}" ] && [ -d .git ]; then
    busy "Scripts bijwerken (git pull)..."
    old=$(git rev-parse HEAD)
    if msg=$(git pull --ff-only 2>&1); then
        TB_START_PULLED=$(git rev-parse --short "$old")     # = HEAD als er niets nieuws was
        if ! git diff --quiet "$old" HEAD -- start.sh; then
            done_busy
            exec env TB_START_PULLED="$TB_START_PULLED" ./start.sh "${ARGS[@]}"
        fi
    else
        row warn "Scripts" "v$VER ($(here)) - niet bijgewerkt, git pull lukt niet:"
        while IFS= read -r l; do more "$l"; done < <(echo "$msg" | grep -v '^\s*$' | tail -n 4)
        more "Eigen wijzigingen in deze map? Zie de GitBook (git checkout .env, git stash)."
        TB_START_PULLED=fout
    fi
fi
# TB_START_PULLED: commit vóór git pull (ook na het herstarten met de nieuwe start.sh)
case ${TB_START_PULLED:-} in
    "")   row ok "Scripts" "v$VER ($(here))" ;;
    fout) ;;
    *)    if [ "$(git rev-parse --short HEAD)" = "$TB_START_PULLED" ]; then
              row ok "Scripts" "v$VER ($(here)) - al de nieuwste"
          else
              row ok "Scripts" "v$VER ($(here)) - bijgewerkt van $TB_START_PULLED:"
              while IFS= read -r l; do more "$l"; done < <(git log --format='- %s' "$TB_START_PULLED"..HEAD | cut -c1-90 | head -6)
          fi ;;
esac
done_busy

# Onder WSL: geheugenlimiet in %USERPROFILE%\.wslconfig? (anders voorstel)
./wsl_check.sh || true

export XAUTH=${XAUTH:-}     # anders waarschuwt docker compose (enkel nodig onder Linux met X11)
IMG=$(docker compose config --images | head -1)
img_id() { docker image inspect -f '{{.Id}}' "$IMG" 2>/dev/null; }
was_running=$(docker ps -q -f name='^turtlebot-vis$')
before=$(img_id)
if [ -z "$before" ]; then
    # eerste keer: image ophalen (~5 GB), met de voortgang van docker
    echo "Image ophalen (eerste keer, ~5 GB)..."
    docker compose pull
elif $PULL; then
    busy "Image controleren (docker pull, bij een update enkele minuten)..."
    out=$(docker compose pull -q 2>&1) || { done_busy; echo "$out"; row warn "Image" "docker pull lukt niet (internet?), verder met de huidige"; }
fi
busy "Container starten..."
out=$(docker compose up -d 2>&1) || { done_busy; echo "$out"; exit 1; }
done_busy
if $PULL || [ -z "$before" ]; then
    [ -n "$before" ] && [ "$(img_id)" = "$before" ] && row ok "Image" "al de nieuwste" || row ok "Image" "nieuwe image binnengehaald"
fi
if [ -n "$was_running" ] && [ "$(img_id)" = "$before" ]; then row ok "Container" "draaide al"; else row ok "Container" "gestart"; fi

# -t enkel met een terminal (anders faalt docker exec, bv. in een script)
TTY=-i; [ -t 0 ] && [ -t 1 ] && TTY=-it
# De eerste shell doet de check vanzelf (.bashrc); hier één keer uitdrukkelijk,
# dus die automatische check uitzetten om hem niet dubbel te tonen.
docker exec turtlebot-vis touch /tmp/vis_checked
if [ -n "$MODE" ]; then
    busy "Onderdelen starten..."
    docker exec turtlebot-vis bash -ic "robot $MODE" > /dev/null 2>&1 || true
    if [ "$MODE" = htmlsim ]; then
        if docker exec turtlebot-vis bash -c 'for i in $(seq 1 40); do (exec 3<>/dev/tcp/127.0.0.1/9090) 2>/dev/null && exit 0; sleep 0.5; done'; then
            row ok "HTML-simulator" "rosbridge gestart"
        else
            row warn "HTML-simulator" "rosbridge start niet, zie hieronder"
        fi
    else
        row ok "Gazebo" "Zenoh-router in de container gestart"
    fi
    if $FENIX; then
        busy "Virtuele robot starten..."
        docker exec turtlebot-vis bash -ic "fenix start" > /dev/null 2>&1 || true
        if docker exec turtlebot-vis bash -c '(exec 3<>/dev/tcp/127.0.0.1/8090) 2>/dev/null'; then
            row ok "Virtuele robot" "gestart (Fenix-pagina)"
        else
            row warn "Virtuele robot" "start niet, zie hieronder"
        fi
    fi
    done_busy
fi

# De rijen naar de container; check toont ze als eerste blok van zijn tabel.
# VIS_QUIET: de shell toont de banner (robot, links) niet, check toont hetzelfde.
if docker exec turtlebot-vis grep -q VIS_START_ROWS /root/vis_check.py 2>/dev/null \
        && printf '%s' "$ROWS" | docker exec -i turtlebot-vis sh -c 'cat > /tmp/vis_start_rows'; then
    docker exec -e VIS_QUIET=1 -e VIS_START_ROWS=/tmp/vis_start_rows $TTY turtlebot-vis bash -ic check || true
else
    printf '%s' "$ROWS" | while IFS=$'\t' read -r s l t; do
        case $s in ok) s=✔ ;; warn|bad) s=! ;; +) s=" " ;; esac
        printf '  %s %-15s %s\n' "$s" "$l" "$t"
    done
    echo
    docker exec -e VIS_QUIET=1 $TTY turtlebot-vis bash -ic check || true
fi

if $OPEN_SHELL; then
    echo
    exec docker exec -e VIS_QUIET=1 $TTY turtlebot-vis bash
fi
