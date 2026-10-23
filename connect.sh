#!/bin/bash
# Een (extra) terminal in de turtlebot-vis-container.
#
#   ./connect.sh                    shell in de container
#   ./connect.sh ros2 topic list    één commando in de container
#
# Draait de container nog niet, dan eerst ./start.sh (starten + check).
cd "$(dirname "$0")"

if [ "$(docker inspect -f '{{.State.Running}}' turtlebot-vis 2>/dev/null)" != true ]; then
    echo "turtlebot-vis draait nog niet - starten met ./start.sh"
    if [ $# -eq 0 ]; then exec ./start.sh; fi
    ./start.sh --no-shell || exit 1
fi

TTY=-i; [ -t 0 ] && [ -t 1 ] && TTY=-it
if [ $# -eq 0 ]; then
    exec docker exec $TTY turtlebot-vis bash
fi
# bash -i: dezelfde omgeving als een shell (ROS, gekozen robot)
exec docker exec $TTY turtlebot-vis bash -ic "$(printf '%q ' "$@")"
