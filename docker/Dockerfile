ARG BASE_IMAGE=osrf/ros:jazzy-desktop-full
FROM $BASE_IMAGE

# ROS 2 Jazzy (Ubuntu 24.04), zoals de robot vanaf image v1.0.0. Eén
# container voor de echte robot én de simulatie (Gazebo, `robot sim`): de
# vroegere ros2-jazzy-gazebo-turtlebot is niet meer nodig.
# TurtleBot3 uit apt (op de laptop zijn de aanpassingen van onze fork niet
# nodig: die gaan over de bringup op de Pi).
RUN apt-get update && apt-get install --no-install-recommends -y \
python3-argcomplete python3-colcon-common-extensions \
build-essential \
ros-${ROS_DISTRO}-turtlebot3 \
ros-${ROS_DISTRO}-turtlebot3-msgs \
ros-${ROS_DISTRO}-turtlebot3-simulations \
ros-${ROS_DISTRO}-ros-gz \
ros-${ROS_DISTRO}-cartographer \
ros-${ROS_DISTRO}-cartographer-ros \
ros-${ROS_DISTRO}-navigation2 \
ros-${ROS_DISTRO}-nav2-bringup \
ros-${ROS_DISTRO}-slam-toolbox \
ros-${ROS_DISTRO}-teleop-twist-keyboard \
ros-${ROS_DISTRO}-rmw-zenoh-cpp \
ros-${ROS_DISTRO}-compressed-image-transport \
&& rm -rf /var/lib/apt/lists/*

WORKDIR /root

# Niet /ros_entrypoint.sh sourcen: dat script doet `set -e`, waardoor de
# interactieve shell (en dus `docker exec -it ... bash`) sluit bij het eerste
# commando dat faalt (typfout, ros2 topic echo met Ctrl+C, ...).
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=burger' >> /root/.bashrc
# Werkmap /root/ros2_ws (bind mount uit docker-compose.yaml): na een
# `colcon build` daar wordt de overlay in elke nieuwe shell gesourcet.
RUN echo '[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash' >> /root/.bashrc
# Lichtblick (open-source web-versie van Foxglove Studio) + foxglove_bridge:
# alternatief voor rviz2 in de browser, zonder WSLg/OpenGL. Gestart door
# vis_start.sh (entrypoint): http://localhost:8080, bridge op ws://localhost:8765.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-foxglove-bridge curl \
    && rm -rf /var/lib/apt/lists/*
ARG LICHTBLICK_VERSION=v1.29.1
RUN mkdir -p /opt/lichtblick \
    && curl -fsSL https://github.com/lichtblick-suite/lichtblick/releases/download/${LICHTBLICK_VERSION}/lichtblick-web.tar.gz \
       | tar xz -C /opt/lichtblick \
    && find /opt/lichtblick -name '*.map' -delete
# Standaard-layout (3D met kaart/scan/robot/pad + camera + teleop naar
# /cmd_vel_lichtblick) in index.html; geldt zolang de student niets aanpast.
COPY docker/lichtblick_layout.json /opt/lichtblick/turtlebot_layout.json
RUN python3 -c "import json; p='/opt/lichtblick/index.html'; s=open(p).read(); \
    ph='/*LICHTBLICK_SUITE_DEFAULT_LAYOUT_PLACEHOLDER*/'; assert ph in s; \
    open(p,'w').write(s.replace(ph, json.dumps(json.load(open('/opt/lichtblick/turtlebot_layout.json')))))"
# http://localhost:8080 zonder parameters -> meteen verbinden met de bridge,
# zodat de student niets hoeft in te stellen.
RUN sed -i 's#<head>#<head><script>if(!new URLSearchParams(location.search).has("ds"))location.replace("?ds=foxglove-websocket\&ds.url=ws://"+location.hostname+":8765")</script>#' \
      /opt/lichtblick/index.html && grep -q 'ds=foxglove-websocket' /opt/lichtblick/index.html
COPY docker/vis_start.sh /root/vis_start.sh
# Lichtblick-Teleop (Twist) -> /cmd_vel (TwistStamped) in de simulatie; op
# de echte robot doet cmd_vel_guard dat (zie sim_teleop_relay.py).
COPY docker/sim_teleop_relay.py /root/sim_teleop_relay.py
# Robot automatisch kiezen (geen .env meer aanpassen); elke interactieve shell
# neemt de keuze over en toont welke robot. Zie robot_select.sh.
COPY docker/robot_select.sh /root/robot_select.sh
COPY docker/vis_check.py /root/vis_check.py
RUN printf '%s\n' 'source /root/robot_select.sh' \
      '[ -s "$ROBOT_ENV" ] || { nr=$(robot_detect) && [ -n "$nr" ] && robot_write_env "$nr"; }' \
      '_robot_refresh' \
      '[ -s "$ROBOT_ENV" ] || echo "Nog geen TurtleBot gevonden (laptop op de wifi TB-AP-<nr>? robot aan?). Opnieuw zoeken: robot   Vast nummer: robot <nr>   Simulatie: robot sim"' \
      'PROMPT_COMMAND="_robot_refresh${PROMPT_COMMAND:+;$PROMPT_COMMAND}"' \
      '[ -e /tmp/vis_checked ] || { touch /tmp/vis_checked; check; }' \
      >> /root/.bashrc
ENTRYPOINT ["/root/vis_start.sh"]
CMD ["bash"]
