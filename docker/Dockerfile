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
# VS Code (Dev Containers, attach): de ROS-omgeving staat enkel in .bashrc. Pylance krijgt het pad naar
# rclpy, de debugger (Run/F5) een env-bestand: basis-ROS hier, de gekozen robot voegt robot_select.sh toe.
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && env | grep -E '^(PATH|PYTHONPATH|LD_LIBRARY_PATH|AMENT_PREFIX_PATH|CMAKE_PREFIX_PATH|ROS_[A-Z_]*|GZ_[A-Z_]*)='" > /root/.ros_base.env \
    && printf "TURTLEBOT3_MODEL=burger\nRMW_IMPLEMENTATION=rmw_zenoh_cpp\n" >> /root/.ros_base.env && cp /root/.ros_base.env /root/.ros.env \
    && mkdir -p /root/.vscode-server/data/Machine \
    && printf '%s\n' '{' \
      '  "python.defaultInterpreterPath": "/usr/bin/python3",' \
      '  "python.analysis.extraPaths": ["/opt/ros/'"${ROS_DISTRO}"'/lib/python3.12/site-packages"],' \
      '  "python.autoComplete.extraPaths": ["/opt/ros/'"${ROS_DISTRO}"'/lib/python3.12/site-packages"],' \
      '  "python.envFile": "/root/.ros.env",' \
      '  "terminal.integrated.defaultProfile.linux": "bash"' \
      '}' > /root/.vscode-server/data/Machine/settings.json
# Lichtblick (open-source web-versie van Foxglove Studio) + foxglove_bridge:
# alternatief voor rviz2 in de browser, zonder WSLg/OpenGL. Gestart door
# vis_start.sh (entrypoint): http://localhost:8080, bridge op ws://localhost:8765.
# rosbridge-msgs: de robot publiceert /connected_clients (rosbridge); zonder dit
# berichttype logt de bridge daar telkens een ERROR, rood op de robotpagina.
# rosbridge-server: voor de HTML-simulator (robot htmlsim, ws://localhost:9090).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-foxglove-bridge curl \
      ros-${ROS_DISTRO}-rosbridge-msgs \
      ros-${ROS_DISTRO}-rosbridge-server \
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
# Eén keer de layouts uit de Humble-tijd weg (Jazzy-overschakeling, 23-10):
# Lichtblick bewaart ze in de browser (IndexedDB lichtblick-layouts, actieve
# layout in localStorage studio.layout) en http://localhost:8080 is dezelfde
# oorsprong als vroeger - de oude teleop publiceerde Twist op /cmd_vel_web en
# blokkeerde zo de webpagina's van de robot. Zonder bewaarde layout neemt
# Lichtblick de standaardlayout hierboven. Nieuwe reset nodig: TB_LAYOUT verhogen.
ARG TB_LAYOUT=jazzy-4
COPY docker/layout_reset.js /tmp/layout_reset.js
RUN python3 -c "p='/opt/lichtblick/index.html'; s=open(p).read(); \
    js='<script>'+open('/tmp/layout_reset.js').read().replace('__TB_LAYOUT__', '${TB_LAYOUT}')+'</script>'; \
    assert '<head>' in s; open(p,'w').write(s.replace('<head>', '<head>'+js, 1))" \
    && grep -q 'tbLayout' /opt/lichtblick/index.html && rm /tmp/layout_reset.js
# HTML-simulator TB3 Lite Sim 3D (htmlsim/, `robot htmlsim`): http://localhost:8080/sim/,
# zonder internet - three.js en Rapier uit de npm-registry, enkel wat hij gebruikt.
ARG THREE_VERSION=0.160.0
ARG RAPIER_VERSION=0.14.0
COPY htmlsim /tmp/htmlsim
RUN cd /tmp/htmlsim && python3 build.py --local \
    && D=/opt/lichtblick/sim && mkdir -p $D/vendor/rapier /tmp/npm && cp dist/index.html $D/ \
    && curl -fsSL https://registry.npmjs.org/three/-/three-${THREE_VERSION}.tgz | tar xz -C /tmp/npm \
    && for f in build/three.module.js examples/jsm/controls/OrbitControls.js examples/jsm/loaders/STLLoader.js \
                examples/jsm/loaders/ColladaLoader.js examples/jsm/loaders/OBJLoader.js examples/jsm/loaders/TGALoader.js; do \
         install -D -m 644 /tmp/npm/package/$f $D/vendor/three/$f; done \
    && rm -rf /tmp/npm/package \
    && curl -fsSL https://registry.npmjs.org/@dimforge/rapier3d-compat/-/rapier3d-compat-${RAPIER_VERSION}.tgz | tar xz -C /tmp/npm \
    && cp /tmp/npm/package/rapier.es.js $D/vendor/rapier/ \
    && rm -rf /tmp/npm /tmp/htmlsim
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
