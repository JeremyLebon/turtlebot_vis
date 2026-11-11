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
# rclpy en de berichttypes voor elke python3 vindbaar (ook voor Pylance in VS Code, dat zijn zoekpaden
# bij de interpreter opvraagt): .pth-bestand. Uitvoeren met ROS vraagt nog altijd de omgeving (bash, .ros.env).
RUN echo /opt/ros/${ROS_DISTRO}/lib/python3.12/site-packages > /usr/lib/python3/dist-packages/ros2-${ROS_DISTRO}.pth
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
# vis_start.sh (entrypoint): http://127.0.0.1:8080, bridge op ws://127.0.0.1:8765.
# rosbridge-msgs: de robot publiceert /connected_clients (rosbridge); zonder dit
# berichttype logt de bridge daar telkens een ERROR, rood op de robotpagina.
# rosbridge-server: voor de HTML-simulator (robot htmlsim, ws://127.0.0.1:9090).
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
# http://127.0.0.1:8080 zonder parameters -> meteen verbinden met de bridge,
# zodat de student niets hoeft in te stellen. Via 127.0.0.1, ook als de pagina op
# localhost openstaat: onder WSL lukte na een wifi-onderbreking tot ~400 s geen
# nieuwe verbinding met "localhost".
RUN sed -i 's#<head>#<head><script>if(!new URLSearchParams(location.search).has("ds"))location.replace("?ds=foxglove-websocket\&ds.url=ws://"+(location.hostname=="localhost"?"127.0.0.1":location.hostname)+":8765")</script>#' \
      /opt/lichtblick/index.html && grep -q 'ds=foxglove-websocket' /opt/lichtblick/index.html
# Eén keer de layouts uit de Humble-tijd weg (Jazzy-overschakeling, 23-10):
# Lichtblick bewaart ze in de browser (IndexedDB lichtblick-layouts, actieve
# layout in localStorage studio.layout) en http://127.0.0.1:8080 is dezelfde
# oorsprong als vroeger - de oude teleop publiceerde Twist op /cmd_vel_web en
# blokkeerde zo de webpagina's van de robot. Zonder bewaarde layout neemt
# Lichtblick de standaardlayout hierboven. Nieuwe reset nodig: TB_LAYOUT verhogen.
ARG TB_LAYOUT=jazzy-4
COPY docker/layout_reset.js /tmp/layout_reset.js
RUN python3 -c "p='/opt/lichtblick/index.html'; s=open(p).read(); \
    js='<script>'+open('/tmp/layout_reset.js').read().replace('__TB_LAYOUT__', '${TB_LAYOUT}')+'</script>'; \
    assert '<head>' in s; open(p,'w').write(s.replace('<head>', '<head>'+js, 1))" \
    && grep -q 'tbLayout' /opt/lichtblick/index.html && rm /tmp/layout_reset.js
# HTML-simulator TB3 Lite Sim 3D (htmlsim/, `robot htmlsim`): http://127.0.0.1:8080/sim/,
# zonder internet - three.js en Rapier uit de npm-registry, enkel wat hij gebruikt.
ARG THREE_VERSION=0.160.0
ARG RAPIER_VERSION=0.14.0
COPY htmlsim /tmp/htmlsim
RUN cd /tmp/htmlsim && python3 build.py --local \
    && D=/opt/lichtblick/sim && mkdir -p $D/vendor/rapier /tmp/npm && cp dist/index.html $D/ \
    && python3 -c "import json; print(json.load(open('package.json'))['version'])" > $D/VERSION \
    && curl -fsSL https://registry.npmjs.org/three/-/three-${THREE_VERSION}.tgz | tar xz -C /tmp/npm \
    && for f in build/three.module.js examples/jsm/controls/OrbitControls.js examples/jsm/loaders/STLLoader.js \
                examples/jsm/loaders/ColladaLoader.js examples/jsm/loaders/OBJLoader.js examples/jsm/loaders/TGALoader.js; do \
         install -D -m 644 /tmp/npm/package/$f $D/vendor/three/$f; done \
    && rm -rf /tmp/npm/package \
    && curl -fsSL https://registry.npmjs.org/@dimforge/rapier3d-compat/-/rapier3d-compat-${RAPIER_VERSION}.tgz | tar xz -C /tmp/npm \
    && cp /tmp/npm/package/rapier.es.js $D/vendor/rapier/ \
    && rm -rf /tmp/npm /tmp/htmlsim
# Virtuele robot (`robot htmlsim`): de software van de echte robot (statuspagina op
# http://127.0.0.1:8090, knoppen, kaarten, zones) met de HTML-simulator als hardware -
# back-up als de robots niet werken. Bestanden uit turtlebot_docker
# (tools/sync_virtual_robot.sh), op dezelfde paden als op de robot. De TurtleBot3-fork
# van de robot (Nav2 zonder rviz2, Cartographer-instellingen) in /root/turtlebot3_ws;
# enkel de virtuele robot sourcet die. Kaarten: ~/ros2_ws/virtuele_robot.
RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-twist-mux \
    && rm -rf /var/lib/apt/lists/*
COPY docker/virtual_robot /tmp/virtual_robot
RUN V=/tmp/virtual_robot && W=/root/turtlebot3_ws \
    && install -m 755 $V/bin/* /usr/local/bin/ \
    && mkdir -p $W/src $W/launch && cp $V/launch/* $W/launch/ && cp $V/*.yaml $W/ \
    && cp -r $V/status_page $W/status_page && cp -r $V/turtlebot3 $W/src/ && cp $V/VERSION $W/ROBOT_VERSION \
    && bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd $W && colcon build --event-handlers console_direct- > /dev/null" \
    && rm -rf $W/build $W/log /tmp/virtual_robot \
    && ln -s /root/ros2_ws/virtuele_robot /root/turtlebot_state
COPY --chmod=755 docker/virtual_robot_start.sh /root/virtual_robot_start.sh
COPY docker/vis_start.sh /root/vis_start.sh
# Lichtblick-Teleop (Twist) -> /cmd_vel (TwistStamped) in de simulatie; op
# de echte robot doet cmd_vel_guard dat (zie sim_teleop_relay.py).
COPY docker/sim_teleop_relay.py /root/sim_teleop_relay.py
# Robot automatisch kiezen (geen .env meer aanpassen); elke interactieve shell
# neemt de keuze over en toont welke robot. Zie robot_select.sh.
COPY docker/robot_select.sh /root/robot_select.sh
COPY docker/vis_check.py /root/vis_check.py
# `diagnose`: geheugen en schrijven per onderdeel (VS Code, Pylance, bridges, eigen nodes); het
# log (~/ros2_ws/.diagnose/geheugen.log) start vanzelf in vis_start.sh. Zie vis_diagnose.py.
COPY docker/vis_diagnose.py /root/vis_diagnose.py
COPY docker/geheugen.html /opt/lichtblick/geheugen.html
RUN chmod +x /root/vis_diagnose.py && ln -s /root/vis_diagnose.py /usr/local/bin/diagnose \
    && printf '#!/bin/sh\nexec python3 /root/vis_diagnose.py --vrijmaken\n' > /usr/local/bin/vrijmaken \
    && chmod +x /usr/local/bin/vrijmaken
RUN printf '%s\n' 'source /root/robot_select.sh' \
      '[ -s "$ROBOT_ENV" ] || { nr=$(robot_detect) && [ -n "$nr" ] && robot_write_env "$nr"; }' \
      '_robot_refresh' \
      '[ -s "$ROBOT_ENV" ] || echo "Nog geen TurtleBot gevonden (laptop op de wifi TB-AP-<nr>? robot aan?). Opnieuw zoeken: robot   Vast nummer: robot <nr>   Simulatie: robot sim"' \
      'PROMPT_COMMAND="_robot_refresh${PROMPT_COMMAND:+;$PROMPT_COMMAND}"' \
      '[ -e /tmp/vis_checked ] || { touch /tmp/vis_checked; check; }' \
      >> /root/.bashrc
# Versie van turtlebot-vis (VERSION + CHANGELOG.md), in `check`. Bouwen met:
#   --build-arg VIS_VERSION="v$(cat VERSION) · $(date +%F) $(git rev-parse --short HEAD)"
# Onderaan: een nieuwe versie bouwt enkel deze laag opnieuw.
ARG VIS_VERSION=dev
ENV TB_VIS_VERSION=$VIS_VERSION
ENTRYPOINT ["/root/vis_start.sh"]
CMD ["bash"]
