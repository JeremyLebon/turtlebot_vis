ARG BASE_IMAGE=osrf/ros:humble-desktop-full
FROM $BASE_IMAGE


RUN apt-get update

RUN apt-get install --no-install-recommends -y \
python3-argcomplete python3-colcon-common-extensions \
libboost-system-dev build-essential \
ros-humble-hls-lfcd-lds-driver \
ros-humble-turtlebot3-msgs \
ros-humble-dynamixel-sdk \
libudev-dev \
ros-humble-cartographer \
ros-humble-cartographer-ros \ 
ros-humble-navigation2 \
ros-humble-nav2-bringup \
ros-humble-urdf \
ros-humble-rmw-zenoh-cpp \
ros-humble-compressed-image-transport



COPY turtlebot3 /root/turtlebot3_ws/src/turtlebot3

WORKDIR /root/turtlebot3_ws/src/turtlebot3

WORKDIR /root/turtlebot3_ws


RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=ON -DBUILD_WITH_VIEWER=ON'

# Niet /ros_entrypoint.sh sourcen: dat script doet `set -e`, waardoor de
# interactieve shell (en dus `docker exec -it ... bash`) sluit bij het eerste
# commando dat faalt (typfout, ros2 topic echo met Ctrl+C, ...).
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
# RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> /root/.bashrc  
# RUN echo 'export LDS_MODEL=LDS-01' >> /root/.bashrc  
RUN echo 'export TURTLEBOT3_MODEL=burger' >> /root/.bashrc
RUN echo "source /root/turtlebot3_ws/install/setup.bash" >> /root/.bashrc
# Werkmap /root/ros2_ws (bind mount uit docker-compose.yaml): na een
# `colcon build` daar wordt de overlay in elke nieuwe shell gesourcet.
RUN echo '[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash' >> /root/.bashrc    
# Lichtblick (open-source web-versie van Foxglove Studio) + foxglove_bridge:
# alternatief voor rviz2 in de browser, zonder WSLg/OpenGL. Gestart door
# vis_start.sh (entrypoint): http://localhost:8080, bridge op ws://localhost:8765.
# rosbridge-msgs: de robot publiceert /connected_clients (rosbridge); zonder dit
# berichttype logt de bridge daar telkens een ERROR, rood op de robotpagina.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-foxglove-bridge curl \
      ros-humble-rosbridge-msgs \
    && rm -rf /var/lib/apt/lists/*
ARG LICHTBLICK_VERSION=v1.29.1
RUN mkdir -p /opt/lichtblick \
    && curl -fsSL https://github.com/lichtblick-suite/lichtblick/releases/download/${LICHTBLICK_VERSION}/lichtblick-web.tar.gz \
       | tar xz -C /opt/lichtblick \
    && find /opt/lichtblick -name '*.map' -delete
# Standaard-layout (3D met kaart/scan/robot/pad + camera + teleop naar
# /cmd_vel_web) in index.html; geldt zolang de student niets aanpast.
COPY docker/lichtblick_layout.json /opt/lichtblick/turtlebot_layout.json
RUN python3 -c "import json; p='/opt/lichtblick/index.html'; s=open(p).read(); \
    ph='/*LICHTBLICK_SUITE_DEFAULT_LAYOUT_PLACEHOLDER*/'; assert ph in s; \
    open(p,'w').write(s.replace(ph, json.dumps(json.load(open('/opt/lichtblick/turtlebot_layout.json')))))"
# http://localhost:8080 zonder parameters -> meteen verbinden met de bridge,
# zodat de student niets hoeft in te stellen.
RUN sed -i 's#<head>#<head><script>if(!new URLSearchParams(location.search).has("ds"))location.replace("?ds=foxglove-websocket\&ds.url=ws://"+location.hostname+":8765")</script>#' \
      /opt/lichtblick/index.html && grep -q 'ds=foxglove-websocket' /opt/lichtblick/index.html
COPY docker/vis_start.sh /root/vis_start.sh
# Robot automatisch kiezen (geen .env meer aanpassen); elke interactieve shell
# neemt de keuze over en toont welke robot. Zie robot_select.sh.
COPY docker/robot_select.sh /root/robot_select.sh
COPY docker/vis_check.py /root/vis_check.py
RUN printf '%s\n' 'source /root/robot_select.sh' \
      '[ -s "$ROBOT_ENV" ] || { nr=$(robot_detect) && [ -n "$nr" ] && robot_write_env "$nr"; }' \
      '_robot_refresh' \
      '[ -s "$ROBOT_ENV" ] || echo "Nog geen TurtleBot gevonden (laptop op de wifi TB-AP-<nr>? robot aan?). Opnieuw zoeken: robot   Vast nummer: robot <nr>"' \
      'PROMPT_COMMAND="_robot_refresh${PROMPT_COMMAND:+;$PROMPT_COMMAND}"' \
      '[ -e /tmp/vis_checked ] || { touch /tmp/vis_checked; check; }' \
      >> /root/.bashrc
ENTRYPOINT ["/root/vis_start.sh"]
CMD ["bash"]

# tf2: nooit 0.25.23 (Humble-sync 2026-09-07) - die heeft een deadlock waardoor
# de TF-listener van een proces bevriest (ros2/geometry2#982/#995; op de robot
# faalde Nav2 erdoor, 2026-10-09). Deze image heeft nu 0.25.22 (niet getroffen),
# maar een rebuild met verse apt-lijsten zou 0.25.23 binnenhalen: dan 0.25.24
# uit ros2-testing. Mag weg zodra 0.25.24 in de gewone ROS-repo staat.
RUN if [ "$(dpkg-query -W -f='${Version}' ros-humble-tf2-ros | cut -d- -f1)" = "0.25.23" ]; then \
      sed -e 's#^Types: .*#Types: deb#' -e 's#/ros2/ubuntu#/ros2-testing/ubuntu#' \
          /etc/apt/sources.list.d/ros2.sources > /etc/apt/sources.list.d/ros2-testing.sources \
      && apt-get update \
      && apt-get install -y --no-install-recommends --only-upgrade \
          $(dpkg-query -W -f='${Package}\n' 'ros-humble-tf2*') \
      && rm /etc/apt/sources.list.d/ros2-testing.sources && rm -rf /var/lib/apt/lists/*; \
    fi \
    && dpkg-query -W -f='${Package} ${Version}\n' ros-humble-tf2-ros
