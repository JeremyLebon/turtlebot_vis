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
ros-humble-rmw-cyclonedds-cpp \
ros-humble-rmw-zenoh-cpp \
ros-humble-compressed-image-transport



COPY turtlebot3 /root/turtlebot3_ws/src/turtlebot3

WORKDIR /root/turtlebot3_ws/src/turtlebot3

WORKDIR /root/turtlebot3_ws

COPY ./cyclonedds_config.xml /root/turtlebot3_ws/cylconedds_config.xml

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
