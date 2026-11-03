#!/usr/bin/env python3
# Headless variant of turtlebot3_cartographer/cartographer.launch.py for the
# SLAM button on the status page: same config and nodes, minus rviz2 (there
# is no display on the robot). Students who type the upstream
# `ros2 launch turtlebot3_cartographer cartographer.launch.py` themselves
# get the same mapping behaviour.
#
# Cartographer instead of slam_toolbox (navigation2.launch.py slam:=True):
# the LDS-02 alternates 220/221 readings per scan and slam_toolbox rejects
# every scan whose length differs from the first one - when the very first
# scan gets rejected the map stays empty until the robot moves (see
# turtlebot_setup/docs/todo.md). Cartographer treats scans as point clouds
# and doesn't care about the reading count.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Virtuele robot (turtlebot-vis): de HTML-simulator stempelt met zijn eigen klok
SIM = os.environ.get('FENIX_SIM') == '1'


def generate_launch_description():
    cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    config_dir = os.path.join(cartographer_prefix, 'config')

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': SIM}],
            arguments=['-configuration_directory', config_dir,
                       '-configuration_basename', 'turtlebot3_lds_2d.lua']),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(cartographer_prefix, 'launch', 'occupancy_grid.launch.py')),
            launch_arguments={'use_sim_time': 'true' if SIM else 'false', 'resolution': '0.05',
                              'publish_period_sec': '1.0'}.items(),
        ),

        # /map_saver/save_map for the status page's "save map" button (came
        # with navigation2.launch.py slam:=True before). Volatile subscription:
        # compatible with any publisher QoS, and the grid is republished every
        # publish_period_sec anyway.
        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_saver',
            output='screen',
            parameters=[{'use_sim_time': SIM, 'save_map_timeout': 5.0,
                         'free_thresh_default': 0.25,
                         'occupied_thresh_default': 0.65,
                         'map_subscribe_transient_local': False}]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_saver']}]),
    ])
