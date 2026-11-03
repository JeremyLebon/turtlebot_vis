# Bringup van de virtuele robot (turtlebot-vis): zie sim_bringup_node.py.
# Zelfde rol als turtlebot3_bringup robot.launch.py op de echte robot.
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['python3', '/usr/local/bin/sim_bringup_node.py'], output='screen'),
    ])
