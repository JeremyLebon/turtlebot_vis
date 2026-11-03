#!/usr/bin/env python3
# Bringup van de virtuele robot (turtlebot-vis, `robot virtueel`): de HTML-
# simulator (http://localhost:8080/sim/) is de hardware en publiceert zelf
# /odom, /scan, /imu, /tf en /battery_state. Deze node start dus niets; hij
# meldt in het log of de simulator er is, zodat "bringup" op de statuspagina
# dezelfde betekenis heeft als op de echte robot.
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class SimBringup(Node):
    def __init__(self):
        super().__init__('sim_bringup')
        self.seen = {}
        self.create_subscription(Odometry, '/odom', lambda m: self.seen.__setitem__('odom', time.monotonic()), 10)
        self.create_subscription(LaserScan, '/scan', lambda m: self.seen.__setitem__('scan', time.monotonic()),
                                 qos_profile_sensor_data)
        self.ok = None
        self.create_timer(2.0, self.check)
        self.get_logger().info('virtuele robot: wacht op de simulator (http://localhost:8080/sim/)')

    def check(self):
        now = time.monotonic()
        ok = all(now - self.seen.get(t, -99) < 3 for t in ('odom', 'scan'))
        if ok != self.ok:
            self.ok = ok
            if ok:
                self.get_logger().info('simulator verbonden: /odom en /scan komen binnen')
            else:
                self.get_logger().warn('geen /odom of /scan van de simulator - is http://localhost:8080/sim/ open?')


def main():
    rclpy.init()
    node = SimBringup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
