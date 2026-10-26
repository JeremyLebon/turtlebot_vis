#!/usr/bin/env python3
"""Lichtblick-Teleop in de simulatie.

Het Teleop-paneel van Lichtblick publiceert enkel geometry_msgs/Twist, op
/cmd_vel_lichtblick (zie lichtblick_layout.json). De TurtleBot3 werkt op
Jazzy met TwistStamped op /cmd_vel. Op de echte robot zet cmd_vel_guard dat
om; in de simulatie (`robot sim`) doet deze node het, gestart door
vis_start.sh.
"""
import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class SimTeleopRelay(Node):
    def __init__(self):
        super().__init__("sim_teleop_relay")
        self.pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.create_subscription(Twist, "cmd_vel_lichtblick", self.on_twist, 10)

    def on_twist(self, twist):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist = twist
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimTeleopRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
