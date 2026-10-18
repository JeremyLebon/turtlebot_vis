#!/usr/bin/env python3
"""Fake robot sensors for at home (no robot): publishes random values once per
second on /sim/battery_state, /sim/temperature, /sim/humidity and /sim/gas_value.

Run it in the simulation container (gazebo_turtlebot_cont):
    python3 simulation_sensors.py
"""
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Temperature, RelativeHumidity
from std_msgs.msg import Float32


class RandomSensorNode(Node):
    def __init__(self):
        super().__init__("random_sensor_node")

        # Topics start with /sim/: the real robot already publishes /battery_state
        self.pub_battery = self.create_publisher(BatteryState, "/sim/battery_state", 10)
        self.pub_temp = self.create_publisher(Temperature, "/sim/temperature", 10)
        self.pub_humidity = self.create_publisher(RelativeHumidity, "/sim/humidity", 10)
        self.pub_gas = self.create_publisher(Float32, "/sim/gas_value", 10)

        self.timer = self.create_timer(1.0, self.publish_random_values)  # 1 Hz
        self.get_logger().info("Random sensors node started: topics /sim/...")

    def publish_random_values(self):
        now = self.get_clock().now().to_msg()

        batt = BatteryState()
        batt.header.stamp = now
        batt.voltage = random.uniform(10.5, 12.6)      # V (3S LiPo)
        batt.percentage = random.uniform(0.0, 1.0)     # 0-1, not 0-100
        self.pub_battery.publish(batt)

        temp = Temperature()
        temp.header.stamp = now
        temp.temperature = random.uniform(18.0, 35.0)  # degrees C
        temp.variance = 0.1
        self.pub_temp.publish(temp)

        hum = RelativeHumidity()
        hum.header.stamp = now
        hum.relative_humidity = random.uniform(0.20, 0.80)  # 0-1
        hum.variance = 0.02
        self.pub_humidity.publish(hum)

        gas = Float32()
        gas.data = random.uniform(0.0, 1024.0)         # raw sensor value
        self.pub_gas.publish(gas)

        self.get_logger().info(
            f"battery {batt.voltage:.2f} V {batt.percentage * 100:.0f} % | "
            f"temp {temp.temperature:.1f} C | humidity {hum.relative_humidity * 100:.0f} % | "
            f"gas {gas.data:.0f}")


def main(args=None):
    rclpy.init(args=args)
    node = RandomSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
