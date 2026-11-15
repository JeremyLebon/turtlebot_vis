#!/usr/bin/env python3
"""Eigen segmentatiemodel (seg_train.py) op het camerabeeld, met enkel OpenCV; de simulator toont het masker en de IoU.

    python3 seg_infer.py [seg_model.onnx]

Leest /camera/image_raw/compressed, publiceert /student/segmentation (mono8, 160 × 120, label-id per pixel) met de
header van het beeld, zodat htmlsim met de echte segmentatie van hetzelfde beeld vergelijkt.
"""
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class SegInfer(Node):
    def __init__(self, model):
        super().__init__("seg_infer")
        self.net = cv2.dnn.readNet(model)
        self.pub = self.create_publisher(Image, "/student/segmentation", 10)
        self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.on_image, 1)
        self.get_logger().info(f"model {model}, wacht op beelden ...")

    def on_image(self, msg):
        img = cv2.resize(cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR), (160, 120), interpolation=cv2.INTER_AREA)
        self.net.setInput(img.astype(np.float32).transpose(2, 0, 1)[None] / 255)   # zelfde voorbewerking als bij het trainen
        mask = self.net.forward()[0].argmax(0).astype(np.uint8)
        self.pub.publish(Image(header=msg.header, height=120, width=160, encoding="mono8", step=160, data=mask.tobytes()))


def main():
    rclpy.init()
    node = SegInfer(sys.argv[1] if len(sys.argv) > 1 else "seg_model.onnx")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
