#!/usr/bin/env python3
"""Dataset uit de simulator om zelf een segmentatiemodel te trainen: camerabeeld + echte labels van hetzelfde beeld.

In de simulator: vinkje Segmentatiecamera aan (Robot > Sensoren). Rijd rond, in verschillende werelden.

    python3 dataset_opnemen.py [map] [--max 500] [--elk 0.5]

Schrijft <map>/beeld_00001.png (160 × 120, kleur), <map>/label_00001.png (mono8: label-id per pixel) en
<map>/labels.json (id → naam, van /segmentation/labels). Beeld en label horen bij elkaar via header.stamp.
Daarna: python3 seg_train.py <map>
"""
import argparse
import json
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

W, H = 160, 120


class Recorder(Node):
    def __init__(self, folder, max_n, every):
        super().__init__("dataset_opnemen")
        self.folder, self.max_n, self.every = folder, max_n, every
        os.makedirs(folder, exist_ok=True)
        self.n = len([f for f in os.listdir(folder) if f.startswith("label_")])
        self.images, self.last_t = {}, -1e9
        self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.on_image, 10)
        self.create_subscription(Image, "/segmentation/labels_map", self.on_labels, 10)
        self.create_subscription(String, "/segmentation/labels", self.on_names, 1)
        self.get_logger().info(f"opnemen naar {folder} (al {self.n}); zet de segmentatiecamera aan in de simulator")

    @staticmethod
    def key(h):
        return (h.stamp.sec, h.stamp.nanosec)

    def on_names(self, msg):
        with open(os.path.join(self.folder, "labels.json"), "w") as f:
            f.write(msg.data)

    def on_image(self, msg):
        self.images[self.key(msg.header)] = msg
        while len(self.images) > 30:                                    # enkel de laatste beelden bijhouden
            self.images.pop(next(iter(self.images)))

    def on_labels(self, msg):
        img_msg = self.images.get(self.key(msg.header))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if img_msg is None or t - self.last_t < self.every or self.n >= self.max_n:
            return
        img = cv2.imdecode(np.frombuffer(img_msg.data, np.uint8), cv2.IMREAD_COLOR)
        lab = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
        self.n += 1; self.last_t = t
        cv2.imwrite(os.path.join(self.folder, f"beeld_{self.n:05d}.png"), cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA))
        cv2.imwrite(os.path.join(self.folder, f"label_{self.n:05d}.png"), cv2.resize(lab, (W, H), interpolation=cv2.INTER_NEAREST))
        if self.n % 25 == 0 or self.n == self.max_n:
            self.get_logger().info(f"{self.n} beelden")


def main():
    ap = argparse.ArgumentParser(description="Dataset opnemen uit htmlsim (beeld + segmentatie)")
    ap.add_argument("map", nargs="?", default="dataset")
    ap.add_argument("--max", type=int, default=500, help="aantal beelden")
    ap.add_argument("--elk", type=float, default=0.5, help="minstens zoveel s sim-tijd tussen twee beelden")
    a = ap.parse_args()
    rclpy.init()
    node = Recorder(a.map, a.max, a.elk)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
