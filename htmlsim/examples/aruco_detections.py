#!/usr/bin/env python3
"""Voorbeeld voor htmlsim: ArUco-markers zoeken in het camerabeeld en het resultaat publiceren.

De simulator tekent de kaders op het camerabeeld en vergelijkt het masker met de echte segmentatie
(IoU in de HUD). Wereld Proefhal: markers 0-3 op de muren, 10 op een standaard (DICT_4X4_50).

    python3 aruco_detections.py

Leest  /camera/image_raw/compressed
Publiceert
  /detections            vision_msgs/Detection2DArray  (kader + "aruco <id>")
  /student/segmentation  sensor_msgs/Image mono8       (pixels van een marker = 13 = aruco, de rest 0)
Zelfde header als het camerabeeld, zodat de simulator met hetzelfde beeld vergelijkt.
Een eigen model (bv. YOLO als ONNX via cv2.dnn) publiceert op dezelfde manier; de label-id's staan op
/segmentation/labels.
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

ARUCO_LABEL = 13   # /segmentation/labels: 13 = aruco


class ArucoDetections(Node):
    def __init__(self):
        super().__init__("aruco_detections")
        aruco = cv2.aruco
        if hasattr(aruco, "ArucoDetector"):            # OpenCV 4.7 en nieuwer
            det = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_4X4_50), aruco.DetectorParameters())
            self.detect = lambda gray: det.detectMarkers(gray)[:2]
        else:                                          # OpenCV 4.6 (container turtlebot-vis)
            d, prm = aruco.Dictionary_get(aruco.DICT_4X4_50), aruco.DetectorParameters_create()
            self.detect = lambda gray: aruco.detectMarkers(gray, d, parameters=prm)[:2]
        self.pub_det = self.create_publisher(Detection2DArray, "/detections", 10)
        self.pub_seg = self.create_publisher(Image, "/student/segmentation", 10)
        self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.on_image, 10)
        self.get_logger().info("wacht op /camera/image_raw/compressed ...")

    def on_image(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        corners, ids = self.detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        out = Detection2DArray(header=msg.header)
        mask = np.zeros(img.shape[:2], np.uint8)
        for c, marker in zip(corners, [] if ids is None else ids.flatten()):
            pts = c.reshape(4, 2)
            (x0, y0), (x1, y1) = pts.min(axis=0), pts.max(axis=0)
            d = Detection2D(header=msg.header)
            d.bbox.center.position.x, d.bbox.center.position.y = float(x0 + x1) / 2, float(y0 + y1) / 2
            d.bbox.size_x, d.bbox.size_y = float(x1 - x0), float(y1 - y0)
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id, h.hypothesis.score = f"aruco {marker}", 1.0
            d.results.append(h)
            out.detections.append(d)
            cv2.fillConvexPoly(mask, pts.astype(np.int32), ARUCO_LABEL)
        self.pub_det.publish(out)
        self.pub_seg.publish(Image(header=msg.header, height=mask.shape[0], width=mask.shape[1],
                                   encoding="mono8", step=mask.shape[1], data=mask.tobytes()))


def main():
    rclpy.init()
    node = ArucoDetections()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
