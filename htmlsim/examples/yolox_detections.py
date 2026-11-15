#!/usr/bin/env python3
"""Objectdetectie met een AI-model (YOLOX, 80 COCO-klassen) op het camerabeeld, met enkel OpenCV.

Werkt in de container turtlebot-vis (OpenCV 4.6, cv2.dnn, CPU, ~0,1 s per beeld) op de simulator en op de echte robot.
De simulator (htmlsim) tekent de kaders op het camerabeeld.

    python3 yolox_detections.py                   ROS-node: /camera/image_raw/compressed → /detections
    python3 yolox_detections.py --test foto.jpg   zonder ROS: detecties tonen en resultaat.jpg schrijven
    opties: --model pad.onnx  --score 0.35  --nms 0.45  --topic /camera/image_raw/compressed

Model: YOLOX-S uit de OpenCV Model Zoo (github.com/opencv/opencv_zoo, models/object_detection_yolox), Apache-2.0,
int8 (9 MB). Gezocht in /opt/models/yolox, ~/ros2_ws/models en naast dit script; anders eenmalig gedownload
(internet nodig). Publiceert vision_msgs/Detection2DArray met de COCO-naam als class_id.

Let op: de simulator bestaat uit blokken en cilinders; een model getraind op foto's herkent daar weinig. Op het
beeld van de echte robot (personen, stoelen, flessen, laptops ...) wel. Zelf trainen kan met beelden uit de
simulator en de echte segmentatie (/segmentation/labels_map) als labels.
"""
import argparse
import os
import sys
import time
import urllib.request

import cv2
import numpy as np

MODEL_FILE = "object_detection_yolox_2022nov_int8.onnx"
MODEL_URL = "https://github.com/opencv/opencv_zoo/raw/main/models/object_detection_yolox/" + MODEL_FILE
SEARCH = ["/opt/models/yolox", os.path.expanduser("~/ros2_ws/models"), os.path.dirname(os.path.abspath(__file__))]

COCO = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
        "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
        "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
        "teddy bear", "hair drier", "toothbrush"]


def find_model(path=None):
    if path:
        return path
    for d in SEARCH:
        p = os.path.join(d, MODEL_FILE)
        if os.path.exists(p):
            return p
    dst = os.path.join(SEARCH[1], MODEL_FILE)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    print(f"model niet gevonden, downloaden naar {dst} (9 MB) ...", file=sys.stderr)
    urllib.request.urlretrieve(MODEL_URL, dst)
    return dst


class Yolox:
    """YOLOX in cv2.dnn: letterbox naar 640 × 640 (grijs 114, BGR, 0-255), uitvoer 8400 × 85 op rasters van 8/16/32 px."""
    SIZE = 640

    def __init__(self, model, score=0.35, nms=0.45):
        self.net = cv2.dnn.readNet(model)
        self.score, self.nms = score, nms
        grids, strides = [], []
        for s in (8, 16, 32):
            n = self.SIZE // s
            yv, xv = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
            grids.append(np.stack((xv, yv), 2).reshape(-1, 2))
            strides.append(np.full((n * n, 1), s))
        self.grids, self.strides = np.concatenate(grids).astype(np.float32), np.concatenate(strides).astype(np.float32)

    def __call__(self, img):
        """img: BGR. Geeft [(x0, y0, x1, y1, klasse, score), ...] in pixels van img."""
        h, w = img.shape[:2]
        r = min(self.SIZE / h, self.SIZE / w)
        pad = np.full((self.SIZE, self.SIZE, 3), 114, np.uint8)
        pad[:int(h * r), :int(w * r)] = cv2.resize(img, (int(w * r), int(h * r)))
        self.net.setInput(np.transpose(pad, (2, 0, 1))[None].astype(np.float32))
        d = self.net.forward()[0]
        xy = (d[:, :2] + self.grids) * self.strides
        wh = np.exp(d[:, 2:4]) * self.strides
        scores = d[:, 4:5] * d[:, 5:]
        cls, best = scores.argmax(1), scores.max(1)
        keep = best > self.score
        xy, wh, cls, best = xy[keep] / r, wh[keep] / r, cls[keep], best[keep]
        boxes = np.concatenate([xy - wh / 2, wh], 1)                       # x, y, b, h voor NMSBoxes
        out = []
        for c in np.unique(cls):                                           # NMS per klasse (OpenCV 4.6 heeft geen batched)
            m = np.where(cls == c)[0]
            for i in np.array(cv2.dnn.NMSBoxes(boxes[m].tolist(), best[m].tolist(), self.score, self.nms)).flatten():
                x, y, bw, bh = boxes[m[i]]
                # gewone Python-floats: ROS weigert numpy-getallen (assert PyFloat_Check, crash van de node)
                out.append((float(max(0.0, x)), float(max(0.0, y)), float(min(w, x + bw)), float(min(h, y + bh)), COCO[int(c)], float(best[m[i]])))
        return out


def draw(img, dets):
    for x0, y0, x1, y1, name, sc in dets:
        cv2.rectangle(img, (int(x0), int(y0)), (int(x1), int(y1)), (0, 212, 255), 2)
        cv2.putText(img, f"{name} {sc:.0%}", (int(x0), max(12, int(y0) - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 212, 255), 1)
    return img


def run_test(model, path):
    img = cv2.imread(path)
    if img is None:
        sys.exit(f"kan {path} niet lezen")
    t = time.time()
    dets = model(img)
    print(f"{len(dets)} detecties in {(time.time() - t) * 1000:.0f} ms")
    for d in dets:
        print(f"  {d[4]:14s} {d[5]:.2f}  ({d[0]:.0f}, {d[1]:.0f}) - ({d[2]:.0f}, {d[3]:.0f})")
    cv2.imwrite("resultaat.jpg", draw(img, dets))
    print("geschreven: resultaat.jpg")


def run_ros(model, topic):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

    class YoloxNode(Node):
        def __init__(self):
            super().__init__("yolox_detections")
            self.pub = self.create_publisher(Detection2DArray, "/detections", 10)
            # queue 1: bij een trage laptop het oudste beeld laten vallen i.p.v. achterop te raken
            self.create_subscription(CompressedImage, topic, self.on_image, 1)
            self.get_logger().info(f"wacht op {topic} ...")

        def on_image(self, msg):
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            out = Detection2DArray(header=msg.header)                     # zelfde stempel als het beeld
            for x0, y0, x1, y1, name, sc in model(img):
                d = Detection2D(header=msg.header)
                d.bbox.center.position.x, d.bbox.center.position.y = (x0 + x1) / 2, (y0 + y1) / 2
                d.bbox.size_x, d.bbox.size_y = x1 - x0, y1 - y0
                h = ObjectHypothesisWithPose()
                h.hypothesis.class_id, h.hypothesis.score = name, sc
                d.results.append(h)
                out.detections.append(d)
            self.pub.publish(out)

    rclpy.init()
    node = YoloxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--test", metavar="FOTO", help="zonder ROS: één foto")
    ap.add_argument("--model", help="pad naar het .onnx-model")
    ap.add_argument("--score", type=float, default=0.35, help="minimale score (0-1)")
    ap.add_argument("--nms", type=float, default=0.45, help="overlapdrempel voor NMS")
    ap.add_argument("--topic", default="/camera/image_raw/compressed")
    a = ap.parse_args()
    model = Yolox(find_model(a.model), a.score, a.nms)
    if a.test:
        run_test(model, a.test)
    else:
        run_ros(model, a.topic)


if __name__ == "__main__":
    main()
