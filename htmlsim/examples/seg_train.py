#!/usr/bin/env python3
"""Een klein segmentatienetwerk (U-Net) trainen op een dataset van dataset_opnemen.py en exporteren naar ONNX.

    python3 seg_train.py dataset/ [--epochs 30] [--out seg_model.onnx]

Daarna draait seg_infer.py het model met enkel OpenCV (cv2.dnn), ook in de container zonder PyTorch.
Nodig om te trainen: PyTorch (CPU volstaat, een GPU gaat sneller). Bijvoorbeeld in de container:
    python3 -m venv ~/venv && ~/venv/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu
    ~/venv/bin/python seg_train.py dataset/
of op een eigen pc / Google Colab. Invoer 160 × 120 BGR, 0-1; uitvoer per pixel een score per label-id.
"""
import argparse
import glob
import json
import os
import random

import cv2
import numpy as np
import torch
import torch.nn as nn


def block(i, o):
    return nn.Sequential(nn.Conv2d(i, o, 3, padding=1), nn.BatchNorm2d(o), nn.ReLU(inplace=True),
                         nn.Conv2d(o, o, 3, padding=1), nn.BatchNorm2d(o), nn.ReLU(inplace=True))


class TinyUNet(nn.Module):
    """3 niveaus (16-32-64 kanalen, 120 × 160 → 30 × 40), ~120 000 parameters."""
    def __init__(self, k):
        super().__init__()
        self.e1, self.e2, self.e3 = block(3, 16), block(16, 32), block(32, 64)
        self.pool, self.up = nn.MaxPool2d(2), nn.Upsample(scale_factor=2, mode="nearest")
        self.d2, self.d1, self.out = block(64 + 32, 32), block(32 + 16, 16), nn.Conv2d(16, k, 1)

    def forward(self, x):
        a = self.e1(x); b = self.e2(self.pool(a)); c = self.e3(self.pool(b))
        b = self.d2(torch.cat([self.up(c), b], 1)); a = self.d1(torch.cat([self.up(b), a], 1))
        return self.out(a)


def load(folder):
    xs, ys = [], []
    for f in sorted(glob.glob(os.path.join(folder, "beeld_*.png"))):
        lab = f.replace("beeld_", "label_")
        if os.path.exists(lab):
            xs.append(cv2.imread(f).astype(np.float32) / 255); ys.append(cv2.imread(lab, cv2.IMREAD_GRAYSCALE).astype(np.int64))
    return np.stack(xs).transpose(0, 3, 1, 2), np.stack(ys)


def miou(pred, y, k):
    per = {}
    for c in range(k):
        u = ((pred == c) | (y == c)).sum()
        if u: per[c] = ((pred == c) & (y == c)).sum() / u
    return per


def main():
    ap = argparse.ArgumentParser(description="U-Net trainen op een htmlsim-dataset")
    ap.add_argument("map"); ap.add_argument("--epochs", type=int, default=30); ap.add_argument("--out", default="seg_model.onnx")
    a = ap.parse_args()
    names = json.load(open(os.path.join(a.map, "labels.json"))) if os.path.exists(os.path.join(a.map, "labels.json")) else {}
    X, Y = load(a.map); k = max(int(Y.max()) + 1, len(names))
    idx = list(range(len(X))); random.seed(1); random.shuffle(idx); nv = max(1, len(X) // 10)
    va, tr = idx[:nv], idx[nv:]
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"{len(tr)} train, {len(va)} validatie, {k} klassen, {dev}")
    net = TinyUNet(k).to(dev); opt = torch.optim.Adam(net.parameters(), 2e-3); lossf = nn.CrossEntropyLoss()
    Xt, Yt = torch.tensor(X), torch.tensor(Y)
    for ep in range(a.epochs):
        net.train(); random.shuffle(tr); tot = 0
        for i in range(0, len(tr), 16):
            b = tr[i:i + 16]; x, y = Xt[b].to(dev), Yt[b].to(dev)
            if random.random() < 0.5: x, y = x.flip(3), y.flip(2)          # spiegelen als augmentatie
            opt.zero_grad(); loss = lossf(net(x), y); loss.backward(); opt.step(); tot += loss.item() * len(b)
        net.eval()
        with torch.no_grad(): pred = net(Xt[va].to(dev)).argmax(1).cpu().numpy()
        per = miou(pred, Y[va], k)
        print(f"epoch {ep + 1:3d}  loss {tot / len(tr):.3f}  mIoU validatie {np.mean(list(per.values())) * 100:.0f} %")
    print("per klasse:", ", ".join(f"{names.get(str(c), c)} {v * 100:.0f}" for c, v in sorted(per.items(), key=lambda t: -t[1])))
    net.cpu().eval()
    torch.onnx.export(net, torch.zeros(1, 3, 120, 160), a.out, input_names=["beeld"], output_names=["scores"], opset_version=11)
    json.dump({"classes": k, "names": names, "size": [160, 120]}, open(os.path.splitext(a.out)[0] + ".json", "w"))
    print("geschreven:", a.out)


if __name__ == "__main__":
    main()
