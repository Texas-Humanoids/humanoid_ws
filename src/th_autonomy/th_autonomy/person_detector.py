"""Finds people in a BGR image with YOLOX-Nano (Apache-2.0) on ONNX Runtime's CPU.

The model (3.7 MB) is fetched by tools/fetch_person_model.py; it is not in git.
It takes a 416x416 BGR image, letterboxed into the top-left corner with gray
(114) padding and 0-255 values, and returns 3549 raw predictions, one per grid
cell at strides 8, 16 and 32. Each is (x, y, w, h) relative to its cell, then the
objectness and 80 COCO class scores (already sigmoided). Class 0 is person.

On the Jetson Orin Nano's CPU one frame takes about 130 ms (about 7 Hz).
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

INPUT_SIZE = 416
STRIDES = (8, 16, 32)
PERSON = 0


@dataclass
class PersonBox:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float

    @property
    def width(self):
        return self.x2 - self.x1

    @property
    def height(self):
        return self.y2 - self.y1

    @property
    def area(self):
        return self.width * self.height

    def as_int(self):
        return int(self.x1), int(self.y1), int(self.x2), int(self.y2)


def _grids():
    grids, strides = [], []
    for stride in STRIDES:
        size = INPUT_SIZE // stride
        ys, xs = np.meshgrid(np.arange(size), np.arange(size), indexing='ij')
        grids.append(np.stack((xs, ys), axis=-1).reshape(-1, 2))
        strides.append(np.full((size * size, 1), stride))
    return np.concatenate(grids).astype(np.float32), np.concatenate(strides).astype(np.float32)


def letterbox(image):
    """The image scaled into the model's input, and the scale used."""
    scale = min(INPUT_SIZE / image.shape[0], INPUT_SIZE / image.shape[1])
    resized = cv2.resize(image, (int(image.shape[1] * scale), int(image.shape[0] * scale)),
                         interpolation=cv2.INTER_LINEAR)
    padded = np.full((INPUT_SIZE, INPUT_SIZE, 3), 114, np.uint8)
    padded[:resized.shape[0], :resized.shape[1]] = resized
    return padded, scale


def decode(output, scale, score_threshold, nms_threshold):
    """People from the raw model output (3549 x 85), in original image pixels."""
    grid, stride = _GRID
    xy = (output[:, :2] + grid) * stride
    wh = np.exp(np.clip(output[:, 2:4], -10, 10)) * stride
    scores = output[:, 4] * output[:, 5 + PERSON]
    keep = scores > score_threshold
    if not keep.any():
        return []
    xy, wh, scores = xy[keep] / scale, wh[keep] / scale, scores[keep]
    boxes = np.concatenate((xy - wh / 2, wh), axis=1)  # x, y, w, h for NMSBoxes
    picked = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), score_threshold, nms_threshold)
    people = []
    for i in np.array(picked).flatten():
        x, y, w, h = boxes[i]
        people.append(PersonBox(float(x), float(y), float(x + w), float(y + h), float(scores[i])))
    people.sort(key=lambda b: b.area, reverse=True)
    return people


_GRID = _grids()


class PersonDetector:

    def __init__(self, model_path, score_threshold=0.45, nms_threshold=0.45, threads=4):
        import onnxruntime  # only the person follower needs it
        path = Path(model_path)
        if not path.is_file():
            raise FileNotFoundError(
                f'No person model at {path.resolve()}; run tools/fetch_person_model.py')
        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = threads
        self.session = onnxruntime.InferenceSession(
            str(path), options, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.score_threshold = score_threshold
        self.nms_threshold = nms_threshold

    def detect(self, bgr):
        """People in the image, largest first, clipped to it."""
        padded, scale = letterbox(bgr)
        blob = padded.transpose(2, 0, 1)[None].astype(np.float32)
        output = self.session.run(None, {self.input_name: blob})[0][0]
        people = decode(output, scale, self.score_threshold, self.nms_threshold)
        h, w = bgr.shape[:2]
        for b in people:
            b.x1, b.x2 = max(0.0, b.x1), min(float(w), b.x2)
            b.y1, b.y2 = max(0.0, b.y1), min(float(h), b.y2)
        return people
