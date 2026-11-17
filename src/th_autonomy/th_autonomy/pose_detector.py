"""Finds people and their body joints in a BGR image with RTMO (OpenMMLab, Apache-2.0).

RTMO is a one-stage multi-person pose model: one pass gives every person's box
and 17 COCO keypoints, so its cost does not grow with the number of people.
tools/fetch_person_model.py fetches it (not in git). The ONNX export takes a
640x640 BGR image, resized to fit into the top-left corner with gray (114)
padding and 0-255 values, and returns, after its own NMS:
  dets       1 x N x 5       x1, y1, x2, y2, score (input pixels)
  keypoints  1 x N x 17 x 3  x, y, score (input pixels)

It runs on the GPU with TensorRT when available (trt_runner.py; the first start
builds and caches an engine, which takes a few minutes), else on the CPU with
ONNX Runtime, which is much slower.
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

INPUT_SIZE = 640

# COCO keypoint order
NOSE, L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW, L_WRIST, R_WRIST = 0, 5, 6, 7, 8, 9, 10
L_HIP, R_HIP, L_KNEE, R_KNEE, L_ANKLE, R_ANKLE = 11, 12, 13, 14, 15, 16
SKELETON = ((5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12), (11, 12),
            (11, 13), (13, 15), (12, 14), (14, 16), (0, 5), (0, 6))


@dataclass
class Person:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    keypoints: np.ndarray = None  # 17 x 3: x, y, score (image pixels)

    @property
    def width(self):
        return self.x2 - self.x1

    @property
    def height(self):
        return self.y2 - self.y1

    @property
    def area(self):
        return max(0.0, self.width) * max(0.0, self.height)

    def as_int(self):
        return int(self.x1), int(self.y1), int(self.x2), int(self.y2)

    def joint(self, index, min_score):
        """(x, y) of a keypoint seen with at least `min_score`, else None."""
        if self.keypoints is None:
            return None
        x, y, s = self.keypoints[index]
        return (float(x), float(y)) if s >= min_score else None


def preprocess(bgr):
    """The model's input tensor and the scale from image to input pixels."""
    scale = min(INPUT_SIZE / bgr.shape[0], INPUT_SIZE / bgr.shape[1])
    resized = cv2.resize(bgr, (round(bgr.shape[1] * scale), round(bgr.shape[0] * scale)),
                         interpolation=cv2.INTER_LINEAR)
    padded = np.full((INPUT_SIZE, INPUT_SIZE, 3), 114, np.uint8)
    padded[:resized.shape[0], :resized.shape[1]] = resized
    return np.ascontiguousarray(padded.transpose(2, 0, 1)[None], np.float32), scale


def postprocess(dets, keypoints, scale, score_threshold, width, height):
    """People above `score_threshold` in image pixels, clipped to the image, largest first."""
    dets = np.asarray(dets).reshape(-1, 5)
    keypoints = np.asarray(keypoints).reshape(-1, 17, 3)
    people = []
    for det, kps in zip(dets, keypoints):
        if det[4] < score_threshold:
            continue
        x1, y1, x2, y2 = (float(v) / scale for v in det[:4])
        kps = kps.astype(np.float32).copy()
        kps[:, :2] /= scale
        person = Person(max(0.0, x1), max(0.0, y1), min(float(width), x2), min(float(height), y2),
                        float(det[4]), kps)
        if person.area > 0.0:
            people.append(person)
    people.sort(key=lambda p: p.area, reverse=True)
    return people


class OnnxRuntimeRunner:
    """CPU fallback (or any ONNX Runtime provider)."""

    backend = 'onnxruntime'

    def __init__(self, model_path, threads):
        import onnxruntime  # only the person follower needs it
        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = threads
        self.session = onnxruntime.InferenceSession(
            str(model_path), options, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name

    def __call__(self, tensor):
        dets, keypoints = self.session.run(['dets', 'keypoints'], {self.input_name: tensor})
        return dets, keypoints


def make_runner(model_path, backend='auto', threads=2, log=print):
    path = Path(model_path)
    if not path.is_file():
        raise FileNotFoundError(
            f'No pose model at {path.resolve()}; run tools/fetch_person_model.py')
    if backend not in ('auto', 'tensorrt', 'onnxruntime'):
        raise ValueError("backend must be 'auto', 'tensorrt' or 'onnxruntime'")
    if backend in ('auto', 'tensorrt'):
        try:
            from th_autonomy.trt_runner import TensorRTRunner
            return TensorRTRunner(path, log=log)
        except Exception as error:  # no TensorRT or no GPU in this container
            if backend == 'tensorrt':
                raise
            log(f'TensorRT unavailable ({error}); running the pose model on the CPU (slow)')
    return OnnxRuntimeRunner(path, threads)


class PoseDetector:

    def __init__(self, model_path, score_threshold=0.3, backend='auto', threads=2, log=print):
        self.runner = make_runner(model_path, backend, threads, log)
        self.score_threshold = score_threshold

    @property
    def backend(self):
        return self.runner.backend

    def detect(self, bgr):
        tensor, scale = preprocess(bgr)
        dets, keypoints = self.runner(tensor)
        return postprocess(dets, keypoints, scale, self.score_threshold, bgr.shape[1], bgr.shape[0])
