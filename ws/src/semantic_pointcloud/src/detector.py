from typing import Tuple

from ultralytics.engine.results import Boxes, Masks

import torch
import torch.nn.functional as F
from ultralytics import YOLO

from utils.detector_config import DetectorConfig


def roundToStride(size: int, stride: int = 32) -> int:
    return max(stride, (size // stride) * stride)


class Detector:
    def __init__(self, config: DetectorConfig, device: str) -> None:
        self.device = device
        self.model = YOLO(config.yolo_model_path)
        self.model.to(self.device)
        self.confidence = config.yolo_confidence
        self.classes = config.yolo_classes
        # class index -> class name, as reported by the loaded model
        self.classNames = self.model.names

    def detect(self, image: torch.Tensor) -> Tuple[Boxes, Masks]:
        """Run YOLO inference on an RGB CxHxW float image.

        The image is resized to the nearest multiple of the model's stride
        (32) in each dimension, since YOLO refuses tensor inputs otherwise.
        Both the returned boxes and masks are in that resized frame, not the
        original image's pixel coordinates.

        Returns tuple:
            Boxes:
                (N, 6) tensor of [x1, y1, x2, y2, confidence, class_id] in the
                resized frame's pixel coordinate system (empty if nothing was
                detected). This is ultralytics' own packed Boxes.data layout.
            Masks:
                (N,H,W) binary masks of the detected objects, H/W matching the
                resized frame.
        """
        _, height, width = image.shape
        resized = F.interpolate(
            image.unsqueeze(0),
            size=(roundToStride(height), roundToStride(width)),
            mode="bilinear",
            align_corners=False,
        )

        results = self.model.predict(
            resized,
            conf=self.confidence,
            classes=self.classes,
            device=self.device,
            verbose=False,
        )
        return results[0].boxes, results[0].masks
