import numpy as np
import torch
from ultralytics import YOLO

from utils.detector_config import DetectorConfig


class Detector:
    def __init__(self, config: DetectorConfig) -> None:
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = YOLO(config.model_path)
        self.model.to(self.device)
        self.confidence = config.confidence
        self.classes = config.classes
        # class index -> class name, as reported by the loaded model
        self.classNames = self.model.names

    def detect(self, image: np.ndarray) -> torch.Tensor:
        """Run YOLO inference on an RGB HxWx3 uint8 image.

        Returns:
            (N, 6) tensor of [x1, y1, x2, y2, confidence, class_id] in the
            input image's own pixel coordinate system (empty if nothing was
            detected). This is ultralytics' own packed Boxes.data layout.
        """
        # ultralytics treats numpy array inputs as already being in OpenCV's
        # BGR order (only PIL/file inputs get an implicit RGB->BGR swap), but
        # ours come from cv_bridge as RGB - swap channels to match.
        bgr = np.ascontiguousarray(image[..., ::-1])
        results = self.model.predict(
            bgr,
            conf=self.confidence,
            classes=self.classes,
            device=self.device,
            verbose=False,
        )
        return results[0].boxes.data.to(self.device)
