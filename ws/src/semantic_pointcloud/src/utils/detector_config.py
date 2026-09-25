from dataclasses import dataclass
from typing import Optional

from utils.base_config import BaseConfig


@dataclass
class DetectorConfig(BaseConfig):
    yolo_model_path: str
    yolo_confidence: float = 0.25
    # restrict detection to specific YOLO class indices; None runs all classes
    # the model knows about
    yolo_classes: Optional[list[int]] = None
