from dataclasses import dataclass
from typing import Optional


@dataclass
class DetectorConfig:
    model_path: str
    confidence: float = 0.25
    # restrict detection to specific YOLO class indices; None runs all classes
    # the model knows about
    classes: Optional[list[int]] = None
