from dataclasses import dataclass
from typing import Optional

from utils.base_config import BaseConfig


@dataclass
class DetectorConfig(BaseConfig):
    yolo_model_path: str
    yolo_confidence: float = 0.75
    # restrict detection to specific YOLO class indices; None runs all classes
    # the model knows about
    yolo_classes: Optional[list[int]] = None
    # None: single pass over the whole (stride-rounded) image, as before.
    # Set to e.g. 320 for small objects (grape berries) that a single pass
    # over a ~1920x1080 frame would shrink past recognizability - runs a full
    # sliding-window grid of tile_size x tile_size tiles instead, each at
    # native pixel density, merged back via per-class NMS (see detector.py).
    tile_size: Optional[int] = None
    tile_overlap: float = 0.1
    tile_nms_iou: float = 0.5
