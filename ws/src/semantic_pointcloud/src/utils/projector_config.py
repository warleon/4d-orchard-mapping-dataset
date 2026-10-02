from dataclasses import dataclass, field

from utils.base_config import BaseConfig
from utils.detection_labeler_config import DetectionLabelerConfig


@dataclass
class ProjectorConfig(BaseConfig):
    calibration_path: str
    half_window: float = 1.5
    world_frame: str = "camera_init"
    camera_frame: str = "spinnaker"
    display_rotation: int = 1
    export_voxel_size: float = 0.02
    image_topic: str = "/semantic_pointcloud/image"
    point_cloud_topic: str = "/semantic_pointcloud/points"
    boxes_topic: str = "/semantic_pointcloud/boxes"
    mask_topic: str = "/semantic_pointcloud/mask"
    detection_labeler: DetectionLabelerConfig = field(
        default_factory=DetectionLabelerConfig
    )
