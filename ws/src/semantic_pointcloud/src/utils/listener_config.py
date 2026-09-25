from dataclasses import dataclass, field

from utils.base_config import BaseConfig


@dataclass
class Topics(BaseConfig):
    point_cloud: str = "/cloud_registered"
    rgb_image: str = "/spinnaker/image_raw"
    odometry: str = "/Odometry"


@dataclass
class ListenerConfig(BaseConfig):
    topic: Topics = field(default_factory=Topics)
    slop: float = 0.02
    sync_queue_size: int = 50
    half_window: float = 1.5
