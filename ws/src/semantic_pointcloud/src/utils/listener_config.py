from dataclasses import dataclass, field


@dataclass
class Topics:
    point_cloud: str = "/cloud_registered"
    rgb_image: str = "/spinnaker/image_raw"
    odometry: str = "/Odometry"


@dataclass
class ListenerConfig:
    topic: Topics = field(default_factory=Topics)
    slop: float = 0.02
    sync_queue_size: int = 50
    half_window: float = 1.5
