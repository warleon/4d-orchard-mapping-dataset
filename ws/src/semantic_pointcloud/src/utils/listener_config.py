from dataclasses import dataclass, field


@dataclass
class _topics:
    point_cloud: str = "/cloud_registered"
    rgb_image: str = "/spinnaker/image_raw"
    odometry: str = "/Odometry"


@dataclass
class _queue_size:
    point_cloud: int = 50
    syncronizer: int = 50


@dataclass
class ListenerConfig:
    topic: _topics = field(default_factory=_topics)
    queue_size: _queue_size = field(default_factory=_queue_size)
    slop: float = 0.02
    # queue_size.point_cloud bounds accumulation by message count, not time;
    # when rendering falls behind the scan rate, "last N scans" can span far
    # more wall/sim time than intended, pulling in scans that no longer
    # match the current camera pose. This bounds accumulation by age (sec)
    # as well, relative to each synced frame's own timestamp.
    max_point_cloud_age: float = 1.0
