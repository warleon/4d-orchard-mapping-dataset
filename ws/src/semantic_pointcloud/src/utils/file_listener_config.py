from dataclasses import dataclass

from utils.base_config import BaseConfig


@dataclass
class FileListenerConfig(BaseConfig):
    dataset_dir: str
    # playback speed multiplier applied to the gaps between consecutive
    # recorded samples' own timestamps; <= 0 replays as fast as possible
    rate: float = 1.0
    # seconds to wait before replay starts, so rviz (or any other late
    # subscriber, including its own tf2 buffer) has time to connect first -
    # with no live bag padding this out, a short recorded dataset can finish
    # playing before a just-started rviz has subscribed to anything
    start_delay: float = 5.0
