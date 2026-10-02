from dataclasses import dataclass, field

from utils.base_config import BaseConfig
from utils.listener_config import ListenerConfig


@dataclass
class RecorderConfig(BaseConfig):
    output_dir: str
    # seconds into the bag's own timeline before the recorder starts writing
    # samples; 0 means start immediately
    offset: float = 0.0
    # seconds after offset to keep recording before the node shuts down;
    # 0 means keep recording until the bag ends or the node is killed
    duration: float = 0.0
    listener: ListenerConfig = field(default_factory=ListenerConfig)
