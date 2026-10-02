from dataclasses import dataclass

from utils.base_config import BaseConfig


@dataclass
class DetectionLabelerConfig(BaseConfig):
    # max depth gap (meters) within a single real-world cluster of candidate
    # points - a mask's candidates often span a near cluster (the actual
    # detected object) plus a far one (background canopy/trunk behind it),
    # and anything wider than this is treated as a seam between two clusters
    cluster_gap_threshold: float = 0.15
