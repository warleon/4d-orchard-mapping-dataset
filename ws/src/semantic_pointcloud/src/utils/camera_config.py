from dataclasses import dataclass


@dataclass
class CameraConfig:
    camera_model: str
    intrinsics: list[float]
    distortion_model: str
    distortion_coeffs: list[float]
    resolution: list[int]
    rostopic: str
