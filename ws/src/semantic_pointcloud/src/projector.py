from typing import Optional

import numpy as np
import torch
from nav_msgs.msg import Odometry

from camera import Camera
from detection_labeling import Box3D, DetectionLabeler
from detector import Detector
from listener import Listener
from point_cloud_export import DetectionExporter
from topic_publisher import TopicPublisher
from utils.listener_config import ListenerConfig
from utils.projector_config import ProjectorConfig


def rotateImage(image: np.ndarray, quarterTurns: int) -> np.ndarray:
    return (
        np.ascontiguousarray(np.rot90(image, k=quarterTurns)) if quarterTurns else image
    )


class Projector:
    def __init__(
        self,
        detector: Detector,
        config: ProjectorConfig = ProjectorConfig(""),
    ) -> None:
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.camera = Camera(
            config.calibration_path,
            config.world_frame,
            config.camera_frame,
            self.device,
        )
        self.labeler = DetectionLabeler(detector, self.camera, self.device)
        self.exporter = DetectionExporter(config.export_voxel_size)
        self.publisher = TopicPublisher(
            config.image_topic, config.point_cloud_topic, config.boxes_topic
        )
        self.displayRotation = config.display_rotation % 4

        self.listener = Listener(
            self.processStream, ListenerConfig(half_window=config.half_window)
        )

    def processStream(
        self, odometry: Odometry, image: np.ndarray, point_cloud: np.ndarray
    ) -> None:
        if not self.camera.updatePose(odometry):
            return

        points = torch.from_numpy(point_cloud).to(
            device=self.device, dtype=torch.float32
        )
        classIds, boxes3D = self.labelPoints(points, image)
        stamp = odometry.header.stamp

        self.publisher.publishImage(
            rotateImage(image, self.displayRotation), stamp, self.camera.cameraFrame
        )
        self.publisher.publishFilteredPoints(
            points, classIds, stamp, self.camera.worldFrame
        )
        self.publisher.publishBoxes(boxes3D, stamp, self.camera.cameraFrame)

    def labelPoints(
        self, points: torch.Tensor, image: np.ndarray
    ) -> tuple[torch.Tensor, list[Box3D]]:

        classIds, boxes3D = self.labeler.label(points, image)
        if self.exporter is not None:
            self.exporter.add(points, classIds)
        return classIds, boxes3D
