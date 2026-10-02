from typing import Callable, Optional

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

from torchvision.transforms.functional import to_tensor, convert_image_dtype

StreamCallback = Callable[[Odometry, np.ndarray, np.ndarray], None]


def rotateImage(image: np.ndarray, quarterTurns: int = 0) -> np.ndarray:
    # cv_bridge needs contiguous memory to compute the message's row
    # stride; rot90 returns a strided view, not a copy
    return np.ascontiguousarray(np.rot90(image, k=quarterTurns)) if quarterTurns else image


class Projector:
    def __init__(
        self,
        detector: Detector,
        config: ProjectorConfig = ProjectorConfig(""),
        # defaults to live ROS subscriptions via Listener; pass e.g.
        # `lambda callback: FileListener(callback, FileListenerConfig(...))`
        # to replay from disk instead (see semantic_pointcloud_file_node.py)
        listenerFactory: Optional[Callable[[StreamCallback], object]] = None,
    ) -> None:
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.displayRotation = config.display_rotation % 4

        # the camera's own calibration is rotated to match displayRotation
        # (see Camera/rotateCalibration), so every image this Projector
        # hands it - rotated once, below, right where it comes in - is
        # already in the orientation the camera natively expects. Nothing
        # downstream (YOLO, the mask image, lidar-to-pixel projection) needs
        # to know about rotation at all.
        self.camera = Camera(
            config.calibration_path,
            config.world_frame,
            config.camera_frame,
            self.device,
            self.displayRotation,
        )
        self.labeler = DetectionLabeler(
            detector, self.camera, self.device, config.detection_labeler
        )
        self.publisher = TopicPublisher(
            config.image_topic,
            config.point_cloud_topic,
            config.boxes_topic,
            config.mask_topic,
        )

        if listenerFactory is None:
            listenerFactory = lambda callback: Listener(
                callback, ListenerConfig(half_window=config.half_window)
            )
        self.listener = listenerFactory(self.processStream)

    def processStream(
        self, odometry: Odometry, image: np.ndarray, point_cloud: np.ndarray
    ) -> None:
        if not self.camera.updatePose(odometry):
            return

        points = torch.from_numpy(point_cloud).to(
            device=self.device, dtype=torch.float32
        )

        rotatedImage = rotateImage(image, self.displayRotation)
        torch_image = to_tensor(rotatedImage)
        torch_image = convert_image_dtype(torch_image)

        instanceIds, boxes3D, maskImage = self.labelPoints(points, torch_image)

        self.publisher.publishImage(rotatedImage, self.camera.cameraFrame)
        self.publisher.publishFilteredPoints(
            points, instanceIds, self.camera.worldFrame
        )
        self.publisher.publishBoxes(boxes3D, self.camera.worldFrame)
        self.publisher.publishMaskImage(
            maskImage.cpu().numpy(), self.camera.cameraFrame
        )

    def labelPoints(
        self, points: torch.Tensor, image: torch.Tensor
    ) -> tuple[torch.Tensor, list[Box3D], torch.Tensor]:

        instanceIds, boxes3D, maskImage = self.labeler.label(points, image)
        return instanceIds, boxes3D, maskImage
