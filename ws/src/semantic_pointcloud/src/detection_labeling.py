from typing import NamedTuple

import numpy as np
import rospy
import torch

from camera import Camera
from detector import Detector


class Box3D(NamedTuple):
    center: torch.Tensor  # (3,) camera-space
    extents: torch.Tensor  # (3,) full width/height/depth
    classId: float


class DetectionLabeler:
    def __init__(self, detector: Detector, camera: Camera, device: torch.device) -> None:
        self.detector = detector
        self.camera = camera
        self.device = device

    def label(
        self, points: torch.Tensor, image: np.ndarray
    ) -> tuple[torch.Tensor, list[Box3D]]:
        """Returns (classIds, boxes3D): classIds is -1 for points outside
        every box, else the class id of whichever 3D box it fell inside.
        """
        boxes = self.detector.detect(image)
        rospy.loginfo_throttle(2.0, "DetectionLabeler: %d detection(s) this frame", boxes.shape[0])
        classIds = torch.full((points.shape[0],), -1.0, device=self.device)
        boxes3D: list[Box3D] = []
        if not boxes.numel():
            return classIds, boxes3D

        pixels, inFront = self.camera.toPixels(points)
        cameraSpace = self.camera.toCameraSpace(points)
        depths = cameraSpace[:, 2]

        for box in boxes:
            pixelMask = self.pixelMask(box, pixels, inFront)
            if not pixelMask.any():
                continue

            box3D = self.extrudeToBox3D(box, depths[pixelMask])
            insideMask = self.pointsInBox3D(box3D, cameraSpace)
            classIds[insideMask] = box3D.classId
            boxes3D.append(box3D)

        return classIds, boxes3D

    def pixelMask(
        self, box: torch.Tensor, pixels: torch.Tensor, inFront: torch.Tensor
    ) -> torch.Tensor:
        x1, y1, x2, y2, _conf, _cls = box
        return (
            inFront
            & (pixels[:, 0] >= x1)
            & (pixels[:, 0] <= x2)
            & (pixels[:, 1] >= y1)
            & (pixels[:, 1] <= y2)
        )

    def extrudeToBox3D(self, box: torch.Tensor, candidateDepths: torch.Tensor) -> Box3D:
        # a 2D box alone can't tell a close object from distant background
        # sharing the same pixels apart - place it at its candidates' own
        # median depth, sized to match there, and extrude it into a cuboid
        # as thick as its largest side.
        x1, y1, x2, y2, _conf, cls = box
        referenceDepth = candidateDepths.median()
        width, height = self.camera.pixelSizeToWorldSize(x2 - x1, y2 - y1, referenceDepth)
        thickness = torch.maximum(width, height)
        center = self.camera.pixelToCameraSpace((x1 + x2) / 2, (y1 + y2) / 2, referenceDepth)
        extents = torch.stack([width, height, thickness])
        return Box3D(center=center, extents=extents, classId=float(cls))

    def pointsInBox3D(self, box: Box3D, cameraSpacePoints: torch.Tensor) -> torch.Tensor:
        offset = (cameraSpacePoints - box.center).abs()
        half = box.extents / 2.0
        return (offset <= half).all(dim=1)
