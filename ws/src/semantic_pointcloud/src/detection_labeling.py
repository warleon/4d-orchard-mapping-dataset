from typing import NamedTuple

import camera
import numpy as np
import rospy
import torch
import torch.nn.functional as F

from camera import Camera
from detector import Detector
from point_cloud_export import classIdsToColors
from utils.detection_labeler_config import DetectionLabelerConfig


class Box3D(NamedTuple):
    center: torch.Tensor  # (3,) world-space
    extents: torch.Tensor  # (3,) full width/height/depth, camera-local axes
    orientation: torch.Tensor  # (4,) ROS-ordered (x,y,z,w), camera's pose at detection
    classId: float
    instanceId: int  # unique per detection across the whole session, for coloring


class DetectionLabeler:
    def __init__(
        self,
        detector: Detector,
        camera: Camera,
        device: torch.device,
        config: DetectionLabelerConfig = DetectionLabelerConfig(),
    ) -> None:
        self.detector = detector
        self.camera = camera
        self.device = device
        self.clusterGapThreshold = config.cluster_gap_threshold
        self.nextInstanceId = 0

    def label(
        self, points: torch.Tensor, image: torch.Tensor
    ) -> tuple[torch.Tensor, list[Box3D], torch.Tensor]:
        """Returns (instanceIds, boxes3D, maskImage): instanceIds is -1 for
        points outside every box, else the instance id of whichever 3D box
        it fell inside. maskImage is the original image, at its own (H,W,3)
        uint8 resolution, with each detection's YOLO mask alpha-blended and
        its calibration-space box outlined, both in that detection's
        instance color.
        """
        boxes, masks = self.detector.detect(image)
        assert boxes.data.shape[0] == masks.data.shape[0]
        rospy.loginfo(
            "DetectionLabeler: %d detection(s) this frame", masks.data.shape[0]
        )
        rospy.loginfo_throttle(
            5.0,
            "DetectionLabeler: image=%s masks=%s camera=%dx%d",
            tuple(image.shape),
            tuple(masks.data.shape),
            self.camera.width,
            self.camera.height,
        )
        instanceIds = torch.full((points.shape[0],), -1.0, device=self.device)
        depths, inBounds, maskPixelX, maskPixelY = self.projectToMaskSpace(
            points, masks
        )

        maskImage = (image.clamp(0, 1) * 255).byte().permute(1, 2, 0).contiguous()

        boxes3D = []
        for i in range(boxes.data.shape[0]):
            mask = masks.data[i]
            if not mask.any():
                continue

            rospy.loginfo(
                "DetectionLabeler: raw box=%s mask nonzero=%d/%d",
                boxes.data[i, :4].tolist(),
                int(mask.sum()),
                mask.numel(),
            )

            maskHits = self.hitTestMask(mask, maskPixelX, maskPixelY, inBounds)
            if not maskHits.any():
                continue

            calibBox = self.rescaleBoxToCalibration(
                boxes.data[i], masks.data.shape[1:]
            )
            rospy.loginfo(
                "DetectionLabeler: calibBox=%s (image CHW was %s)",
                calibBox[:4].tolist(),
                tuple(image.shape),
            )
            box3D = self.extrudeToBox3D(calibBox, depths[maskHits], self.nextInstanceId)
            self.nextInstanceId += 1
            instanceIds[maskHits] = box3D.instanceId

            color = classIdsToColors(
                torch.tensor([box3D.instanceId], device=self.device)
            )[0]
            self.paintDetection(maskImage, mask, calibBox, color)

            boxes3D.append(box3D)

        return instanceIds, boxes3D, maskImage

    def paintDetection(
        self,
        image: torch.Tensor,
        mask: torch.Tensor,
        calibBox: torch.Tensor,
        color: torch.Tensor,
        alpha: float = 0.5,
        thickness: int = 2,
    ) -> None:
        """Alpha-blends mask (in the detector's own resolution) and draws
        calibBox's outline (in image's resolution) into image, in-place,
        both in color. Pure tensor ops - no cv2 - since image, mask and
        calibBox are already torch tensors and this stays easiest to keep
        consistent (dtype/device) by never leaving torch at all."""
        height, width = image.shape[:2]

        maskUpscaled = (
            F.interpolate(
                mask.float()[None, None], size=(height, width), mode="nearest"
            )[0, 0]
            > 0
        )
        image[maskUpscaled] = (
            image[maskUpscaled].float() * (1 - alpha) + color.float() * alpha
        ).to(torch.uint8)

        x1, y1, x2, y2 = calibBox[:4]
        x1, x2 = sorted((int(x1.clamp(0, width - 1)), int(x2.clamp(0, width - 1))))
        y1, y2 = sorted((int(y1.clamp(0, height - 1)), int(y2.clamp(0, height - 1))))

        top = slice(y1, min(y1 + thickness, y2 + 1))
        bottom = slice(max(y2 - thickness + 1, y1), y2 + 1)
        left = slice(x1, min(x1 + thickness, x2 + 1))
        right = slice(max(x2 - thickness + 1, x1), x2 + 1)

        image[top, x1 : x2 + 1] = color
        image[bottom, x1 : x2 + 1] = color
        image[y1 : y2 + 1, left] = color
        image[y1 : y2 + 1, right] = color

    def projectToMaskSpace(
        self, points: torch.Tensor, masks
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Projects points to the detector's mask frame via normalized
        [0,1) image fractions, since masks/boxes come back sized to whatever
        resized frame Detector.detect used this call, not the calibration
        resolution these points were projected against.

        Returns (depths, inBounds, maskPixelX, maskPixelY).
        """
        pixels = self.camera.toPixels(points)
        depths = self.camera.toCameraSpace(points)[:, 2]

        u = pixels[:, 0] / self.camera.width
        v = pixels[:, 1] / self.camera.height
        inBounds = (depths > 0) & (u >= 0) & (u < 1) & (v >= 0) & (v < 1)

        maskHeight, maskWidth = masks.data.shape[1:]
        maskPixelX = (u * maskWidth).long()
        maskPixelY = (v * maskHeight).long()
        return depths, inBounds, maskPixelX, maskPixelY

    def hitTestMask(
        self,
        mask: torch.Tensor,
        maskPixelX: torch.Tensor,
        maskPixelY: torch.Tensor,
        inBounds: torch.Tensor,
    ) -> torch.Tensor:
        maskHits = torch.zeros_like(inBounds)
        maskHits[inBounds] = mask[maskPixelY[inBounds], maskPixelX[inBounds]].bool()
        return maskHits

    def rescaleBoxToCalibration(
        self, box: torch.Tensor, maskShape: tuple[int, int]
    ) -> torch.Tensor:
        maskHeight, maskWidth = maskShape
        x1, y1, x2, y2, conf, cls = box
        return torch.stack(
            [
                x1 / maskWidth * self.camera.width,
                y1 / maskHeight * self.camera.height,
                x2 / maskWidth * self.camera.width,
                y2 / maskHeight * self.camera.height,
                conf,
                cls,
            ]
        )

    def closestClusterDepth(self, candidateDepths: torch.Tensor) -> torch.Tensor:
        """Mean depth of the nearest-to-camera cluster among candidateDepths.

        LIDAR hits behind a mask tend to bunch into distinct depth clusters
        (the detected object itself, then whatever's behind it) rather than
        spreading continuously, so a plain median can land in the gap
        between them or get pulled toward a denser far cluster. Splitting on
        the first depth gap wider than self.clusterGapThreshold isolates the
        near cluster - the one that's actually the detected object - and
        averaging just that gives a tighter, more representative depth.
        """
        sortedDepths = candidateDepths.sort().values
        gaps = sortedDepths[1:] - sortedDepths[:-1]
        seams = torch.nonzero(gaps > self.clusterGapThreshold, as_tuple=True)[0]
        clusterEnd = int(seams[0]) + 1 if seams.numel() else sortedDepths.shape[0]
        return sortedDepths[:clusterEnd].mean()

    def extrudeToBox3D(
        self, box: torch.Tensor, candidateDepths: torch.Tensor, instanceId: int
    ) -> Box3D:
        # a 2D box alone can't tell a close object from distant background
        # sharing the same pixels apart - place it at its candidates' own
        # nearest depth cluster, sized to match there, and extrude it into a
        # cuboid as thick as its largest side.
        x1, y1, x2, y2, _conf, cls = box
        referenceDepth = self.closestClusterDepth(candidateDepths)
        width, height = self.camera.pixelSizeToWorldSize(
            x2 - x1, y2 - y1, referenceDepth
        )
        thickness = torch.maximum(width, height)
        cameraSpaceCenter = self.camera.pixelToCameraSpace(
            (x1 + x2) / 2, (y1 + y2) / 2, referenceDepth
        )
        worldCenter = self.camera.toWorldSpace(cameraSpaceCenter.unsqueeze(0))[0]
        extents = torch.stack([width, height, thickness])
        return Box3D(
            center=worldCenter,
            extents=extents,
            orientation=self.camera.currentOrientation(),
            classId=float(cls),
            instanceId=instanceId,
        )
