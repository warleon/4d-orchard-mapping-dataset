import torch
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
import yaml
from utils.camera_config import CameraConfig
from utils.projector_config import ProjectorConfig
from typing import cast
from pytorch3d.transforms import quaternion_to_matrix
from pytorch3d.renderer.fisheyecameras import FishEyeCameras
from pytorch3d.renderer.points.rasterizer import (
    PointsRasterizer,
    PointsRasterizationSettings,
)
from pytorch3d.renderer.points.renderer import PointsRenderer
from pytorch3d.renderer.points.compositor import AlphaCompositor
from pytorch3d.structures import Pointclouds
from pytorch3d.io import IO
from listener import Listener
from utils.listener_config import ListenerConfig, _queue_size
from detector import Detector
import numpy as np
from torchvision.transforms import v2
from torchvision.utils import draw_bounding_boxes
from typing import Callable, Optional

# deterministic, qualitative palette (tab20) for coloring exported PLY points
# by YOLO class id; cycles via modulo if a model has more classes than this.
_CLASS_PALETTE = (
    torch.tensor(
        [
            [31, 119, 180], [174, 199, 232], [255, 127, 14], [255, 187, 120],
            [44, 160, 44], [152, 223, 138], [214, 39, 40], [255, 152, 150],
            [148, 103, 189], [197, 176, 213], [140, 86, 75], [196, 156, 148],
            [227, 119, 194], [247, 182, 210], [127, 127, 127], [199, 199, 199],
            [188, 189, 34], [219, 219, 141], [23, 190, 207], [158, 218, 229],
        ],
        dtype=torch.float32,
    )
    / 255.0
)


def classIdsToColors(classIds: torch.Tensor) -> torch.Tensor:
    palette = _CLASS_PALETTE.to(classIds.device)
    indices = classIds.long().clamp(min=0) % palette.shape[0]
    return palette[indices]


def voxelDownsample(
    points: torch.Tensor, classIds: torch.Tensor, voxelSize: float
) -> tuple[torch.Tensor, torch.Tensor]:
    """Dedupe points onto a voxel grid: one output point per occupied voxel,
    at the mean position of the points that fell in it, labeled with
    whichever class was most common among them (majority vote - points from
    the same detected object are expected to agree within a small voxel).
    """
    voxelIndex = torch.floor(points / voxelSize).long()
    uniqueVoxels, inverse = torch.unique(voxelIndex, dim=0, return_inverse=True)
    numVoxels = uniqueVoxels.shape[0]

    sums = torch.zeros((numVoxels, 3), dtype=points.dtype, device=points.device)
    sums.index_add_(0, inverse, points)
    counts = torch.zeros(numVoxels, dtype=points.dtype, device=points.device)
    counts.index_add_(0, inverse, torch.ones_like(inverse, dtype=points.dtype))
    meanPoints = sums / counts.unsqueeze(-1)

    uniqueClasses = classIds.unique()
    votes = torch.zeros((numVoxels, uniqueClasses.shape[0]), device=points.device)
    for i, cls in enumerate(uniqueClasses):
        mask = classIds == cls
        votes[:, i].index_add_(
            0, inverse[mask], torch.ones(mask.sum(), device=points.device)
        )
    voxelClasses = uniqueClasses[votes.argmax(dim=1)]

    return meanPoints, voxelClasses


class Projector:
    def __init__(
        self,
        config: ProjectorConfig = ProjectorConfig(""),
        onFrame: Optional[Callable[[torch.Tensor], None]] = None,
        detector: Optional[Detector] = None,
    ) -> None:
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.image: torch.Tensor
        # the ingested cloud, managed as a PyTorch3D Pointclouds object; its
        # single feature channel is the YOLO class id of whichever detection
        # box each point's projected pixel fell inside, or -1 if none/no
        # detector (see processStream). world-frame coordinates.
        self.pointCloud: Pointclouds
        self.camera: FishEyeCameras
        self.renderedImage: torch.Tensor
        # invoked with each new rendered frame from processStream(); runs on
        # whichever thread ROS dispatches the synchronized callback on, so it
        # must not do GUI work directly (see Viewer, which requires the main
        # thread) - hand the frame off instead of rendering it here.
        self.onFrame = onFrame
        # optional; when set, points that fall inside a detected box are
        # colored insideBoxColor instead of pointColor (see processStream/
        # render). Detection runs on the full-resolution raw image for
        # quality, decoupled from the (possibly tiny) render_scale canvas.
        self.detector = detector
        self.detectionBoxes: Optional[torch.Tensor] = None
        # every frame's detected (class_id >= 0) points get appended here,
        # in world-frame coordinates, *before* render()'s max_points
        # subsampling - so the exported cloud isn't missing detections just
        # because the live preview dropped them for render speed. A physical
        # point seen as "detected" across many frames is appended once per
        # frame; exportDetections voxel-downsamples to dedupe this.
        self.accumulatedDetectedPoints: list[torch.Tensor] = []
        self.accumulatedDetectedClasses: list[torch.Tensor] = []
        self.exportVoxelSize = config.export_voxel_size

        self.pointRadius = config.point_radius
        self.pointsPerPixel = config.points_per_pixel
        self.pointColor = torch.tensor(config.point_color, device=self.device)
        self.insideBoxColor = torch.tensor(config.inside_box_color, device=self.device)
        self.boxColor = tuple(config.box_color)
        self.boxLineWidth = config.box_line_width
        self.maxPoints = config.max_points
        self.displayRotation = config.display_rotation % 4

        self.worldFrame = config.world_frame
        self.cameraFrame = config.camera_frame
        # /Odometry gives the body frame's pose, not the camera's - see
        # loadExtrinsics for why a tf lookup is used instead.
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.loadCalibration(config.calibration_path)

        # NDC intrinsics are resolution independent, so the render canvas can
        # be much smaller than the calibration's native resolution without
        # affecting projection accuracy - only the output size.
        self.nativeHeight, self.nativeWidth = (int(v) for v in self.camera.image_size[0])
        self.renderHeight = max(1, round(self.nativeHeight * config.render_scale))
        self.renderWidth = max(1, round(self.nativeWidth * config.render_scale))

        raster_settings = PointsRasterizationSettings(
            image_size=(self.renderHeight, self.renderWidth),
            radius=self.pointRadius,
            points_per_pixel=self.pointsPerPixel,
        )
        # PointsRenderer.forward reads its splat-weight radius from
        # rasterizer.raster_settings directly, ignoring any raster_settings passed
        # as a call kwarg - it must be set here, not just handed to render() per call.
        self.rasterizer = PointsRasterizer(
            cameras=self.camera, raster_settings=raster_settings
        )
        self.compositor = AlphaCompositor()
        self.renderer = PointsRenderer(
            rasterizer=self.rasterizer, compositor=self.compositor
        )

        # /cloud_registered is the current scan re-registered into the world
        # frame, not the accumulated map - Listener vstacks the last
        # queue_size.point_cloud of those. On CPU, PyTorch3D's rasterizer
        # cost scales with point count, so the default queue (50 scans, up
        # to millions of points for a 128-beam LiDAR) makes rendering
        # impractically slow; keep the accumulation window short here.
        listenerConfig = ListenerConfig(
            queue_size=_queue_size(point_cloud=config.point_cloud_queue_size),
            max_point_cloud_age=config.max_point_cloud_age,
        )
        self.listener = Listener(self.processStream, listenerConfig)

    def loadCalibration(self, calibrationPath):
        with open(calibrationPath) as file:
            calibrationData = cast(dict, yaml.safe_load(file))
            calibration = CameraConfig(**calibrationData["cam0"])
            fx, fy, u0, v0 = calibration.intrinsics
            # Kalibr's own convention is resolution: [width, height] (verified
            # against kalibr_camera_calibration/CameraUtils.py, which builds
            # np.zeros((resolution[1], resolution[0]))).
            width, height = calibration.resolution
            k1, k2, p1, p2 = calibration.distortion_coeffs

            # FishEyeCameras.in_ndc() is hard-coded True, so PointsRasterizer treats
            # whatever transform_points() returns as final NDC coordinates - there is
            # no automatic pixel/screen -> NDC conversion for this camera type (unlike
            # e.g. PerspectiveCameras). The pixel-space intrinsics from the calibration
            # file must therefore be pre-converted to PyTorch3D's NDC convention here
            # (centered on the image, smaller side spanning [-1, 1]); the negation is
            # required to match the rasterizer's NDC axis orientation, verified
            # empirically - dropping it mirrors the render relative to the source image.
            scale = min(height, width) / 2.0
            fx, fy = -fx / scale, -fy / scale
            u0, v0 = (width / 2.0 - u0) / scale, (height / 2.0 - v0) / scale

            # FishEyeCameras always evaluates a 6-term radial polynomial; pad any
            # shorter calibration (here: a 2-term radtan-style k1, k2) with zeros.
            radialParams = [k1, k2] + [0.0] * 4

            self.camera = FishEyeCameras(
                focal_length=torch.tensor([[fx, fy]], device=self.device),
                principal_point=torch.tensor([[u0, v0]], device=self.device),
                radial_params=torch.tensor([radialParams], device=self.device),
                tangential_params=torch.tensor([[p1, p2]], device=self.device),
                use_thin_prism=False,
                image_size=torch.tensor([[height, width]], device=self.device),
                world_coordinates=True,
                device=self.device,
            )

    def loadExtrinsics(self, odometry: Odometry) -> bool:
        # /Odometry gives the body (LiDAR/IMU) frame's pose, not the camera's
        # - faster-lio also broadcasts that same pose as a world_frame->
        # body_frame tf, so a single world_frame->camera_frame lookup here
        # (tf2 chains it through body_frame automatically) gives the camera's
        # actual world pose directly, without manually composing transforms.
        try:
            worldToCamera = self.tfBuffer.lookup_transform(
                self.worldFrame,
                self.cameraFrame,
                odometry.header.stamp,
                rospy.Duration(0.2),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ) as e:
            rospy.logwarn_throttle(
                5.0,
                "Projector: tf lookup %s->%s failed: %s",
                self.worldFrame,
                self.cameraFrame,
                e,
            )
            return False

        translation = worldToCamera.transform.translation
        rotation = worldToCamera.transform.rotation
        cameraCenter = torch.tensor(
            [[translation.x, translation.y, translation.z]], device=self.device
        )
        R = quaternion_to_matrix(
            torch.tensor(
                [[rotation.w, rotation.x, rotation.y, rotation.z]], device=self.device
            )
        )

        # PyTorch3D's row-vector convention maps X_view = X_world @ R + T, which
        # makes T the camera center expressed in the rotated view frame
        # (T = -cameraCenter @ R), not the raw world-frame position.
        T = -torch.bmm(cameraCenter.unsqueeze(1), R).squeeze(1)

        self.camera.R = R
        self.camera.T = T
        return True

    def projectToRenderPixels(
        self, points: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Project world-space points to pixel coordinates in the render canvas.

        Mirrors the NDC->pixel conversion PointsRasterizer applies internally
        (same sign/scale as loadCalibration's pixel->NDC conversion, inverted
        and re-scaled to the render canvas instead of the native resolution -
        NDC is resolution independent, so only the target size differs).

        Returns (pixels, inFront): inFront must be applied by the caller -
        the fisheye x/z, y/z projection formula doesn't check the sign of z,
        so a point behind (or beside) the camera can still land on a
        plausible-looking pixel and accidentally alias into a detection box.
        PointsRasterizer avoids this by separately clipping on real
        camera-space depth before splatting (see loadExtrinsics/render); this
        path has to do the same check explicitly.
        """
        viewPoints = self.camera.get_world_to_view_transform(
            R=self.camera.R, T=self.camera.T
        ).transform_points(points)
        inFront = viewPoints[:, 2] > 0

        ndc = self.camera.transform_points(points)[..., :2]
        scale = min(self.renderHeight, self.renderWidth) / 2.0
        pixelsX = self.renderWidth / 2.0 - ndc[:, 0] * scale
        pixelsY = self.renderHeight / 2.0 - ndc[:, 1] * scale
        return torch.stack([pixelsX, pixelsY], dim=-1), inFront

    def processStream(
        self, odometry: Odometry, image: np.ndarray, point_cloud: np.ndarray
    ):
        if not self.loadExtrinsics(odometry):
            return None

        points = torch.from_numpy(point_cloud).to(device=self.device, dtype=torch.float32)
        classIds = torch.full((points.shape[0],), -1.0, device=self.device)

        if self.detector is not None:
            boxes = self.detector.detect(image)
            if boxes.numel():
                # detect() runs on the full-resolution raw image, but points
                # get projected into the (possibly much smaller) render
                # canvas - rescale box coordinates (not confidence/class_id,
                # the trailing two columns) to match that space.
                nativeHeight, nativeWidth = image.shape[:2]
                scale = torch.tensor(
                    [self.renderWidth / nativeWidth, self.renderHeight / nativeHeight],
                    device=self.device,
                ).repeat(2)
                boxes = torch.cat([boxes[:, :4] * scale, boxes[:, 4:]], dim=1)

                pixels, inFront = self.projectToRenderPixels(points)
                for x1, y1, x2, y2, _conf, cls in boxes:
                    inside = (
                        inFront
                        & (pixels[:, 0] >= x1)
                        & (pixels[:, 0] <= x2)
                        & (pixels[:, 1] >= y1)
                        & (pixels[:, 1] <= y2)
                    )
                    classIds[inside] = cls
            self.detectionBoxes = boxes
            rospy.loginfo_throttle(
                2.0, "Projector: %d YOLO detection(s) this frame", boxes.shape[0]
            )

            detectedMask = classIds >= 0
            if detectedMask.any():
                self.accumulatedDetectedPoints.append(points[detectedMask].cpu())
                self.accumulatedDetectedClasses.append(classIds[detectedMask].cpu())

        self.pointCloud = Pointclouds(points=[points], features=[classIds.unsqueeze(-1)])

        self.image = v2.functional.to_image(image)
        self.image = v2.functional.to_dtype(
            self.image, dtype=torch.float32, scale=True
        ).to(self.device)
        self.image = v2.functional.resize(
            self.image, [self.renderHeight, self.renderWidth]
        )

        self.renderedImage = self.render()

        if self.onFrame is not None:
            self.onFrame(self.renderedImage)

        return self.renderedImage

    def render(self) -> torch.Tensor:
        points = self.pointCloud.points_packed()
        classIds = self.pointCloud.features_packed()[:, 0]

        if points.shape[0] > self.maxPoints:
            indices = torch.randperm(points.shape[0], device=self.device)[: self.maxPoints]
            points = points[indices]
            classIds = classIds[indices]

        numPoints = points.shape[0]
        colors = self.pointColor.expand(numPoints, 3).clone()
        colors[classIds >= 0] = self.insideBoxColor

        coverage = torch.ones((numPoints, 1), device=self.device)
        features = torch.cat([colors, coverage], dim=1)

        splatPointClouds = Pointclouds(points=[points], features=[features])

        rendered = self.renderer(splatPointClouds)[0]
        splatColor, splatAlpha = rendered[..., :3], rendered[..., 3:4].clamp(0.0, 1.0)

        background = self.image.permute(1, 2, 0)
        composited = splatColor * splatAlpha + background * (1 - splatAlpha)
        composited = composited.permute(2, 0, 1).clamp(0.0, 1.0)

        # drawn here (pre-rotation, same space detectionBoxes/points already
        # live in) so the outline rotates along with everything else below,
        # rather than needing its own separate rotation.
        if self.detectionBoxes is not None and self.detectionBoxes.numel():
            composited = draw_bounding_boxes(
                composited,
                self.detectionBoxes[:, :4],
                colors=self.boxColor,
                width=self.boxLineWidth,
            )

        # display-only: the camera is mounted rotated, so the already-
        # composited (image + points) output needs reorienting for viewing -
        # rotating post-composite keeps the overlay aligned with the image.
        if self.displayRotation:
            composited = torch.rot90(composited, k=self.displayRotation, dims=(1, 2))

        return composited

    def exportDetections(self, path: str) -> None:
        """Write every accumulated detected (class_id >= 0) point, across the
        whole session, to a PLY file - voxel-downsampled to dedupe points
        from the same physical object detected repeatedly across frames, and
        colored by class_id via _CLASS_PALETTE so the classes stay
        distinguishable when opened in a mesh viewer. Intended to be called
        once, on shutdown.
        """
        if not self.accumulatedDetectedPoints:
            rospy.logwarn(
                "Projector: no detected points accumulated, skipping export to %s",
                path,
            )
            return

        points = torch.cat(self.accumulatedDetectedPoints, dim=0)
        classIds = torch.cat(self.accumulatedDetectedClasses, dim=0)
        rawCount = points.shape[0]
        points, classIds = voxelDownsample(points, classIds, self.exportVoxelSize)
        colors = classIdsToColors(classIds)

        pointCloud = Pointclouds(points=[points], features=[colors])
        IO().save_pointcloud(pointCloud, path, colors_as_uint8=True)
        rospy.loginfo(
            "Projector: exported %d detected points (%d classes, deduped from "
            "%d with %gm voxels) to %s",
            points.shape[0],
            classIds.unique().numel(),
            rawCount,
            self.exportVoxelSize,
            path,
        )
