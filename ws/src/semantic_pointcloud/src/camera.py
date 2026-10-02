from typing import cast

import rospy
import tf2_ros
import torch
import yaml
from nav_msgs.msg import Odometry
from pytorch3d.renderer.fisheyecameras import FishEyeCameras
from pytorch3d.transforms import matrix_to_quaternion, quaternion_to_matrix

from utils.camera_config import CameraConfig


def rotateCalibration(
    width: float,
    height: float,
    fx: float,
    fy: float,
    u0: float,
    v0: float,
    p1: float,
    p2: float,
    quarterTurns: int,
) -> tuple[float, float, float, float, float, float, float, float]:
    """Rotates a pinhole/fisheye calibration to match a quarterTurns 90-
    degree CCW rotation of the image itself (np.rot90/torch.rot90
    convention). Applying this once here - instead of rotating pixel
    coordinates or images at various points downstream - lets every image
    handed to this Camera be expected pre-rotated the same way, with
    toPixels/pixelToCameraSpace/etc. then needing no rotation awareness at
    all; they just see a camera natively mounted in that orientation.

    k1,k2 (radial distortion) are rotation-invariant (they're a function of
    r^2 = x^2+y^2 alone) and don't need adjusting. p1,p2 (tangential) and
    the principal point/focal/resolution do; derived by rotating the
    Brown-Conrady tangential-distortion terms and the pixel-index mapping
    np.rot90 itself applies, one 90-degree step at a time.
    """
    for _ in range(quarterTurns % 4):
        u0, v0 = v0, width - u0
        width, height = height, width
        fx, fy = fy, fx
        p1, p2 = -p2, p1
    return width, height, fx, fy, u0, v0, p1, p2


def loadFisheyeCamera(calibrationPath: str, device: torch.device, quarterTurns: int = 0):
    with open(calibrationPath) as file:
        calibrationData = cast(dict, yaml.safe_load(file))
    calibration = CameraConfig(**calibrationData["cam0"])
    fx, fy, u0, v0 = calibration.intrinsics
    # despite the [width, height] convention kalibr documents, this
    # dataset's calibration files list [height, width] - confirmed against
    # the recorded images themselves (2448w x 2048h) and against the
    # intrinsics' own principal point (u0~=1252 is half of 2448, not 2048)
    height, width = calibration.resolution
    k1, k2, p1, p2 = calibration.distortion_coeffs

    width, height, fx, fy, u0, v0, p1, p2 = rotateCalibration(
        width, height, fx, fy, u0, v0, p1, p2, quarterTurns
    )

    # FishEyeCameras.in_ndc() is hard-coded True, so its raw pixel-space
    # intrinsics must be pre-converted to NDC here; the negation matches the
    # rasterizer's NDC axis orientation (verified empirically - dropping it
    # mirrors the projection).
    scale = min(height, width) / 2.0
    fx, fy = -fx / scale, -fy / scale
    u0, v0 = (width / 2.0 - u0) / scale, (height / 2.0 - v0) / scale
    radialParams = [k1, k2, 0.0, 0.0, 0.0, 0.0]  # always evaluated as 6 terms

    return (
        FishEyeCameras(
            focal_length=torch.tensor([[fx, fy]], device=device),
            principal_point=torch.tensor([[u0, v0]], device=device),
            radial_params=torch.tensor([radialParams], device=device),
            tangential_params=torch.tensor([[p1, p2]], device=device),
            use_thin_prism=False,
            image_size=torch.tensor([[height, width]], device=device),
            world_coordinates=True,
            device=device,
        ),
        height,
        width,
        scale,
    )


class Camera:
    def __init__(
        self,
        calibrationPath: str,
        worldFrame: str,
        cameraFrame: str,
        device: torch.device,
        quarterTurns: int = 0,
    ) -> None:
        self.device = device
        self.worldFrame = worldFrame
        self.cameraFrame = cameraFrame
        # default cache is 10s; CPU-bound YOLO inference can push the
        # effective processing lag (on top of the half_window buffering)
        # past that, so lookups for the buffered odometry stamp start
        # missing the tf history.
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(60))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # every image this Camera is ever handed is expected pre-rotated by
        # quarterTurns (see rotateCalibration) - the caller rotates once,
        # up front, and this Camera's own notion of pixel space already
        # matches that, so nothing downstream rotates again
        self.fisheye, self.height, self.width, self.scale = loadFisheyeCamera(
            calibrationPath, device, quarterTurns
        )

    def updatePose(self, odometry: Odometry) -> bool:
        # /Odometry gives the body frame's pose, not the camera's; faster-lio
        # also broadcasts that same pose as world_frame->body, so this single
        # lookup (chained through body by tf2) resolves the camera's own pose.
        try:
            transform = self.tfBuffer.lookup_transform(
                self.worldFrame,
                self.cameraFrame,
                odometry.header.stamp,
            )
        except Exception as error:
            rospy.logerr_throttle(
                5.0,
                "Camera: tf lookup %s->%s failed: %s",
                self.worldFrame,
                self.cameraFrame,
                error,
            )
            return False

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        center = torch.tensor(
            [[translation.x, translation.y, translation.z]], device=self.device
        )
        R = quaternion_to_matrix(
            torch.tensor(
                [[rotation.w, rotation.x, rotation.y, rotation.z]], device=self.device
            )
        )
        # row-vector convention: X_view = X_world @ R + T, so T is the camera
        # center re-expressed in the rotated view frame, not the raw position.
        T = -torch.bmm(center.unsqueeze(1), R).squeeze(1)

        self.fisheye.R = R
        self.fisheye.T = T
        return True

    def toCameraSpace(self, points: torch.Tensor) -> torch.Tensor:
        return self.fisheye.get_world_to_view_transform(
            R=self.fisheye.R, T=self.fisheye.T
        ).transform_points(points)

    def toWorldSpace(self, points: torch.Tensor) -> torch.Tensor:
        return (
            self.fisheye.get_world_to_view_transform(R=self.fisheye.R, T=self.fisheye.T)
            .inverse()
            .transform_points(points)
        )

    def currentOrientation(self) -> torch.Tensor:
        """Camera's current world-space orientation as a ROS-ordered
        (x, y, z, w) quaternion, for anchoring a camera-local direction to a
        world-frame marker."""
        w, x, y, z = matrix_to_quaternion(self.fisheye.R[0])
        return torch.stack([x, y, z, w])

    def toPixels(self, points: torch.Tensor):
        ndc = self.fisheye.transform_points(points)[..., :2]
        pixelsX = self.width / 2.0 - ndc[:, 0] * self.scale
        pixelsY = self.height / 2.0 - ndc[:, 1] * self.scale
        return torch.stack([pixelsX, pixelsY], dim=-1)

    def pixelSizeToWorldSize(
        self, pixelWidth: torch.Tensor, pixelHeight: torch.Tensor, depth: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        focalX, focalY = self.fisheye.focal[0].abs()
        worldWidth = depth * (pixelWidth / self.scale) / focalX
        worldHeight = depth * (pixelHeight / self.scale) / focalY
        return worldWidth, worldHeight

    def pixelToCameraSpace(
        self, pixelX: torch.Tensor, pixelY: torch.Tensor, depth: torch.Tensor
    ) -> torch.Tensor:
        ndcX = (self.width / 2.0 - pixelX) / self.scale
        ndcY = (self.height / 2.0 - pixelY) / self.scale
        focalX, focalY = self.fisheye.focal[0]
        principalX, principalY = self.fisheye.principal_point[0]
        x = depth * (ndcX - principalX) / focalX
        y = depth * (ndcY - principalY) / focalY
        return torch.stack([x, y, depth])
