from typing import cast

import rospy
import tf2_ros
import torch
import yaml
from nav_msgs.msg import Odometry
from pytorch3d.renderer.fisheyecameras import FishEyeCameras
from pytorch3d.transforms import quaternion_to_matrix

from utils.camera_config import CameraConfig


def loadFisheyeCamera(calibrationPath: str, device: torch.device):
    with open(calibrationPath) as file:
        calibrationData = cast(dict, yaml.safe_load(file))
    calibration = CameraConfig(**calibrationData["cam0"])
    fx, fy, u0, v0 = calibration.intrinsics
    width, height = calibration.resolution  # kalibr's own convention
    k1, k2, p1, p2 = calibration.distortion_coeffs

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
    ) -> None:
        self.device = device
        self.worldFrame = worldFrame
        self.cameraFrame = cameraFrame
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.fisheye, self.height, self.width, self.scale = loadFisheyeCamera(
            calibrationPath, device
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
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ) as error:
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

    def toPixels(self, points: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Returns (pixels, inFront).

        inFront must be applied by the caller: the fisheye x/z, y/z formula
        doesn't check the sign of z, so a point behind the camera can still
        land on a plausible pixel.
        """
        inFront = self.toCameraSpace(points)[:, 2] > 0
        ndc = self.fisheye.transform_points(points)[..., :2]
        pixelsX = self.width / 2.0 - ndc[:, 0] * self.scale
        pixelsY = self.height / 2.0 - ndc[:, 1] * self.scale
        return torch.stack([pixelsX, pixelsY], dim=-1), inFront

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
