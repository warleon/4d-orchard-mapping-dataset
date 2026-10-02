from torchvision.transforms.functional import convert_image_dtype

import numpy as np
import rospy
import torch
from cv_bridge import CvBridge
from ros_numpy.point_cloud2 import array_to_pointcloud2, merge_rgb_fields
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from detection_labeling import Box3D
from point_cloud_export import classIdsToColors

_POINT_DTYPE = [
    ("x", np.float32),
    ("y", np.float32),
    ("z", np.float32),
    ("r", np.uint8),
    ("g", np.uint8),
    ("b", np.uint8),
]


class TopicPublisher:
    def __init__(
        self,
        imageTopic: str,
        pointCloudTopic: str,
        boxesTopic: str,
        maskTopic: str,
    ) -> None:
        self.bridge = CvBridge()
        self.imagePub = rospy.Publisher(imageTopic, Image, queue_size=1)
        self.pointCloudPub = rospy.Publisher(pointCloudTopic, PointCloud2, queue_size=1)
        self.boxesPub = rospy.Publisher(boxesTopic, MarkerArray, queue_size=1)
        self.maskPub = rospy.Publisher(maskTopic, Image, queue_size=1)

    def publishImage(self, image: np.ndarray, frameId: str) -> None:
        self._publishImage(self.imagePub, image, frameId)

    def publishMaskImage(self, image: np.ndarray, frameId: str) -> None:
        self._publishImage(self.maskPub, image, frameId)

    def _publishImage(self, publisher: rospy.Publisher, image: np.ndarray, frameId: str) -> None:
        message = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
        message.header.frame_id = frameId
        publisher.publish(message)

    def publishFilteredPoints(
        self, points: torch.Tensor, instanceIds: torch.Tensor, frameId: str
    ) -> None:
        mask = instanceIds >= 0
        filtered = points[mask].cpu().numpy()
        colors = (classIdsToColors(instanceIds[mask])).cpu().numpy()

        cloud = np.zeros(filtered.shape[0], dtype=_POINT_DTYPE)
        cloud["x"], cloud["y"], cloud["z"] = (
            filtered[:, 0],
            filtered[:, 1],
            filtered[:, 2],
        )
        cloud["r"], cloud["g"], cloud["b"] = colors[:, 0], colors[:, 1], colors[:, 2]

        message = array_to_pointcloud2(merge_rgb_fields(cloud), frame_id=frameId)
        self.pointCloudPub.publish(message)

    def publishBoxes(self, boxes: list[Box3D], frameId: str) -> None:
        markers = [self.boxMarker(box, frameId) for box in boxes]
        self.boxesPub.publish(MarkerArray(markers=markers))

    def boxMarker(self, box: Box3D, frameId: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = frameId
        marker.ns = "detections"
        # unique per detection (not per-frame index) so each one gets its own
        # persistent marker slot instead of being overwritten by whatever
        # else lands at the same index next frame
        marker.id = box.instanceId
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            box.center.tolist()
        )
        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = (
            box.orientation.tolist()
        )
        marker.scale.x, marker.scale.y, marker.scale.z = box.extents.tolist()
        color = classIdsToColors(torch.tensor([box.instanceId]))[0].tolist()
        # Marker color fields are floats in [0,1]; the palette is uint8 [0,255]
        marker.color.r, marker.color.g, marker.color.b = (c / 255.0 for c in color)
        marker.color.a = 0.25
        marker.lifetime = rospy.Duration(0)  # forever, to accumulate like the point cloud
        return marker
