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
    def __init__(self, imageTopic: str, pointCloudTopic: str, boxesTopic: str) -> None:
        self.bridge = CvBridge()
        self.imagePub = rospy.Publisher(imageTopic, Image, queue_size=1)
        self.pointCloudPub = rospy.Publisher(pointCloudTopic, PointCloud2, queue_size=1)
        self.boxesPub = rospy.Publisher(boxesTopic, MarkerArray, queue_size=1)

    def publishImage(self, image: np.ndarray, stamp, frameId: str) -> None:
        message = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
        message.header.stamp = stamp
        message.header.frame_id = frameId
        self.imagePub.publish(message)

    def publishFilteredPoints(
        self, points: torch.Tensor, classIds: torch.Tensor, stamp, frameId: str
    ) -> None:
        mask = classIds >= 0
        filtered = points[mask].cpu().numpy()
        colors = (classIdsToColors(classIds[mask]) * 255).byte().cpu().numpy()

        cloud = np.zeros(filtered.shape[0], dtype=_POINT_DTYPE)
        cloud["x"], cloud["y"], cloud["z"] = (
            filtered[:, 0],
            filtered[:, 1],
            filtered[:, 2],
        )
        cloud["r"], cloud["g"], cloud["b"] = colors[:, 0], colors[:, 1], colors[:, 2]

        message = array_to_pointcloud2(
            merge_rgb_fields(cloud), stamp=stamp, frame_id=frameId
        )
        self.pointCloudPub.publish(message)

    def publishBoxes(self, boxes: list[Box3D], stamp, frameId: str) -> None:
        markers = [
            self.boxMarker(box, i, stamp, frameId) for i, box in enumerate(boxes)
        ]
        self.boxesPub.publish(MarkerArray(markers=markers))

    def boxMarker(self, box: Box3D, index: int, stamp, frameId: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = frameId
        marker.header.stamp = stamp
        marker.ns = "detections"
        marker.id = index
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            box.center.tolist()
        )
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = box.extents.tolist()
        color = classIdsToColors(torch.tensor([box.classId]))[0].tolist()
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 0.25
        # auto-expires if a newer MarkerArray doesn't refresh it in time,
        # instead of needing an explicit DELETEALL when detections stop
        marker.lifetime = rospy.Duration(1)
        return marker
