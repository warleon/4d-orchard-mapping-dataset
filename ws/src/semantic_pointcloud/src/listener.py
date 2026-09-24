from typing import Callable

from rospy import Subscriber
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber as SynchronizedSubscriber
from collections import deque
from utils.listener_config import ListenerConfig
from utils.point_cloud_message import point_cloud_message_to_numpy
import numpy as np
from cv_bridge import CvBridge


class Listener:
    type _callback_t = Callable[[Odometry, np.ndarray, np.ndarray], None]

    def __init__(
        self, callback: _callback_t, config: ListenerConfig = ListenerConfig()
    ) -> None:
        self.callback = callback
        self.bridge = CvBridge()
        self.point_cloud_listener = Subscriber(
            config.topic.point_cloud,
            PointCloud2,
            self.point_cloud_listener_callback,
        )

        self.odometry_listener = SynchronizedSubscriber(config.topic.odometry, Odometry)
        self.image_listener = SynchronizedSubscriber(config.topic.rgb_image, Image)
        self.point_cloud_queue = deque(maxlen=config.queue_size.point_cloud)
        self.max_point_cloud_age = config.max_point_cloud_age

        self.syncronizer = ApproximateTimeSynchronizer(
            [self.odometry_listener, self.image_listener],
            config.queue_size.syncronizer,
            config.slop,
        )
        self.syncronizer.registerCallback(self.syncronizer_callback)

    def point_cloud_listener_callback(self, point_cloud_message: PointCloud2):
        xyz = point_cloud_message_to_numpy(point_cloud_message)
        self.point_cloud_queue.append((point_cloud_message.header.stamp, xyz))

    def syncronizer_callback(self, odometry_message: Odometry, image_message: Image):
        # point_cloud_queue is bounded by count, not time - when rendering is
        # slower than the scan rate (e.g. PyTorch3D's CPU point rasterizer,
        # which can take ~1s/frame), the "last N" scans by the time a frame
        # actually gets rendered can span a much wider time window than this
        # synced pair, so scans that no longer match the current camera pose
        # get projected too, showing up as points in the wrong place (e.g.
        # the sky). Bound by recency relative to this frame as well.
        accumulated_point_cloud = [
            xyz
            for stamp, xyz in self.point_cloud_queue
            if abs((image_message.header.stamp - stamp).to_sec())
            <= self.max_point_cloud_age
        ]
        if not len(accumulated_point_cloud):
            return

        accumulated_point_cloud = np.vstack(accumulated_point_cloud)

        image = self.bridge.imgmsg_to_cv2(image_message, "rgb8")

        self.callback(odometry_message, image, accumulated_point_cloud)
