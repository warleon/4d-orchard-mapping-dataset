from collections import deque
from typing import Callable, List

import numpy as np
import rospy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber as SynchronizedSubscriber
from nav_msgs.msg import Odometry
from rospy import Subscriber
from sensor_msgs.msg import Image, PointCloud2

from utils.listener_config import ListenerConfig

Callback = Callable[[Odometry, np.ndarray, np.ndarray], None]

from timed_queue import TimedPointCloudQueue


class Listener:
    def __init__(
        self, callback: Callback, config: ListenerConfig = ListenerConfig()
    ) -> None:
        self.callback = callback
        self.bridge = CvBridge()
        self.halfWindow = config.half_window

        self.pointCloudWindow = TimedPointCloudQueue(
            config.half_window, config.half_window
        )
        self.pendingPairs: deque[tuple[Odometry, Image]] = deque()

        Subscriber(config.topic.point_cloud, PointCloud2, self.onPointCloud)
        odometrySub = SynchronizedSubscriber(config.topic.odometry, Odometry)
        imageSub = SynchronizedSubscriber(config.topic.rgb_image, Image)
        synchronizer = ApproximateTimeSynchronizer(
            [odometrySub, imageSub], config.sync_queue_size, config.slop
        )
        synchronizer.registerCallback(self.onSyncedPair)

    def onPointCloud(self, message: PointCloud2) -> None:
        self.pointCloudWindow.add(message)
        self.tryDispatch()

    def onSyncedPair(self, odometry: Odometry, image: Image) -> None:
        self.pendingPairs.append((odometry, image))
        self.tryDispatch()

    def tryDispatch(self) -> None:
        if self.pointCloudWindow.isFull() and len(self.pendingPairs):
            odom, imageMsg = self.pendingPairs.popleft()
            image = self.bridge.imgmsg_to_cv2(imageMsg, "rgb8")

            points = self.pointCloudWindow.around(odom)
            if not len(points):
                rospy.logerr_throttle(
                    5.0,
                    "Listener: no point clouds found around time %d",
                    odom.header.stamp.to_sec(),
                )
                return

            pointCloud = np.vstack(points)
            self.callback(odom, image, pointCloud)
