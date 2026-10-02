import os
from typing import Callable

import cv2
import numpy as np
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from utils.file_listener_config import FileListenerConfig

Callback = Callable[[Odometry, np.ndarray, np.ndarray], None]


def _odometryFromDict(data: dict) -> Odometry:
    odometry = Odometry()
    odometry.header.stamp = rospy.Time.from_sec(data["stamp"])
    odometry.header.frame_id = data["frame_id"]
    odometry.child_frame_id = data["child_frame_id"]
    (
        odometry.pose.pose.position.x,
        odometry.pose.pose.position.y,
        odometry.pose.pose.position.z,
    ) = data["position"]
    (
        odometry.pose.pose.orientation.x,
        odometry.pose.pose.orientation.y,
        odometry.pose.pose.orientation.z,
        odometry.pose.pose.orientation.w,
    ) = data["orientation"]
    return odometry


class FileListener:
    """Replays samples written by Recorder in place of Listener's live ROS
    subscriptions, invoking the same (odometry, image, point_cloud) callback
    but sourced from disk.

    Each recorded sample's odometry is also re-broadcast over /tf (as
    faster-lio would live), since Camera.updatePose() resolves the camera's
    pose via a tf lookup rather than from the Odometry message directly.

    Unlike Listener, which returns immediately and is driven by ROS's own
    subscription callbacks, the constructor here blocks until every sample
    under dataset_dir has been replayed.
    """

    def __init__(self, callback: Callback, config: FileListenerConfig) -> None:
        self.callback = callback
        self.rate = config.rate
        self.startDelay = config.start_delay
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        self.sampleDirs = sorted(
            (
                os.path.join(config.dataset_dir, name)
                for name in os.listdir(config.dataset_dir)
            ),
            key=lambda path: int(os.path.basename(path)),
        )
        rospy.loginfo(
            "FileListener: found %d sample(s) under %s",
            len(self.sampleDirs),
            config.dataset_dir,
        )

        if rospy.get_param("/use_sim_time", False):
            rospy.logwarn(
                "FileListener: /use_sim_time is true but this launch has no "
                "/clock source - every rospy.sleep() (start_delay, inter-"
                "sample gaps) will hang forever. Set use_sim_time:=false, or "
                "restart roscore if it's a stale param from a previous run."
            )

        self._playAll()

    def _playAll(self) -> None:
        rospy.loginfo(
            "FileListener: waiting start_delay=%.2fs (ros time now=%.3f)",
            self.startDelay,
            rospy.get_time(),
        )
        if self.startDelay > 0.0:
            rospy.sleep(self.startDelay)
        rospy.loginfo(
            "FileListener: start_delay elapsed (ros time now=%.3f), beginning replay",
            rospy.get_time(),
        )

        previousStamp = None
        for index, sampleDir in enumerate(self.sampleDirs):
            if rospy.is_shutdown():
                rospy.loginfo("FileListener: shutdown requested, stopping replay")
                return

            rospy.loginfo(
                "FileListener: loading sample %d/%d from %s",
                index + 1,
                len(self.sampleDirs),
                sampleDir,
            )
            odometry, image, pointCloud = self._loadSample(sampleDir)

            if self.rate > 0.0 and previousStamp is not None:
                gap = (odometry.header.stamp - previousStamp).to_sec() / self.rate
                if gap > 0.0:
                    rospy.loginfo("FileListener: sleeping %.3fs for pacing", gap)
                    rospy.sleep(gap)
            previousStamp = odometry.header.stamp

            self._broadcastTransform(odometry)
            rospy.logdebug(
                "FileListener: broadcast tf %s -> %s at stamp %s",
                odometry.header.frame_id,
                odometry.child_frame_id,
                odometry.header.stamp,
            )
            # tf2's TransformListener populates its buffer asynchronously on
            # receiving /tf, on a background thread - give it a moment before
            # the callback below triggers a lookup for this exact transform.
            rospy.sleep(0.05)

            rospy.loginfo(
                "FileListener: dispatching sample %d/%d to callback",
                index + 1,
                len(self.sampleDirs),
            )
            self.callback(odometry, image, pointCloud)

        rospy.loginfo("FileListener: replay complete (%d sample(s))", len(self.sampleDirs))

    def _loadSample(self, sampleDir: str) -> tuple[Odometry, np.ndarray, np.ndarray]:
        with open(os.path.join(sampleDir, "odometry.yaml")) as file:
            odometry = _odometryFromDict(yaml.safe_load(file))

        image = cv2.cvtColor(
            cv2.imread(os.path.join(sampleDir, "image.png")), cv2.COLOR_BGR2RGB
        )

        pointCloud = np.load(os.path.join(sampleDir, "point_cloud.npy"))

        return odometry, image, pointCloud

    def _broadcastTransform(self, odometry: Odometry) -> None:
        transform = TransformStamped()
        transform.header.stamp = odometry.header.stamp
        transform.header.frame_id = odometry.header.frame_id
        transform.child_frame_id = odometry.child_frame_id
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self.tfBroadcaster.sendTransform(transform)
