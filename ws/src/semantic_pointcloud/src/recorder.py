import os

import cv2
import numpy as np
import rospy
import yaml
from nav_msgs.msg import Odometry

from listener import Listener
from utils.recorder_config import RecorderConfig


def odometryToDict(odometry: Odometry) -> dict:
    position = odometry.pose.pose.position
    orientation = odometry.pose.pose.orientation
    return {
        "stamp": odometry.header.stamp.to_sec(),
        "frame_id": odometry.header.frame_id,
        "child_frame_id": odometry.child_frame_id,
        "position": [position.x, position.y, position.z],
        "orientation": [orientation.x, orientation.y, orientation.z, orientation.w],
    }


class Recorder:
    def __init__(self, config: RecorderConfig) -> None:
        self.outputDir = config.output_dir
        os.makedirs(self.outputDir, exist_ok=True)
        self.offset = config.offset
        self.duration = config.duration

        # rospy.get_time() (bag time under use_sim_time) returns 0.0 until the
        # first /clock arrives, then jumps straight to the bag's recorded
        # epoch - wait for that jump so offset/duration are measured against
        # the bag's own timeline instead of the pre-jump 0.0.
        while not rospy.is_shutdown() and rospy.get_time() <= 0.0:
            rospy.sleep(0.05)
        self.startTime = rospy.get_time()

        # subscribe immediately regardless of offset, so the listener's
        # point cloud window is already warm by the time recording starts
        self.listener = Listener(self.onData, config.listener)

    def onData(
        self, odometry: Odometry, image: np.ndarray, point_cloud: np.ndarray
    ) -> None:
        elapsed = odometry.header.stamp.to_sec() - self.startTime
        if elapsed < self.offset:
            return
        if self.duration > 0.0 and elapsed >= self.offset + self.duration:
            return

        sampleDir = os.path.join(self.outputDir, str(odometry.header.stamp.to_nsec()))
        os.makedirs(sampleDir, exist_ok=True)

        with open(os.path.join(sampleDir, "odometry.yaml"), "w") as file:
            yaml.safe_dump(odometryToDict(odometry), file)

        cv2.imwrite(
            os.path.join(sampleDir, "image.png"),
            cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
        )

        np.save(os.path.join(sampleDir, "point_cloud.npy"), point_cloud)

        rospy.loginfo("Recorder: wrote sample to %s", sampleDir)
