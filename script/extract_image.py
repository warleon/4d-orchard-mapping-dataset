#!/usr/bin/env python3
"""ROS node that extracts --num_images images, each paired with its
synchronized /Odometry message and its latest /cloud_registered point cloud,
starting --offset seconds after the first image is received. Each sample is
written to its own numbered subdirectory of --output_dir, then the node
shuts itself down.

/Odometry and /cloud_registered only exist while faster-lio's laserMapping
is running -- they aren't recorded in dataset/data.bag -- so this must run
online, alongside mapping_ouster128_with_driver.launch and a bag playback,
e.g.:
    roslaunch launch/mapping_ouster128_with_driver.launch
    rosbag play dataset/data.bag --clock --rate 0.5    # separate terminal
    python3 script/extract_image.py --output_dir ~/out --offset 30 --num_images 20

Play the bag at --rate 1.0 on a loaded machine and laserMapping's cloud
registration falls behind real time (its published /cloud_registered stamps
lag further and further behind /Odometry/image stamps, growing without bound
as the run goes on), so most/all point clouds end up skipped. --rate 0.5 was
enough to keep the image/cloud gap bounded (typically ~0.2-0.4s, since
/cloud_registered publishes at ~10Hz vs ~30Hz for images) on this project's
dev machine, but how much CPU faster-lio actually gets varies run to run
with whatever else is running on the machine -- if you're seeing lots of
"no point cloud" warnings even at --rate 0.5, either lower --rate further or
raise --pc_max_age; occasional skips are expected and non-fatal."""

import argparse
import os

import cv2
import message_filters
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2

from utils import modifyPcMsgFields, pcMsg2NumpyXYZL


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output_dir",
        required=True,
        help="directory to write one numbered subdirectory per extracted image into",
    )
    parser.add_argument(
        "--offset",
        type=float,
        default=0.0,
        help="seconds after the first received image to begin extracting from",
    )
    parser.add_argument(
        "--num_images",
        type=int,
        default=10,
        help="number of images to extract starting at --offset, then the node exits",
    )
    parser.add_argument("--image_topic", default="/spinnaker/image_raw")
    parser.add_argument("--odom_topic", default="/Odometry")
    parser.add_argument("--pc_topic", default="/cloud_registered")
    parser.add_argument(
        "--slop",
        type=float,
        default=0.02,
        help="max seconds between an image and its synchronized odometry message "
        "(same tolerance pc_odom_rgb_sync_node.py uses)",
    )
    parser.add_argument(
        "--pc_max_age",
        type=float,
        default=1.0,
        help="max seconds between an image and the latest received point cloud "
        "for it to still be saved (point clouds are matched to whichever "
        "/cloud_registered message most recently arrived, not looked up by "
        "nearest timestamp, since this runs online). The healthy gap is "
        "usually ~0.2-0.4s (cloud publishes at ~10Hz vs ~30Hz for images) "
        "but spikes with momentary system load even at --rate 0.5, hence the "
        "margin; if misses are still frequent, lower rosbag play's --rate",
    )
    return parser.parse_args()


def odom_to_dict(odom_msg):
    p = odom_msg.pose.pose.position
    o = odom_msg.pose.pose.orientation
    return {
        "stamp": odom_msg.header.stamp.to_sec(),
        "frame_id": odom_msg.header.frame_id,
        "child_frame_id": odom_msg.child_frame_id,
        "position": {"x": p.x, "y": p.y, "z": p.z},
        "orientation": {"x": o.x, "y": o.y, "z": o.z, "w": o.w},
    }


class ImageExtractor:
    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()
        self.start_stamp = None
        self.count = 0
        self.last_pc_msg = None

        os.makedirs(args.output_dir, exist_ok=True)

        rospy.Subscriber(args.pc_topic, PointCloud2, self.pc_cb, queue_size=1)

        odom_sub = message_filters.Subscriber(args.odom_topic, Odometry)
        img_sub = message_filters.Subscriber(args.image_topic, Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [odom_sub, img_sub], queue_size=50, slop=args.slop
        )
        self.sync.registerCallback(self.sync_cb)

    def pc_cb(self, msg):
        self.last_pc_msg = msg

    def sync_cb(self, odom_msg, img_msg):
        t = img_msg.header.stamp.to_sec()
        if self.start_stamp is None:
            self.start_stamp = t
            rospy.loginfo(
                "first image received at t=%.3f, waiting %.1fs before extracting",
                t,
                self.args.offset,
            )
        if t - self.start_stamp < self.args.offset:
            return

        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        image = np.rot90(image)

        sample_dir = os.path.join(self.args.output_dir, f"{self.count:06d}")
        os.makedirs(sample_dir, exist_ok=True)
        cv2.imwrite(os.path.join(sample_dir, "image.png"), image)
        with open(os.path.join(sample_dir, "odom.yaml"), "w") as f:
            yaml.safe_dump(odom_to_dict(odom_msg), f)

        pc_msg = self.last_pc_msg
        if (
            pc_msg is not None
            and abs(pc_msg.header.stamp.to_sec() - t) <= self.args.pc_max_age
        ):
            # world-frame xyz + intensity, same layout pc_odom_rgb_sync_node.py works with
            pc_xyzl = pcMsg2NumpyXYZL(modifyPcMsgFields(pc_msg))
            np.save(os.path.join(sample_dir, "pointcloud.npy"), pc_xyzl)
        else:
            rospy.logwarn(
                "no point cloud within %.2fs of image %d (t=%.3f); skipping pointcloud.npy",
                self.args.pc_max_age,
                self.count,
                t,
            )

        self.count += 1
        rospy.loginfo("[%d/%d] saved %s", self.count, self.args.num_images, sample_dir)

        if self.count >= self.args.num_images:
            rospy.loginfo("extracted %d images, shutting down", self.count)
            rospy.signal_shutdown("done")


def main():
    args = parse_args()
    rospy.init_node("extract_image_node", anonymous=True)
    ImageExtractor(args)
    rospy.spin()


if __name__ == "__main__":
    main()
