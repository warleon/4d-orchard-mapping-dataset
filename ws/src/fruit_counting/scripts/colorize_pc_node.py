#!/usr/bin/env python3
# Colorizes /cloud_registered by projecting it into the Spinnaker image using
# utils.debugPc2depthImg (the existing lidar->image projection helper), then
# republishes it as an XYZRGB PointCloud2 so it can be viewed directly as a
# colored point cloud in rviz -- instead of (or alongside) rviz's own Camera
# overlay, which only shows the fusion from the live camera viewpoint.
import time

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
import message_filters
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
from fruit_counting import utils


class ColorizePcNode:
    def __init__(self, world_frame, camera_frame, max_rate):
        self.world_frame = world_frame
        self.camera_frame = camera_frame
        self.bridge = CvBridge()
        self.camera_info = None
        # Debayering the full-res image + projecting the whole registered
        # cloud is expensive; running it on every synced pair competes with
        # faster-lio's real-time LIO thread for CPU and visibly degrades its
        # own trajectory/map. Throttle by wall-clock time, independent of
        # sim time/bag rate, so this node only does the expensive work at
        # most max_rate times per second.
        self.min_interval = 1.0 / max_rate
        self.last_processed = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher("colorized_cloud", PointCloud2, queue_size=1)

        rospy.Subscriber("camera_info", CameraInfo, self.camera_info_cb, queue_size=1)

        pc_sub = message_filters.Subscriber("cloud", PointCloud2)
        img_sub = message_filters.Subscriber("image", Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [pc_sub, img_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.sync_cb)

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def sync_cb(self, pc_msg, img_msg):
        if self.camera_info is None:
            return
        now = time.monotonic()
        if now - self.last_processed < self.min_interval:
            return
        self.last_processed = now
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.world_frame, self.camera_frame, pc_msg.header.stamp, rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(5.0, "colorize_pc_node: tf lookup failed: %s", e)
            return

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        T_world_camera = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T_world_camera[:3, 3] = [t.x, t.y, t.z]

        # /cloud_registered's recorded PointField layout doesn't match what
        # ros_numpy expects by default (see CLOUD_REGISTERED_FIELD) -- correct
        # it before reading xyz, same as pc_odom_rgb_sync_node.py does.
        pc_msg = utils.modifyPcMsgFields(pc_msg)
        pc_xyz_camera = utils.transformPcMsg(T_world_camera, pc_msg)
        # only points in front of the camera can land on the image plane
        pc_xyz_camera = pc_xyz_camera[pc_xyz_camera[:, 2] > 0]

        cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        K = np.array(self.camera_info.K).reshape(3, 3)
        D = np.array(self.camera_info.D)
        image_size = (self.camera_info.height, self.camera_info.width)

        _, colorized_pc_msg = utils.debugPc2depthImg(
            pc_xyz_camera, K, D, image_size,
            original_rgb_img=cv_img,
            frame_id=self.camera_frame,
            stamp=pc_msg.header.stamp)

        if colorized_pc_msg is not None:
            self.pub.publish(colorized_pc_msg)


if __name__ == "__main__":
    rospy.init_node("colorize_pc_node")
    ColorizePcNode(
        rospy.get_param("~world_frame", "camera_init"),
        rospy.get_param("~camera_frame", "spinnaker"),
        rospy.get_param("~max_rate", 2.0))
    rospy.spin()
