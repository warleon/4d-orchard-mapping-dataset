#!/usr/bin/env python3
# The Spinnaker driver never fills in /spinnaker/camera_info (K/D/height/width
# all come through zeroed), so tools that need real intrinsics -- rviz's Camera
# display, image rectification, lidar->image projection -- can't use it as-is.
# This republishes a proper CameraInfo, timestamped/framed to match each
# incoming image, built from a kalibr cam0 intrinsics yaml (see dataset/*.yaml,
# produced by kalibr_calibrate_cameras).
import rospy
from sensor_msgs.msg import Image, CameraInfo
from fruit_counting import utils


class CameraInfoPublisher:
    def __init__(self, calib_file):
        cam0 = utils.load_params(calib_file)["cam0"]
        fx, fy, cx, cy = cam0["intrinsics"]
        self.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.D = list(cam0["distortion_coeffs"])
        # kalibr's "radtan" distortion model is OpenCV's plumb_bob, same
        # [k1, k2, p1, p2] order.
        self.distortion_model = "plumb_bob"

        self.pub = rospy.Publisher("camera_info", CameraInfo, queue_size=10)
        self.sub = rospy.Subscriber("image", Image, self.image_callback, queue_size=10)

    def image_callback(self, img_msg):
        info = CameraInfo()
        info.header = img_msg.header
        info.height = img_msg.height
        info.width = img_msg.width
        info.distortion_model = self.distortion_model
        info.D = self.D
        info.K = self.K
        info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.P = [
            self.K[0], 0.0, self.K[2], 0.0,
            0.0, self.K[4], self.K[5], 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self.pub.publish(info)


if __name__ == "__main__":
    rospy.init_node("camera_info_publisher")
    CameraInfoPublisher(rospy.get_param("~calib_file"))
    rospy.spin()
