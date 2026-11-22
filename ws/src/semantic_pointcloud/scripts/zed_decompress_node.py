#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def main():
    rospy.init_node("zed_decompress_node")
    bridge = CvBridge()
    # the ZED driver's own recorded frame_id (e.g. "front_left_camera_optical_frame")
    # has no corresponding tf frame in this rig - only body->zed_front_camera (or
    # whatever camera_frame is configured to) exists, so anything that resolves a
    # frame via tf (e.g. rviz's Camera display) needs it rewritten to match.
    frameId = rospy.get_param("~frame_id", "zed_front_camera")

    imagePub = rospy.Publisher("~out", Image, queue_size=1)

    def onImage(msg: CompressedImage):
        image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        outMsg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        outMsg.header = msg.header
        outMsg.header.frame_id = frameId
        imagePub.publish(outMsg)

    rospy.Subscriber("~in", CompressedImage, onImage, queue_size=1)

    cameraInfoInTopic = rospy.get_param("~camera_info_in", "")
    if cameraInfoInTopic:
        cameraInfoPub = rospy.Publisher("~camera_info_out", CameraInfo, queue_size=1)

        def onCameraInfo(msg: CameraInfo):
            msg.header.frame_id = frameId
            cameraInfoPub.publish(msg)

        rospy.Subscriber(cameraInfoInTopic, CameraInfo, onCameraInfo, queue_size=1)

    rospy.spin()


if __name__ == "__main__":
    main()
