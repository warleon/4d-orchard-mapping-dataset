#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image


def main():
    rospy.init_node("zed_decompress_node")
    bridge = CvBridge()
    pub = rospy.Publisher("~out", Image, queue_size=1)

    def callback(msg: CompressedImage):
        image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        outMsg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        outMsg.header = msg.header
        pub.publish(outMsg)

    rospy.Subscriber("~in", CompressedImage, callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
