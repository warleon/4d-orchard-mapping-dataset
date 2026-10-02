#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _venv_bootstrap import activate

activate(__file__)

package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(package_root, "src"))

import rospy
from recorder import Recorder
from utils.recorder_config import RecorderConfig


def main():
    rospy.init_node("recorder_node")
    recorder = Recorder(RecorderConfig.load())

    rate = rospy.Rate(rospy.get_param("~poll_rate", 10))
    while not rospy.is_shutdown():
        if recorder.duration > 0.0 and (
            rospy.get_time() - recorder.startTime
        ) >= recorder.offset + recorder.duration:
            rospy.loginfo("recorder_node: offset+duration elapsed, shutting down")
            break

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


if __name__ == "__main__":
    main()
