#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _venv_bootstrap import activate

activate(__file__)

package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(package_root, "src"))

import rospy
from detector import Detector
from file_listener import FileListener
from projector import Projector
from utils.detector_config import DetectorConfig
from utils.file_listener_config import FileListenerConfig
from utils.projector_config import ProjectorConfig


def buildDetector() -> Detector:
    return Detector(DetectorConfig.load(), "cpu")


def main():
    rospy.init_node("semantic_pointcloud_file_node")

    fileListenerConfig = FileListenerConfig.load()

    # FileListener's constructor blocks until every recorded sample has been
    # replayed through the pipeline, so by the time this returns there's
    # nothing left to do but keep the node alive for rviz/subscribers.
    Projector(
        buildDetector(),
        ProjectorConfig.load(),
        listenerFactory=lambda callback: FileListener(callback, fileListenerConfig),
    )

    rospy.spin()


if __name__ == "__main__":
    main()
