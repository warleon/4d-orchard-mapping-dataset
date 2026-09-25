#!/usr/bin/env python3
from dataclasses import fields
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _venv_bootstrap import activate

activate(__file__)

package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(package_root, "src"))

import rospy
from detector import Detector
from projector import Projector
from utils.detector_config import DetectorConfig
from utils.projector_config import ProjectorConfig


def buildProjectorConfig() -> ProjectorConfig:
    config = ProjectorConfig(calibration_path=rospy.get_param("~calibration_path"))


def buildDetector() -> Detector | None:
    modelPath = rospy.get_param("~yolo_model_path")
    return Detector(
        DetectorConfig(
            model_path=modelPath,
            confidence=rospy.get_param("~yolo_confidence", 0.25),
            classes=rospy.get_param("~yolo_classes", None),
        )
    )


def main():
    rospy.init_node("semantic_pointcloud_node")

    projector = Projector(
        buildDetector(),
        buildProjectorConfig(),
    )

    detectedPointcloudPath = rospy.get_param(
        "~detected_pointcloud_path", "/tmp/semantic_pointcloud_detections.ply"
    )
    exportInterval = rospy.get_param("~export_interval", 30.0)
    maxDuration = rospy.get_param("~max_duration", 0.0)

    # rospy.get_time() (bag time under use_sim_time) returns 0.0 until the
    # first /clock arrives, then jumps straight to the bag's recorded epoch -
    # wait for that jump instead of baselining against the pre-jump 0.0.
    while not rospy.is_shutdown() and rospy.get_time() == 0.0:
        time.sleep(0.05)
    startBagTime = rospy.get_time()
    lastExportTime = time.time()

    rate = rospy.Rate(rospy.get_param("~poll_rate", 10))
    while not rospy.is_shutdown():
        if maxDuration > 0 and rospy.get_time() - startBagTime >= maxDuration:
            rospy.loginfo(
                "semantic_pointcloud_node: max_duration elapsed, shutting down"
            )
            break

        if (
            projector.exporter is not None
            and exportInterval > 0
            and time.time() - lastExportTime >= exportInterval
        ):
            projector.exporter.export(detectedPointcloudPath)
            lastExportTime = time.time()

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break

    if projector.exporter is not None:
        projector.exporter.export(detectedPointcloudPath)


if __name__ == "__main__":
    main()
