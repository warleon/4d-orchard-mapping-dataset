#!/usr/bin/env python3
import os
import queue
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _venv_bootstrap import activate

activate(__file__)

package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(package_root, "src"))

import rospy
import torch
from projector import Projector
from viewer import Viewer
from detector import Detector
from utils.projector_config import ProjectorConfig
from utils.detector_config import DetectorConfig


def main():
    rospy.init_node("semantic_pointcloud_node")

    calibrationPath = rospy.get_param("~calibration_path")
    config = ProjectorConfig(
        calibration_path=calibrationPath,
        point_radius=rospy.get_param("~point_radius", 0.01),
        points_per_pixel=rospy.get_param("~points_per_pixel", 8),
        point_color=rospy.get_param("~point_color", [1.0, 0.0, 0.0]),
        inside_box_color=rospy.get_param("~inside_box_color", [0.0, 0.0, 1.0]),
        box_color=rospy.get_param("~box_color", [0, 255, 0]),
        box_line_width=rospy.get_param("~box_line_width", 2),
        # CPU has no binned point rasterizer in PyTorch3D (CUDA-only), so
        # render cost is render_scale^2 * max_points; raise both freely if
        # running on a CUDA GPU.
        render_scale=rospy.get_param("~render_scale", 0.08),
        max_points=rospy.get_param("~max_points", 5000),
        point_cloud_queue_size=rospy.get_param("~point_cloud_queue_size", 5),
        max_point_cloud_age=rospy.get_param("~max_point_cloud_age", 1.0),
        world_frame=rospy.get_param("~world_frame", "camera_init"),
        camera_frame=rospy.get_param("~camera_frame", "spinnaker"),
        display_rotation=rospy.get_param("~display_rotation", 1),
        export_voxel_size=rospy.get_param("~export_voxel_size", 0.02),
    )

    # optional: only enabled if a model path is given, since no model ships
    # with this package - see models/ at the repo root.
    yoloModelPath = rospy.get_param("~yolo_model_path", "")
    detector = None
    if yoloModelPath:
        detector = Detector(
            DetectorConfig(
                model_path=yoloModelPath,
                confidence=rospy.get_param("~yolo_confidence", 0.25),
                classes=rospy.get_param("~yolo_classes", None),
            )
        )

    # Projector's onFrame runs on whichever thread ROS dispatches the
    # synchronized callback on; hand the frame off through a single-slot
    # queue instead of touching the viewer there, since the viewer's GUI
    # toolkit must only be driven from the main thread.
    frameQueue: "queue.Queue[torch.Tensor]" = queue.Queue(maxsize=1)

    def onFrame(image: torch.Tensor):
        if frameQueue.full():
            frameQueue.get_nowait()
        frameQueue.put_nowait(image)

    projector = Projector(config, onFrame=onFrame, detector=detector)
    viewer = Viewer("semantic pointcloud")

    detectedPointcloudPath = rospy.get_param(
        "~detected_pointcloud_path", "/tmp/semantic_pointcloud_detections.ply"
    )
    # PLY's header declares the vertex count up front, so it can't be
    # appended to like a log file - instead, periodically re-export
    # (overwriting the same file) so a crash or a Ctrl+C caught mid-render
    # loses at most one interval's worth of detections, not the whole
    # session. 0 disables this and only exports once, at shutdown.
    exportInterval = rospy.get_param("~export_interval", 30.0)
    lastExportTime = rospy.get_time()

    # 0 (default) runs until the bag/topics stop or the process is killed;
    # set to bound the session and exit on its own, without needing Ctrl+C -
    # avoids interrupting a live render/export mid-computation (torch's
    # worker threads don't always tolerate a forced signal-driven unwind,
    # which can abort the process instead of exiting cleanly).
    maxDuration = rospy.get_param("~max_duration", 0.0)
    startTime = rospy.get_time()

    rate = rospy.Rate(rospy.get_param("~display_rate", 30))
    while not rospy.is_shutdown():
        if maxDuration > 0 and rospy.get_time() - startTime >= maxDuration:
            rospy.loginfo("semantic_pointcloud_node: max_duration elapsed, shutting down")
            break

        try:
            image = frameQueue.get_nowait()
            viewer.show(image)
        except queue.Empty:
            pass

        if (
            detector is not None
            and exportInterval > 0
            and rospy.get_time() - lastExportTime >= exportInterval
        ):
            projector.exportDetections(detectedPointcloudPath)
            lastExportTime = rospy.get_time()

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            # raised by sleep() itself on shutdown (e.g. Ctrl+C) - without
            # catching this here, it propagates out of main() and skips the
            # cleanup/export below entirely.
            break

    viewer.close()

    if detector is not None:
        projector.exportDetections(detectedPointcloudPath)


if __name__ == "__main__":
    main()
