#!/usr/bin/env python3
"""Interactive 6-DOF rviz marker for tuning a static body->child extrinsic by eye.

Drag/rotate the marker directly in rviz's 3D view; each move rebroadcasts the
body->child tf immediately, so any PointCloud2/Camera display using that tf
updates live - no edit-launch-file-and-relaunch loop. Right-click the marker
for a "Save" menu entry that appends the current translation+quaternion to
~output_path (default /tmp/extrinsic_tuner_output.txt) in the same
"x y z qx qy qz qw" order a tf2_ros static_transform_publisher's args expect,
ready to paste into a launch file.
"""
import time

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
)


def makeAxisControl(name: str, x: float, y: float, z: float, w: float, mode: int) -> InteractiveMarkerControl:
    control = InteractiveMarkerControl()
    control.name = name
    control.orientation.x = x
    control.orientation.y = y
    control.orientation.z = z
    control.orientation.w = w
    control.interaction_mode = mode
    return control


class ExtrinsicTuner:
    def __init__(self) -> None:
        self.bodyFrame = rospy.get_param("~body_frame", "body")
        self.childFrame = rospy.get_param("~child_frame", "zed_front_camera")
        self.outputPath = rospy.get_param("~output_path", "/tmp/extrinsic_tuner_output.txt")

        initialTranslation = rospy.get_param("~initial_translation", [0.0, 0.0, 0.0])
        initialRotation = rospy.get_param("~initial_rotation", [0.5, -0.5, 0.5, -0.5])

        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.transform = TransformStamped()
        self.transform.header.frame_id = self.bodyFrame
        self.transform.child_frame_id = self.childFrame
        self.transform.transform.translation.x = initialTranslation[0]
        self.transform.transform.translation.y = initialTranslation[1]
        self.transform.transform.translation.z = initialTranslation[2]
        self.transform.transform.rotation.x = initialRotation[0]
        self.transform.transform.rotation.y = initialRotation[1]
        self.transform.transform.rotation.z = initialRotation[2]
        self.transform.transform.rotation.w = initialRotation[3]

        self.menuHandler = MenuHandler()
        self.menuHandler.insert("Save", callback=self.onSave)

        self.server = InteractiveMarkerServer("extrinsic_tuner")
        self.buildMarker()

        self.publishInterval = 1.0 / rospy.get_param("~publish_rate", 30)

    def buildMarker(self) -> None:
        marker = InteractiveMarker()
        marker.header.frame_id = self.bodyFrame
        marker.name = f"{self.bodyFrame}_to_{self.childFrame}"
        marker.description = f"{self.bodyFrame} -> {self.childFrame}\n(right-click to save)"
        marker.scale = 0.3
        marker.pose.position = self.transform.transform.translation
        marker.pose.orientation = self.transform.transform.rotation

        visual = InteractiveMarkerControl()
        visual.always_visible = True
        visual.orientation.w = 1.0
        axisMarker = Marker()
        axisMarker.type = Marker.ARROW
        axisMarker.pose.orientation.w = 1.0  # Marker() defaults to an invalid
        # all-zero quaternion, not identity - renders nothing (or garbage) if left unset
        axisMarker.scale.x = 0.2
        axisMarker.scale.y = 0.04
        axisMarker.scale.z = 0.04
        axisMarker.color.r = 1.0
        axisMarker.color.a = 1.0
        visual.markers.append(axisMarker)
        marker.controls.append(visual)

        for name, x, y, z, w in [
            ("move_x", 1.0, 0.0, 0.0, 1.0),
            ("rotate_x", 1.0, 0.0, 0.0, 1.0),
            ("move_z", 0.0, 1.0, 0.0, 1.0),
            ("rotate_z", 0.0, 1.0, 0.0, 1.0),
            ("move_y", 0.0, 0.0, 1.0, 1.0),
            ("rotate_y", 0.0, 0.0, 1.0, 1.0),
        ]:
            mode = (
                InteractiveMarkerControl.MOVE_AXIS
                if name.startswith("move")
                else InteractiveMarkerControl.ROTATE_AXIS
            )
            marker.controls.append(makeAxisControl(name, x, y, z, w, mode))

        self.server.insert(marker, feedback_cb=self.onFeedback)
        self.menuHandler.apply(self.server, marker.name)
        self.server.applyChanges()

    def onFeedback(self, feedback) -> None:
        self.transform.transform.translation.x = feedback.pose.position.x
        self.transform.transform.translation.y = feedback.pose.position.y
        self.transform.transform.translation.z = feedback.pose.position.z
        self.transform.transform.rotation = feedback.pose.orientation

    def onSave(self, feedback) -> None:
        t = self.transform.transform.translation
        r = self.transform.transform.rotation
        line = (
            f"{t.x:.6f} {t.y:.6f} {t.z:.6f} "
            f"{r.x:.6f} {r.y:.6f} {r.z:.6f} {r.w:.6f} "
            f"{self.bodyFrame} {self.childFrame}\n"
        )
        with open(self.outputPath, "a") as f:
            f.write(line)
        rospy.loginfo("ExtrinsicTuner: saved to %s: %s", self.outputPath, line.strip())

    def spin(self) -> None:
        # plain wall-clock sleep, not rospy.Rate/rospy.sleep - the bag this is
        # played against loops, and sim time jumping backwards at the loop
        # point makes Rate.sleep() raise ROSTimeMovedBackwardsException; the
        # tf broadcast cadence itself doesn't need to track sim time at all.
        while not rospy.is_shutdown():
            self.transform.header.stamp = rospy.Time.now()
            self.broadcaster.sendTransform(self.transform)
            time.sleep(self.publishInterval)


def main() -> None:
    rospy.init_node("extrinsic_tuner_node")
    ExtrinsicTuner().spin()


if __name__ == "__main__":
    main()
