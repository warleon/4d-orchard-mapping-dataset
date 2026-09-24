from dataclasses import dataclass, field


@dataclass
class ProjectorConfig:
    calibration_path: str
    point_radius: float = 0.01
    points_per_pixel: int = 8
    point_color: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0])
    # color for points whose projected pixel falls inside a YOLO detection
    # box (see Projector's detector param); irrelevant if no detector is set
    inside_box_color: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    # outline drawn for each YOLO detection box on the final rendered image;
    # 0-255 RGB, unlike point_color/inside_box_color's 0-1 range (matches
    # torchvision.utils.draw_bounding_boxes' expected color format)
    box_color: list[int] = field(default_factory=lambda: [0, 255, 0])
    box_line_width: int = 2
    # PyTorch3D has no binned/coarse-to-fine point rasterizer on CPU (only
    # CUDA) - it always falls back to a brute-force O(points x pixels) scan
    # there, so cost is the *product* of these two knobs. Benchmarked on
    # CPU: 5k points at 128x160 ~0.4s, 50k points at the same size ~3.6s.
    # These defaults target a still-usable, if not truly real-time, refresh
    # on CPU; with a CUDA GPU both can be raised substantially. NDC
    # intrinsics are resolution independent, so render_scale only affects
    # the output canvas, not projection accuracy.
    render_scale: float = 0.08
    max_points: int = 5_000
    point_cloud_queue_size: int = 5
    # bounds point cloud accumulation by age (sec) as well as count, so a
    # slow render doesn't pull in scans stale enough to no longer match the
    # current camera pose (see Listener.syncronizer_callback).
    max_point_cloud_age: float = 1.0
    # /Odometry publishes the body (LiDAR/IMU) frame's pose, not the
    # camera's - faster-lio also broadcasts that same pose as a
    # world_frame->body tf, so a single world_frame->camera_frame tf lookup
    # (chained through body automatically) gives the camera's actual world
    # pose at each frame.
    world_frame: str = "camera_init"
    camera_frame: str = "spinnaker"
    # The Spinnaker camera is mounted rotated on the rig, so raw frames (and
    # the calibration, which matches them) are sideways relative to how a
    # human would view the scene. This is purely a display-time rotation of
    # the already-composited output (image + projected points together) -
    # it must not be applied before compositing, since that would misalign
    # the point overlay with the image. Number of 90-degree counter-
    # clockwise turns (0-3); default guessed from one sample frame - flip to
    # 3 (or 2) if it comes out wrong way.
    display_rotation: int = 1
    # side length (meters) of the voxel grid used to dedupe the exported
    # detected-points PLY on shutdown; a physical point/object seen as
    # "detected" across many frames otherwise ends up duplicated many times
    # over in the accumulated export.
    export_voxel_size: float = 0.02
