# MY CHANGES

## to extract_image
run in separate terminals
```
nixGLIntel roslaunch /home/warleon/Projects/macanudo/study-repos/4d-orchard-mapping-dataset/launch/mapping_ouster128_with_driver.launch
python3 script/extract_image.py --output_dir ./dataset/extracted --offset 45 --num_images 5
rosbag play dataset/data.bag --clock --rate 0.2
```


# 4d-orchard-mapping-dataset

This repository holds scripts and tutorials on using the dataset presented in the paper **4D Metric-Semantic Mapping for Persistent Orchard Monitoring: Method and Dataset**

Project website: [https://4d-metric-semantic-mapping.org/overview/](https://4d-metric-semantic-mapping.org/overview/)

## Scripts
* manual_counting.py: helper script to get the ground truth of fruit count
* pc_odom_rgb_sync_node.py: sychronize pointcloud, rgb images, and odometry in the bag
* PrepareImagesForTraining.py: Extract images and labels from a folder, convert labels in json format to yolo required format, and automatically divide them into training and validation set
* spinnaker_exposure.py: adjust the exposure level of rosbag images as some of them looks dark
* utils.py: helper functions
* extract_image.py: get the images from rosbags 

## How to use [Faster-lio](https://github.com/gaoxiang12/faster-lio) to get point cloud from lidar packets

It's recommended to use ROS as our dataset is recorded as ros bags.

Dependency: 
* ROS1: Faster-lio is implemented to use with ROS. If you don't have experience in ROS, refer to [ROS](https://wiki.ros.org/Installation) for more information on installation and tutorial. 
* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html): build ros workspace
* dependencies of [faster-lio](https://github.com/gaoxiang12/faster-lio)
* dependencies of [ouster-ros](https://github.com/ouster-lidar/ouster-ros)

After installing ROS1, following the steps below:

1. create ros workspace

    ```
    cd ~
    mkdir ws
    cd ws
    mkdir src
    ```

2. faster-lio

    ```
    cd ~/ws/src
    git clone https://github.com/gaoxiang12/faster-lio.git
    ```

3. ouster-ros

    ```
    cd ~/ws/src
    git clone https://github.com/ouster-lidar/ouster-ros.git
    ```


4. build the workspace
   ```
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

5. config files
   1. Put the all the files in the launch folder into faster-lio's launch folder. 
   2. Put ouster128.yaml inside faster-lio's config
   3. Change the value of *metadata* to be the path to the metadata provided.

6. launch the ouster driver and faster-lio
   ```
    roslaunch faster_lio mapping_ouster128_with_driver.launch
   ```


## How to do calibration using calibration bags

1. use the same workspace, compile kalibr
    ```
    cd ~/ws/src
    git clone https://github.com/ethz-asl/kalibr.git
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

2. run kalibr
    ```
    rosrun kalibr kalibr_calibrate_cameras --target path_to_apriltag.yaml --models pinhole-radtan --topics image_topic --bag path_to_your_bag --bag-freq 4.0
    ```

## Grape rig: remote GPU pipeline and local visualization

The grape rig differs from the apple rig this dataset was built around: a
16-channel Ouster that publishes already-decoded points (no packet driver
needed) and a ZED front camera instead of the FLIR Spinnaker. Its bags live
on a remote GPU box (`anthony-gpu`), reached over SSH via an AWS SSM
`ProxyCommand` (no public SSH port). All commands below assume you're in the
repo root, with `ws/devel/setup.bash` sourced (and, on the remote, inside the
project's Nix devShell).

### Running the live pipeline on a full bag

```
roslaunch launch/semantic_pointcloud_grape.launch \
  data_bag:=/path/to/rosbag_compressed_*.bag \
  rate:=0.2
```

Run this on `anthony-gpu`, not locally - it needs the GPU for tiled YOLO
inference. `rate` controls bag playback speed; the tiled detector doesn't
keep up with real-time (`rate:=1.0`, the default) on a loaded GPU, so slow
it down if detections lag or drop. Pass `rviz:=false` to skip rviz - its
indirect-GLX rendering over `ssh -X` segfaults reliably on a Wayland/XWayland
client, so prefer local visualization instead (see below).

`mapping_ouster_grape.launch` (faster-lio only, no detection) is the piece
`semantic_pointcloud_grape.launch` includes for lidar odometry/mapping - run
it alone if you only need `/Odometry` and `/cloud_registered`.

### Visualizing the remote stream locally

`script/run_remote_viz.sh` wraps the whole remote-to-local workflow: it
starts the `anthony-gpu` instance if it's stopped, clears any leftover ROS
processes from a previous run, launches the given launch file and arguments
on the remote, opens an `sshuttle` tunnel (so your local machine can reach
the remote's ROS topics directly, sidestepping ROS's per-topic dynamic port
negotiation), and opens local `rviz` once the remote topics are reachable.

```
script/run_remote_viz.sh semantic_pointcloud_grape.launch \
  rate:=0.2 data_bag:=/home/ubuntu/repos/rosbag_compressed_2022-09-15-14-23-20.bag
```

Run `script/run_remote_viz.sh --help` for its full option list (custom rviz
config, skipping rviz or the tunnel). It needs the `anthony-gpu` SSH host
alias and Nix (for `nix shell nixpkgs#sshuttle`); the remote pipeline and the
tunnel both keep running in the background after the script exits.

If you restart the remote pipeline while an old local `rviz` is still open,
restart `rviz` too - each `roslaunch` spins up a brand-new `rosmaster`
process, and existing clients stay registered with the old (now-dead) one.

### Taking rviz screenshots without the remote GPU

`semantic_pointcloud_grape_snippet.launch` runs the full detection pipeline
(tiled YOLO, instance-colored filtered cloud, boxes, mask) against a short
locally recorded snippet (`dataset/grape_zed_tuning_snippet.bag`), entirely
on your own machine:

```
roslaunch launch/semantic_pointcloud_grape_snippet.launch
```

It loops the snippet continuously and defaults to `rate:=0.2` for the same
reason as above. Record your own snippet with `rosbag record`, capturing at
least `/tf`, `/Odometry`, `/cloud_registered`, the ZED's compressed image,
and its `camera_info` - `/Odometry` especially, since
`semantic_pointcloud_node` won't dispatch any frame without it.

### Tuning the camera extrinsic

`launch/tune_zed_extrinsic.launch` opens an interactive rviz marker (an
arrow at `zed_front_camera`'s current pose, attached to `body`) for tuning
the body-to-camera extrinsic by eye, against the same local snippet:

```
roslaunch launch/tune_zed_extrinsic.launch
```

Drag or rotate the marker in rviz's 3D view - the transform broadcasts live
as you move it, so the Camera Overlay display's point-cloud projection
updates immediately, no relaunching needed. Right-click the marker and
choose **Save** to append the current translation and quaternion to
`/tmp/extrinsic_tuner_output.txt`; copy that line into
`semantic_pointcloud_grape.launch`'s own `lidar2camera` static transform.

### Extracting images for training

`extract_grape_images.launch` brings up lidar odometry/mapping, ZED
decompression, and full-bag playback - `script/extract_image.py` needs all
three running live, since it saves synced odometry and point-cloud samples
that only exist while faster-lio is actually mapping:

```
roslaunch launch/extract_grape_images.launch \
  data_bag:=/path/to/rosbag_compressed_*.bag
```

Once that's up, run the extractor in a separate terminal against the same
ROS master:

```
python3 script/extract_image.py --output_dir ~/grape_extracted \
  --image_topic /front/zed_node/rgb/image_rect_color_raw \
  --rotate 0 --offset 30 --num_images 20
```

`--rotate 0` matters: the script's default (`--rotate 1`) assumes the apple
rig's rotated Spinnaker mount, which would incorrectly rotate the grape
rig's upright ZED images.

## Questions
Please feel free to open an issue if there are any questions or bugs.


## License
This code and data is released using the Penn Software Licence. Please refer to LICENSE.txt for details.
