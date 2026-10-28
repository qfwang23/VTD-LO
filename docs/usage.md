# Build and run

[Project home](../README.md) · [Method and parameters](method.md)

## Environment

The reference setup is Ubuntu 20.04 with ROS Noetic and a C++17 compiler. The package requires catkin, PCL, Eigen, Ceres, and the ROS message and visualization packages below. Install ROS Noetic and configure its package repository before these steps.

```bash
sudo apt update
sudo apt install \
  build-essential cmake git \
  libeigen3-dev libpcl-dev libceres-dev \
  ros-noetic-catkin ros-noetic-roscpp ros-noetic-rospy ros-noetic-rosbag \
  ros-noetic-std-msgs ros-noetic-geometry-msgs ros-noetic-nav-msgs \
  ros-noetic-sensor-msgs ros-noetic-tf ros-noetic-eigen-conversions \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  ros-noetic-jsk-recognition-msgs ros-noetic-rviz \
  ros-noetic-hector-trajectory-server
```

## Build a catkin workspace

The repository contains the ROS package inside `VTD-LO(main)/src/vtd_lo`. Link that package into a fresh workspace:

```bash
source /opt/ros/noetic/setup.bash
mkdir -p "$HOME/code" "$HOME/vtd_ws/src"
git clone https://github.com/qfwang23/VTD-LO.git "$HOME/code/VTD-LO"
ln -s "$HOME/code/VTD-LO/VTD-LO(main)/src/vtd_lo" "$HOME/vtd_ws/src/vtd_lo"
cd "$HOME/vtd_ws"
catkin_make -j4
source devel/setup.bash
```

If the repository is already cloned, use its existing path in the `ln -s` command. This layout lets catkin create the workspace's top-level build files and keeps the ROS package in its original source location.

Source the workspace in each new terminal:

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/vtd_ws/devel/setup.bash"
```

## Input data

The launch files expect ROS 1 bags containing `sensor_msgs/PointCloud2` scans on `/velodyne_points`. Point clouds are read as PCL `PointXYZI`: provide `x`, `y`, `z`, and `intensity` fields, consistent timestamps, and the original scan ordering.

The source assigns scan lines using elevation angles. The supplied KITTI launch selects its 64-line branch; M2DGR selects its 32-line branch. Both use a nominal 0.1 s scan period. Obtain the recordings from the [KITTI](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) and [M2DGR](https://github.com/SJTU-ViSYS/M2DGR) dataset pages.

Inspect a bag before starting:

```bash
rosbag info /absolute/path/sequence.bag
```

## KITTI

Place the converted ROS bag at `~/DATASET/KITTI/08.bag`, or use another two-digit sequence name from `00` to `10`.

```bash
mkdir -p "$HOME/vtd_results"
roslaunch vtd_lo kitti.launch \
  seq:=08 \
  tum_path:="$HOME/vtd_results/kitti08.txt"
```

This single launch starts bag playback with simulated time, the three processing nodes, RViz, and trajectory visualization. The `seq` argument selects the bag at `~/DATASET/KITTI/<seq>.bag`.

Append `rviz:=false` to disable the RViz window while keeping trajectory visualization topics available.

## M2DGR

Provide the absolute bag path:

```bash
mkdir -p "$HOME/vtd_results"
roslaunch vtd_lo m2dgr.launch \
  bag_file:="/absolute/path/hall_01.bag" \
  tum_path:="$HOME/vtd_results/hall01.txt"
```

The launch includes bag playback and selects the 32-line configuration. To use another sequence, change `bag_file` and the output filename.

## Launch arguments and ROS parameters

| Argument | KITTI | M2DGR | Description |
| --- | --- | --- | --- |
| `seq` | Yes | No | Two-digit KITTI sequence; default `08` |
| `bag_file` | No | Yes | M2DGR bag path |
| `tum_path` | Yes | Yes | Output trajectory filename |
| `rviz` | Yes | Yes | Start RViz; default `true` |
| `use_dynamic_filter` | Yes | Yes | Enable both temporal-filtering stages; default `true` |
| `use_adaptive_weight` | Yes | Yes | Enable adaptive residual weighting; default `true` |

For example, the switches can be set explicitly:

```bash
roslaunch vtd_lo kitti.launch \
  seq:=08 \
  use_dynamic_filter:=true \
  use_adaptive_weight:=true \
  rviz:=false \
  tum_path:="$HOME/vtd_results/kitti08.txt"
```

Configure `topic`, `scan_line`, `map_resolution`, and the voxel thresholds through the `<param>` entries in the launch files. For a different input topic or sensor setup, copy the appropriate launch file and edit those entries before starting the nodes. Use the arguments in the table for command-line overrides. See [Method and parameters](method.md#launch-configuration) for parameter meanings and supplied values.

## ROS interfaces

| Topic | Type | Description |
| --- | --- | --- |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Input scans; configurable through the `topic` parameter |
| `/laser_cloud_edge` | `sensor_msgs/PointCloud2` | Extracted edge features in `base_link` |
| `/laser_cloud_surf` | `sensor_msgs/PointCloud2` | Extracted planar features in `base_link` |
| `/velodyne_points_filtered` | `sensor_msgs/PointCloud2` | Combined extracted edge and planar features in `base_link` |
| `/odom` | `nav_msgs/Odometry` | Estimated pose; `map` parent and `base_link` child |
| `/edge_local_map` | `sensor_msgs/PointCloud2` | Odometry module's local corner map in `map` |
| `/surf_local_map` | `sensor_msgs/PointCloud2` | Odometry module's local surface map in `map` |
| `/map` | `sensor_msgs/PointCloud2` | Accumulated feature visualization map in `map` |

The odometry node broadcasts `map → base_link`. The launch files also publish an identity `world → map` transform. `/edge_local_map` and `/surf_local_map` are the relevant outputs for viewing the temporally filtered registration map. `/map` is produced separately by the visualization mapping node.

The default RViz configuration uses `map` as its fixed frame. Matching camera topics and dataset transforms enable its optional image and ground-truth trajectory displays. The pose estimator receives the extracted LiDAR features.

## Trajectory export

Set `tum_path` to a writable file and create its parent directory before launch. Each row contains:

```text
timestamp tx ty tz qx qy qz qw
```

Timestamps follow the incoming scan messages. A run opens the output file for writing, so use a distinct filename to preserve an earlier trajectory. After bag playback completes, allow queued scans to finish processing and stop the launch with `Ctrl+C`.

## Run checklist

| Item | Verification |
| --- | --- |
| Workspace | Source `~/vtd_ws/devel/setup.bash`; `rospack find vtd_lo` should return the package path. |
| Dependencies | Install the listed Ceres, JSK, and Hector packages before building and launching. |
| Playback | Verify the bag path, input topic, `/topic` parameter, and simulated clock. |
| Trajectory output | Create the output directory and set a writable `tum_path` for each run. |
| Build jobs | Choose a parallel job count suited to available memory, for example `catkin_make -j2` or `-j4`. |

The logged `avg FPS` summarizes timed calls to `updatePointsToMap` in the odometry node. This timing scope covers the scan-to-map odometry update.
