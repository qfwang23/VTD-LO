# The manuscript has been submitted to IEEE Transactions on Transportation Electrification

# VTD-LO: Voxel-Temporal Dynamic Filtering LiDAR Odometry with Two-Stage Dynamic Suppression and Adaptive Residual Weighting

 
## Introduction
	LiDAR odometry is a key enabler for autonomous navigation and localization in mobile robots and has been widely deployed in autonomous driving and robotic systems. However, existing local-map-based geometric matching methods are vulnerable in dynamic scenes. Erroneous correspondences induced by moving objects and transient structural changes can bias pose updates, while the long-term fusion of dynamic points gradually contaminates the local map, leading to unstable estimation and degraded accuracy. To address these issues, this paper proposes VTD-LO (Voxel-Temporal Dynamic LiDAR Odometry), a LiDAR odometry method with voxel-temporal dynamic filtering. VTD-LO constructs geometric residual constraints using edge and planar features against a local map, and maintains online, cross-frame accumulated voxel statistics. The temporal stability of each voxel is characterized by the cross-frame dispersion (accumulated variance) of points observed in that voxel. Based on this stability measure, VTD-LO introduces a two-stage dynamic suppression mechanism: (i) during constraint construction, a gating rule evaluates the temporal instability of neighboring voxels and rejects correspondences with high dynamic risk; (ii) during map update, map points inside unstable voxels are removed to prevent long-term accumulation of dynamic regions. In addition, to mitigate optimization bias caused by heterogeneous constraint reliability, VTD-LO adopts an adaptive residual weighting scheme that combines voxel staticness, map-point age, and range-dependent factors with bounded scaling, and integrates it with the Huber robust kernel to form a weighted robust pose optimization objective for improved convergence stability. Extensive experiments on the public KITTI and M2DGR datasets demonstrate that VTD-LO achieves more stable trajectory estimation and lower localization error than competing methods in dynamic environments.
## Demo
![示例图片](https://github.com/qfwang23/ALO/blob/6aa048cc49058d78d20e694bbfcd3c419e20cf9a/demo.gif)

## Dependence
```bash
Ubuntu 18.04 or 20.04
ROS Melodic（roscpp、std_msgs、sensor_msgs、geometry_msgs、pcl_ros）
C++ 14
CMake ≥ 3.16
PCL≥ 1.10.0
Eigen ≥ 3.3.7
```

## Install
 
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/qfwang23/LO.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslauch LO lo.launch
rosbag play [topic]
```

## Rusult

![示例图片](https://github.com/qfwang23/LO/blob/3eaeaeebb11cf3b1210e6e47297447f7a408c7fe/LO_png/tab1.png)

![示例图片](https://github.com/qfwang23/LO/blob/3eaeaeebb11cf3b1210e6e47297447f7a408c7fe/LO_png/traij.png)

## Acknowledgements
```bash
@article{vizzo2023ral,
  author    = {Vizzo, Ignacio and Guadagnino, Tiziano and Mersch, Benedikt and Wiesmann, Louis and Behley, Jens and Stachniss, Cyrill},
  title     = {{KISS-ICP: In Defense of Point-to-Point ICP -- Simple, Accurate, and Robust Registration If Done the Right Way}},
  journal   = {IEEE Robotics and Automation Letters (RA-L)},
  pages     = {1029--1036},
  doi       = {10.1109/LRA.2023.3236571},
  volume    = {8},
  number    = {2},
  year      = {2023}
}

@article{chen2022direct,
  author={Kenny Chen and Brett T. Lopez and Ali-akbar Agha-mohammadi and Ankur Mehta},
  journal={IEEE Robotics and Automation Letters}, 
  title={Direct LiDAR Odometry: Fast Localization With Dense Point Clouds}, 
  year={2022},
  volume={7},
  number={2},
  pages={2000-2007},
  doi={10.1109/LRA.2022.3142739}
}
```
