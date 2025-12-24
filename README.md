# The manuscript has been submitted to IEEE Transactions on Geoscience and Remote Sensing

# VTD-LO: Voxel-Temporal Dynamic Filtering LiDAR Odometry with Two-Stage Dynamic Suppression and Adaptive Residual Weighting

[Paper Details / 论文详情](https://qfwang23.github.io/VTD-LO/)

<p align="center">
  <img src="https://github.com/qfwang23/VTD-LO/blob/feb5ae156f1869c22bf65f3fa8e5bede164cca43/result/02map.png?raw=1" width="45%" />
  <img src="https://github.com/qfwang23/VTD-LO/blob/ac9b0cd4dfe308cf0a35bb80ec29678d6c88f410/result/08map.png?raw=1" width="45%" />
</p>






## Dependence
```bash
Ubuntu 20.04
ROS Noetic（roscpp、std_msgs、sensor_msgs、geometry_msgs、pcl_ros）
C++ 14
CMake ≥ 3.16
PCL≥ 1.10.0
Eigen ≥ 3.3.7
```

## Install
 
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/qfwang23/VTD-LO.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslauch vtd_lo kitti.launch
```

## Acknowledgements
```bash
@INPROCEEDINGS{9636655,
  author={Wang, Han and Wang, Chen and Chen, Chun-Lin and Xie, Lihua},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={F-LOAM : Fast LiDAR Odometry and Mapping}, 
  year={2021},
  volume={},
  number={},
  pages={4390-4396},
  keywords={Location awareness;Simultaneous localization and mapping;Laser radar;Remotely guided vehicles;Pose estimation;Feature extraction;Distortion},
  doi={10.1109/IROS51168.2021.9636655}}


```
