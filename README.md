# The manuscript has been submitted to IEEE Transactions on Geoscience and Remote Sensing

# VTD-LO: Voxel-Temporal Dynamic Filtering LiDAR Odometry with Two-Stage Dynamic Suppression and Adaptive Residual Weighting

 
## Introduction

LiDAR odometry is a key enabler for autonomous navigation and localization in mobile robots and has been widely deployed in autonomous driving and robotic systems. However, existing local-map-based geometric matching methods are vulnerable in dynamic scenes. Erroneous correspondences induced by moving objects and transient structural changes can bias pose updates, while the long-term fusion of dynamic points gradually contaminates the local map, leading to unstable estimation and degraded accuracy. To address these issues, this paper proposes VTD-LO (Voxel-Temporal Dynamic LiDAR Odometry), a LiDAR odometry method with voxel-temporal dynamic filtering. VTD-LO constructs geometric residual constraints using edge and planar features against a local map, and maintains online, cross-frame accumulated voxel statistics. The temporal stability of each voxel is characterized by the cross-frame dispersion (accumulated variance) of points observed in that voxel. Based on this stability measure, VTD-LO introduces a two-stage dynamic suppression mechanism: (i) during constraint construction, a gating rule evaluates the temporal instability of neighboring voxels and rejects correspondences with high dynamic risk; (ii) during map update, map points inside unstable voxels are removed to prevent long-term accumulation of dynamic regions. In addition, to mitigate optimization bias caused by heterogeneous constraint reliability, VTD-LO adopts an adaptive residual weighting scheme that combines voxel staticness, map-point age, and range-dependent factors with bounded scaling, and integrates it with the Huber robust kernel to form a weighted robust pose optimization objective for improved convergence stability. Extensive experiments on the public KITTI and M2DGR datasets demonstrate that VTD-LO achieves more stable trajectory estimation and lower localization error than competing methods in dynamic environments.

## Result



![示例图片](https://github.com/qfwang23/VTD-LO/blob/a63b95719870e0c7074aaa2081af4e2649f99c9f/result/image.png)

<p align="center">
  <img src="https://github.com/qfwang23/VTD-LO/blob/feb5ae156f1869c22bf65f3fa8e5bede164cca43/result/02map.png?raw=1" width="45%" />
  <img src="https://github.com/qfwang23/VTD-LO/blob/ac9b0cd4dfe308cf0a35bb80ec29678d6c88f410/result/08map.png?raw=1" width="45%" />
</p>







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
