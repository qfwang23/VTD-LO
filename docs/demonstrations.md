# Demonstrations

[Project home](../README.md) · [Method and parameters](method.md) · [Build and run](usage.md)

The three demonstrations present recorded VTD-LO local maps and estimated trajectories. The README presents looping animations; the video gallery provides the corresponding 1280 × 720 MP4 recordings.

## Recording guide

| Demonstration | Recorded content | Presentation |
| --- | --- | --- |
| Temporal local-map replay | KITTI 08 edge and surface local maps from three short time windows | 1× recorded time; approximately 2.5 map snapshots per second |
| Local-map geometry | Three KITTI 08 local-map snapshots | Eight seconds of changing viewpoint per fixed point set |
| Five recorded urban runs | Five UrbanLoco pose trajectories under one shared sensor configuration | 40× recorded time, with a brief hold between runs |

Map views use slate blue for edge features and sienna for surface features. The colors identify feature types. A route locator marks the position of each displayed local map along the saved trajectory.

## KITTI recordings

| Sequence | Saved poses | Recording duration |
| --- | ---: | ---: |
| 08 | 4,071 | 422.940 s |

The temporal map replay uses these three KITTI 08 windows, measured from the start of the saved trajectory:

| Window | Elapsed time | Saved map instants |
| --- | --- | ---: |
| 1 | 149.517–157.415 s | 21 |
| 2 | 300.942–308.944 s | 21 |
| 3 | 352.480–360.481 s | 21 |

The geometry video uses the snapshots at 153.465 s, 304.993 s, and 356.433 s. Each view uses a 48 m-radius local crop and a 10 m scale. Each snapshot is presented through an eight-second viewpoint orbit.

## UrbanLoco recordings

All five displayed runs use one shared 32-line sensor configuration with calibrated elevation-to-scan-line assignment. The dataset launch files distributed in this repository cover the KITTI and M2DGR execution paths.

| Recording | Saved poses | Recording duration |
| --- | ---: | ---: |
| HK-Data20190426-2 | 2,607 | 260.825 s |
| HK-Data20190117 | 3,178 | 317.979 s |
| 20190331HH | 1,508 | 150.830 s |
| HK-Data20190317 | 3,181 | 318.280 s |
| 20190331_NJ_SL | 1,753 | 175.352 s |

The video contains all 12,227 saved poses across these five runs. Each sequence starts in its own local map frame. The recordings are shown sequentially with explicit sequence labels and progress indicators.

## Coordinates and playback

Trajectory views show estimated positions in meters in each run's local map frame. Elapsed time is measured from the first saved pose. The full path provides spatial context, while the highlighted trace and pose marker follow the recording timeline.

The GIF versions use fewer display frames for efficient inline playback while retaining the video duration and the stated playback speed. The MP4 versions use 24 frames per second. The geometry orbit changes the viewing camera around recorded point sets; the temporal map replay follows saved map updates.

## Datasets

The demonstrations use the [KITTI odometry dataset](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) and [UrbanLoco](https://github.com/weisongwen/UrbanLoco). Please cite the corresponding dataset papers when using these recordings in research.
