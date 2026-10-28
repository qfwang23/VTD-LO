# Method and parameters

[Project home](../README.md) · [Build and run](usage.md)

VTD-LO uses geometric consistency across repeated observations to maintain a useful registration map in changing scenes. The implementation combines voxel-temporal statistics, correspondence screening, local-map filtering, and adaptive residual weighting in a feature-based LiDAR odometry pipeline.

## Processing stages

1. **Extract geometric features.** Organize points into scan lines from their elevation angles, reject out-of-range points, and select edge and planar features using local curvature. Occlusion and neighboring-point checks guide feature selection.
2. **Predict the pose.** Extrapolate the preceding relative motion to initialize the next scan-to-map alignment.
3. **Build constraints.** Search for five neighboring map features, use voxel-temporal states to screen their stability, and construct point-to-line and point-to-plane residuals.
4. **Optimize the pose.** Apply adaptive residual weights and a Huber loss in Ceres.
5. **Update the local map.** Transform new features, crop and downsample the map, update its voxel statistics, and remove map points in unstable voxels.

The LiDAR-only odometry frontend estimates poses and maintains its maps through sequential scan-to-map registration.

## Voxel-temporal statistics

Corner and surface maps have separate voxel states. Each state maintains an observation count, a mean position, and the accumulated squared deviations needed for an online variance estimate. For coordinate variances $\sigma_x^2$, $\sigma_y^2$, and $\sigma_z^2$, the instability statistic is

$$
s_v = \left\|[\sigma_x^2,\sigma_y^2,\sigma_z^2]^\top\right\|_2.
$$

A voxel is considered unstable once its count reaches the minimum and $s_v$ exceeds its threshold. The statistic has units of square meters. Each voxel's count is the cumulative number of downsampled map representatives processed across map updates. Its statistics retain this accumulated observation history.

This geometric criterion treats observed stability directly. Stopped objects can provide stable constraints, while points that change position across observations can increase the voxel statistic. The accumulated spatial consistency determines the voxel state.

## Two stages of dynamic suppression

**Correspondence screening.** During registration, the five nearest map neighbors are checked against their voxel states. A candidate constraint is discarded when at least three neighbors lie in voxels marked unstable.

**Local-map filtering.** After map insertion, cropping, and downsampling, the voxel states are updated and points in unstable voxels are removed from the local feature maps. These maps provide the geometry for subsequent registration.

Both operations use the same temporal criterion and are controlled by `use_dynamic_filter`. Adaptive weighting is controlled separately through `use_adaptive_weight`.

## Adaptive residual weighting

For a retained correspondence, VTD-LO combines three terms:

- **Stability:** the mean of $1-\min(s_v/\tau,1)$ over neighboring voxels with sufficient observations. The term defaults to one until qualifying voxel history is available.
- **Age:** $0.5+0.5\bar a$, where $\bar a=\frac{1}{5}\sum_{j=1}^{5}g_j/255$ is the average normalized age of the five map neighbors. Existing points increment their stored age before each map update, up to 255; newly inserted points start at zero. Voxel averaging combines these values when map points are merged.
- **Range:** $\min(1,1/\max(d,1))$, with $d$ expressed in meters and measured from the current LiDAR origin.

The combined weight is

$$
\omega=\mathrm{clip}(\omega_{\mathrm{static}}\,\omega_{\mathrm{age}}\,\omega_{\mathrm{range}},0.1,2.0).
$$

It scales the robust loss of each scalar geometric residual:

$$
E(\mathbf T)=\frac12\sum_i \omega_i\,\rho\!\left(r_i(\mathbf T)^2\right).
$$

Here $\rho$ is a Huber loss with scale 0.1. The three terms combine temporal stability, observation age, and measurement range into a residual-confidence weight. Clipping provides a positive floor for retained constraints. With these normalized factors, the resulting weights lie in $[0.1,1.0]$ within the configured clipping interval.

## Launch configuration

The following values are set by the supplied KITTI and M2DGR launch files. The launch settings take precedence over defaults inside the node source.

| ROS parameter | KITTI | M2DGR | Role |
| --- | ---: | ---: | --- |
| `scan_line` | 64 | 32 | Scan-line assignment branch |
| `scan_period` | 0.1 | 0.1 | Nominal scan period in seconds |
| `min_dis` | 3.0 | 3.0 | Minimum horizontal range in meters |
| `max_dis` | 90.0 | 90.0 | Maximum horizontal range in meters |
| `map_resolution` | 0.4 | 0.4 | Corner-map voxel width in meters |
| `voxel_var_thresh_corner` | 0.01 | 0.01 | Corner-voxel instability threshold in m² |
| `voxel_var_thresh_surf` | 0.01 | 0.01 | Surface-voxel instability threshold in m² |
| `voxel_min_count_corner` | 5 | 5 | Minimum corner-voxel observation count |
| `voxel_min_count_surf` | 5 | 5 | Minimum surface-voxel observation count |
| `use_dynamic_filter` | true | true | Enable correspondence screening and local-map filtering |
| `use_adaptive_weight` | true | true | Enable adaptive residual weighting |

Surface-map voxels use twice `map_resolution`, giving 0.8 m with the supplied settings. The range gate uses $\sqrt{x^2+y^2}$; the residual range weight uses the full three-dimensional distance.

### Configuration guidance

Voxel width determines geometric aggregation. The instability threshold controls how much accumulated variation is tolerated inside a voxel, and the minimum count delays classification until sufficient updates are available. These quantities should be considered together because changing voxel width also changes the observations grouped into each state.

The implementation provides elevation-angle assignment branches for 16-, 32-, and 64-line sensors in `src/laserProcessingClass.cpp`. Select the appropriate branch and match its elevation mapping to the sensor calibration. Preserve point ordering within each scan because feature extraction uses neighboring points along each line.

The C++ implementation defines the weighting constants, five-neighbor search, three-neighbor rejection rule, and Huber scale described above. The launch files configure sensor input, voxel thresholds, and feature-map resolution through the parameters in the table.

## Map outputs

`/edge_local_map` and `/surf_local_map` publish the local feature maps maintained by the odometry module, including its voxel-temporal filtering. Their RGB fields encode per-point algorithm state.

`/map` is a separate visualization map accumulated from the extracted scan features using the estimated poses. Use the local-map topics when inspecting the geometry used for registration and temporal filtering. See [ROS interfaces](usage.md#ros-interfaces) for message types and coordinate frames.

## Source guide

All paths below are relative to `VTD-LO(main)/src/vtd_lo/`.

| File | Responsibility |
| --- | --- |
| `src/laserProcessingClass.cpp` | Scan-line organization and feature extraction |
| `src/odomEstimationClass.cpp` | Temporal statistics, correspondences, weighting, optimization, and local maps |
| `src/lidarOptimization.cpp` | Geometric residuals and pose parameterization |
| `src/odomEstimationNode.cpp` | ROS odometry interface and TUM export |
| `src/laserMappingClass.cpp` | Accumulated visualization map |
| `launch/kitti.launch`, `launch/m2dgr.launch` | Dataset playback and node configuration |
