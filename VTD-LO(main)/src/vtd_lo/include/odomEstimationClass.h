#ifndef _ODOM_ESTIMATION_CLASS_H_
#define _ODOM_ESTIMATION_CLASS_H_

// std lib
#include <string>
#include <math.h>
#include <vector>
#include <unordered_map> 
#include <cstddef>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>

// ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// LOCAL LIB
#include "lidar.h"
#include "lidarOptimization.h"
#include <ros/ros.h>

typedef pcl::PointXYZRGB PointType;

/**

* Odometry estimation class

*
* Functions:

* - Perform pose optimization (Ceres) using LOAM-type edge/surf features

* - Use a local map (odom ±100m + voxel filtering)

* - Maintain voxel time statistics (mean / var) to filter "dynamic voxels"

* - Calculate and print the number of corners/faces deleted for each frame

*/
class OdomEstimationClass 
{
public:
    OdomEstimationClass();
    
	void init(lidar::Lidar lidar_param,
			double map_resolution,
			double voxel_var_thresh_corner_in,
			double voxel_var_thresh_surf_in,
			int    voxel_min_count_corner_in,
			int    voxel_min_count_surf_in);

    void initMapWithPoints(const pcl::PointCloud<PointType>::Ptr& edge_in,
                           const pcl::PointCloud<PointType>::Ptr& surf_in);

    void updatePointsToMap(const pcl::PointCloud<PointType>::Ptr& edge_in,
                           const pcl::PointCloud<PointType>::Ptr& surf_in);

    void getMap(pcl::PointCloud<PointType>::Ptr& laserCloudMap);

    Eigen::Isometry3d odom;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfMap;

    float map_resolution;

    void setUseAdaptiveWeight(bool v);
    void setUseDynamicFilter(bool v);

private:
    // parameters = [qw, qx, qy, qz, tx, ty, tz]
    double parameters[7] = {0, 0, 0, 1, 0, 0, 0};

    Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
    Eigen::Map<Eigen::Vector3d>    t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

    Eigen::Isometry3d last_odom;

    // ==================== Kd-tree ====================
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeEdgeMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfMap;

    pcl::VoxelGrid<PointType> downSizeFilterEdge;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;


    pcl::CropBox<PointType> cropBoxFilter;

    int optimization_count;


    void updateStaticProbabilityForMap(pcl::PointCloud<PointType>::Ptr &cloud);

// ==================== Voxel Temporal Statistics: Used for Dynamic Object Filtering ===================
    struct VoxelState
    {
        Eigen::Vector3d mean; // Geometric center of cumulative observations
        Eigen::Vector3d m2; // Welford uses as an intermediate variable to calculate variance
        int count; // Number of frames/points observed
        VoxelState()
        {
            mean.setZero();
            m2.setZero();
            count = 0;
        }
    };

    // key: voxel hash, value: time statistics for this voxel
    std::unordered_map<long long, VoxelState> voxel_states_corner_;
    std::unordered_map<long long, VoxelState> voxel_states_surf_;

    // Variance threshold & minimum number of observations:
    // - count < voxel_min_count_* indicates the statistics are not yet reliable and should not be considered dynamic.
    // - var_norm > voxel_var_thresh_* indicates the voxel "fluctuates greatly" over time and is classified as a dynamic voxel.
    double voxel_var_thresh_corner_;
    double voxel_var_thresh_surf_;
    int    voxel_min_count_corner_;
    int    voxel_min_count_surf_;

    // Statistics: Currently processing to which frame number, used to print "How many corner points/face points were deleted in frame N".
    std::size_t frame_count_;

    // Voxel hash: Given a point and the size of a voxel, calculate a long long key
    long long computeVoxelKey(const PointType &pt, float leaf) const;
    
    // Calculate a scalar (norm) of "variance strength" from VoxelState.
    double voxelVarianceNorm(const VoxelState &vs) const;


    void addEdgeCostFactor(const pcl::PointCloud<PointType>::Ptr& pc_in,
                           const pcl::PointCloud<PointType>::Ptr& map_in,
                           ceres::Problem& problem,
                           ceres::LossFunction *loss_function);

    void addSurfCostFactor(const pcl::PointCloud<PointType>::Ptr& pc_in,
                           const pcl::PointCloud<PointType>::Ptr& map_in,
                           ceres::Problem& problem,
                           ceres::LossFunction *loss_function);

    // Add current frame points into the map, and maintain local map & dynamic filtering
    void addPointsToMap(const pcl::PointCloud<PointType>::Ptr& downsampledEdgeCloud,
                        const pcl::PointCloud<PointType>::Ptr& downsampledSurfCloud);

    // Transform points from current frame coordinate system -> map coordinate system
    void pointAssociateToMap(PointType const *const pi, PointType *const po);

    // Voxel downsample edge/surf points in current frame once to reduce optimization points
    void downSamplingToMap(const pcl::PointCloud<PointType>::Ptr& edge_pc_in,
                           pcl::PointCloud<PointType>::Ptr& edge_pc_out,
                           const pcl::PointCloud<PointType>::Ptr& surf_pc_in,
                           pcl::PointCloud<PointType>::Ptr& surf_pc_out);

    // === Ablation switches ===
    bool use_adaptive_weight_;  // Innovation 1: adaptive residual weighting
    bool use_dynamic_filter_;   // Innovation 2: dynamic point removal via voxel temporal variance
};

#endif // _ODOM_ESTIMATION_CLASS_H_
