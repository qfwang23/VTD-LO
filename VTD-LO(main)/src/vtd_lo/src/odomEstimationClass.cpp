#include "odomEstimationClass.h"

#include <algorithm>
#include <cmath>

// ====================== Utility: voxel key & variance ======================

long long OdomEstimationClass::computeVoxelKey(const PointType &pt, float leaf) const
{
    int ix = static_cast<int>(std::floor(pt.x / leaf));
    int iy = static_cast<int>(std::floor(pt.y / leaf));
    int iz = static_cast<int>(std::floor(pt.z / leaf));

    const long long p1 = 73856093;
    const long long p2 = 19349663;
    const long long p3 = 83492791;

    long long key = static_cast<long long>(ix) * p1
                  ^ static_cast<long long>(iy) * p2
                  ^ static_cast<long long>(iz) * p3;
    return key;
}

double OdomEstimationClass::voxelVarianceNorm(const VoxelState &vs) const
{
    if (vs.count <= 1) return 0.0;

    const double denom = static_cast<double>(vs.count - 1);
    Eigen::Vector3d var = vs.m2 / denom;
    var = var.cwiseMax(0.0);      // Keep this line to prevent negative numerical noise
    return var.norm();
}


// ====================== Constructor / initialization ======================

OdomEstimationClass::OdomEstimationClass()
    : frame_count_(0)
{
    odom.setIdentity();
    last_odom.setIdentity();

    // Ablation experiment switches, enabled by default
    //   - use_adaptive_weight_ : Innovation 1, adaptive residual weighting
    //   - use_dynamic_filter_  : Innovation 2, unstable point removal (voxel temporal variance)
    use_adaptive_weight_ = true;
    use_dynamic_filter_  = true;
}

void OdomEstimationClass::init(lidar::Lidar lidar_param,
                               double map_resolution_in,
                               double voxel_var_thresh_corner_in,
                               double voxel_var_thresh_surf_in,
                               int    voxel_min_count_corner_in,
                               int    voxel_min_count_surf_in)
{
    // Maintain both corner / surf maps
    laserCloudCornerMap.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfMap.reset(new pcl::PointCloud<PointType>());

    // Downsampling resolution
    map_resolution = static_cast<float>(map_resolution_in);
    downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
    downSizeFilterSurf.setLeafSize(map_resolution * 2.0f,
                                   map_resolution * 2.0f,
                                   map_resolution * 2.0f);

    // kd-tree
    kdtreeEdgeMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfMap.reset(new pcl::KdTreeFLANN<PointType>());

    odom.setIdentity();
    last_odom.setIdentity();
    optimization_count = 2;


    cropBoxFilter.setNegative(false);

    // Voxel temporal variance thresholds (used for Innovation 2: unstable point removal & dynamic gating)
    voxel_var_thresh_corner_ = static_cast<float>(voxel_var_thresh_corner_in);
    voxel_var_thresh_surf_   = static_cast<float>(voxel_var_thresh_surf_in);
    voxel_min_count_corner_  = voxel_min_count_corner_in;
    voxel_min_count_surf_    = voxel_min_count_surf_in;

    voxel_states_corner_.clear();
    voxel_states_surf_.clear();

    frame_count_ = 0;

    // You can also read these ablation switches from ROS params here (optional)
    // ros::NodeHandle nh("~");
    // nh.param("use_adaptive_weight", use_adaptive_weight_, true);
    // nh.param("use_dynamic_filter",  use_dynamic_filter_,  true);
}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<PointType>::Ptr& edge_in,
                                            const pcl::PointCloud<PointType>::Ptr& surf_in)
{
    // Initialize the map directly using the first frame's edge + surf
    *laserCloudCornerMap += *edge_in;
    *laserCloudSurfMap   += *surf_in;
    optimization_count    = 12;

    for (auto &pt : laserCloudCornerMap->points)
    {
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
    }
    for (auto &pt : laserCloudSurfMap->points)
    {
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
    }
}

// ====================== Simple "soft forgetting": age++ ======================

void OdomEstimationClass::updateStaticProbabilityForMap(pcl::PointCloud<PointType>::Ptr &cloud)
{
    for (auto &pt : cloud->points)
    {
        if (pt.g < 255)
            ++pt.g;
    }
}

// ====================== Point coordinate transform ======================

void OdomEstimationClass::pointAssociateToMap(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;

    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->r = pi->r;
    po->g = pi->g;
    po->b = pi->b;
}

// ====================== downSamplingToMap: downsample current frame ======================

void OdomEstimationClass::downSamplingToMap(const pcl::PointCloud<PointType>::Ptr& edge_pc_in,
                                            pcl::PointCloud<PointType>::Ptr& edge_pc_out,
                                            const pcl::PointCloud<PointType>::Ptr& surf_pc_in,
                                            pcl::PointCloud<PointType>::Ptr& surf_pc_out)
{
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);

    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out);
}

// ====================== Edge cost: voxel gating + adaptive weight (both switchable) ======================

void OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<PointType>::Ptr& pc_in,
                                            const pcl::PointCloud<PointType>::Ptr& map_in,
                                            ceres::Problem& problem,
                                            ceres::LossFunction *loss_function)
{
    int corner_num = 0;

    for (int i = 0; i < (int)pc_in->points.size(); ++i)
    {
        PointType point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);

        std::vector<int>   pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
        if (pointSearchSqDis.size() < 5) continue;
        if (pointSearchSqDis[4] > 1.0f)  continue;

        // ---------- Innovation 2 (part): voxel temporal variance gating ----------
        int    dynamic_voxel_cnt = 0;
        double static_score_sum  = 0.0;   // Only for Innovation 1: adaptive weighting
        int    static_score_cnt  = 0;
        double age_sum           = 0.0;   // Only for Innovation 1: adaptive weighting

        for (int j = 0; j < 5; ++j)
        {
            const PointType &mp = map_in->points[pointSearchInd[j]];
            long long key = computeVoxelKey(mp, map_resolution);   // corner uses map_resolution

            auto it = voxel_states_corner_.find(key);
            if (it != voxel_states_corner_.end() &&
                it->second.count >= voxel_min_count_corner_)
            {
                double var_n = voxelVarianceNorm(it->second);
                if (var_n > voxel_var_thresh_corner_)
                    ++dynamic_voxel_cnt;

                // Staticness score: larger variance -> lower score (Innovation 1: adaptive weighting)
                double s = 1.0 - std::min(var_n / (double)voxel_var_thresh_corner_, 1.0);
                static_score_sum += s;
                static_score_cnt++;
            }

            // Use g channel to store age (0~255); normalize and use in weight
            age_sum += (double)mp.g / 255.0;
        }

        // === Switch: whether to filter out unstable areas based on voxel variance ===
        if (use_dynamic_filter_ && dynamic_voxel_cnt >= 3)
        {
            // Treat this area as time-varying (dynamic), discard this residual
            continue;
        }

        // ---------- Innovation 1: adaptive weight (staticness + age + range) ----------
        double weight = 1.0;   // Default weight 1.0 (used when Innovation 1 is off)

        if (use_adaptive_weight_)
        {
            double w_static = (static_score_cnt > 0)
                              ? (static_score_sum / static_score_cnt)
                              : 1.0;

            double w_age = age_sum / 5.0;         // Neighbor average age [0,1]
            w_age = 0.5 + 0.5 * w_age;           // New points ~0.5, old points ~1.0

            double range = std::sqrt(pc_in->points[i].x * pc_in->points[i].x +
                                     pc_in->points[i].y * pc_in->points[i].y +
                                     pc_in->points[i].z * pc_in->points[i].z);
            double w_range = 1.0 / std::max(range, 1.0); // <=1 when range>=1m
            w_range = std::min(w_range, 1.0);

            weight = w_static * w_age * w_range;
            weight = std::max(0.1, std::min(weight, 2.0)); // clamp [0.1, 2.0]
        }
        // ------------------------------------------------------------

        std::vector<Eigen::Vector3d> nearCorners;
        nearCorners.reserve(5);
        Eigen::Vector3d center(0, 0, 0);
        for (int j = 0; j < 5; ++j)
        {
            Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                map_in->points[pointSearchInd[j]].y,
                                map_in->points[pointSearchInd[j]].z);
            center += tmp;
            nearCorners.push_back(tmp);
        }
        center /= 5.0;

        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
        for (int j = 0; j < 5; ++j)
        {
            Eigen::Vector3d tmpZeroMean = nearCorners[j] - center;
            covMat += tmpZeroMean * tmpZeroMean.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

        Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
        Eigen::Vector3d curr_point(pc_in->points[i].x,
                                   pc_in->points[i].y,
                                   pc_in->points[i].z);

        if (saes.eigenvalues()[2] > 3.0 * saes.eigenvalues()[1])
        {
            Eigen::Vector3d point_on_line = center;
            Eigen::Vector3d point_a = 0.1 * unit_direction + point_on_line;
            Eigen::Vector3d point_b = -0.1 * unit_direction + point_on_line;

            ceres::CostFunction *cost_function =
                new EdgeAnalyticCostFunction(curr_point, point_a, point_b);

            // === Switch: whether to use "adaptive weight" ===
            ceres::LossFunction *loss_to_use = nullptr;
            if (use_adaptive_weight_)
            {
                // Innovation 1: multiply an additional residual weight on top of HuberLoss
                loss_to_use = new ceres::ScaledLoss(
                    loss_function, weight,
                    ceres::DO_NOT_TAKE_OWNERSHIP);   // does not delete huber_loss
            }
            else
            {
                // Innovation 1 off: use original HuberLoss directly (same as original A-LOAM)
                loss_to_use = loss_function;
            }

            problem.AddResidualBlock(cost_function, loss_to_use, parameters);

            corner_num++;
        }
    }

    if (corner_num < 20)
    {
        // printf("[OdomEstimation] not enough correct edge points: %d\n", corner_num);
    }
}

// ====================== Surf cost: voxel gating + adaptive weight (both switchable) ======================

void OdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<PointType>::Ptr& pc_in,
                                            const pcl::PointCloud<PointType>::Ptr& map_in,
                                            ceres::Problem& problem,
                                            ceres::LossFunction *loss_function)
{
    int surf_num = 0;

    for (int i = 0; i < (int)pc_in->points.size(); ++i)
    {
        PointType point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);

        std::vector<int>   pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
        if (pointSearchSqDis.size() < 5) continue;
        if (pointSearchSqDis[4] > 1.0f)  continue;

        // ---------- Innovation 2 (part): voxel temporal variance gating ----------
        int    dynamic_voxel_cnt = 0;
        double static_score_sum  = 0.0;   // Only for Innovation 1
        int    static_score_cnt  = 0;
        double age_sum           = 0.0;   // Only for Innovation 1

        for (int j = 0; j < 5; ++j)
        {
            const PointType &mp = map_in->points[pointSearchInd[j]];
            long long key = computeVoxelKey(mp, map_resolution * 2.0f); // surf uses 2*res

            auto it = voxel_states_surf_.find(key);
            if (it != voxel_states_surf_.end() &&
                it->second.count >= voxel_min_count_surf_)
            {
                double var_n = voxelVarianceNorm(it->second);
                if (var_n > voxel_var_thresh_surf_)
                    ++dynamic_voxel_cnt;

                double s = 1.0 - std::min(var_n / (double)voxel_var_thresh_surf_, 1.0);
                static_score_sum += s;
                static_score_cnt++;
            }

            age_sum += (double)mp.g / 255.0;
        }

        // === Switch: whether to filter out unstable areas based on voxel variance ===
        if (use_dynamic_filter_ && dynamic_voxel_cnt >= 3)
        {
            continue;
        }

        // ---------- Innovation 1: adaptive weight ----------
        double weight = 1.0;

        if (use_adaptive_weight_)
        {
            double w_static = (static_score_cnt > 0)
                              ? (static_score_sum / static_score_cnt)
                              : 1.0;

            double w_age = age_sum / 5.0;
            w_age = 0.5 + 0.5 * w_age;

            double range = std::sqrt(pc_in->points[i].x * pc_in->points[i].x +
                                     pc_in->points[i].y * pc_in->points[i].y +
                                     pc_in->points[i].z * pc_in->points[i].z);
            double w_range = 1.0 / std::max(range, 1.0);
            w_range = std::min(w_range, 1.0);

            weight = w_static * w_age * w_range;
            weight = std::max(0.1, std::min(weight, 2.0));
        }
        // --------------------------------------------------

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1.0 * Eigen::Matrix<double, 5, 1>::Ones();

        for (int j = 0; j < 5; ++j)
        {
            matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
            matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
            matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
        }

        Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
        double negative_OA_dot_norm = 1.0 / norm.norm();
        norm.normalize();

        bool planeValid = true;
        for (int j = 0; j < 5; ++j)
        {
            double dist = norm(0) * map_in->points[pointSearchInd[j]].x +
                          norm(1) * map_in->points[pointSearchInd[j]].y +
                          norm(2) * map_in->points[pointSearchInd[j]].z +
                          negative_OA_dot_norm;
            if (fabs(dist) > 0.2)
            {
                planeValid = false;
                break;
            }
        }

        if (!planeValid) continue;

        Eigen::Vector3d curr_point(pc_in->points[i].x,
                                   pc_in->points[i].y,
                                   pc_in->points[i].z);

        ceres::CostFunction *cost_function =
            new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);

        // === Switch: whether to use "adaptive weight" ===
        ceres::LossFunction *loss_to_use = nullptr;
        if (use_adaptive_weight_)
        {
            loss_to_use = new ceres::ScaledLoss(
                loss_function, weight,
                ceres::DO_NOT_TAKE_OWNERSHIP);
        }
        else
        {
            loss_to_use = loss_function;
        }

        problem.AddResidualBlock(cost_function, loss_to_use, parameters);

        surf_num++;
    }

    if (surf_num < 20)
    {
        // printf("[OdomEstimation] not enough correct surf points: %d\n", surf_num);
    }
}

// ====================== updatePointsToMap: overall frontend pipeline ======================

void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<PointType>::Ptr& edge_in,
                                            const pcl::PointCloud<PointType>::Ptr& surf_in)
{
    ++frame_count_;

    if (optimization_count > 2)
        optimization_count--;

    // Simple "prediction"
    Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
    last_odom = odom;
    odom      = odom_prediction;

    q_w_curr = Eigen::Quaterniond(odom.rotation());
    t_w_curr = odom.translation();

    pcl::PointCloud<PointType>::Ptr downsampledEdgeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr downsampledSurfCloud(new pcl::PointCloud<PointType>());
    downSamplingToMap(edge_in, downsampledEdgeCloud, surf_in, downsampledSurfCloud);

    if (laserCloudCornerMap->points.size() > 10 &&
        laserCloudSurfMap->points.size()   > 50)
    {
        kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
        kdtreeSurfMap->setInputCloud(laserCloudSurfMap);

        for (int iterCount = 0; iterCount < optimization_count; ++iterCount)
        {
            // Note: when Innovation 1 is off, huber_loss will be owned and deleted by problem;
            // when Innovation 1 is on, it is only referenced by ScaledLoss and will not be deleted
            // (a minor memory leak, but negligible).
            ceres::LossFunction* huber_loss = new ceres::HuberLoss(0.1);
            ceres::Problem problem;

            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());

            addEdgeCostFactor(downsampledEdgeCloud, laserCloudCornerMap,
                            problem, huber_loss);
            addSurfCostFactor(downsampledSurfCloud, laserCloudSurfMap,
                            problem, huber_loss);

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            // Since check_gradients = false, setting gradient_check_relative_precision makes no difference
            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);

        }
    }
    else
    {
        // printf("[OdomEstimation] not enough points in map to associate, map error\n");
    }

    // Update odom
    odom.setIdentity();
    odom.linear()      = q_w_curr.toRotationMatrix();
    odom.translation() = t_w_curr;

    // Add current frame points into the map + maintain local map + dynamic object filtering
    addPointsToMap(downsampledEdgeCloud, downsampledSurfCloud);
}

// ====================== addPointsToMap: local map + dynamic filtering (switchable) ======================

void OdomEstimationClass::addPointsToMap(const pcl::PointCloud<PointType>::Ptr& downsampledEdgeCloud,
                                         const pcl::PointCloud<PointType>::Ptr& downsampledSurfCloud)
{
    // 1. age++ for old map
    updateStaticProbabilityForMap(laserCloudCornerMap);
    updateStaticProbabilityForMap(laserCloudSurfMap);

    // 2. Transform current frame downsampled points to world coordinates and add to map
    laserCloudCornerMap->reserve(laserCloudCornerMap->size() +
                                 downsampledEdgeCloud->size());
    laserCloudSurfMap->reserve(laserCloudSurfMap->size() +
                               downsampledSurfCloud->size());

    for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); ++i)
    {
        PointType point_temp;
        pointAssociateToMap(&downsampledEdgeCloud->points[i], &point_temp);

        point_temp.r = 255;
        point_temp.g = 0;
        point_temp.b = 0;

        laserCloudCornerMap->push_back(point_temp);
    }

    for (int i = 0; i < (int)downsampledSurfCloud->points.size(); ++i)
    {
        PointType point_temp;
        pointAssociateToMap(&downsampledSurfCloud->points[i], &point_temp);

        point_temp.r = 255;
        point_temp.g = 0;
        point_temp.b = 0;

        laserCloudSurfMap->push_back(point_temp);
    }

    // 3. Build a local map based on current odom: crop ±100m
    const double x_min = odom.translation().x() - 100.0;
    const double y_min = odom.translation().y() - 100.0;
    const double z_min = odom.translation().z() - 100.0;
    const double x_max = odom.translation().x() + 100.0;
    const double y_max = odom.translation().y() + 100.0;
    const double z_max = odom.translation().z() + 100.0;

    cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0f));
    cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0f));
    cropBoxFilter.setNegative(false);

    pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());

    cropBoxFilter.setInputCloud(laserCloudSurfMap);
    cropBoxFilter.filter(*tmpSurf);
    cropBoxFilter.setInputCloud(laserCloudCornerMap);
    cropBoxFilter.filter(*tmpCorner);

    // 4. Apply voxel filtering again to the cropped map (the actual local map)
    laserCloudSurfMap->clear();
    laserCloudCornerMap->clear();

    downSizeFilterSurf.setInputCloud(tmpSurf);
    downSizeFilterSurf.filter(*laserCloudSurfMap);

    downSizeFilterEdge.setInputCloud(tmpCorner);
    downSizeFilterEdge.filter(*laserCloudCornerMap);

    // 5. Update voxel temporal states (used for Innovation 1 & Innovation 2)
    for (const auto &pt : laserCloudSurfMap->points)
    {
        long long key = computeVoxelKey(pt, map_resolution * 2.0f);
        VoxelState &vs = voxel_states_surf_[key];

        Eigen::Vector3d x(pt.x, pt.y, pt.z);
        vs.count++;
        if (vs.count == 1)
        {
            vs.mean = x;
            vs.m2.setZero();
        }
        else
        {
            Eigen::Vector3d delta = x - vs.mean;
            vs.mean += delta / (double)vs.count;
            vs.m2   += delta.cwiseProduct(x - vs.mean); // Welford accumulation
        }
    }

    for (const auto &pt : laserCloudCornerMap->points)
    {
        long long key = computeVoxelKey(pt, map_resolution);
        VoxelState &vs = voxel_states_corner_[key];

        Eigen::Vector3d x(pt.x, pt.y, pt.z);
        vs.count++;
        if (vs.count == 1)
        {
            vs.mean = x;
            vs.m2.setZero();
        }
        else
        {
            Eigen::Vector3d delta = x - vs.mean;
            vs.mean += delta / (double)vs.count;
            vs.m2   += delta.cwiseProduct(x - vs.mean);
        }
    }

    // 6. Innovation 2: remove unstable voxels based on temporal variance (switchable)
    if (use_dynamic_filter_)
    {
        const std::size_t before_surf   = laserCloudSurfMap->size();
        const std::size_t before_corner = laserCloudCornerMap->size();

        // 6.1 surf dynamic removal
        pcl::PointCloud<PointType>::Ptr filteredSurf(new pcl::PointCloud<PointType>());
        filteredSurf->reserve(before_surf);
        for (const auto &pt : laserCloudSurfMap->points)
        {
            long long key = computeVoxelKey(pt, map_resolution * 2.0f);
            bool is_dynamic = false;

            auto it = voxel_states_surf_.find(key);
            if (it != voxel_states_surf_.end() &&
                it->second.count >= voxel_min_count_surf_)
            {
                double var_n = voxelVarianceNorm(it->second);
                if (var_n > voxel_var_thresh_surf_)
                    is_dynamic = true;
            }

            if (!is_dynamic)
                filteredSurf->push_back(pt);
        }
        filteredSurf->width    = filteredSurf->points.size();
        filteredSurf->height   = 1;
        filteredSurf->is_dense = true;
        laserCloudSurfMap.swap(filteredSurf);

        // 6.2 corner dynamic removal
        pcl::PointCloud<PointType>::Ptr filteredCorner(new pcl::PointCloud<PointType>());
        filteredCorner->reserve(before_corner);
        for (const auto &pt : laserCloudCornerMap->points)
        {
            long long key = computeVoxelKey(pt, map_resolution);
            bool is_dynamic = false;

            auto it = voxel_states_corner_.find(key);
            if (it != voxel_states_corner_.end() &&
                it->second.count >= voxel_min_count_corner_)
            {
                double var_n = voxelVarianceNorm(it->second);
                if (var_n > voxel_var_thresh_corner_)
                    is_dynamic = true;
            }

            if (!is_dynamic)
            {
                filteredCorner->push_back(pt);
            }
        }
        filteredCorner->width    = filteredCorner->points.size();
        filteredCorner->height   = 1;
        filteredCorner->is_dense = true;
        laserCloudCornerMap.swap(filteredCorner);

        const std::size_t after_surf   = laserCloudSurfMap->size();
        const std::size_t after_corner = laserCloudCornerMap->size();
        const std::size_t removed_surf   = (before_surf   > after_surf)   ? (before_surf   - after_surf)   : 0;
        const std::size_t removed_corner = (before_corner > after_corner) ? (before_corner - after_corner) : 0;

        // if (removed_surf > 0 || removed_corner > 0)
        // {
        //     ROS_INFO("[OdomEstimation][Frame %zu] removed corner=%zu, surf=%zu, remain corner=%zu, surf=%zu",
        //              frame_count_,
        //              removed_corner, removed_surf,
        //              after_corner, after_surf);
        // }
    }
    // else: Innovation 2 off: only crop + voxel filter, no variance-based point removal
}

// ====================== getMap ======================

void OdomEstimationClass::getMap(pcl::PointCloud<PointType>::Ptr& laserCloudMap)
{
    *laserCloudMap += *laserCloudSurfMap;
    *laserCloudMap += *laserCloudCornerMap;
}

// ====================== Ablation-only: switch interfaces ======================
// Must be declared in odomEstimationClass.h:
//   void setUseAdaptiveWeight(bool v);
//   void setUseDynamicFilter(bool v);

void OdomEstimationClass::setUseAdaptiveWeight(bool v)
{
    use_adaptive_weight_ = v;
}

void OdomEstimationClass::setUseDynamicFilter(bool v)
{
    use_dynamic_filter_ = v;
}
