#include "laserProcessingClass.h"

// For disable PCL compile lib, to use PointXYZI with custom fields if needed
#define PCL_NO_PRECOMPILE

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <cmath>
#include <algorithm>
#include <memory>

// ============================================================================
// LaserProcessingClass
// ============================================================================

LaserProcessingClass::LaserProcessingClass() {}

void LaserProcessingClass::init(lidar::Lidar lidar_param_in)
{
    lidar_param = lidar_param_in;
}

// Main feature extraction function
// Requirements:
//   - Do not distinguish ground / non-ground; extract features from all points together
//   - Refer to A-LOAM:
//        * Curvature calculation
//        * cloudNeighborPicked preprocessing (occluded points / parallel-beam denoising)
//        * Sector-based selection of edge points / planar points
//   - Output:
//        pc_out_edge -> similar to cornerPointsLessSharp
//        pc_out_surf -> similar to surfPointsLessFlat (voxel filtering within each scan)
void LaserProcessingClass::featureExtraction(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_edge,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_surf)
{
    pc_out_edge->clear();
    pc_out_surf->clear();

    if (!pc_in || pc_in->empty())
    {
        // ROS_WARN("[LaserProcessing] input cloud is empty.");
        pc_out_edge->width = pc_out_edge->points.size();
        pc_out_edge->height = 1;
        pc_out_edge->is_dense = true;
        pc_out_surf->width = pc_out_surf->points.size();
        pc_out_surf->height = 1;
        pc_out_surf->is_dense = true;
        return;
    }

    // 1. Remove NaN
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc_in, *cloud, indices);

    int N_SCANS = lidar_param.num_lines;
    if (N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        // ROS_ERROR("[LaserProcessing] lidar_param.num_lines = %d, only support 16 / 32 / 64", N_SCANS);
        // return;
    }

    // 2. Split point cloud by scan line
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudScans;
    laserCloudScans.resize(N_SCANS);
    for (int i = 0; i < N_SCANS; ++i)
    {
        laserCloudScans[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    const double min_d = lidar_param.min_distance;
    const double max_d = lidar_param.max_distance;

    for (std::size_t i = 0; i < cloud->points.size(); ++i)
    {
        pcl::PointXYZI pt = cloud->points[i];
        double xy_dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        if (xy_dist < min_d || xy_dist > max_d)
            continue;

        double angle = std::atan2(pt.z, xy_dist) * 180.0 / M_PI;
        int scanID = 0;

        if (N_SCANS == 16)
        {
            scanID = int((angle + 15.0) / 2.0 + 0.5);
            if (scanID < 0 || scanID > N_SCANS - 1) continue;
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
            if (scanID < 0 || scanID > N_SCANS - 1) continue;
        }
        else if (N_SCANS == 64)
        {
            if (angle >= -8.83)
                scanID = int((2.0 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);
            if (angle > 2.0 || angle < -24.33 || scanID < 0 || scanID > 50)
                continue;
        }

        laserCloudScans[scanID]->points.push_back(pt);
    }

    // 3. For each scan, compute curvature, do occlusion/noise preprocessing, and select edge/surf points
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
    downSizeFilterSurf.setLeafSize(0.2f, 0.2f, 0.2f);

    for (int i = 0; i < N_SCANS; ++i)
    {
        auto &scanCloud = laserCloudScans[i];
        int cloudSize = (int)scanCloud->points.size();
        if (cloudSize < 11)
            continue;

        std::vector<float> cloudCurvature(cloudSize, 0.0f);
        std::vector<int>   cloudNeighborPicked(cloudSize, 0);
        std::vector<int>   cloudLabel(cloudSize, 0);
        std::vector<int>   cloudSortInd(cloudSize, 0);

        // 3.1 Curvature
        for (int j = 5; j < cloudSize - 5; ++j)
        {
            float diffX =
                scanCloud->points[j - 5].x +
                scanCloud->points[j - 4].x +
                scanCloud->points[j - 3].x +
                scanCloud->points[j - 2].x +
                scanCloud->points[j - 1].x -
                10 * scanCloud->points[j].x +
                scanCloud->points[j + 1].x +
                scanCloud->points[j + 2].x +
                scanCloud->points[j + 3].x +
                scanCloud->points[j + 4].x +
                scanCloud->points[j + 5].x;

            float diffY =
                scanCloud->points[j - 5].y +
                scanCloud->points[j - 4].y +
                scanCloud->points[j - 3].y +
                scanCloud->points[j - 2].y +
                scanCloud->points[j - 1].y -
                10 * scanCloud->points[j].y +
                scanCloud->points[j + 1].y +
                scanCloud->points[j + 2].y +
                scanCloud->points[j + 3].y +
                scanCloud->points[j + 4].y +
                scanCloud->points[j + 5].y;

            float diffZ =
                scanCloud->points[j - 5].z +
                scanCloud->points[j - 4].z +
                scanCloud->points[j - 3].z +
                scanCloud->points[j - 2].z +
                scanCloud->points[j - 1].z -
                10 * scanCloud->points[j].z +
                scanCloud->points[j + 1].z +
                scanCloud->points[j + 2].z +
                scanCloud->points[j + 3].z +
                scanCloud->points[j + 4].z +
                scanCloud->points[j + 5].z;

            cloudCurvature[j] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            cloudSortInd[j] = j;
            cloudNeighborPicked[j] = 0;
            cloudLabel[j] = 0;
        }

        // 3.2 A-LOAM-style noise preprocessing: occluded points / parallel beams
        for (int j = 5; j < cloudSize - 6; ++j)
        {
            const auto &pt    = scanCloud->points[j];
            const auto &pt_n1 = scanCloud->points[j + 1];

            float depth1 = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            float depth2 = std::sqrt(pt_n1.x * pt_n1.x + pt_n1.y * pt_n1.y + pt_n1.z * pt_n1.z);

            if (depth1 < 0.1f || depth2 < 0.1f)
                continue;

            float diff1 = depth2 - depth1;

            // Occlusion: large depth difference between adjacent points
            if (diff1 > 0.3f)
            {
                float ratio = diff1 / depth1;
                if (ratio > 0.1f)
                {
                    for (int k = 1; k <= 5 && j + k < cloudSize; ++k)
                        cloudNeighborPicked[j + k] = 1;
                }
            }
            else if (-diff1 > 0.3f)
            {
                float ratio = -diff1 / depth2;
                if (ratio > 0.1f)
                {
                    for (int k = 0; k <= 5 && j - k >= 0; ++k)
                        cloudNeighborPicked[j - k] = 1;
                }
            }

            // Parallel beam: very close in space, but direction is essentially the same
            float diffX = pt_n1.x - pt.x;
            float diffY = pt_n1.y - pt.y;
            float diffZ = pt_n1.z - pt.z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ < 1e-4f)
            {
                cloudNeighborPicked[j] = 1;
                cloudNeighborPicked[j + 1] = 1;
            }
        }

        // 3.3 Split into 6 sectors for feature extraction
        pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>());

        int total_points = cloudSize - 10; // remove the first and last 5 points
        for (int s = 0; s < 6; ++s)
        {
            int sector_start = 5 + total_points * s / 6;
            int sector_end   = 5 + total_points * (s + 1) / 6 - 1;
            if (sector_end >= cloudSize - 5)
                sector_end = cloudSize - 6;
            if (sector_start >= sector_end)
                continue;

            // Local sort
            std::sort(cloudSortInd.begin() + sector_start,
                      cloudSortInd.begin() + sector_end + 1,
                      [&cloudCurvature](int i, int j)
                      {
                          return cloudCurvature[i] < cloudCurvature[j];
                      });

            // Select edge points first (high curvature)
            int largestPickedNum = 0;
            for (int k = sector_end; k >= sector_start; --k)
            {
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1f)
                {
                    largestPickedNum++;
                    if (largestPickedNum <= 20) // max 20 edge points per sector
                    {
                        cloudLabel[ind] = 1; // edge
                        pc_out_edge->push_back(scanCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;

                    // Neighborhood suppression (forward)
                    for (int l = 1; l <= 5; ++l)
                    {
                        int ind_n = ind + l;
                        if (ind_n >= cloudSize) break;
                        float diffX = scanCloud->points[ind_n].x - scanCloud->points[ind_n - 1].x;
                        float diffY = scanCloud->points[ind_n].y - scanCloud->points[ind_n - 1].y;
                        float diffZ = scanCloud->points[ind_n].z - scanCloud->points[ind_n - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05f)
                            break;
                        cloudNeighborPicked[ind_n] = 1;
                    }
                    // Neighborhood suppression (backward)
                    for (int l = -1; l >= -5; --l)
                    {
                        int ind_n = ind + l;
                        if (ind_n < 0) break;
                        float diffX = scanCloud->points[ind_n].x - scanCloud->points[ind_n + 1].x;
                        float diffY = scanCloud->points[ind_n].y - scanCloud->points[ind_n + 1].y;
                        float diffZ = scanCloud->points[ind_n].z - scanCloud->points[ind_n + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05f)
                            break;
                        cloudNeighborPicked[ind_n] = 1;
                    }
                }
            }

            // Then select planar points: points not marked as edge and with small curvature
            for (int k = sector_start; k <= sector_end; ++k)
            {
                int ind = cloudSortInd[k];
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1f)
                {
                    cloudLabel[ind] = -1; // surf candidate
                }
            }

            // Put all "non-edge" points of this sector into less-flat set; overall voxel downsampling later
            for (int k = sector_start; k <= sector_end; ++k)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(scanCloud->points[k]);
                }
            }
        }

        // Voxel downsample less-flat points per scan once, then add to global surf
        pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;
        downSizeFilterSurf.setInputCloud(surfPointsLessFlatScan);
        downSizeFilterSurf.filter(surfPointsLessFlatScanDS);

        *pc_out_surf += surfPointsLessFlatScanDS;
    }

    // Finalize width / height
    pc_out_edge->width  = pc_out_edge->points.size();
    pc_out_edge->height = 1;
    pc_out_edge->is_dense = true;

    pc_out_surf->width  = pc_out_surf->points.size();
    pc_out_surf->height = 1;
    pc_out_surf->is_dense = true;

    // ROS_INFO("[LaserProcessing] featureExtraction done: edge = %zu, surf = %zu",
    //          pc_out_edge->points.size(), pc_out_surf->points.size());
}

// If this function is still declared in the header, provide an empty implementation to avoid linker errors
void LaserProcessingClass::featureExtractionFromSector(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &/*pc_in*/,
    std::vector<Double2d> &/*cloudCurvature*/,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &/*pc_out_edge*/,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &/*pc_out_surf*/)
{
    // This function is no longer used in the current version
}

// ============================================================================
// Small struct constructors
// ============================================================================

Double2d::Double2d(int id_in, double value_in)
{
    id    = id_in;
    value = value_in;
}

PointsInfo::PointsInfo(int layer_in, double time_in)
{
    layer = layer_in;
    time  = time_in;
}
