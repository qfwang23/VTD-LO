//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//local lib
#include "lidar.h"
#include "odomEstimationClass.h"

// ==================== Header section ====================
#include <fstream>   // Added: for writing TUM trajectory file
#include <iomanip>   // Added: control floating-point output precision

// Global variables related to TUM trajectory file
std::ofstream tum_file;         // TUM trajectory output file
bool tum_file_inited = false;   // Whether initialized (file opened)
std::string tum_file_path = "/home/wqf/MyPFilter(V3)/result/trajectory.txt";  // Can be overridden by param


// Registration timing statistics
std::size_t total_frame = 0;    // Total number of frames
double total_time_ms = 0.0;     // Total registration time (ms)

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
lidar::Lidar lidar_param;

ros::Publisher pubLaserOdometry;
ros::Publisher pubmapSurfPoints;
ros::Publisher pubmapEdgePoints;

void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    std::lock_guard<std::mutex> lock(mutex_lock);
    pointCloudSurfBuf.push(laserCloudMsg);
}

void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    std::lock_guard<std::mutex> lock(mutex_lock);
    pointCloudEdgeBuf.push(laserCloudMsg);
}

bool is_odom_inited = false;

void odom_estimation(){
    while(1){

        // Initialize the TUM file inside odom_estimation() (only once)
        if (!tum_file_inited) {
            tum_file.open(tum_file_path.c_str(), std::ios::out);
            if (!tum_file.is_open()) {
                ROS_ERROR("Failed to open TUM trajectory file: %s", tum_file_path.c_str());
            } else {
                tum_file << std::fixed << std::setprecision(9);
                ROS_INFO("TUM trajectory will be saved to: %s", tum_file_path.c_str());
            }
            tum_file_inited = true;
        }


        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){

            //read data
            mutex_lock.lock();
            if(!pointCloudSurfBuf.empty() &&
               (pointCloudSurfBuf.front()->header.stamp.toSec()
                < pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudSurfBuf.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }

            if(!pointCloudEdgeBuf.empty() &&
               (pointCloudEdgeBuf.front()->header.stamp.toSec()
                < pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudEdgeBuf.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }
            //if time aligned 
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_tmp);
            pcl::copyPointCloud(*pointcloud_tmp, *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_tmp);
            pcl::copyPointCloud(*pointcloud_tmp, *pointcloud_surf_in);

            ros::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutex_lock.unlock();

            if(is_odom_inited == false){
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                ROS_INFO("odom inited");
            }else{
                // Timing + print per-frame time, frame count, average FPS
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();

                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);

                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;

                float curr_time_ms = elapsed_seconds.count() * 1000.0f;

                total_frame++;
                total_time_ms += curr_time_ms;

                double avg_time_ms = total_time_ms / static_cast<double>(total_frame);
                double avg_fps     = 1000.0 / avg_time_ms;

                ROS_INFO("odom estimation: current = %.3f ms, frame = %zu, avg = %.3f ms, avg FPS = %.2f Hz",
                         curr_time_ms, total_frame, avg_time_ms, avg_fps);
            }

            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            Eigen::Vector3d    t_current = odomEstimation.odom.translation();

            // Write one line of TUM trajectory
            if (tum_file_inited && tum_file.is_open()) {
                double timestamp = pointcloud_time.toSec();
                tum_file << timestamp << " "
                         << t_current.x() << " " 
                         << t_current.y() << " " 
                         << t_current.z() << " "
                         << q_current.x() << " " 
                         << q_current.y() << " " 
                         << q_current.z() << " " 
                         << q_current.w() << "\n";

                tum_file.flush();   // Force flush to disk every frame
            }

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map";
            laserOdometry.child_frame_id = "base_link";
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();
            pubLaserOdometry.publish(laserOdometry);

            sensor_msgs::PointCloud2 cloudMsg;
            if(pubmapEdgePoints.getNumSubscribers() > 0){
                pcl::toROSMsg(*odomEstimation.laserCloudCornerMap, cloudMsg);
                cloudMsg.header.stamp = pointcloud_time;
                cloudMsg.header.frame_id = "/map";
                pubmapEdgePoints.publish(cloudMsg);
            }
            
            if(pubmapSurfPoints.getNumSubscribers() > 0){
                pcl::toROSMsg(*odomEstimation.laserCloudSurfMap, cloudMsg);
                cloudMsg.header.stamp = pointcloud_time;
                cloudMsg.header.frame_id = "/map";
                pubmapSurfPoints.publish(cloudMsg);
            }
        }

        //sleep 2 ms every time
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");   // Private handle, corresponding to <param> inside <node>

    int    scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period    = 0.1;
    double max_dis        = 60.0;
    double min_dis        = 2.0;
    double map_resolution = 0.4;

    // === Original parameters (global params) ===
    nh.getParam("/scan_period",     scan_period); 
    nh.getParam("/vertical_angle",  vertical_angle); 
    nh.getParam("/max_dis",         max_dis);
    nh.getParam("/min_dis",         min_dis);
    nh.getParam("/scan_line",       scan_line);
    nh.getParam("/map_resolution",  map_resolution);

    // === Parameters related to dynamic point filtering (global params) ===
    double voxel_var_thresh_corner = 0.0125;
    double voxel_var_thresh_surf   = 0.025;
    int    voxel_min_count_corner  = 5;
    int    voxel_min_count_surf    = 5;

    nh.param("voxel_var_thresh_corner", voxel_var_thresh_corner, voxel_var_thresh_corner);
    nh.param("voxel_var_thresh_surf",   voxel_var_thresh_surf,   voxel_var_thresh_surf);
    nh.param("voxel_min_count_corner",  voxel_min_count_corner,  voxel_min_count_corner);
    nh.param("voxel_min_count_surf",    voxel_min_count_surf,    voxel_min_count_surf);

    // === Ablation switches (private params) ===
    // Corresponding launch:
    // <node ...>
    //   <param name="use_adaptive_weight" type="bool" value="true" />
    //   <param name="use_dynamic_filter"  type="bool" value="true" />
    // </node>
    bool use_adaptive_weight = true;
    bool use_dynamic_filter  = true;
    pnh.param("use_adaptive_weight", use_adaptive_weight, true);  // Innovation 1: adaptive weighting
    pnh.param("use_dynamic_filter",  use_dynamic_filter,  true);  // Innovation 2: dynamic point removal

    // === TUM trajectory output path (private param) ===
    // Recommended naming: ua_[true|false]_ud_[true|false]_seq_[00-10].txt
    pnh.param("tum_path", tum_file_path, tum_file_path);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    odomEstimation.init(lidar_param,
                        map_resolution,
                        voxel_var_thresh_corner,
                        voxel_var_thresh_surf,
                        voxel_min_count_corner,
                        voxel_min_count_surf);

    // Set the two innovation switches (for ablation experiments)
    odomEstimation.setUseAdaptiveWeight(use_adaptive_weight);
    odomEstimation.setUseDynamicFilter(use_dynamic_filter);

    ros::Subscriber subEdgeLaserCloud =
        nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud =
        nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    pubmapSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/surf_local_map", 1000); 
    pubmapEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/edge_local_map", 1000); 
    
    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    return 0;
}
