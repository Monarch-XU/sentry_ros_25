#include <thread>
#include <pcl_ros/point_cloud.h>  
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include "ros/ros.h"
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;


void showPointCloud(PointCloudXYZI::Ptr cloud, std::string name, int R, int G, int B);
void showPointCloud(PointCloudXYZIN::Ptr cloud, std::string name, int R, int G, int B);
void pclViewer(PointCloudXYZIN::Ptr prior_point_cloud, PointCloudXYZIN::Ptr pointcloud_before_tf, PointCloudXYZIN::Ptr pointcloud_after_tf){
