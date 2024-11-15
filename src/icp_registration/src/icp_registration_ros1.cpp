// std
#include <filesystem>
#include <mutex>
#include <thread>
#include <functional>

// ROS1
#include "ros/ros.h"
#include <ros/console.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>

// TF_ROS1
#include <tf/transform_broadcaster.h>  
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// PCL
#include <pcl_ros/point_cloud.h>  
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>

// Eigen
#include <Eigen/Dense>

// #include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <sensor_msgs/PointCloud2.h>

#include <boost/shared_ptr.hpp>

using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;

std::string pointcloud_topic_;
ros::Subscriber pointcloud_sub_;  
ros::Subscriber initial_pose_sub_;

std::mutex mutex_;
std::unique_ptr<std::thread> tf_publisher_thread_;
pcl::VoxelGrid<pcl::PointXYZI> voxel_rough_filter_;
pcl::VoxelGrid<pcl::PointXYZI> voxel_refine_filter_;
int rough_iter_;
int refine_iter_;
pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_rough_;
pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_refine_;
PointCloudXYZI::Ptr cloud_in_;
PointCloudXYZIN::Ptr rough_map_;
PointCloudXYZIN::Ptr refine_map_;
// robot_pose_pub=nh.advertise<geometry_msgs::PoseStamped>("/robot_pose",10);
                // geometry_msgs::PoseStamped robot_pose;
                // robot_pose.pose.position.x
                // std::lock_guard lock(mutex_);
                // robot_pose_pub.publish(robot_pose);
geometry_msgs::TransformStamped map_to_odom_;
tf::Transform map_to_odom_transform;
std::string pcd_path_;
std::string map_frame_id_, odom_frame_id_, range_odom_frame_id_, laser_frame_id_;
bool success_;
double rough_leaf_size_, refine_leaf_size_;
double score_;
double thresh_;
double xy_offset_;
double yaw_offset_;
double yaw_resolution_;
geometry_msgs::Pose initial_pose_;
bool is_ready_;
bool first_scan_;

void 
void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr msg);
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg);
static PointCloudXYZIN::Ptr addNorm(PointCloudXYZI::Ptr cloud);
Eigen::Matrix4d multiAlignSync(PointCloudXYZI::Ptr source, const Eigen::Matrix4d &init_guess);
void quaternionToEigenMatrix(const tf::Quaternion& q, Eigen::Matrix3d& rotation_matrix);