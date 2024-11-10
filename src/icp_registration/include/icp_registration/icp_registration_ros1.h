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


// TF_ROS2
// #include "tf2/LinearMath/Transform.h"
// #include "tf2_ros/transform_broadcaster.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// PCL
#include <pcl_ros/point_cloud.h>  
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>  

// Eigen
#include<Eigen/Dense>

#include <geometry_msgs/PoseWithCovarianceStamped.h>  
#include <sensor_msgs/PointCloud2.h>

#include <boost/shared_ptr.hpp>



// namespace icp{
using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;

// Get initial map to odom pose estimation using ICP algorithm
// 使用ICP算法获取初始映射到odom姿态估计
class IcpNode{
public:
    IcpNode(ros::NodeHandle& nh);

    ros::NodeHandle& nh_;  
    ros::Subscriber pointcloud_sub_;  
    std::string pointcloud_topic_;
    ros::Subscriber initial_pose_sub_;

  void livox_pcl_cbk(const std::string &msg);

  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  // void pointcloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &msg);
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  // void initialPoseCallback(const boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> &msg);

  /**
   * 传入：pcl::PointCloud<pcl::PointXYZI>::Ptr cloud
   * 传出：PointCloudXYZIN::Ptr out
   * 功能：为点云添加法线
   */
  static PointCloudXYZIN::Ptr addNorm(PointCloudXYZI::Ptr cloud);

  // Eigen::Matrix4d align(PointCloudXYZI::Ptr source,
  //                       const Eigen::Matrix4d &init_guess);

  Eigen::Matrix4d multiAlignSync(PointCloudXYZI::Ptr source, const Eigen::Matrix4d &init_guess);

  void quaternionToEigenMatrix(const tf::Quaternion& q, Eigen::Matrix3d& rotation_matrix);


  // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // tf::TransformBroadcaster tf_broadcaster_;

  tf::TransformBroadcaster tf_broadcaster_;

  tf::TransformListener tf_listener_;
  

  std::mutex mutex_;
  std::unique_ptr<std::thread> tf_publisher_thread_;

  // Voxelfilter used to downsample the pointcloud
  // 使用 Voxelfilter 对点云进行下采样
  /**
   * 类型：pcl::VoxelGrid<pcl::PointXYZI>
   * 含义：粗糙滤波器
   */
  pcl::VoxelGrid<pcl::PointXYZI> voxel_rough_filter_;
  /**
   * 类型：pcl::VoxelGrid<pcl::PointXYZI>
   * 含义：精细滤波器，用于点云采样
   */
  pcl::VoxelGrid<pcl::PointXYZI> voxel_refine_filter_;

  // ICP
  /**
   * 类型：int
   * 含义：粗略配准阶段的迭代次数
   */
  int rough_iter_;
  /**
   * 类型：int
   * 含义：精细配准阶段的迭代次数
   */
  int refine_iter_;
  /**
   * 类型：pcl::IterativeClosestPointWithNormals<PointType, PointType>
   * 含义：粗略ICP/初步配准器
   */
  pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_rough_;
  /**
   * 类型：pcl::IterativeClosestPointWithNormals<PointType, PointType>
   * 含义：精细ICP/精细配准器
   */
  pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_refine_;

  // Store
  /**
   * 类型：PointCloudXYZI::Ptr
   * 含义：输入点云指针
   */
  PointCloudXYZI::Ptr cloud_in_;

  /**
   * 类型：PointCloudXYZI::Ptr
   * 含义：粗处理地图点云指针
   */
  PointCloudXYZIN::Ptr rough_map_;
  /**
   * 类型：PointCloudXYZI::Ptr
   * 含义：精处理后的地图点云指针
   */
  PointCloudXYZIN::Ptr refine_map_;

  /**
   * 类型：geometry_msgs::msg::TransformStamped
   * 含义：map到odom坐标系的变换
   */
  geometry_msgs::TransformStamped map_to_odom_;

  /**
   * 类型：std::filesystem::path
   * 含义：PCD文件路径
   */
  // std::filesystem::path pcd_path_;
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

  /**
   * 类型：bool
   * 含义：节点是否准备完毕
   */
  bool is_ready_;

  /**
   * 类型：bool
   * 含义：是否为第一次扫描
   */
  bool first_scan_;
private:

};
// }