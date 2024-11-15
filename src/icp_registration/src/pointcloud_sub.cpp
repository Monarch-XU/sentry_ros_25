#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>  
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>

using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
PointCloudXYZI::Ptr cloud_in_; 

// 回调函数，当接收到点云消息时被调用
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 计算点的数量，对于PointCloud2，点的数量是height * width（除非height为1，表示无序点云）
    size_t pointCount = msg->height * msg->width;
    cloud_in_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud_in_);
    // 打印点云大小
    ROS_INFO("Received PointCloud2 with %lu points.", pointCount);
    ROS_INFO("pointcloud size: %ld\n",cloud_in_->points.size());
    
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "pointcloud_sub");
    
    // 创建节点句柄
    ros::NodeHandle nh;
    
    // 订阅点云主题，假设主题名为"/pointcloud"
    // 根据你的实际情况修改主题名
    ros::Subscriber sub = nh.subscribe("/livox/points", 1, pointCloudCallback);
    
    // 进入ROS的spin循环，等待并处理回调
    ros::spin();
    
    return 0;
}