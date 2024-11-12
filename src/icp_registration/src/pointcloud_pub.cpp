// 测试文件

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>



int main(int argc, char** argv) {

    std::string pcd_file_path = "PCD/205lib.pcd";
    std::string topic_name = "/livox/points";

    // 初始化ROS节点
    ros::init(argc, argv, "pcd_to_ros_publisher");
    ros::NodeHandle nh;

    // 创建发布者
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1);

    // 读取PCD文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1) {
        ROS_ERROR("Couldn't read file %s \n", pcd_file_path.c_str());
        return 0;
    }

    // 转换点云数据为ROS消息
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    ros_cloud.header.stamp = ros::Time::now();
    ros_cloud.header.frame_id = "map"; // 或其他适当的帧ID

    while (ros::ok())
    {
        pub.publish(ros_cloud);
        ros::spinOnce();
        ROS_INFO("pub point cloud!\n");
    }
    

    return 0;
}