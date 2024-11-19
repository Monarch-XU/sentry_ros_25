// 测试文件

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

int main(int argc, char** argv) {

    std::string pcd_file_path = "/home/hj/sentry_ros_25/PCD/205lib.pcd";
    std::string topic_name = "/livox/points";

    // 初始化ROS节点
    ros::init(argc, argv, "pointcloud_pub");
    ros::NodeHandle nh;


    // The point clouds we will be using

    PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
    PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

    // 创建发布者
    // sensor_msgs::PointCloud2ConstPtr msg
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1);

    // 读取PCD文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1) {
        ROS_ERROR("Couldn't read file %s \n", pcd_file_path.c_str());
        return 0;
    }


    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    double theta = M_PI / 14;  // The angle of rotation in radians
    transformation_matrix (0, 0) = std::cos (theta);
    transformation_matrix (0, 1) = -sin (theta);
    transformation_matrix (1, 0) = sin (theta);
    transformation_matrix (1, 1) = std::cos (theta);

    // A translation on Z axis (0.4 meters)
    transformation_matrix(0, 3) = 6;  // Translation on X axis
    transformation_matrix(1, 3) = -5; // Translation on Y axis
    transformation_matrix(2, 3) = 1;  // Translation on Z axis

    // Display in terminal the transformation matrix
    std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
    print4x4Matrix (transformation_matrix);

    // Executing the transformation
    pcl::transformPointCloud (*cloud, *cloud_icp, transformation_matrix);
    *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

    // 转换点云数据为ROS消息
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud_icp, ros_cloud);
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