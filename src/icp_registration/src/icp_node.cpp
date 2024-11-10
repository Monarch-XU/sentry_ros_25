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
#include<Eigen/Dense>

#include <geometry_msgs/PoseWithCovarianceStamped.h>  
#include <sensor_msgs/PointCloud2.h>

#include <boost/shared_ptr.hpp>

using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;

std::string pointcloud_topic_;
ros::Subscriber pointcloud_sub_;  
ros::Subscriber initial_pose_sub_;

void livox_pcl_cbk(const std::string &msg);
void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

static PointCloudXYZIN::Ptr addNorm(PointCloudXYZI::Ptr cloud);
Eigen::Matrix4d multiAlignSync(PointCloudXYZI::Ptr source, const Eigen::Matrix4d &init_guess);
void quaternionToEigenMatrix(const tf::Quaternion& q, Eigen::Matrix3d& rotation_matrix);
// tf::TransformBroadcaster tf_broadcaster_;
// tf::TransformListener tf_listener_;
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
geometry_msgs::TransformStamped map_to_odom_;
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



void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    ROS_INFO("get point cloud msg\n");
// void IcpNode::pointcloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &msg) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 将ROS的PointCloud2消息转换为PCL的PointCloud对象
    pcl::fromROSMsg(*msg, *cloud);
    pcl::fromROSMsg(*msg, *cloud_in_);
    if (first_scan_) {
    // auto pose_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    geometry_msgs::PoseWithCovarianceStamped::Ptr pose_msg(new geometry_msgs::PoseWithCovarianceStamped);
    pose_msg->header = msg->header;
    pose_msg->pose.pose = initial_pose_;
    initialPoseCallback(pose_msg);
    first_scan_ = false;
    }
}


void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
    ROS_INFO("get geometry msg\n");
// void IcpNode::initialPoseCallback(const boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> &msg){

    // Set the initial pose
    Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    Eigen::Quaterniond q(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    Eigen::Matrix4d initial_guess;
    initial_guess.block<3, 3>(0, 0) = q.toRotationMatrix();
    initial_guess.block<3, 1>(0, 3) = pos;
    initial_guess(3, 3) = 1;
    ROS_INFO("Aligning the pointcloud\n");
    
    Eigen::Matrix4d map_to_laser = multiAlignSync(cloud_in_, initial_guess);
    if (!success_) {
        map_to_laser = initial_guess;
        ROS_INFO("ICP failed!\n");
    }
    // Eigen::Isometry3d laser_to_odom;
    Eigen::Matrix4d laser_to_odom = Eigen::Matrix4d::Identity();
    try {
        // Get odom to laser transform
        tf::StampedTransform transform;
        // 尝试查找从激光帧到里程计帧的变换，超时时间为10秒
        tf::TransformListener tf_listener_;
        tf_listener_.waitForTransform(laser_frame_id_, range_odom_frame_id_, ros::Time(0), ros::Duration(10));
        tf_listener_.lookupTransform(laser_frame_id_, range_odom_frame_id_, ros::Time(0), transform);
        // RCLCPP_INFO(get_logger(), "%s", transform.header.frame_id.c_str());

        Eigen::Vector3d t(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        tf::Quaternion q(transform.getRotation());
        Eigen::Matrix3d rotation_matrix;
        quaternionToEigenMatrix(q, rotation_matrix);
        // laser_to_odom.prerotate(q);
        // laser_to_odom.translate(t);
        laser_to_odom.block<3, 3>(0, 0) = rotation_matrix;
        laser_to_odom.block<3, 1>(0, 3) = t;
    } catch (tf2::TransformException &ex) {
        std::lock_guard<std::mutex> lock(mutex_);
        ROS_INFO("%s", ex.what());
        is_ready_ = false;
        return;
    }
    
    Eigen::Matrix4d result = map_to_laser * laser_to_odom.matrix().cast<double>();
    {
        // std::lock_guard lock(mutex_);
        std::lock_guard<std::mutex> lock(mutex_);
        map_to_odom_.transform.translation.x = result(0, 3);
        map_to_odom_.transform.translation.y = result(1, 3);
        map_to_odom_.transform.translation.z = result(2, 3);

        Eigen::Matrix3d rotation = result.block<3, 3>(0, 0);
        q = Eigen::Quaterniond(rotation);

        map_to_odom_.transform.rotation.w = q.w();
        map_to_odom_.transform.rotation.x = q.x();
        map_to_odom_.transform.rotation.y = q.y();
        map_to_odom_.transform.rotation.z = q.z();
        is_ready_ = true;
    }
}


void quaternionToEigenMatrix(const tf::Quaternion& q, Eigen::Matrix3d& rotation_matrix) {
    // 设置旋转矩阵的元素
    rotation_matrix(0, 0) = 1 - 2 * q.y() * q.y() - 2 * q.z() * q.z();
    rotation_matrix(0, 1) = 2 * q.x() * q.y() - 2 * q.z() * q.w();
    rotation_matrix(0, 2) = 2 * q.x() * q.z() + 2 * q.y() * q.w();
    rotation_matrix(1, 0) = 2 * q.x() * q.y() + 2 * q.z() * q.w();
    rotation_matrix(1, 1) = 1 - 2 * q.x() * q.x() - 2 * q.z() * q.z();
    rotation_matrix(1, 2) = 2 * q.y() * q.z() - 2 * q.x() * q.w();
    rotation_matrix(2, 0) = 2 * q.x() * q.z() - 2 * q.y() * q.w();
    rotation_matrix(2, 1) = 2 * q.y() * q.z() + 2 * q.x() * q.w();
    rotation_matrix(2, 2) = 1 - 2 * q.x() * q.x() - 2 * q.y() * q.y();
}


Eigen::Matrix4d multiAlignSync(PointCloudXYZI::Ptr source, const Eigen::Matrix4d &init_guess){
    static auto rotate2rpy = [](Eigen::Matrix3d &rot) -> Eigen::Vector3d {
        double roll = std::atan2(rot(2, 1), rot(2, 2));
        double pitch = asin(-rot(2, 0));
        double yaw = std::atan2(rot(1, 0), rot(0, 0));
        return Eigen::Vector3d(roll, pitch, yaw);
    };
    success_ = false;
    Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);
    Eigen::Vector3d rpy = rotate2rpy(rotation);
    Eigen::AngleAxisf rollAngle(rpy(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(rpy(1), Eigen::Vector3f::UnitY());
    std::vector<Eigen::Matrix4f> candidates;
    Eigen::Matrix4f temp_pose;

    ROS_INFO("initial guess: %f, %f, %f, %f, %f, %f",xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2));
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            for (int k = -yaw_offset_; k <= yaw_offset_; k++) {
                Eigen::Vector3f pos(xyz(0) + i * xy_offset_, xyz(1) + j * xy_offset_,
                                    xyz(2));
                Eigen::AngleAxisf yawAngle(rpy(2) + k * yaw_resolution_,
                                        Eigen::Vector3f::UnitZ());
                temp_pose.setIdentity();
                temp_pose.block<3, 3>(0, 0) =
                    (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
                temp_pose.block<3, 1>(0, 3) = pos;
                candidates.push_back(temp_pose);
            }
        }
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr rough_source(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(
        new pcl::PointCloud<pcl::PointXYZI>);

    voxel_rough_filter_.setInputCloud(source);
    voxel_rough_filter_.filter(*rough_source);
    voxel_refine_filter_.setInputCloud(source);
    voxel_refine_filter_.filter(*refine_source);

    PointCloudXYZIN::Ptr rough_source_norm = addNorm(rough_source);
    PointCloudXYZIN::Ptr refine_source_norm = addNorm(refine_source);
    PointCloudXYZIN::Ptr align_point(new PointCloudXYZIN);

    Eigen::Matrix4f best_rough_transform;
    double best_rough_score = 10.0;
    bool rough_converge = false;
    auto tic = std::chrono::system_clock::now();
    for (Eigen::Matrix4f &init_pose : candidates) {
        icp_rough_.setInputSource(rough_source_norm);
        icp_rough_.align(*align_point, init_pose);
        if (!icp_rough_.hasConverged()) continue;
        double rough_score = icp_rough_.getFitnessScore();
        if (rough_score > 2 * thresh_) continue;
        if (rough_score < best_rough_score) {
            best_rough_score = rough_score;
            rough_converge = true;
            best_rough_transform = icp_rough_.getFinalTransformation();
        }
    }

    if (!rough_converge) return Eigen::Matrix4d::Zero();

    icp_refine_.setInputSource(refine_source_norm);
    icp_refine_.align(*align_point, best_rough_transform);
    score_ = icp_refine_.getFitnessScore();

    if (!icp_refine_.hasConverged()) return Eigen::Matrix4d::Zero();
    if (score_ > thresh_) return Eigen::Matrix4d::Zero();
    success_ = true;
    auto toc = std::chrono::system_clock::now();
    std::chrono::duration<double> duration = toc - tic;
    ROS_INFO("align used: %f ms\n", duration.count() * 1000);
    ROS_INFO("score: %f\n", score_);

    return icp_refine_.getFinalTransformation().cast<double>();
}


// 给点云添加法线
PointCloudXYZIN::Ptr addNorm(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZI>);
    searchTree->setInputCloud(cloud);

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud);
    normalEstimator.setSearchMethod(searchTree);
    normalEstimator.setKSearch(15);
    normalEstimator.compute(*normals);
    PointCloudXYZIN::Ptr out(new PointCloudXYZIN);
    pcl::concatenateFields(*cloud, *normals, *out);
    return out;
}



int main(int argc, char *argv[])  {  
    ros::init(argc,argv,"icp_node");  

    // ros::NodeHandle nh;
    ros::NodeHandle nh("~");

    setlocale(LC_ALL,"");

    is_ready_ = false;

    // 初始化一个指向pcl::PointCloud<pcl::PointXYZI>的智能指针，用于存储输入的点云数据
    cloud_in_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    //// TODO:改成用服务器读参
    // double rough_leaf_size = (double)fs["rough_leaf_size"];
    // double refine_leaf_size = (double)fs["refine_leaf_size"];
    double rough_leaf_size, refine_leaf_size;
    nh.param<double>("rough_leaf_size", rough_leaf_size, 0.4);
    nh.param<double>("refine_leaf_size", refine_leaf_size, 0.1);

    ROS_INFO("refine_leaf_size: %lf\n",refine_leaf_size);


    // 设置粗糙和精细体素滤波器叶大小。
    voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size, rough_leaf_size);
    voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size, refine_leaf_size);

    // 读取PCD文件路径参数
    nh.param<std::string>("pcd_path", pcd_path_, "/home/hj/sentry_ros_25/PCD/205lib.pcd");
   
    //// TODO:
    // if (!std::filesystem::exists(pcd_path_)) {
    //     ROS_ERROR("Invalid pcd path: %s",pcd_path_.c_str());
    //     throw std::runtime_error("Invalid pcd path");
    // }
    // 用PCL的PCDReader类读取PCD文件，并将点云数据存储在cloud指针指向的对象中
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    reader.read(pcd_path_, *cloud);
    // 将读取的点云数据作为输入传递给精细体素滤波器，并应用滤波器进行下采样
    voxel_refine_filter_.setInputCloud(cloud);
    voxel_refine_filter_.filter(*cloud);

    // Add normal to the pointcloud
    // 调用addNorm函数为精细处理后的点云添加法线信息
    refine_map_ = addNorm(cloud);
    // 创建粗糙处理点云的指针
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_rough(new pcl::PointCloud<pcl::PointXYZI>); 
    // 创建滤波后粗糙点云的指针
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_point_rough(new pcl::PointCloud<pcl::PointXYZI>);
    // 复制精细处理后的点云到粗糙处理点云(不复制法线)
    pcl::copyPointCloud(*refine_map_, *point_rough);
    /*调用voxel_rough_filter_的filter方法，对point_rough中的点进行下采样处理，并将结果存储在filterd_point_rough中。
    经过这一步，filterd_point_rough中的点数量会减少，点的分布会变得更加稀疏，但整体上仍然保留了原始点云的形状特征。*/
    voxel_rough_filter_.setInputCloud(point_rough);
    voxel_rough_filter_.filter(*filterd_point_rough);
    // 为滤波后的粗糙点云添加法线信息
    rough_map_ = addNorm(filterd_point_rough);
    // 设置粗略ICP/初步配准器的最大迭代次数为rough_iter_
    icp_rough_.setMaximumIterations(rough_iter_);
    // 设置精细ICP/精细配准器的目标点云为rough_map_
    icp_rough_.setInputTarget(rough_map_);
    // 设置精细ICP/精细配准器的最大迭代次数为refine_iter_
    icp_refine_.setMaximumIterations(refine_iter_);
    // 设置精细ICP/精细配准器的目标点云为refine_map_
    icp_refine_.setInputTarget(refine_map_);

    // 打印点云大小
    ROS_INFO("PCD大小： %ld, %ld", refine_map_->size(), rough_map_->size());

    // Parameters
    //// TODO:改成用服务器读参
    nh.param<std::string>("map_frame_id", map_frame_id_, "map");
    nh.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
    nh.param<std::string>("range_odom_frame_id", range_odom_frame_id_, "range_odom");
    nh.param<std::string>("laser_frame_id", laser_frame_id_, "laser");
    nh.param<double>("thresh", thresh_,  0.15);
    nh.param<double>("xy_offset", xy_offset_,  0.2);
    nh.param<double>("yaw_offset", yaw_offset_,  30.0);
    yaw_offset_ *= M_PI / 180.0;

    std::vector<double> initial_pose_vec; 
    nh.param<std::vector<double>>("initial_pose", initial_pose_vec,  {0, 0, 0, 0, 0, 0});

    try {
    // 设置初始位置三维坐标
    initial_pose_.position.x = initial_pose_vec.at(0);
    initial_pose_.position.y = initial_pose_vec.at(1);
    initial_pose_.position.z = initial_pose_vec.at(2);
    tf::Quaternion q; 
    // 通过四元数设置初始位置row、pitch、yaw角度
    q.setRPY(initial_pose_vec.at(3), initial_pose_vec.at(4), initial_pose_vec.at(5));
    } catch (const std::out_of_range &ex) {
        // 若initial_pose_vec不是一个包含6个元素的向量，报错
        ROS_INFO("initial_pose is not a vector with 6 elements, what(): %s", ex.what()); 
    }

    std::string pointcloud_topic;
    
    nh.param<std::string>("pointcloud_topic", pointcloud_topic,  "/livox/points");

    ROS_INFO("pointcloud_topic: %s",pointcloud_topic.c_str());



    pointcloud_sub_ = nh.subscribe(pointcloud_topic,10,pointcloudCallback);
    
    initial_pose_sub_ = nh.subscribe("/initialpose",10,initialPoseCallback);

    tf_publisher_thread_ = std::make_unique<std::thread>([]() {
        ros::Rate rate(100);
        // while (ros::ok()) {
        if (ros::ok()) {
            {
                // std::lock_guard lock(mutex_);
                std::lock_guard<std::mutex> lock(mutex_);
                if (is_ready_) {
                    map_to_odom_.header.stamp = ros::Time::now();
                    map_to_odom_.header.frame_id = map_frame_id_;
                    map_to_odom_.child_frame_id = odom_frame_id_;
                    // 通过tf_broadcaster_对象发送TF变换信息到TF树中
                    tf::TransformBroadcaster tf_broadcaster_;
                    tf_broadcaster_.sendTransform(map_to_odom_);
                }
            }
            rate.sleep();
        }
    });



    ROS_INFO("icp_registration: initialized\n");


  
    ros::spin(); // 保持节点运行，直到接收到关闭信号  


    return 0;  
}
