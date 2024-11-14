#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "math.h"

double x_pos;
double y_pos;
double z_pos;
double x_q;
double y_q;
double z_q;
double w_q;


#define TEST

int main(int argc, char **argv)
{
    ros::init(argc, argv, "initial_pose_pub");
    ros::NodeHandle nh;
    ros::Publisher init_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_pub", 1); // initialpose, 改之前没有"/"

    nh.param<double>("position_x", x_pos, 0);
    nh.param<double>("position_y", y_pos, 0);
    nh.param<double>("position_z", z_pos, 0);
    nh.param<double>("orientation_x", x_q, 0);
    nh.param<double>("orientation_y", y_q, 0);
    nh.param<double>("orientation_z", z_q, 0);
    nh.param<double>("orientation_w", w_q, 0);

    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    while (ros::ok())
    {
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = x_pos;
        pose_msg.pose.pose.position.y = y_pos;
        pose_msg.pose.pose.position.z = z_pos;
        pose_msg.pose.covariance[0] = 0;
        pose_msg.pose.covariance[6 * 1 + 1] = 0;
        pose_msg.pose.covariance[6 * 5 + 5] = 0.0;
        pose_msg.pose.pose.orientation.x = x_q;
        pose_msg.pose.pose.orientation.y = y_q;
        pose_msg.pose.pose.orientation.z = z_q;
        pose_msg.pose.pose.orientation.w = w_q;
        init_pose_pub.publish(pose_msg);
        ROS_INFO("Setting to :(%f,%f)", x_pos, y_pos);
       
    }
    return 0;
}
