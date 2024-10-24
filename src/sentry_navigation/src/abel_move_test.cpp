#include"chassis_move.h"

int main(int argc,char **argv)
{
    // ros::NodeHandle nh("~");
    ros::init(argc, argv, "abel_move_test");
    ros::NodeHandle nh("~");
    ros::Publisher cmd_vel_pub_;
    // cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/abel_cmd_vel",10);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    cmd_vel.linear.z = 1;   // bool success or not
    cmd_vel_pub_.publish(cmd_vel);
    ROS_INFO("Planning Success!\n");
    
    bool cmd_vel_init_ok = (cmd_vel.linear.z = 1) ? true : false;

    if (cmd_vel_init_ok)
    {
        cmd_vel.linear.x = 0.1;
        for (int i = 0; i < 30; ++i) {
            cmd_vel_pub_.publish(cmd_vel);
            ROS_INFO("Start moving! x = %f\n",cmd_vel.linear.x);
            ros::Duration(0.1).sleep(); // 暂停0.1秒
        }
        cmd_vel.linear.x = 0;
        for (int i = 0; i < 10; ++i) {
            cmd_vel_pub_.publish(cmd_vel);
            ROS_INFO("Stop !\n");
            ros::Duration(0.1).sleep(); // 暂停0.1秒
        }
    }
    
    ROS_INFO("Moving complite !\n");
    
    ros::spin();
    return 0;

}
