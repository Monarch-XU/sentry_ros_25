#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  // 初始化ROS节点
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  // 创建一个tf广播器
  tf::TransformBroadcaster br;
  tf::Transform transform;

  tf::Transform transform2;

  ros::Rate rate(10.0); // 设置发布频率为10Hz

  while (node.ok()){
    // 设置变换
    transform.setOrigin(tf::Vector3(1.0, 0.0, 0.0)); // 例如，从map到odom的平移是(1, 0, 0)
    tf::Quaternion q;
    q.setRPY(0, 0, 1.57); // 例如，从map到odom的旋转是90度（1.57弧度）绕Z轴
    transform.setRotation(q);

    transform2.setOrigin(tf::Vector3(0.0, 1.0, 0.0));
    tf::Quaternion q2;
    q2.setRPY(0, 1, 0); // 例如，从map到odom的旋转是90度（1.57弧度）绕Z轴
    transform2.setRotation(q2);

    // 发送变换
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
    br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "map", "odom2"));

    ROS_INFO("tf pub!\n");

    rate.sleep();
  }

  return 0;
}