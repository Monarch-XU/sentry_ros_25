#!/bin/bash
source /opt/ros/noetic/setup.bash
cd /home/hj/sentry_ros_25
source devel/setup.bash
rosrun icp_registration pointcloud_pub &
rosrun sentry_navigation initial_pose_pub &
