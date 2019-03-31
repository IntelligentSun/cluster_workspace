#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/shansu/Apps/ros/devel/setup.bash
export ROS_MASTER_URI=http://192.168.99.51:11311
export ROS_IP=192.168.99.51
sleep 3
roslaunch yikun_navigation bringup.launch

