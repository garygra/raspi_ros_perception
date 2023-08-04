#!/usr/bin/bash

export ROS_OS_OVERRIDE=debian:bullseye
source /home/pracsys-pi1/ros_catkin_ws/devel_isolated/setup.bash
source /home/pracsys-pi1/raspi_ros_perception/devel/setup.bash
export ROS_IP=192.168.0.150
export ROS_MASTER_URI=http://192.168.0.100:11311/

#source /home/pracsys-pi1/ros_catkin_ws/devel_isolated/setup.bash
#source /home/pracsys-pi1/raspi_ros_perception/devel/setup.bash
exec "$@"
