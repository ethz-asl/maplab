#!/usr/bin/env zsh

# Install ROS melodic
apt -y install lsb-release

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - 2> /dev/null
apt -y update
apt -y install ros-melodic-desktop-full ros-melodic-tf2-* ros-melodic-camera-info-manager* python-catkin-tools
