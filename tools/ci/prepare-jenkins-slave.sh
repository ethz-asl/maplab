#!/bin/bash -e
echo "Running the prepare script for maplab.";

if [[ $(uname) == "Linux" ]]; then
  sudo apt-get install -y doxygen liblapack-dev libblas-dev autotools-dev \
    dh-autoreconf libboost-all-dev python-setuptools git g++ cppcheck \
    default-jre libreadline-dev libgtest-dev libglew-dev python-git pylint \
    checkstyle python-termcolor liblog4cplus-dev cimg-dev python-wstool \
    python-catkin-tools libssh2-1-dev libatlas3-base libv4l-dev python-scipy

  if lsb_release -c 2> /dev/null | grep trusty > /dev/null ; then
    # Ubuntu 14.04 / ROS Indigo.
    sudo apt-get install -y clang-format-3.4 ros-indigo-camera-info-manager* \
      ros-indigo-rviz-animated-view-controller ros-indigo-octomap-ros
  elif lsb_release -c 2> /dev/null | grep xenial > /dev/null ; then
    # Ubuntu 16.04 / ROS Kinetic.
    sudo apt-get install -y clang-format-3.8 ros-kinetic-camera-info-manager* \
      ros-kinetic-octomap-ros
  else
    echo "Unknown Ubuntu version. Couldn't install all necessary dependencies."
  fi
else
  echo "Platform not supported by the prepare script."
  echo "Please ensure that all necessary dependencies are installed."
fi
