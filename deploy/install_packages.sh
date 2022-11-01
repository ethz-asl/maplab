#!/usr/bin/env zsh

apt -y install clang-format-6.0 ros-melodic-pcl-conversions \
  libpcl-dev libnlopt-dev ros-melodic-pcl-ros wget \
  ccache liblapack-dev libblas-dev libgtest-dev libreadline-dev libssh2-1-dev pylint python-autopep8 python-pip python-git python-setuptools libatlas3-base libv4l-dev

pip install requests
