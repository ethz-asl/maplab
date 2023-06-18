#!/bin/bash -e
echo "Running the prepare script for maplab.";

sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y autotools-dev ccache doxygen dh-autoreconf git \
                        liblapack-dev libblas-dev libgtest-dev libreadline-dev \
                        libssh2-1-dev libatlas3-base libv4l-dev libjpeg-dev \
                        python3 python3-catkin-tools python3-pip python-git-doc \
                        python3-setuptools python3-termcolor python3-wstool

# Python package for end to end test
pip3 install --user --upgrade pip
pip3 install --user requests evo opencv-python opencv-contrib-python tqdm pillow \
                    numpy matplotlib scikit-learn torch torchvision
