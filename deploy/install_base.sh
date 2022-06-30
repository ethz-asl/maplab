#!/usr/bin/env zsh

# Base install
apt update
apt -y upgrade
apt -y install doxygen autotools-dev \
    dh-autoreconf libboost-all-dev python-setuptools git g++ cppcheck \
    libgtest-dev python-git pylint \
    python-termcolor liblog4cplus-dev cimg-dev \
