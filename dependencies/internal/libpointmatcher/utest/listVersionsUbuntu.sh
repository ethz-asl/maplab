#!/bin/bash 

echo -e "\n Copy-paste those information when reporting a bug in libpointmatcher:\n"

echo -e "Name \t\t| Version"
echo -e "----------------|-------------------------------"
echo -e "ubuntu: \t|" $(lsb_release -d)
echo -e "architecture: \t|" $(getconf LONG_BIT)"-bit"
echo -e "gcc: \t\t|" $(gcc --version | grep gcc)
echo -e "git: \t\t|" $(git --version)
echo -e "cmake: \t\t|" $(cmake --version)
echo -e "boost: \t\t|" $(dpkg -s libboost-dev | grep Version)
echo -e "eigen3: \t|" $(dpkg -s libeigen3-dev  | grep Version)
echo -e "doxygen: \t|" $(dpkg -s doxygen  | grep Version)
echo -e "\n\n"
