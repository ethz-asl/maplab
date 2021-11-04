#!/bin/bash
CATKIN_WS=$HOME/maplab_ws # Change this if you want.
SECONDS=0
echo "
WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWx'...'ckXWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM0:.....lXMMMMMMMMMWXXWMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMKc...cKMNKKXWMMMMN0KWMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMNd'..cKMWKkxONMMXxdOXWMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMNX0Okxdddddc'..:KMMMWXXNWMXl...,ckNMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMWXOdl:,'...........:KMMMMWNXWMXl......,0MMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMWXkl;.....';:clloollcoKMMMMMWK0XXl...cxddOWMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMW0o;....;ldOKXWWMMMMMMWWMMWXXXNWWXl...lXMMMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMNOdc'...,oOXWMMMMMMMMMMMMMMMN0kkkONNo...'l0NMMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMM0c.....;xXWMMMMMMMMMMMMMMMWWMNOkkkOXNk:....'lKMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMXc....'dXMMMMMMWXKKXXKK0kdlccokKXKXNWMMNx,....lNMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMNkl'...;OWMMMMMNkc,'',,,'........;oONMMMMMW0:...'lONMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMWO:.....;OMMMMMWKl...................':d0NMMMMK:.....c0WMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMK:.....'kWMMMMNx,.............,:c:'.....':d0WMM0;.....:KMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMM0;.....lNMMMWKl....':oddl;..;kKOOX0l'......'cONWd.....:KMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMWx'...'xMMMNx,....'xNN0KNXd:kMXl.0WXc.........oXO,...,kWMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMNx,...,OMMKc......:XMKl.0M0okNMNKNWXkl:'......,O0;...,kWMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMk'....'kMK:.......;ONNKXNKc.':xOOko:kWNKkdllllkN0;....,OMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMWd......dNo......'lO0ocooc,..........oNMMMMMMMMMMk'....'xMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMKc.....:0k,..':d0WMNo...............:KMMMMMMMMMXc.....cKMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMXx;....oXXO0KNMMMMMk'..............,OMMMMMMMMWd'...;kNMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMXc....'oNMMMMMMMMMK:...:c:;',ld:..:KMMMMMMMWx'....oNMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMNl......lKWMMMMMMMWO;..:odoxOd;..;OWMMMMMMXo'.....oWMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMW0c......;xXMMMMMMMMKd:'...l0l,cxXWMMMMMNk:......cKWWMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMWNNNXXKKK00Okoc:;'...;dKWMMMMMMMNKOOk0WNXWMMMMMWXx:....,;:oxkOO000KKKXWMMMMMMMMMMMMMM
MMMMMMMMMMMMMMN0Okkkkkkkkkkkkkkko;....'cx0XWMMMMMMMMMMMMMMWN0xl,...';ldxxxxxxxkkkkO0XNXXNMMMMMMMMMMM
MMMMMMMMMMMMMN0kkkkkkkkkkkkkkkkkxl'......';coxkO0KKKK00kxol;'......cxkxxxxxxkKNWNNNK0OkxONMMMMMMMMMM
MMMMMMMMMMMMN0kkkkkkkkkkkkkkkkkxxl'.............''''''............'lkkxxxxxxONMMMNOxxxxxx0WMMMMMMMMM
MMMMMMMMMMMN0kkkkkkkkkkkkkkkkkkxxl'......,::;,,'''''''',;;:,......'lkkxxxxxxxO0XWNOxxxxxxkXMMMMMMMMM
MMMMMMMMMMN0kkkkkkkkkkkkkkkkkkxxxl'......;dxxxddooddddxxxkx:......'lkkxxxxxxxxxx0NNOxxxxxxONMMMMMMMM
MMMMMMMMMN0kkkkkkkkkkkkkkkkkkkdl:,.......;dxxxxxxxxxkkkkkkx:.......,:ldxxxxxxxxxxONN0xxxxxx0WMMMMMMM
MMMMMMMMW0kkkkkkkkkkkkkkkkkkkd:..........:kOOO000000000000Oc..........;oxxxxxxkO0KNWXkxxxxxxKWMMMMMM
MMMMMMMW0kkkkkkkkkkkkkkkO00KKO;..........lNMMMMMMMMMMMMMMMWd..........,x00O0KXXXXK0OkxxxxxxxkXMMMMMM
MMMMMMW0kkkkkkkkkkkkk0XNWWMMMWOc'......,lKMMMMMMMMMMMMMMMMMXo,......':kNMMMMMWXOxxxxxxxxxxxxxONMMMMM
MMMMMW0kkkkkkkkkkkkkkKNWMMMMMMMNK0O0OO0XWMMMMMMMMMMMMMMMMMMMWX0OO0O0KNMMMMMMWNXOxxxxxxxxxxxxxx0WMMMM
MMMMWKkkkkkkkkkkkkkkkkO00KNWWNNWWWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMWWWNNXKK0OkxxxxxxxxxxxxxxxxkXWMMM
MMMWKkkkkkk00K00000KKKKK0KNN0kkkOOO000000KKKKKKKKKKXXXXXKKKKKKKK0000OOOkkxxxxxxxxxxxxxxxxxxxxxxONMMM
MMWKkkkkOKXNXXXXXXXXXXXXXXKOxxxxxxxxxxxxxxxxxxxxxxxkkkkkkkkkkkkkkkkkkkkkkxxxxxxxxxxxxxxxxxxxxxxx0WMM
MWKkkO0XNNKOkkkkkkkkkkkkkxxxxxxxxxxxxxxxxxxxxxxxxxxkkkkkkkkkkkkkkkkkkkkkkxxxxxxxxxxxxxxxxxxxxxxxkKWM
WKO0KNNX0OkkkkkkkkkkkkOO0OOkxxxxxxxxxxxxxxxxxxxxxxxkkkkkkkkkkkkkkkkkkkOO0OOkkxxxxxxxxxxxxxxxxxxxxkXM
XKXNXKOkkkkkkOO00KKXXNNWWWWWNXXKK00OOkkxxxxxxxxxxxxkkkkkkkkkkkO00KKXXNNWWWWNNXXK00OOkxxxxxxxxxxxxxON
WNK0OO00KKXXNNWWMMMMMMMMMMMMMMMMMMMWWNNXXKK00OOkkxxkOO000KKXNNWWMMMMMMMMMMMMMMMMMMWWNNXKK00OkkxxxxxK
XKKXXNWWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMWWWNXKKXXNWWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMWWNXXK0OkO"

echo "
====================================================================================================
||                                                                                                ||
||  Welcome to the maplab installation script!                                                    ||
||                                                                                                ||
===================================================================================================="

if [ -z "$1" ]
then
  echo -e "\e[92m\e[1mInstalling maplab develop.\e[39m\e[0m"
  EXPERIMENTAL=false
elif [ "$1" == "experimental" ]
then
  echo -e "\e[92m\e[1mInstalling maplab experimental.\e[39m\e[0m"
  EXPERIMENTAL=true
else
  echo -e "\e[41m\e1mERROR: Argument $1 not defined.\e[39m\e[0m"
  exit 1
fi

UBUNTU_VERSION=$(lsb_release -cs)
if [ "$UBUNTU_VERSION" == "trusty" ]
then
  ROS_VERSION="indigo"
  echo -e "\e[93m\e[1mWARNING: 14.04 support is depricated. Try at your own risk.\e[39m\e[0m"
elif [ "$UBUNTU_VERSION" == "xenial" ]
then
  ROS_VERSION="kinetic"
elif [ $UBUNTU_VERSION == "bionic" ]
then
  ROS_VERSION="melodic"
elif [ $UBUNTU_VERSION == "focal" ]
then
  ROS_VERSION="noetic"
  echo -e "\e[93m\e[1mWARNING: 20.04 support is experimental. Try at your own risk.\e[39m\e[0m"
else
  echo -e "\e[41m\e[1mERROR: Operating system is not supported.\e[39m\e[0m"
  exit 1
fi
echo -e "\e[92m\e[1mInstalling ros-$ROS_VERSION for Ubuntu $UBUNTU_VERSION.\e[39m\e[0m"
echo -e "\e[92m\e[1mInstalling system dependencies...\e[39m\e[0m"

sudo apt install software-properties-common
sudo add-apt-repository "deb http://packages.ros.org/ros/ubuntu $UBUNTU_VERSION main"
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt update
sudo apt install ros-$ROS_VERSION-desktop-full "ros-$ROS_VERSION-tf2-*" "ros-$ROS_VERSION-camera-info-manager*" --yes
sudo apt install autotools-dev ccache doxygen dh-autoreconf git liblapack-dev libblas-dev libgtest-dev libreadline-dev libssh2-1-dev pylint clang-format-3.9 python-autopep8 python-catkin-tools python-pip python-git python-setuptools python-termcolor python-wstool libatlas3-base libv4l-dev --yes

sudo pip install requests

echo -e "\e[92m\e[1mUpdate ROS environment...\e[39m\e[0m"
sudo rosdep init
rosdep update
echo ". /opt/ros/$ROS_VERSION/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo -e "\e[92m\e[1mInstall ccache...\e[39m\e[0m"
sudo apt install -y ccache &&\
echo 'export PATH="/usr/lib/ccache:$PATH"' | tee -a ~/.bashrc &&\
source ~/.bashrc && echo $PATH
ccache --max-size=10G

echo -e "\e[92m\e[1mCreate a catkin workspace...\e[39m\e[0m"
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-fdiagnostics-color
cd src

echo -e "\e[92m\e[1mCloning maplab repository and dependencies...\e[39m\e[0m"
if [ $EXPERIMENTAL ] 
then
  git clone git@github.com:ethz-asl/maplab_experimental.git --recursive
else
  git clone git@github.com:ethz-asl/maplab.git --recursive -b develop
fi

echo -e "\e[92m\e[1mInstalling the linter...\e[39m\e\0m"

if [ $EXPERIMENTAL ]
then
  cd $CATKIN_WS/src/maplab
else
  cd $CATKIN_WS/src/maplab_experimental/maplab
fi
./dependencies/internal/linter/init-git-hooks.py

echo -e "\e[92m\e[1mBuilding maplab...\e[39m\e[0m"
catkin build maplab

echo -e "\e[92m\e[1mDone in $SECONDS seconds. Enjoy maplab 🐑\e[39m\e[0m"
