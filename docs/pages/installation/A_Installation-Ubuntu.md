## Installing on Ubuntu 14.04, 16.04, (experimental: 18.04)

### Install required system packages

```bash
# Install ROS 
export UBUNTU_VERSION=xenial #(Ubuntu 16.04: xenial, Ubuntu 14.04: trusty, Ubuntu 18.04: bionic)
export ROS_VERSION=kinetic #(Ubuntu 16.04: kinetic, Ubuntu 14.04: indigo, Ubuntu 18.04: melodic)

# NOTE: Follow the official ROS installation instructions for melodic.
sudo apt install software-properties-common
sudo add-apt-repository "deb http://packages.ros.org/ros/ubuntu $UBUNTU_VERSION main"
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt update
sudo apt install ros-$ROS_VERSION-desktop-full "ros-$ROS_VERSION-tf2-*" "ros-$ROS_VERSION-camera-info-manager*" --yes


# Install framework dependencies.
# NOTE: clang-format-3.8 is not available anymore on bionic, install a newer version.
sudo apt install autotools-dev ccache doxygen dh-autoreconf git liblapack-dev libblas-dev libgtest-dev libreadline-dev libssh2-1-dev pylint clang-format-3.8 python-autopep8 python-catkin-tools python-pip python-git python-setuptools python-termcolor python-wstool libatlas3-base --yes

sudo pip install requests
```

### Update ROS environment

```bash
sudo rosdep init
rosdep update
echo ". /opt/ros/$ROS_VERSION/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### (OPTIONAL) Install ccache for faster rebuilds.
ccache is a tool that caches intermediate build files to speed up rebuilds of the same code. On Ubuntu it can be set up with the following command. The max. cache size is set to 10GB and can be adapted in the lines below:

```bash
sudo apt install -y ccache &&\
echo 'export PATH="/usr/lib/ccache:$PATH"' | tee -a ~/.bashrc &&\
source ~/.bashrc && echo $PATH
ccache --max-size=10G
```
Your path (at least the beginning) should look like:
```
/usr/lib/ccache:/usr/local/cuda-5.5/bin/:/usr/lib/lightdm/lightdm:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games
```
And g++/gcc should now point to:
```
which g++ gcc
/usr/lib/ccache/g++
/usr/lib/ccache/gcc
```
Show cache statistics:
```
ccache -s
```
Empty the cache and reset the stats:
```
ccache -C -z
```
ccache only works for a clean workspace. You will need a `make clean` otherwise.
## Create a catkin workspace

To create a workspace, run:
```bash
export ROS_VERSION=kinetic #(Ubuntu 16.04: kinetic, Ubuntu 14.04: indigo)
export CATKIN_WS=~/maplab_ws
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src
```

Now you can clone maplab and its dependencies, either via HTTPS or SSH.

### Cloning over HTTPS (no github account needed)
```bash
git clone https://github.com/ethz-asl/maplab.git --recursive
git clone https://github.com/ethz-asl/maplab_dependencies --recursive
```

### Cloning over SSH (github account needed)
```bash
git clone git@github.com:ethz-asl/maplab.git --recursive
git clone git@github.com:ethz-asl/maplab_dependencies.git --recursive
```

### Setting up the linter
This setups a linter which checks if the code conforms to our style guide during commits.
These steps are only necessary if you plan on contributing to maplab.

```bash
cd $CATKIN_WS/src/maplab
./tools/linter/init-git-hooks.py
```

### Building maplab
```bash
cd $CATKIN_WS
catkin build maplab
```
**Note:** Currently some of our dependencies contain superfluous packages that will not have all the necessary dependencies. Therefore compilation will fail for these packages, if you try to build the complete workspace with: `catkin build`. Please have a look at the [FAQ](https://github.com/ethz-asl/maplab/wiki/FAQ#q-why-do-i-get-missing-dependencies-when-building-the-maplab-workspace)

### Troubleshooting

Please visit the [[FAQ]] section.
