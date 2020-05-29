## Installing on Ubuntu 
14.04 (depricated), 16.04, 18.04, 20.04 (experimental)
### Automatic installation
For your convenience, we provide an install script to automate the whole installtion. Please make sure that you have an [SSH key activated on your GitHub account.](https://help.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
To install maplab, use the following commands
```bash
wget https://raw.githubusercontent.com/ethz-asl/maplab/docs/install-instructions/install.sh
bash install.sh # Use the argument "experimental" to also install our latest features if your are eligible. Do not use sudo.
```

### Manual installation
If you prefer manually installing all compoments, following this guideline.

```bash 
export UBUNTU_VERSION=$(lsb_release -cs) #(Ubuntu 14.04: trusty (depricated), Ubuntu 16.04: xenial, Ubuntu 18.04: bionic, Ubuntu 20.04: focal (experimental))
export ROS_VERSION=melodic #(Ubuntu 14.04: indigo (depricated), Ubuntu 16.04: kinetic, Ubuntu 18.04: melodic, Ubuntu 20.04: noetic (experimental))
export CATKIN_WS=~/maplab_ws
```

#### Install required system packages
```bash
# Install ROS
# NOTE: Follow the official ROS installation instructions for melodic.
sudo apt install software-properties-common
sudo add-apt-repository "deb http://packages.ros.org/ros/ubuntu $UBUNTU_VERSION main"
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt update
sudo apt install ros-$ROS_VERSION-desktop-full "ros-$ROS_VERSION-tf2-*" "ros-$ROS_VERSION-camera-info-manager*" --yes


# Install framework dependencies.
sudo apt install autotools-dev ccache doxygen dh-autoreconf git liblapack-dev libblas-dev libgtest-dev libreadline-dev libssh2-1-dev pylint clang-format-3.9 python-autopep8 python-catkin-tools python-pip python-git python-setuptools python-termcolor python-wstool libatlas3-base --yes

sudo pip install requests
```

#### Update ROS environment

```bash
sudo rosdep init
rosdep update
echo ". /opt/ros/$ROS_VERSION/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Install ccache for faster rebuilds.
(OPTIONAL but HIGHLY RECOMMENDED)
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
#### Create a catkin workspace

To create a workspace, run:
```bash
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# For more informative and readable output.
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-fdiagnostics-color
cd src
```

#### Cloning maplab repository
Now you can clone maplab and its dependencies via SSH, https clone is not supported for developer version.
SSH keys need to be installed and connected to your GitHub account, as explained [here.](https://help.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)

```bash
git clone git@github.com:ethz-asl/maplab.git --recursive -b develop
```

#### Setting up the linter
This setups a linter which checks if the code conforms to our style guide during commits.
These steps are only necessary if you plan on contributing to maplab.

```bash
cd $CATKIN_WS/src/maplab
./dependencies/internal/linter/init-git-hooks.py
```

#### Building maplab
```bash
cd $CATKIN_WS
catkin build maplab
```
**Note:** Currently some of our dependencies contain superfluous packages that will not have all the necessary dependencies. Therefore compilation will fail for these packages, if you try to build the complete workspace with: `catkin build`. Please have a look at the [FAQ](../overview_and_introduction/D_FAQ.md).

#### Troubleshooting

Please visit the [FAQ](../overview_and_introduction/D_FAQ.md) section.
