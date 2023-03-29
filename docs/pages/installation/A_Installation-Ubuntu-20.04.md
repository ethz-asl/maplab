## Installing on Ubuntu 20.04

#### Install required system packages
```bash
# Install ROS (follow the official ROS installation instructions).
sudo apt install software-properties-common
sudo add-apt-repository "deb http://packages.ros.org/ros/ubuntu focal main"
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full ros-noetic-tf2-* ros-noetic-camera-info-manager*


# Install framework dependencies.
sudo apt install -y autotools-dev ccache doxygen dh-autoreconf git \
                    liblapack-dev libblas-dev libgtest-dev libreadline-dev \
                    libssh2-1-dev pylint clang-format-12 python3-autopep8 \
                    python3 python3-catkin-tools python3-pip python-git-doc \
                    python3-setuptools python3-termcolor python3-wstool \
                    libatlas3-base libv4l-dev libjpeg-dev

pip3 install --user --upgrade pip
pip3 install --user requests opencv-python opencv-contrib-python
```

#### Update the ROS environment
Follow this if you freshly installed ROS, if you already had ROS installed you can skip this.

```bash
sudo rosdep init
rosdep update
echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Install ccache for faster rebuilds.
(OPTIONAL but HIGHLY RECOMMENDED)

ccache is a tool that caches intermediate build files to speed up rebuilds of the same code. On Ubuntu it can be set up with the following command. The max. cache size is set to 10GB and can be adapted in the lines below:

```bash
sudo apt install -y ccache
echo 'export PATH="/usr/lib/ccache:$PATH"' | tee -a ~/.bashrc
source ~/.bashrc
ccache --max-size=10G
```

Now g++/gcc should now point to:
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

#### Create a catkin workspace
To create a workspace, run:
```bash
mkdir -p maplab_ws/src
cd maplab_ws
catkin init
catkin config --merge-devel
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

For debugging and a more informative and readable output use:
```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-fdiagnostics-color
```

#### Cloning maplab repository
Now you can clone maplab and its dependencies.

```bash
cd src
git clone git@github.com:ethz-asl/maplab.git --recursive
```

#### Building maplab
```bash
cd ~/maplab_ws
catkin build maplab
```
**Note:** Currently some of our dependencies contain superfluous packages that will not have all the necessary dependencies. Therefore compilation will fail for these packages, if you try to build the complete workspace with: `catkin build`.

#### Troubleshooting

Please visit the [FAQ](../overview_and_introduction/D_FAQ.html#installation) and the additional instructions on debugging provided here [here](B_Compilation-and-Debugging.html).
