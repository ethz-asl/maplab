## Compilation and Debugging

### Use ccache
ccache is a fast compiler cache. It is a program that is sitting in front of gcc and monitors what is being compiled. If a file was compiled before in the exact same state, then ccache will serve a compilation request from cache and thus lead to "instant" compilation.

To install ccache:
```bash
sudo apt-get install ccache
sudo ln -s /usr/bin/ccache /usr/local/bin/gcc
sudo ln -s /usr/bin/ccache /usr/local/bin/g++
sudo ln -s /usr/bin/ccache /usr/local/bin/cc
```

### Compile the package you need
Avoid running ```catkin build``` without specifying a package since this builds the entire workspace which takes a long time.
You can specify individual packages simply by adding their names:
```
catkin build maplab
```

### Build single packages efficiently
If you only changed a single package you can also restrict the build process to this single package:
```
catkin build maplab --no-deps
```
Even more efficient is invoking ```make -j8``` in the build folder of the package
```
cd ~/catkin_ws/build/maplab && make -j8
```

### Compile in Release/Debug mode
Most packages are not built in Release mode. To build packages in Release mode:
```
catkin build maplab --cmake-args -DCMAKE_BUILD_TYPE=Release
```
To set this as the default behavior in your current catkin profile:
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```
If you are actively developing on maplab, debug symbols will sooner or later be helpful, therefore, we recommend to include them when building. This flag will enable all the optimizations of the release mode, but include debug symbols to give the debugger a change to figure out where in the code the program currently is:
```
-DCMAKE_BUILD_TYPE=RelWithDebInfo
```
In order to compile all packages in debug mode, i.e. no optimizations plus debug symbols, use:
```
-DCMAKE_BUILD_TYPE=Debug
```
**Important:** Switching these modes does not automatically rebuild packages that have been build before, therefore, clean the workspace first to be sure the new mode is adopted by all packages.
```
catkin clean --yes
```

### Change individual packages to Debug mode
If you want to efficiently debug a program it makes sense to switch only the package that the error is contained in to ```Debug``` mode.
```
cd ~/catkin_ws/build/maplab
ccmake .
```
You will now see a window similar to this:
```
 BUILD_SHARED_LIBS                ON
 CATKIN_DEVEL_PREFIX              /Users/sheep/catkin_ws/devel
 CATKIN_ENABLE_TESTING            ON
 CATKIN_PACKAGE_PREFIX
 CATKIN_SKIP_TESTING              OFF
 CMAKE_BUILD_TYPE                 Release
 CMAKE_INSTALL_PREFIX             /Users/sheep/catkin_ws/install
 ...
```
Navigate to the line which says ```CMAKE_BUILD_TYPE``` using the cursor keys and press ```Enter```. Now replace ```Debug``` with ```Release```. Press ```c``` to configure, followed by ```g``` to generate. (you might now press ```e``` to hide warnings.). Finally press ```q``` to quit.
Now make the package by invoking:
```
cd ~/catkin_ws/build/maplab && make -j8
```

**HINT:** Don't forget to switch back to ```Release``` once you are done with debugging.

### Run gdb with arguments
In order to run a process with arguments in gdb, the most efficient way is to use:
```
gdb --ex run --args ~/devel/lib/maplab/maplab --use_external_memory=false
```
Where ```--use_external_memory=false``` is just an example flag.

### Clang on Ubuntu 14.04
Clang 3.5 can be installed with
```
sudo apt-get install clang-3.5
```
and set as the default compiler with:
```
sudo update-alternatives --config c++
sudo update-alternatives --config cc
```

If you are using clang together with ccache on Ubuntu 14.04 you should consider updating the system's ccache version.

### Compile clang with openmp support on Ubuntu 14.04

Get the sources:

```
mkdir -p /tmp/llvm-clang-omp
cd /tmp/llvm-clang-omp
git clone https://github.com/clang-omp/llvm
git clone https://github.com/clang-omp/compiler-rt llvm/projects/compiler-rt
git clone -b clang-omp https://github.com/clang-omp/clang llvm/tools/clang
```

Build and time for several coffees:

```
mkdir -p /tmp/llvm-clang-omp/build
cd /tmp/llvm-clang-omp/build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release ../llvm
make -j8
```

Remove old packages and install the compiler and Intel runtime.

```
sudo apt-get install checkinstall libiomp-dev
sudo apt-get remove clang-* llvm-* libclang*

cd /tmp/llvm-clang-omp/build
checkinstall
```
Set a meaningful name for the package and let it conflict with the system's clang packages if you want to exclude the possibility of different versions conflicting.

Set as default compiler:

```
sudo rm /etc/alternatives/cc
sudo rm /etc/alternatives/c++

sudo ln -s /usr/local/bin/clang-3.5 /etc/alternatives/cc
sudo ln -s /usr/local/bin/clang++ /etc/alternatives/c++
```

### Upgrade ccache on Ubuntu 14.04
Ubuntu 14.04 delivers ccache 3.1.9 that has no official clang support. A manual update to a version of ccache >= 3.2.0 is recommended when using clang. The following steps can be used to perform this update:

```
sudo apt-get install devscripts
mkdir -p /tmp/ccache
```

Get the current distro source package:
```
cd /tmp/ccache
apt-get build-dep ccache
apt-get source ccache
```

Upgrade the distro source package with the recent upstream source:
```
cd /tmp/ccache
wget http://samba.org/ftp/ccache/ccache-3.2.1.tar.gz
cd ccache-3.1.9
uupdate ../ccache-3.2.1.tar.gz
```

Build binary package and install:
```
cd /tmp/ccache/ccache-3.2.1
dpkg-buildpackage -us -uc -nc
sudo dpkg -i ../ccache_3.2.1-0ubuntu1_amd64.deb
```
