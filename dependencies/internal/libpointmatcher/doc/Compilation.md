| [Tutorials Home](index.md)    | | [Next](Datafilters.md) |
| ------------- |:-------------:| -----:|

# Compiling and Installing libpointmatcher on your Computer

## In short...
If you are used to development projects, here is what you need:

|Name           |Version  <br> (Tested Feb. 20, 2015) |Version  <br> (Tested Sept. 6, 2016) | Version  <br> (Tested Jan. 8, 2019) |
|---------------|-----------------------|-----------------------|-----------------------|
|Ubuntu         | 12.04.5 LTS (64 bit)  | 14.04.5 LTS (64 bit)  | 18.04.1 LTS (64 bit) |
|gcc            | 4.6.3                 | 4.8.4                 | 7.3.0 |
|git            | 1.7.9.5               | 1.9.1                 | 2.17.1 |
|cmake          | 2.8.11.2              | 2.8.12.2              | 3.10.2 |
|doxygen (opt.) | 1.7.6.1               | 1.8.6-2               | 1.8.13-10 |
|||||
| _Dependency:_|  |  |
|boost          | 1.48.0.2             | 1.54.0                 | 1.65.1 |
|eigen          | 3.0.5                | 3.2.0-8                | 3.3.4-4 |
|libnabo        | [from source](https://github.com/ethz-asl/libnabo) | [from source](https://github.com/ethz-asl/libnabo) | [from source](https://github.com/ethz-asl/libnabo) |

__Note:__ we only support 64-bit systems because of some issues with Eigen. Other versions will most probably work but you'll have to try yourself to know for sure.

The rest of this tutorial will guide you through the different requirements step by step.

## Detailed Installation Instructions 
### Some Basic Requirements 

#### a. Installing Boost
[Boost](www.boost.org) is a widely-used C++ library and is included in most Linux distributions.  You can quickly check if Boost is installed on your system by running

```
ldconfig -p | grep libboost
```

If you see a list of results then you can skip to the next section.  If not, you most likely have to install Boost.

Instructions for downloading and installing boost to Unix systems can be found [here](http://www.boost.org/doc/libs/1_55_0/more/getting_started/unix-variants.html).  Boost can also be installed as a package by running

```
sudo apt-get install libboost-all-dev
```

#### b. Installing Git
[Git](http://git-scm.com/) is a version control system similar to SVN designed for collaboration on large code projects.  Because libpointmatcher is hosted on Github, you should the git application to keep track of code revisions, and bug fixes pushed to the public repository.

Git should already be installed on your system but you can check if it is by running

```
git --version 
```

If Git is installed, you should see a message of the form

```
git version 2.17.1
```

If not refer to the Git homepage for installation instructions or install via the package manager by running

```
sudo apt-get install git-core
```

#### c. Installing CMake
[CMake](http://www.cmake.org/) is a cross-platform build system and is used for building the libpointmatcher library.  Refer to the homepage for installation instructions, or you can once again use the package manager

```
sudo apt-get install cmake cmake-gui
```

*NOTE:* CMake has a GUI called cmake-gui which can be useful for configuring builds.  We recommend you install this as well since it will be referred to in this tutorial.

*NOTE 2:* Some functionalities like find_package() (only useful if you intent to link with your own project) will only work if you have CMake 2.8.11 or over. On Ubuntu 12.04, only 2.8.7 is available using apt-get. If you intent to use such functionality, you will have to compile CMake from sources.

### 1. Installing Eigen
The Eigen linear algebra library is required before installing libpointmatcher and can be found [here](http://eigen.tuxfamily.org/).  Either download and compile Eigen following instructions from the package website or simply install package via apt by running:

```
sudo apt-get install libeigen3-dev
```

### 2. Installing libnabo
libnabo is a library for performing fast nearest-neighbor searches in low-dimensional spaces.  It can be found [here](https://github.com/ethz-asl/libnabo).  Clone the source repository into a local directory of your choice.
```
mkdir ~/Libraries/
cd ~/Libraries
git clone git://github.com/ethz-asl/libnabo.git
cd libnabo
``` 

Now you can compile libnabo by entering the following commands
```
SRC_DIR=`pwd`
BUILD_DIR=${SRC_DIR}/build
mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
make
sudo make install
```

This will compile libnabo in a `/build` directory and install it on your system.

*Note:* If Eigen or Boost are not in their regular system locations you will have to indicate their location by setting the corresponding CMake flags.
<!--
### 3. Installing yaml-cpp 0.3.0 (Optional)
Configuration files can be managed using YAML in libpointmatcher.  This allows users to edit configuration files in a readable format.  To support this, you need to install [yaml-cpp](http://code.google.com/p/yaml-cpp/).  **It is important that you install the older version (0.3.0) of lib-yaml or you will not be able to install Pointmatcher.**  If you are using versions of Ubuntu newer than 12.04, see the warning below. Either compile and install from the source or install package by running:

```
sudo apt-get install libyaml-cpp-dev
```

#### Warning for users of Ubuntu 14.04 (Trusty Tahr) or more recent versions
The yaml-cpp package for Trusty Tahr provides yaml-cpp0.5. Libpointmatcher is so far only compatible with yaml-cpp0.3 and thus an older version of yaml-cpp should be installed manually.
-->

### 4. Compiling the Documentation (optional)
Libpointmatcher is documented directly in the source-code using [Doxygen](http://www.stack.nl/~dimitri/doxygen/).  If Doxygen is installed on your system, an html version of the documentation will be compiled in `/usr/local/share/doc/libpointmatcher/`.  To install Doxygen in Ubuntu, run:

```
sudo apt-get install doxygen
```

Once you have compiled libpointmatcher in step 6, you can simply open `/usr/local/share/doc/libpointmatcher/api/html/index.html` in a browser to view the API documentation.

You will also need Latex for the rendering of equations:

```
sudo apt-get install texlive-full
```

### 5. Installing libpointmatcher
Clone the source repository into a local directory.  As an example we reuse the Libraries directory that was created to contain the libnabo sources.

```
cd ~/Libraries/
git clone git://github.com/ethz-asl/libpointmatcher.git
cd libpointmatcher
```
Now, libpointmatcher is compiled into a `/build` directory.
```
SRC_DIR=`pwd`
BUILD_DIR=${SRC_DIR}/build
mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
make
```

If you have doxygen and latex installed, you can enable the documentation using the CMake variable `GENERATE_API_DOC` to `true`. This can be achived through CMake GUI or by the command line:

```
cmake <otherflags> -D GENERATE_API_DOC=true ${SRC_DIR}
```

You can optionally verify that the version of libpointmatcher you have compiled is stable by running the unit tests.
```
utest/utest --path [libpointmatcher root]/examples/data/
```

Finally, to install libpointmatcher to your system run the following:
```
sudo make install
```

#### Possible Caveats
If Eigen, libnabo, yaml-cpp are not found during the installation, you will have to manually supply their installation locations by setting the CMake flags.  You can do so using the CMake GUI.

```
cd build
cmake-gui .
```
![alt text](images/cmake_screenshot.png "Screenshot of CMake GUI")

<!--If yaml-cpp was installed using apt-get as described above, it will not be found by the default CMake configuration.  You should set the `yaml-cpp_INCLUDE_DIRS` and `yaml-cpp_LIBRARIES` to `/usr/include/yaml-cpp` and `/usr/lib/x86_64-linux-gnu/` respectively.  These locations could be different on your machine.  You can find them by the files installed by the libyaml package:
```
dpkg -L libyaml-cpp-dev
```
-->
You can then set `EIGEN_INCLUDE_DIR`, `NABO_INCLUDE_DIR`, `NABO_LIBRARY`, `yaml-cpp_INCLUDE_DIRS`, `yaml-cpp_LIBRARIES` to point to your installation directories as shown in the screenshot above.  Then, generate the make files by clicking generate and rerun the following inside `/build`:
```
make
sudo make install
```

# Having problems?
Some dependencies changed and we don't keep track of all combinations possible. Before reporting a problem, make sure to include the versions you are using. You can run the bash script `./utest/listVersionsUbuntu.sh` and copy-paste its output when [reporting an issue on github](https://github.com/ethz-asl/libpointmatcher/issues). You may need to ensure that the file is executable:

```
chmod +x ./utest/listVersionsUbuntu.sh 
./utest/listVersionsUbuntu.sh 
```


Here are the list of useful commands used in the bash script:

Ubuntu version:

    lsb_release -r

32-bit or 64-bit architecture:

    getconf LONG_BIT

Compiler version:

    gcc --version

Git version:

    git --version
  
CMake:

    cmake --version

Boost version:

    dpkg -s libboost-dev | grep Version
    
Eigen3:

    dpkg -s libeigen3-dev | grep Version

Doxygen:

    dpkg -s doxygen | grep Version
