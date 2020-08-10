| [Tutorials Home](index.md)    | | [Next](Datafilters.md) |
| ------------- |:-------------:| -----:|

# Compiling and Installing libpointmatcher on your Computer (Mac OS X Instructions)


## In short...
If you are used to development project, here is what you need:

|Name           |Version  <br> (Tested Feb. 16, 2015)          |
|---------------|-----------------------|
|MacOS          | 10.10.2               |
|Xcode          | todo                  |      
|gcc            | 4.2.1                 |
|brew           | 0.9.5                 |
|git            | 1.9.3                 |
|cmake          | 3.0.2                 |
|doxygen (opt.) | 1.8.9.1               |
| | |
|_Dependency:_ ||
|boost          | 1.57.0                |
|eigen          | 3.2.4                 |
|libnabo        | [from source](https://github.com/ethz-asl/libnabo)       |


__Note:__ Other versions will most probably work but you'll have to try yourself to know for sure.

The rest of this tutorial will guide you through the different requirements step by step.

### Some Basic Requirements 
#### a. Installing Xcode via the App Store (OS X 10.10.2  and later)
Mac OS X does not come with a built-in C++ command-line compiler.  You must therefore install XCode by visiting the App Store.

Once Xcode is installed on your machine, launch it.  Navigate to preferences, to the downloads tab.  In the components list, install the Command Line Tools component.

You should now have a working version of gcc.  You can check by running the following command in your terminal:

	gcc --version

A message similar to the following should appear

	Configured with: --prefix=/Applications/Xcode.app/Contents/Developer/usr --with-gxx-include-dir=/usr/include/c++/4.2.1
	Apple LLVM version 6.0 (clang-600.0.56) (based on LLVM 3.5svn)
	Target: x86_64-apple-darwin14.1.0
	Thread model: posix

#### b. Installing Homebrew
Because Mac OS X does not come with a built-in package manager like in Ubuntu, you need to install one on your own.  A package manager is handy because it allows you to install, uninstall, update and maintain software packages with ease.  There are several possibilities including [Macports](http://www.macports.org/) and [Homebrew](http://brew.sh/).  While both are good options, we have a slight preference for homebrew which is easier to use.

You do not need a package manager to install libpointmatcher, but it simplifies things.  The following instructions will make use of homebrew and will thus assume that it is installed on your system.

Installing Homebrew is extremely easy and can be done by entering the following single command in your terminal

```
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```
Once the scripts finishes installing, you are good to go!

#### c. Installing Boost
[Boost](www.boost.org) is a widely-used C++ library and is included in most Linux distributions.  Mac OS X does not ship with Boost so if you have never used it before, you probably need to install it.  You can install the latest version of boost with the following homebrew command:

	brew install boost

#### d. Installing Git
[Git](http://git-scm.com/) is a version control system similar to SVN designed for collaboration on large code projects.  Because libpointmatcher is hosted on Github, you should the git application to keep track of code revisions, and bug fixes pushed to the public repository.

After installing the Xcode Command Line Tools, Git should already be installed on your system but you can check that it is there by running

```
git --version 
```
If Git is installed, you should see a message of the form
```
git version 1.8.3.2
```
If not refer to the Git homepage for installation instructions or install via homebrew by running
```
brew install git
```
#### e. Installing CMake
[CMake](http://www.cmake.org/) is a cross-platform build system and is used for building the libpointmatcher library.  Refer to the homepage for installation instructions, or you can once again use homebrew

	brew install cmake

### 1. Installing Eigen
The Eigen linear algebra is required before installing libpointmatcher and can be found [here](http://eigen.tuxfamily.org/).  Either download and compile Eigen following instructions from the package website or simply install the package via homebrew by running:

	brew install eigen

### 2. Compiling the Documentation (optional)
Libpointmatcher is documented directly in the source-code using [Doxygen](http://www.stack.nl/~dimitri/doxygen/).  If Doxygen is installed on your system, an html version of the documentation will be compiled in `/usr/local/share/doc/libpointmatcher/`.  To install Doxygen in Ubuntu, run:

	brew install doxygen

Once you have compiled libpointmatcher in step 6, you can simply open `/usr/local/share/doc/libpointmatcher/api/html/index.html` in a browser to view the API documentation.

### 3. Installing libnabo
libnabo is a library for performing fast nearest-neighbor searches in low-dimensional spaces.  It can be found [here](https://github.com/ethz-asl/libnabo).  Clone the source repository into a local directory of your choice.

	mkdir ~/Libraries/
	cd ~/Libraries
	git clone git://github.com/ethz-asl/libnabo.git
	cd libnabo


Now you can compile libnabo by entering the following commands

	SRC_DIR=`pwd`
	BUILD_DIR=${SRC_DIR}/build
	mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
	make


To make sure that everything is working properly, run the unit tests:

	make test

This will run multiple nearest-neighbor searches performances and may take some minutes. 

	sudo make install

*Note:* If Eigen or Boost are not in their regular system locations you will have to indicate their location by setting the corresponding CMake flags. You can use the following command to default flags:

	ccmake .




### 4. Installing libpointmatcher
Clone the source repository into a local directory.  As an example we reuse the Libraries directory that was created to contain the libnabo sources.

	cd ~/Libraries/
	git clone git://github.com/ethz-asl/libpointmatcher.git
	cd libpointmatcher

Now, libpointmatcher is compiled into a `/build` directory.

	SRC_DIR=`pwd`
	BUILD_DIR=${SRC_DIR}/build
	mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}
	make


You can optionally verify that the version of libpointmatcher you have compiled is stable by running the unit tests.

	utest/utest --path ../examples/data/


Finally, to install libpointmatcher to your system run the following:

	sudo make install


#### Possible Caveats
If Eigen, libnabo, yaml-cpp, or GTest are not found during the installation, you will have to manually supply their installation locations by setting the CMake flags.  You can do so using the ccmake tool.
```
cd build
ccmake ..
```

You can then set `EIGEN_INCLUDE_DIR`, `NABO_INCLUDE_DIR`, `NABO_LIBRARY`, `yaml-cpp_INCLUDE_DIRS`, `yaml-cpp_LIBRARIES` to point to your installation directories as shown in the screenshot above.  Then, generate the make files by clicking generate and rerun the following inside `/build`:
```
make
sudo make install
```

# Having problems?

Some dependencies changed and we don't keep track of all combinations possible. Before reporting a problem, make sure to include the versions you are using. 

Here are useful commands for the different version:

MacOS version:

	sw_vers -productVersion 

Compiler version:

	gcc --version

Homebrew:

	brew --version

Boost:

	brew info boost

Git:

	git --version

CMake:

	cmake --version

Eigen:

	brew info eigen

Doxygen:

	brew info doxygen

	


