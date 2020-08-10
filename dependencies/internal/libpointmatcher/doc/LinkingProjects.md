| [Tutorials Home](index.md)    | [Previous](Configuration.md) | [Next](UsingInRos.md) |
| ------------- |:-------------:| -----:|

# Linking Projects to libpointmatcher

Once you have followed the [compilation instructions](Compilation.md) and installed libpointmatcher to your system, you can use libpointmatcher in your project.

## Option 1: Using CMake (Recommended)
Because libpointmatcher was build using CMake, it can be conveniently included in other CMake projects.  You can simply use the `find_package` functionality of CMake to locate the installation directory of libpointmatcher.  Add `$POINTMATCHER_INCLUDE_DIRS` to the list of include directories in your project and link the appropriate executables to `$POINTMATCHER_LIBRARIES`.

In this following example, we build a very simple CMake project containing one executable in `myProgram.cpp` which depends on libpointmatcher.

```cmake
cmake_minimum_required (VERSION 2.6)
project (myProject)

find_package(libpointmatcher 1.1.0 REQUIRED)
include_directories("${libpointmatcher_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")

add_executable(myProgram myProgram.cpp)
target_link_libraries(myProgram ${libpointmatcher_LIBRARIES})
```
A working example of how to link to an external project can be found in [./examples/demo_cmake](../examples/demo_cmake).

## Option 2: Using Eclipse
### Using the Native Eclipse Builder
We will demonstrate how to create an Eclipse project containing a simple executable which depends on libpointmatcher.  You must have [Eclipse CDT](http://www.eclipse.org/cdt/) installed to develop with libpointmatcher in Eclipse.  

Create a new C++ project by clicking `File > New > C++ Project`.  You can name your project "PointmatcherEclipseDemo" and in toolchains select the default toolchain for your system (most likely Linux GCC).  Click `Finish` to add your project to your Eclipse workspace.  

You must then configure the project by going to `Project > Properties > C/C++Build > Settings`.  Navigate to `C++ Compiler > Includes` and add the libpointmatcher include path to the `Include paths (-I)` list.  Next, go to `C++ Linker/Libraries` and add the the following three dependencies to the "Libraries (-l)" list: 

* pointmatcher
* boost_system
* nabo

Click `Ok` to save the configuration.  Create a new source file by clicking `File > New > Source File` and name it "Demo.cpp".  In this file you can type the following:
 
```cpp
#include <pointmatcher/PointMatcher.h>
#include <iostream>

typedef PointMatcher<float> PM;


int main(int argc, char *argv[]) {
	PM::ICP icp;
	icp.setDefault();

	std::cout << "ICP configured to default." << std::endl;
	return 0;
}
```
The program will create an ICP chain, configure it to the default settings and exit subsequently.  Click on `Project > Build Project` and check that the project compiles successfully.  Finally run the program by clicking `Run > Run`. The message "ICP configured to default." should be displayed in the console.       

## Option 3: Using Eclipse
You will need to generate a `.pro` file containing your project information. This file would look like this:

```
QT       	+= core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
TARGET    	 = LAUPointMatcher
TEMPLATE  	 = app
SOURCES  	+= main.cpp

INCLUDEPATH += 	/Users/francoispomerleau/Research/Code/libpointmatcher/pointmatcher \
                /Users/francoispomerleau/Research/Code/libnabo/ \
                /usr/local/Cellar/eigen/3.2.4/include/eigen3 \
		/usr/local/include/

CONFIG          += c++11
#QMAKE_CXXFLAGS += -mmacosx-version-min=10.7
#QMAKE_LFLAGS   += -mmacosx-version-min=10.7

LIBS     	+= 	/usr/local/lib/libboost_thread-mt.dylib \
                	/usr/local/lib/libboost_filesystem-mt.dylib \
                        /usr/local/lib/libboost_system-mt.dylib \
                        /usr/local/lib/libboost_program_options-mt.dylib \
                        /usr/local/lib/libboost_date_time-mt.dylib \
                        /usr/local/lib/libboost_chrono-mt.dylib \
                        /Users/francoispomerleau/Research/Code/libpointmatcher/build/libpointmatcher.a \
                        /Users/francoispomerleau/Research/Code/libnabo/build/libnabo.a \
                        /Users/francoispomerleau/Research/Code/libpointmatcher/build/contrib/yaml-cpp-pm/libyaml-cpp-pm.a
```

A working example of how to link to an external project can be found in [./examples/demo_Qt](../examples/demo_Qt).

## Option 4: Using Compiler Flags
If you are compiling a very simple program without the use of a builder, simply include the libpointmatcher header files by setting the include flag in your compiler.  Example:
```
g++ -I/usr/local/include/pointmatcher -o myProgram.o -c myProgram.cpp
```
You can then link to the pointmatcher library using:
```
g++ myProgram.o   -o myProgram -lpointmatcher -lnabo -lboost_system -lyaml-cpp -lboost_filesystem -lrt
```
Nevertheless, it is always more convenient to use a builder such as CMake.
