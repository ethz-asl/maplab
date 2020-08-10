| [Tutorials Home](index.md)    | | [Next](Datafilters.md) |
| ------------- |:-------------:| -----:|

# Compiling and Installing libpointmatcher on Windows

## Compiling using MSVC (Microsoft Visual Studio)

### In Short...

If you are used to development project, here is what you need:


| Name   | Link | Version <br> (Tested December 29, 2018)|
| ------ | ---- | ------------- |
| Windows |  | 10 |
|  git | <https://git-scm.com/> | v2.20.1 |
|  libpointmatcher sources | <https://github.com/ethz-asl/libpointmatcher> |  |
| libnabo sources | <https://github.com/ethz-asl/libnabo> |  |
| Visual Studio |  <https://visualstudio.microsoft.com/>  | Visual Studio 2017 15.9.4 |
| CMake | <https://cmake.org/> | v3.13.2 |
| Eigen 3 | <http://eigen.tuxfamily.org/index.php> | v3.3.7 |
| Boost | <https://www.boost.org/> | v1.69.0 |

The rest of this tutorial will guide you through the different requirements step by step.

### Building Boost
1. Open a console that knows the path to the MSVC compiler command (cl). We suggest using **Windows PowerShell**. An alternative is **Developer Command Prompt**, which can be located in the Start menu in the Visual Studio section.
1. Go to your Boost source directory, and do:

    ```
    $ .\bootstrap.bat
    $ .\b2.exe install --prefix=build address-model=64
    ```

1. It may take a while to finish.


### Build libnabo
You may need to install grep to build libnabo. You can get the Windows version [here](http://gnuwin32.sourceforge.net/packages/grep.htm).

1. Start **CMake GUI**

1. Add the path of your libnabo source folder in the field _Where is the source code_.
1. Add a folder named build in the field _Where to build the binary_. This will allow you to do out-of-source compilation.
1. Click on the button Configure
    1. Select the generator for the project (Visual Studio 15 2017 Win64)
    1. Typically you will select "Use default native compilers" for the generator
    1. Click "Finish"
    1. An error will be reported, because CMake does not know yet where to find the libraries. The next steps will tell it where to find them.

1. Locate _your eigen folder_ in the field **EIGEN_INCLUDE_DIR**

1. Add the following boolean variable and set it to `true`: **Boost_USE_STATIC_LIBS**

1. Add the following PATH variable and set it to _(your boost folder)_/build: **BOOST_ROOT**

1. Change the variable **CMAKE_CONFIGURATION_TYPES** to `RelWithDebInfo`

1. Click on the button Configure, then on Generate. Here is an example of what your CMake should look like:

	![alt text](images/win_cmake_libnabo.png "CMake libnabo")


1. Locate the Microsoft Visual Studio Solution file (libnabo.sln) in your build folder and open it. Visual Studio should open.

1. Build the solution: BUILD -> Build Solution

    Alternatively, you can build the solution from the command line. In _(your libnabo folder)_/build:

    ```
    $ msbuild /m:2 libnabo.sln
    ```

    (Note that the flag /m:X defines the number of cores msbuild will use while building the solution.)


### Build libpointmatcher
1. Start **CMake GUI**, follow the same steps to configure the source and build folders for _libpointmatcher_, then click the Configure button.

1. Locate _your eigen folder_ in the field **EIGEN_INCLUDE_DIR**

1. Add the following boolean variable and set it to `true`: **Boost_USE_STATIC_LIBS**

1. Add the following PATH variable and set it to _(your Boost folder)_/build: **BOOST_ROOT**

1. Add the following PATH variable and set it to _(your libnabo source folder)_: **NABO_INCLUDE_DIR**

1. Add the following FILEPATH variable and set it to _(your libnabo source folder)_/build/RelWithDebInfo/nabo.lib: **NABO_LIBRARY**

1. Change the variable **CMAKE_CONFIGURATION_TYPES** to `RelWithDebInfo`

1. Click on the button Configure, then on Generate. Here is an example of what your CMake should look like:

	![alt text](images/win_cmake_libpointmatcher.png "CMake libpointmatcher")

1. In Visual Studio, build the solution: BUILD -> Build Solution

    Alternatively, you can build the solution from the command line. In _(your libpointmatcher folder)_/build:

    ```
    $ msbuild /m:2 libpointmatcher.sln
    ```

    (Note that the flag /m:X defines the number of cores msbuild will use while building the solution.)


## Reporting Issues

Currently, we don't have a developer fully supporting compilation on Windows. If you can help refreshing this documentation, your help is more than welcome.

Before reporting new building issues, have a look in the current/past list of issues. Add as much details as you can since you will most probably receive answers from developers that cannot reproduce the problem on their side. Here are some of them:

- Your directory structure need to be well organized as mention in [Issue #136](https://github.com/ethz-asl/libpointmatcher/issues/136).
- There might be some problems related to libnabo as mention in [Issue #128](https://github.com/ethz-asl/libpointmatcher/issues/118).

## Special Thanks

Special thanks to the following users in helping us with the Windows support:

- [kwill](https://github.com/kwill) for keeping the documentation up-to-date and investing time to make libpointmatcher compiling on Windows.
- [braddodson](https://github.com/braddodson) for porting a version of libpointmacher in `C#` with a limited set of features. The code can be found here: https://github.com/braddodson/pointmatcher.net


