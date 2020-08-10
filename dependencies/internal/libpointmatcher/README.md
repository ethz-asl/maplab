![alt tag](doc/images/banner_light.jpeg)


---


Documentation and Tutorials
===========================
libpointmatcher is a modular library implementing the Iterative Closest Point (ICP) algorithm for aligning point clouds. It has applications in robotics and computer vision.

**Quick link for the tutorial pages: [Tutorials](doc/index.md) (also available on [readthedocs.org](http://libpointmatcher.readthedocs.org/)).**

Those tutorials are written using Markdown syntax and stored in the project's `/doc` folder.  Their scope ranges from introductory material on performing point cloud registration to instructions for the more experienced developer on how to extend the library's codebase. 

Libpointmatcher's source code is fully documented based on doxygen to provide an easy API to developers. An example of this API can be found [here](https://norlab.ulaval.ca/libpointmatcher-doc/), but it is suggested to use the one build for your version in `doc/html`. 

libpointmatcher is being developed by [François Pomerleau](mailto:f.pomerleau@gmail.com) and [Stéphane Magnenat](http://stephane.magnenat.net) as part of our work at [ASL-ETH](http://www.asl.ethz.ch).

You can read the latest changes in the [release notes](doc/ReleaseNotes.md).

Quick Start
==================

Although we suggest to use the [tutorials](doc/index.md), here is a quick version of it:

The library has a light dependency list:

 * [Eigen] version 3, a modern C++ matrix and linear-algebra library,
 * [boost] version 1.48 and up, portable C++ source libraries,
 * [libnabo] version 1.0.7, a fast K Nearest Neighbour library for low-dimensional spaces,
 
and was compiled on:
  * Ubuntu ([see how](/doc/Compilation.md))
  * Mac OS X ([see how](/doc/CompilationMac.md))
  * Windows ([see how](/doc/CompilationWindows.md) - partially supported)

### Compilation & Installation 

For beginner users who are not familiar with compiling and installing a library in Linux, go [here](doc/Compilation.md) for detailed instructions on how to compile libpointmatcher from the source code.  If you are comfortable with Linux and CMake and have already installed the prerequisites above, the following commands should install libpointmatcher on your system.

```
mkdir build && cd build
cmake ..
make
sudo make install
```

### Testing

Libpointmatcher ships with a version of the Google testing framework [GTest](https://code.google.com/p/googletest/).  Unit tests are located in utest/ and are compiled with libpointmatcher.  To run the tests and make sure that your compiled version is working correctly, run the test executable in your build directory:
```
cd build
utest/utest --path ../examples/data/
```

### Linking to external projects.
We mainly develop for __cmake projects__ and we provide example files under [`examples/demo_cmake/`](https://github.com/ethz-asl/libpointmatcher/tree/master/examples/demo_cmake) to help you in your own project. We also provide a __QT Creator__ example in [`examples/demo_QT/`](https://github.com/ethz-asl/libpointmatcher/tree/master/examples/demo_Qt), which manually list all the dependencies in the file [`demo.pro`](https://github.com/ethz-asl/libpointmatcher/blob/master/examples/demo_Qt/demo.pro). You would need to ajust those paths to point at the appropriate locations on your system.


### Bug reporting

Please use our [github's issue tracker](http://github.com/ethz-asl/libpointmatcher/issues) to report bugs. If you are running the library on Ubuntu, copy-paste the output of the script [listVersionsUbuntu.sh](https://github.com/ethz-asl/libpointmatcher/blob/master/utest/listVersionsUbuntu.sh) to simplify the search of an answer.

## File formats
The library support different file formats for importing or exporting data:
  * csv (Comma Separated Values)
  * vtk (Visualization Toolkit Files)
  * ply (Polygon File Format)
  * pcd (Point Cloud Library Format)

Those functionnalities are available without increasing the list of dependencies at the expense of a limited functionality support. For more details, see the tutorial [Importing and Exporting Point Clouds](doc/ImportExport.md). Example executables using those file formats from the command line can be found in `./example/` and are described [here](doc/ICPIntro.md) in more details 

Citing
======

If you use libpointmatcher in an academic context, please cite the following publication:

	@article{Pomerleau12comp,
		author = {Pomerleau, Fran{\c c}ois and Colas, Francis and Siegwart, Roland and Magnenat, St{\'e}phane},
		title = {{Comparing ICP Variants on Real-World Data Sets}},
		journal = {Autonomous Robots},
		year = {2013},
		volume = {34},
		number = {3},
		pages = {133--148},
		month = feb
	}

and/or

	@INPROCEEDINGS{pomerleau11tracking,
		author = {Fran{\c c}ois Pomerleau and St{\'e}phane Magnenat and Francis Colas and Ming Liu and Roland Siegwart},
		title = {Tracking a Depth Camera: Parameter Exploration for Fast ICP},
		booktitle = {Proc. of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
		publisher = {IEEE Press},
		pages = {3824--3829},
		year = {2011}
	}

Extra Reading
=============
If you are interested in learning more about different registration algorithms, we recently put together a literature review surveying multiple solutions. The review is organized in the same way as the library and many examples are provided based on real deployments.

F. Pomerleau, F. Colas and R. Siegwart (2015), "_A Review of Point Cloud Registration Algorithms for Mobile Robotics_", __Foundations and Trends® in Robotics__: Vol. 4: No. 1, pp 1-104.  https://doi.org/10.1561/2300000035 

If you don't have access to the journal, you can download it from [here](https://www.researchgate.net/publication/277558596_A_Review_of_Point_Cloud_Registration_Algorithms_for_Mobile_Robotics).

More Point Clouds
=================
We also produced those freely available data sets to test different registration solutions:

[_Challenging data sets for point cloud registration algorithms_](http://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:laserregistration)

![alt tag](http://projects.asl.ethz.ch/datasets/lib/exe/fetch.php?cache=&media=laserregistration:asldataset_weblarge.jpg)

You can download the files in CSV or VTK formats, which are directly supported by the library I/O module. 


Projects and Partners
=====================

If you are using libpointmatcher in your project and you would like to have it listed here, please contact [François Pomerleau](mailto:f.pomerleau@gmail.com).

 * European Project [NIFTi](http://www.nifti.eu/) (FP7 ICT-247870): Search and rescue project in dynamic environments. Results: [video of multi-floor reconstruction](http://www.youtube.com/watch?v=lP5Mj-TGaiw) and [video of railyard reconstruction](http://www.youtube.com/watch?v=ygIvzWVfPYk). All results with real-time computation.
 * NASA Ames [Stereo Pipeline](https://ti.arc.nasa.gov/tech/asr/groups/intelligent-robotics/ngt/stereo/): Planetary reconstruction from satellite observations. Results: used for Mars, Moon and Earth point clouds.
 * Armasuisse S+T UGV research program [ARTOR](http://www.artor.ethz.ch/): Development of techniques for reliable autonomous navigation of a wheeled robot in rough, outdoor terrain. Results: [video of urban and dynamic 3D reconstruction](http://www.youtube.com/watch?v=UCCAUf64tD0) and [video of open space 3D reconstruction](http://www.youtube.com/watch?v=M5Y99o7um88) with real-time computation.
 * Swiss National Science Foundation - [Limnobotics](http://www.limnobotics.ch/): Robotic solution for toxic algae monitoring in lacs. Result: [video of 3D shore reconstruction](http://www.youtube.com/watch?v=g8l-Xq4qYeE) with real-time computation.

For a larger list of work realized with libpointmatcher, please see the page [Applications And Publications](/doc/ApplicationsAndPub.md).


License
=======

libpointmatcher is released under a permissive BSD license. Enjoy!

[Ubuntu]: http://www.ubuntu.com
[CMake]: http://www.cmake.org
[CMake documentation]: http://www.cmake.org/cmake/help/cmake2.6docs.html
[git]: http://git-scm.com
[Eigen]: http://eigen.tuxfamily.org
[libnabo]: http://github.com/ethz-asl/libnabo
[ROS]: http://www.ros.org/
[Paraview]: http://www.paraview.org/
[yaml-cpp]: http://code.google.com/p/yaml-cpp/
[Doxygen]: http://www.stack.nl/~dimitri/doxygen/
[boost]: http://www.boost.org/


---


![alt tag](doc/images/banner_dark.jpeg)
