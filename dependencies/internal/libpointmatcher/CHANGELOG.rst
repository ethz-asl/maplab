^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libpointmatcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2019-03-04)
------------------
* Added documentation for people using ROS.
* Increased libnabo minimal version to 1.0.7.
* Added interface to inform if maximum number of iterations was reached.
* Fixed portability issue of the FileLogger.
* Fixed unit tests on Windows.
* Fixed parameter-less modules having 'unknown' as class name.
* Updated Windows compilation tutorial.
* Fixed compilation problems on Windows.
* Fixed PointToPlan error residual.
* Changed DOI resolver link in documentation.
* Added validation for the input transformation matrix in ICP.cpp.
* Removed duplication of PointToPoint compute in PointToPointWithCov.
* Added the RemoveSensorBias filter.
* Splitted ErrorMinimizersImpl.cpp into multiple files.

1.3.0 (2018-10-26)
------------------
* Removed some boost utilities supported by c++11
* Replaced raw pointers by std smart pointers

1.2.4 (2018-10-15)
------------------
* Support of Eigen 3.3
* Introduced SurfaceNormalDataPointsFilter, OctreeGridDataPointsFilter and NormalSpaceDataPointsFilter
* A lot of bugs were fixed

1.2.3 (2015-05-15)
------------------
* Support including other versions of YAML in compilation units that also include the YAML version packed with libpointmatcher (PR #80)
* Changed immutability concept for SupportLabel to support MSVC 2012 (#78)
* Fixed build system related bugs (#79, #70, ..).
* updated build_map example, added better error message, added better information prints
* cleaned CMakeList and added missing dependencies for external projetcs
* avoid possibility of building dynamic library on MacOS
* updated Mac build instructions
* Tim3xx laser support on Simple Noise filter (#64)
* Modified default covariance return in PointToPlaneWithCovErrorMinimizer (#59)
* update usage text and retab
* Removed compilation warnings
* add unit test for ICPSequence
* added application of reference data points filters for ICPSequence objects (#56)
* Merge branch 'master' of github.com:ethz-asl/libpointmatcher
* fix problem with libnabo linking (#54)
* Adapted the code to handle 2D point clouds and decided to split the initial/icp/complete transformation matrices in 3 different files. It should be easier to post process the transformations.
* Changed matrix for matrices as output suffix
* Changed the ICP example (pmicp) to accept initial translation/rotation input and allow to output the transformation matrices
* CutBelowLevelDataPointsFilter (PR #48)
* split unit tests (PR #47)
* Delete roadmap.txt
* change year to 2014
* correct bug in DataPoints operator==
* add a method to remove features or descriptors
* add empty function for removing features and descriptors
* add functions to DataPoints avoiding error on rows and cols
* fill missing documentation
* resolve warning from unsigned to int in IO.cpp
* add extra empty line in utest
* add extra unit tests and resolve remaining bugs
* Refactored how to load PLY files
* Allow 2D descriptors (##45)
* Allow saving 2D descriptors coming from a 2Dmap, that are converted to 3D when writing to the file but needed after if we want to load the map as 2D.
* Contributors: Francis Colas, Francisco J Perez Grau, François Pomerleau, HannesSommer, Philipp Kruesi, Renaud Dube, Simon Lynen, chipironcin, pomerlef, smichaud, v01d

1.2.2 (2014-08-05)
------------------
* Yaml-cpp0.3 now built with libpointmatcher for compatibility with newer Ubuntu systems using yaml-cpp0.5

1.2.1
-----------
* Fixed bug with soft outlier weights in error minimization
* Fixed some issues for releasing into ROS ecosystem
* Contributors: François Pomerleau, Mike Bosse, Samuel Charreyron, Simon Lynen
