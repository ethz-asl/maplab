Release Notes
=============

Wish list for next release
--------------------------

 * Change the API to allow only on kind of transformation, not a list of transformations. See issue [#164](https://github.com/ethz-asl/libpointmatcher/issues/164).
 * Migrate ffrom yaml-cpp 0.3 to 0.5
 * Fix portability problem with FileLogger on Windows
 * Support for OpenMP parallel computing
 * Handle larger point clouds (100 million points and more)

Already implemented in the current master:

 * add a new OutlierFilter: RobustWelschOutlierFilter (for robust cost function using Welsch weighting).
 * Add a new ErrorMinimizer: PointToPointSimilarityErrorMinimizer (rotation + translation + scale)
 * Add a new Transformation: SimilarityTransformation.
 * Add a new data filter: [VoxelGrid](https://github.com/ethz-asl/libpointmatcher/blob/master/doc/Datafilters.md#voxel-grid-filter-) (April 2014, thanks to Samuel Charreyron) 
 * Add support for PCL ([Point Cloud Library](http://pointclouds.org/)) data format: PCD (April 2014, thanks to Samuel Charreyron) 
 * Improve *.vtk legacy format to handle UNSTRUCTURED_GRID format (April 2014, thanks to Samuel Charreyron)
 * New ErrorMinimizer producing covariance matrix (1 April, 2014, thanks to Francisco J Perez Grau)
 * Better user [documentation and tutorials](https://github.com/ethz-asl/libpointmatcher/blob/master/doc/index.md) (5 Fev. 2014 thanks to Samuel Charreyron)
 * Avoid point cloud copies when filtering (10 Jan. 2014 thanks to Oleg Alexandrov)
 * Add CMake support for find_package (20 Sept. 2013)
 * Added support for PLY ([Polygon File Format or the Stanford Triangle Format](http://en.wikipedia.org/wiki/PLY_(file_format))) for ascii encoding (14 March, 2014)
 * Improved CSV file support with import/export of selected descriptors


Version 1.1.0
--------------

 * Removed C++0x dependency 
 * Added compatibility with Visual Studio 11
 * Fixed various bugs
 * Added optimisation of 2-D pose with 3-D data
 
Version 1.0.0
-------------

 * First release with a stable API
