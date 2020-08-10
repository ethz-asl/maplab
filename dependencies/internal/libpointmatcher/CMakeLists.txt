cmake_minimum_required(VERSION 2.8.11)

include(CheckSymbolExists)

#========================
# Project details / setup
#========================

# Extract version from header, done first to satisfy CMP0048,
# see `cmake --help-policy CMP0048` for more information.
file(
	STRINGS                               # Read in a file to memory.
	pointmatcher/PointMatcher.h           # File to parse for version number.
	POINTMATCHER_PROJECT_VERSION          # Where to store the results (should just be one line)
	REGEX "#define POINTMATCHER_VERSION " # The space after is important to ignore 'POINTMATCHER_VERSION_INT'
)
# If no matches found, something is wrong with PointMatcher.h
if (NOT POINTMATCHER_PROJECT_VERSION)
	message(SEND_ERROR "Cannot find version number in '${CMAKE_CURRENT_SOURCE_DIR}/pointmatcher/PointMatcher.h'.")
endif (NOT POINTMATCHER_PROJECT_VERSION)
# Transform '#define POINTMATCHER_VERSION "X.Y.Z"' into 'X.Y.Z'
string(REGEX REPLACE ".*\"(.*)\".*" "\\1" POINTMATCHER_PROJECT_VERSION "${POINTMATCHER_PROJECT_VERSION}")

# In 3.0+, project(...) should specify VERSION to satisfy CMP0048
if (CMAKE_VERSION VERSION_LESS 3.0.0)
	project(libpointmatcher)
else (CMAKE_VERSION VERSION_LESS 3.0.0)
	cmake_policy(SET CMP0048 NEW)
	project(libpointmatcher VERSION ${POINTMATCHER_PROJECT_VERSION})
endif (CMAKE_VERSION VERSION_LESS 3.0.0)

# Check if 32 bit platform
# By default, libpointmatcher is not compatible with and will not build on a
# 32 bit system
if( NOT CMAKE_SIZEOF_VOID_P EQUAL 8 )
    MESSAGE(SEND_ERROR "32 bits compiler detected. Libpointmatcher is only supported in 64 bits." )
    SET( EX_PLATFORM 32 )
    SET( EX_PLATFORM_NAME "x86" )
endif( NOT CMAKE_SIZEOF_VOID_P EQUAL 8 )

## WARNING: unsupported
## To force install as a 32 bit library, set BUILD_32 to true
if( BUILD_32 )
	MESSAGE(STATUS "Building as a 32 bit library")
	SET(CMAKE_CXX_FLAGS "-m32")
endif( BUILD_32 )

# Ensure proper build type
if (NOT CMAKE_BUILD_TYPE)
  message("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
else (NOT CMAKE_BUILD_TYPE)
  if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("\n=================================================================================")
    message("\n-- Build type: Debug. Performance will be terrible!")
    message("-- Add -DCMAKE_BUILD_TYPE=Release to the CMake command line to get an optimized build.")
    message("\n=================================================================================")
  endif (CMAKE_BUILD_TYPE STREQUAL "Debug")
endif (NOT CMAKE_BUILD_TYPE)


#================= extra building definitions ==============================
if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-O3)
endif(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")

# For Windows
if( MSVC )
	add_definitions( /D _VARIADIC_MAX=10 ) # VS2012 does not support tuples correctly yet
	add_definitions( /D _USE_MATH_DEFINES) # defines M_PI for Visual Studio
	add_definitions( /D _ENABLE_EXTENDED_ALIGNED_STORAGE) # this variable must be defined with VS2017 to acknowledge alignment changes of aligned_storage
endif()

#======================= installation =====================================

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
	set(DEF_INSTALL_CMAKE_DIR CMake)
else()
	set(DEF_INSTALL_CMAKE_DIR lib/cmake/pointmatcher)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")


# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
	set(var INSTALL_${p}_DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
endforeach()

# Install package.xml for catkin
install(FILES package.xml DESTINATION "share/${PROJECT_NAME}")

#===========================================================================


#======================== External Dependencies ===============================

# initially
set(EXTERNAL_LIBS "")
set(EXTRA_DEPS "")

# compile local version of gtest and yaml-cpp
add_subdirectory(contrib)


#--------------------
# DEPENDENCY:  boost
#--------------------
find_package(Boost COMPONENTS thread filesystem system program_options date_time REQUIRED)
if (Boost_MINOR_VERSION GREATER 47)
	find_package(Boost COMPONENTS thread filesystem system program_options date_time chrono REQUIRED)
endif (Boost_MINOR_VERSION GREATER 47)
include_directories(${Boost_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${Boost_LIBRARIES})


#--------------------
# DEPENDENCY: eigen 3
#--------------------
find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen3
	/usr/include/eigen3
)

# Suppress Eigen's warning by adding it to the system's library
include_directories(SYSTEM "${EIGEN_INCLUDE_DIR}")

#TODO: this should be a more standard way
#find_package(Eigen3 REQUIRED)
#message("-- eigen3 , version ${EIGEN_VERSION}")



#--------------------
# DEPENDENCY: nabo
#--------------------
find_package(libnabo REQUIRED PATHS ${LIBNABO_INSTALL_DIR})
#include(libnaboConfig)
include_directories(${libnabo_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${libnabo_LIBRARIES})
message(STATUS "libnabo found, version ${libnabo_VERSION} (include=${libnabo_INCLUDE_DIRS} libs=${libnabo_LIBRARIES})")

#--------------------
# DEPENDENCY: OpenMP (optional)
#--------------------
set(USE_OPEN_MP FALSE CACHE BOOL "Set to TRUE to use OpenMP")
if (USE_OPEN_MP)
	find_package(OpenMP)
	if (OPENMP_FOUND)
		add_definitions(-fopenmp -DHAVE_OPENMP)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		if (CMAKE_COMPILER_IS_GNUCC)
			set(EXTERNAL_LIBS ${EXTERNAL_LIBS} gomp)
    	message("-- OpenMP found, parallel computer enabled")
		endif(CMAKE_COMPILER_IS_GNUCC)
	endif(OPENMP_FOUND)
endif (USE_OPEN_MP)

#--------------------
# DEPENDENCY: OpenCL (optional)
#--------------------
set(USE_OPEN_CL "false" CACHE BOOL "Set to ON to look for OpenCL, if your libnabo was compiled with CL support")
if (USE_OPEN_CL)
	if (WIN32)
		find_library(OPENCL_LIBRARIES opencl64)
		if (!OPENCL_LIBRARIES)
			find_library(OPENCL_LIBRARIES opencl32)
		endif (!OPENCL_LIBRARIES)
	else (WIN32)
		find_library(OPENCL_LIBRARIES OpenCL ENV LD_LIBRARY_PATH)
	endif (WIN32)
	# if found, add
	if (OPENCL_LIBRARIES)
		set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${OPENCL_LIBRARIES})
    message("-- openCL found, parallel computer enabled for kd-tree")
	endif (OPENCL_LIBRARIES)
endif(USE_OPEN_CL)


#--------------------
# DEPENDENCY: yaml-cpp (local or system, optional)
#--------------------
option(USE_SYSTEM_YAML_CPP "Use system version of yaml-cpp rather than one packaged with libpointmatcher" FALSE)

if(USE_SYSTEM_YAML_CPP)
    message(STATUS "Looking for yaml-cpp on system")
    find_path(yaml-cpp_INCLUDE_DIRS yaml-cpp/yaml.h
            /usr/local/include
    )
    find_library(yaml-cpp_LIBRARIES yaml-cpp PATHS
            /usr/local/lib
    )
    if(yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)
            include_directories(${yaml-cpp_INCLUDE_DIRS})
            add_definitions(-DSYSTEM_YAML_CPP)
            set(yamlcpp_FOUND)
            set (EXTERNAL_LIBS ${EXTERNAL_LIBS} ${yaml-cpp_LIBRARIES} )
            message("-- yaml-cpp found, text-based configuration enabled")
    else(yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)
            message("-- yaml-cpp not found, text-based configuration and related applications disabled")
    endif(yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)
else(USE_SYSTEM_YAML_CPP)
        include_directories(contrib/yaml-cpp-pm/include)

#note: this is not working....
        #get_property(yaml-cpp-pm_INCLUDE TARGET yaml-cpp-pm PROPERTY INCLUDE_DIRECTORIES)
        #include_directories(${yaml-cpp-pm_INCLUDE})

        get_property(yaml-cpp-pm_LIB TARGET yaml-cpp-pm PROPERTY LOCATION)
        set (EXTERNAL_LIBS ${EXTERNAL_LIBS} ${yaml-cpp-pm_LIB} )
        set (EXTRA_DEPS ${EXTRA_DEPS} yaml-cpp-pm)
        set(yamlcpp_FOUND)

        get_property(yaml-cpp-pm_VERSION TARGET yaml-cpp-pm PROPERTY VERSION)
        message("-- using built-in yaml-cpp, version ${yaml-cpp-pm_VERSION}")
        message("   -- text-based configuration enabled")
endif(USE_SYSTEM_YAML_CPP)


#--------------------
# DEPENDENCY: rt (optional)
#--------------------
# link rt support if POSIX timers are used
check_symbol_exists(_POSIX_TIMERS "unistd.h;time.h" POSIX_TIMERS)
if (POSIX_TIMERS AND NOT APPLE)
	set(EXTERNAL_LIBS ${EXTERNAL_LIBS} rt)
endif (POSIX_TIMERS AND NOT APPLE)

#============================= end dependencies =================================


#========================== libpointmatcher itself ==============================

# Check the compiler version as we need full C++11 support.
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	# using Clang
	if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.3")
		message(WARNING, "Your clang compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while only version 3.3 or later is supported")
	endif ()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
	# using AppleClang
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.1")
		message(WARNING "Your XCode environment has version ${CMAKE_CXX_COMPILER_VERSION}, while only version 5.1 or later is supported")
	endif()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	# using GCC
	if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "4.8.2")
		message(WARNING, "Your g++ compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while only version 4.8.2 or later is supported")
	endif ()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	# using MSVC
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "19.0.23506")
		message(WARNING "Your Microsoft Visual C++ compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while only version MSVC 2015 Update 1+ or later is supported")
	endif()
endif ()

# enable C++11 support.
if (CMAKE_VERSION VERSION_LESS "3.1")
	if (MSVC)
		message(FATAL_ERROR, "CMake version 3.1 or later is required to compiler ${PROJECT_NAME} with Microsoft Visual C++")
	endif ()
	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		set (CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
	else ()
		set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
	endif ()
else ()
	set (CMAKE_CXX_STANDARD 11)
endif ()

# SOURCE

# Pointmatcher lib and install
set(POINTMATCHER_SRC
	pointmatcher/Logger.cpp
	pointmatcher/Exceptions.cpp
	pointmatcher/DataPoints.cpp
	pointmatcher/Matches.cpp
	pointmatcher/ICP.cpp
	pointmatcher/Registry.cpp
	pointmatcher/Registrar.cpp
	pointmatcher/DataPointsFilter.cpp
	pointmatcher/Matcher.cpp
	pointmatcher/OutlierFilter.cpp
	pointmatcher/ErrorMinimizer.cpp
	pointmatcher/Transformation.cpp
	pointmatcher/TransformationChecker.cpp
	pointmatcher/Inspector.cpp
	pointmatcher/IO.cpp
	pointmatcher/IOFunctions.cpp
	pointmatcher/Bibliography.cpp
	pointmatcher/Timer.cpp
	pointmatcher/Histogram.cpp
	pointmatcher/Parametrizable.cpp
	pointmatcher/LoggerImpl.cpp
	pointmatcher/MatchersImpl.cpp
	pointmatcher/OutlierFiltersImpl.cpp
	pointmatcher/TransformationsImpl.cpp
	pointmatcher/TransformationCheckersImpl.cpp
	pointmatcher/InspectorsImpl.cpp
#ErrorMinimizers
	pointmatcher/ErrorMinimizers/PointToPlane.cpp
	pointmatcher/ErrorMinimizers/PointToPlaneWithCov.cpp
	pointmatcher/ErrorMinimizers/PointToPoint.cpp
	pointmatcher/ErrorMinimizers/PointToPointWithCov.cpp
	pointmatcher/ErrorMinimizers/PointToPointSimilarity.cpp
	pointmatcher/ErrorMinimizers/Identity.cpp
#DataPointsFilters	
	pointmatcher/DataPointsFilters/Identity.cpp
	pointmatcher/DataPointsFilters/RemoveNaN.cpp
	pointmatcher/DataPointsFilters/MaxDist.cpp
	pointmatcher/DataPointsFilters/MinDist.cpp
	pointmatcher/DataPointsFilters/BoundingBox.cpp
	pointmatcher/DataPointsFilters/MaxQuantileOnAxis.cpp
	pointmatcher/DataPointsFilters/MaxDensity.cpp
	pointmatcher/DataPointsFilters/SurfaceNormal.cpp
	pointmatcher/DataPointsFilters/SamplingSurfaceNormal.cpp
	pointmatcher/DataPointsFilters/OrientNormals.cpp
	pointmatcher/DataPointsFilters/IncidenceAngle.cpp
	pointmatcher/DataPointsFilters/RandomSampling.cpp
	pointmatcher/DataPointsFilters/MaxPointCount.cpp
	pointmatcher/DataPointsFilters/FixStepSampling.cpp
	pointmatcher/DataPointsFilters/Shadow.cpp
	pointmatcher/DataPointsFilters/SimpleSensorNoise.cpp
	pointmatcher/DataPointsFilters/ObservationDirection.cpp
	pointmatcher/DataPointsFilters/VoxelGrid.cpp
	pointmatcher/DataPointsFilters/CutAtDescriptorThreshold.cpp
	pointmatcher/DataPointsFilters/Elipsoids.cpp
	pointmatcher/DataPointsFilters/Gestalt.cpp
	pointmatcher/DataPointsFilters/OctreeGrid.cpp
	pointmatcher/DataPointsFilters/NormalSpace.cpp
	pointmatcher/DataPointsFilters/CovarianceSampling.cpp
	pointmatcher/DataPointsFilters/DistanceLimit.cpp
	pointmatcher/DataPointsFilters/RemoveSensorBias.cpp
)

include_directories(${CMAKE_SOURCE_DIR})


file(GLOB_RECURSE POINTMATCHER_HEADERS "pointmatcher/*.h")

# MacOS and Windows deal with shared/dynamic library differently. For
# simplicity, we only authorize static library in those cases.
if(APPLE OR WIN32)

	add_library(pointmatcher ${POINTMATCHER_SRC} ${POINTMATCHER_HEADERS} )
	install(TARGETS pointmatcher ARCHIVE DESTINATION ${INSTALL_LIB_DIR})

else(APPLE OR WIN32)
	set(SHARED_LIBS "true" CACHE BOOL "To build shared (true) or static (false) library")

	if (SHARED_LIBS)
		add_library(pointmatcher SHARED ${POINTMATCHER_SRC} ${POINTMATCHER_HEADERS})
		install(TARGETS pointmatcher
						ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
						LIBRARY DESTINATION ${INSTALL_LIB_DIR}
						RUNTIME DESTINATION ${INSTALL_BIN_DIR})
		#install(TARGETS pointmatcher LIBRARY DESTINATION ${INSTALL_LIB_DIR})
	else(SHARED_LIBS)
		add_library(pointmatcher ${POINTMATCHER_SRC} ${POINTMATCHER_HEADERS} )
		install(TARGETS pointmatcher ARCHIVE DESTINATION ${INSTALL_LIB_DIR})
	endif(SHARED_LIBS)
endif(APPLE OR WIN32)

target_include_directories(pointmatcher PRIVATE ${CMAKE_SOURCE_DIR}/pointmatcher)
target_include_directories(pointmatcher PRIVATE ${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters)
target_include_directories(pointmatcher PRIVATE ${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters/utils)

# link all libraries to libpointmatcher
add_definitions(-Wall)
#target_link_libraries(pointmatcher ${yaml-cpp_LIBRARIES} ${libnabo_LIBRARIES})
target_link_libraries(pointmatcher ${EXTERNAL_LIBS})

if(EXTRA_DEPS)
	add_dependencies(pointmatcher ${EXTRA_DEPS})
endif(EXTRA_DEPS)
set_target_properties(pointmatcher PROPERTIES VERSION "${POINTMATCHER_PROJECT_VERSION}" SOVERSION 1)

#========================= Install header ===========================

install(FILES
	pointmatcher/DeprecationWarnings.h
	pointmatcher/PointMatcher.h
	pointmatcher/PointMatcherPrivate.h
	pointmatcher/Parametrizable.h
	pointmatcher/Registrar.h
	pointmatcher/Timer.h
	pointmatcher/Functions.h
	pointmatcher/IO.h
	DESTINATION ${INSTALL_INCLUDE_DIR}/pointmatcher
)

#========================= Documentation ===========================

set(GENERATE_API_DOC false CACHE BOOL "Set to true to build the documentation using Doxygen")

if(GENERATE_API_DOC)

	message("-- API Documentation (doxygen): enabled")

	# Note: there seems to be equations in the documentation leading to the
	# use of Latex anyway. This cause problems for user without latex...

	set(DOXYFILE_LATEX "NO")
	include(UseDoxygen.cmake)

	set(DOC_INSTALL_TARGET "share/doc/${PROJECT_NAME}/api" CACHE STRING "Target where to install doxygen documentation")

	add_dependencies(pointmatcher doc)

	install(FILES README.md DESTINATION share/doc/${PROJECT_NAME})
	if (DOXYGEN_FOUND)
		install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/html DESTINATION ${DOC_INSTALL_TARGET})
	endif (DOXYGEN_FOUND)

else(GENERATE_API_DOC)

	message("-- API Documentation (doxygen): disabled")

endif(GENERATE_API_DOC)


#=============== trigger other makefile ======================

# Example programs
add_subdirectory(examples)
# Evaluation programs
add_subdirectory(evaluations)

# Unit testing

option(BUILD_TESTS "Build all tests." OFF)

if (BUILD_TESTS)
	enable_testing()
	add_subdirectory(utest)
endif()

#=================== allow find_package() =========================
#
# the following case be used in an external project requiring libpointmatcher:
#  ...
#  find_package(libpointmatcher)
#  include_directories(${libpointmatcher_INCLUDE_DIRS})
#  target_link_libraries(executableName ${libpointmatcher_LIBRARIES})
#  ...

# 1- local build #

# Register the local build in case one doesn't use "make install"
export(PACKAGE libpointmatcher)

# Create variable for the local build tree
get_property(CONF_INCLUDE_DIRS DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

# Create variable with the library location
get_target_property(POINTMATCHER_LIB pointmatcher LOCATION)

# Configure config file for local build tree
configure_file(libpointmatcherConfig.cmake.in
  "${PROJECT_BINARY_DIR}/libpointmatcherConfig.cmake" @ONLY)

# 2- installation build #

# Change the include location for the case of an install location
set(CONF_INCLUDE_DIRS ${INSTALL_INCLUDE_DIR} ${CONF_INCLUDE_DIRS} )

#FIXME: this will only be applied to installed files. Confirm that we want that.
# gather all the includes but remove ones in the source tree
# because we added an include for the local yaml-cpp-pm we should also remove it
list(REMOVE_ITEM CONF_INCLUDE_DIRS ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/contrib/yaml-cpp-pm/include)

if(SHARED_LIBS AND (NOT USE_SYSTEM_YAML_CPP))
  list(REMOVE_ITEM EXTERNAL_LIBS ${yaml-cpp-pm_LIB})
endif(SHARED_LIBS AND (NOT USE_SYSTEM_YAML_CPP))

# Change the library location for an install location
get_filename_component(POINTMATCHER_LIB_NAME ${POINTMATCHER_LIB} NAME)
set(POINTMATCHER_LIB ${INSTALL_LIB_DIR}/${POINTMATCHER_LIB_NAME})

# We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
configure_file(libpointmatcherConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libpointmatcherConfig.cmake" @ONLY)

# The same versioning file can be used for both cases
configure_file(libpointmatcherConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/libpointmatcherConfigVersion.cmake" @ONLY)


# Install the libpointmatcherConfig.cmake and libpointmatcherConfigVersion.cmake
install(
  FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libpointmatcherConfig.cmake"
    "${PROJECT_BINARY_DIR}/libpointmatcherConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev
)

# useful for TRADR european project. TODO: check to use the other install
# required for ros deployment, too
install (
  FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libpointmatcherConfig.cmake"
    "${PROJECT_BINARY_DIR}/libpointmatcherConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake/"
)

#Handle pkg-config file
set(LIBRARY_CC_ARGS "")
foreach(_LIB IN LISTS POINTMATCHER_LIB EXTERNAL_LIBS)
  get_filename_component(_FILE_NAME ${_LIB} NAME)
  if(${_FILE_NAME} STREQUAL ${_LIB}) # not an absolute path
    set(LIBRARY_CC_ARGS "${LIBRARY_CC_ARGS} -l${_LIB}")
  else()
    set(LIBRARY_CC_ARGS "${LIBRARY_CC_ARGS} ${_LIB}")
  endif()
endforeach()
unset(_LIB)
unset(_FILE_NAME)

configure_file(pointmatcher.pc.in libpointmatcher.pc @ONLY)
configure_file(pointmatcher.pc.in pointmatcher.pc @ONLY) # for backward compatibility
install(FILES
  ${CMAKE_BINARY_DIR}/libpointmatcher.pc
  ${CMAKE_BINARY_DIR}/pointmatcher.pc # for backward compatibility
  DESTINATION lib/pkgconfig
)
unset(LIBRARY_CC_ARGS)

#====================== uninstall target ===============================
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
