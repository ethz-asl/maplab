#include <iomanip>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "maplab-common/global-coordinate-tools.h"
#include "maplab-common/test/testing-entrypoint.h"
#include "maplab-common/test/testing-predicates.h"

namespace common {

TEST(GlobalCoordinateToolsTest, testLlhToEcefAndBack) {
  // Zurich coordinates in longitude, latitude, height.
  const double latitude_degree = 47.3666700;
  const double longitude_degree = 8.5500000;
  // Height above WGS84 ellipsoid.
  const double height_meter = 429.0;
  const Eigen::Vector3d llh(latitude_degree, longitude_degree, height_meter);

  // Conversion from LLH to ECEF coordinates.
  Eigen::Vector3d ecef;

  common::llhToEcef(llh, &ecef);

  // Conversion from ECEF back to LLH coordinates using iterative method.
  Eigen::Vector3d llh_back_transformed_iterative;
  common::ecefToLlhIterative(ecef, &llh_back_transformed_iterative);
  EXPECT_NEAR_EIGEN(llh, llh_back_transformed_iterative, 1.0e-5);

  // Conversion from ECEF back to LLH coordinates using closed method.
  Eigen::Vector3d llh_back_transformed_closed;
  common::ecefToLlh(ecef, &llh_back_transformed_closed);
  EXPECT_NEAR_EIGEN(llh, llh_back_transformed_closed, 1.0e-5);
}

TEST(GlobalCoordinateToolsTest, testEcefToNedAndBack) {
  // Zurich coordinates in longitude, latitude, height.
  const double latitude_degree = 47.3666700;
  const double longitude_degree = 8.5500000;
  // Height above WGS84 ellipsoid.
  const double height_meter = 429.0;
  const Eigen::Vector3d llh(latitude_degree, longitude_degree, height_meter);

  // Conversion from LLH to ECEF coordinates.
  Eigen::Vector3d ecef;
  common::llhToEcef(llh, &ecef);

  // Conversion from ECEF to NED coordinates.
  const Eigen::Vector3d offset(10.0, 50.0, 20.0);
  const Eigen::Vector3d origin_ecef(ecef + offset);
  Eigen::Vector3d north_east_down;
  common::ecefToNed(ecef, origin_ecef, &north_east_down);

  // Conversion from NED back to ECEF coordinates.
  Eigen::Vector3d ecef_back_transformed;
  common::nedToEcef(north_east_down, origin_ecef, &ecef_back_transformed);

  EXPECT_NEAR_EIGEN(ecef, ecef_back_transformed, 1.0e-12);
}
}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
