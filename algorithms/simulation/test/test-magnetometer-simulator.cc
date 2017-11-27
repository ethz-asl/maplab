#include "simulation/magnetometer-simulator.h"

#include <iomanip>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "simulation/generic-path-generator.h"

namespace simulation {

class MagnetometerSimulatorTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual void SetUp() {
    settings_.mode = test_trajectory_gen::Path::kCircular;
    settings_.imu_sigmas.acc_bias_random_walk_noise_density = 0.1;
    settings_.imu_sigmas.acc_noise_density = 0.1;
    settings_.gravity_meter_by_second2 = 9.81;
    settings_.imu_sigmas.gyro_bias_random_walk_noise_density = 0.1;
    settings_.imu_sigmas.gyro_noise_density = 0.1;
    settings_.imu_noise_bias_seed = 10u;
    settings_.num_of_path_constraints = 8;
    settings_.sampling_time_second = 0.1;

    simulation::YearMonthDay utc_time;
    utc_time.year = 2017;
    utc_time.month = 7;
    utc_time.day = 1;
    mag_sim_.reset(new MagnetometerSimulator(utc_time));

    if (kSaveMagnetometerSimulationToFile) {
      const std::string filename = "/tmp/simulated_magnetometer.txt";
      logger_.reset(new common::FileLogger(filename));
    }
  }

  test_trajectory_gen::PathAndLandmarkSettings settings_;
  std::shared_ptr<test_trajectory_gen::GenericPathGenerator> path_generator_;
  aslam::TransformationVector ground_truth_transformations_;
  std::unique_ptr<MagnetometerSimulator> mag_sim_;

  const bool kSaveMagnetometerSimulationToFile = false;
  std::unique_ptr<common::FileLogger> logger_;
};

// Test magnetometer simulator using official world magnetic model test data
// from 2015.
TEST_F(MagnetometerSimulatorTest, testWorldMagneticModel) {
  const double latitude_degree = -80.0;
  const double longitude_degree = 240.0;
  const double height_above_wgs84_ellipsoid_meter = 100.0 * 1.0e3;
  Eigen::Vector3d llh(
      latitude_degree, longitude_degree, height_above_wgs84_ellipsoid_meter);

  // No rotation w.r.t. earth magnetic field.
  Eigen::Matrix3d R_B_N;
  R_B_N.setIdentity();

  Eigen::Vector3d simulated_magnetic_field_tesla;
  mag_sim_->calculateMagneticField(llh, R_B_N, &simulated_magnetic_field_tesla);

  // Trusted values from world magnetic model, do not change.
  Eigen::Vector3d true_magnetic_field_tesla;
  true_magnetic_field_tesla << 5683.5175495763 * 1e-9, 14808.8492023104 * 1e-9,
      -50163.0133654779 * 1e-9;

  EXPECT_NEAR_EIGEN(
      simulated_magnetic_field_tesla, true_magnetic_field_tesla, 1e-7);
}

// The test goes as follows:
// 1. Simulate a circular trajectory (rotation on spot).
// 2. Calculate magnetic field for every time step.
// 3. Calculate absolute yaw heading from simulated magnetometer values.
// 4. Compare true heading to calculated heading from magnetometer values.
TEST_F(MagnetometerSimulatorTest, testCircularPath) {
  // 1. Simulate a circular trajectory (rotation on spot).
  path_generator_ =
      aligned_shared<test_trajectory_gen::GenericPathGenerator>(settings_);
  path_generator_->generatePath();
  path_generator_->getGroundTruthTransformations(
      &ground_truth_transformations_);
  // Location in global coordinates.
  const double latitude_degree = 47.6027992;
  const double longitude_degree = 8.5341138;
  const double height_above_wgs84_ellipsoid_meter = 411.083;
  Eigen::Vector3d llh(
      latitude_degree, longitude_degree, height_above_wgs84_ellipsoid_meter);

  // Retrieve magnetometer offset.
  Eigen::Matrix3d R_B0_G;
  R_B0_G.setIdentity();
  Eigen::Vector3d magnetic_field_offset_tesla;
  mag_sim_->calculateMagneticField(llh, R_B0_G, &magnetic_field_offset_tesla);
  double yaw_offset_degree =
      -std::atan2(
          magnetic_field_offset_tesla(1), magnetic_field_offset_tesla(0)) *
      180.0 / M_PI;

  for (size_t sample_idx = 0; sample_idx < ground_truth_transformations_.size();
       ++sample_idx) {
    // 2. Calculate magnetic field for every time step.
    Eigen::Vector3d simulated_magnetic_field_tesla;
    const aslam::Transformation& T_G_B =
        ground_truth_transformations_[sample_idx];
    aslam::Transformation T_B_G = T_G_B.inverted();
    Eigen::Matrix3d R_B_G = T_B_G.getRotationMatrix();
    mag_sim_->calculateMagneticField(
        llh, R_B_G, &simulated_magnetic_field_tesla);

    // 3. Calculate absolute yaw heading from simulated magnetometer values.
    double yaw_simulated_degree = -std::atan2(
                                      simulated_magnetic_field_tesla(1),
                                      simulated_magnetic_field_tesla(0)) *
                                  180.0 / M_PI;
    double yaw_true_degree =
        -std::atan2(R_B_G(1, 0), R_B_G(1, 1)) * 180.0 / M_PI;

    // 4. Compare true heading to calculated heading from magnetometer values.
    double error =
        std::abs(yaw_true_degree - (yaw_simulated_degree - yaw_offset_degree));
    error += (error > 180.0) ? -360.0 : (error < -180.0) ? 360.0 : 0.0;
    // Expect error smaller than 1.0e-13 degrees.
    EXPECT_NEAR(error, 0.0, 1.0e-13);

    // Save simulation results to file for debugging.
    if (kSaveMagnetometerSimulationToFile) {
      *logger_ << std::setprecision(15) << simulated_magnetic_field_tesla(0)
               << " " << simulated_magnetic_field_tesla(1) << " "
               << simulated_magnetic_field_tesla(2) << " "
               << yaw_simulated_degree << " " << yaw_offset_degree << " "
               << yaw_true_degree << " " << error << std::endl;
    }
  }
}

}  // namespace simulation

MAPLAB_UNITTEST_ENTRYPOINT
