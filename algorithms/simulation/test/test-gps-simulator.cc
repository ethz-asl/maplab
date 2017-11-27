#include <iomanip>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <aslam/common/statistics/statistics.h>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/feature-descriptor-ref.h>
#include <maplab-common/global-coordinate-tools.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "simulation/generic-path-generator.h"
#include "simulation/gps-simulator.h"

namespace simulation {

namespace gps_simulator {

class GpsSimulatorTest : public ::testing::Test {
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
    settings_.landmark_seed = 5u;
    settings_.landmark_variance_meter = 0.25;
    settings_.num_of_landmarks = 1500;
    settings_.sampling_time_second = 0.1;
    settings_.distance_to_keypoints_meter = 7.5;

    gps_simulator_.reset(new GpsSimulator);

    if (kSaveGnssSimulationToFile) {
      const std::string filename = "/tmp/simulated_gnss.txt";
      logger_.reset(new common::FileLogger(filename));
    }
  }
  test_trajectory_gen::PathAndLandmarkSettings settings_;
  std::shared_ptr<test_trajectory_gen::GenericPathGenerator> path_generator_;
  std::unique_ptr<GpsSimulator> gps_simulator_;

  const bool kSaveGnssSimulationToFile = false;
  std::unique_ptr<common::FileLogger> logger_;
};

TEST_F(GpsSimulatorTest, testSimulateGps_noErrors) {
  path_generator_ =
      aligned_shared<test_trajectory_gen::GenericPathGenerator>(settings_);

  path_generator_->generatePath();

  aslam::TransformationVector T_G_Bs;
  path_generator_->getGroundTruthTransformations(&T_G_Bs);

  GpsSimulator::GpsSimulationSettings no_noise_settings;
  no_noise_settings.residual_zenith_ionospheric_delay_mean_s = 0.0;
  no_noise_settings.residual_zenith_ionospheric_delay_sigma_s = 0.0;
  no_noise_settings.receiver_clock_error_mean_s = 0.0;
  no_noise_settings.receiver_clock_error_sigma_s = 0.0;
  no_noise_settings.residual_satellite_clock_error_mean_s = 0.0;
  no_noise_settings.residual_satellite_clock_error_sigma_s = 0.0;
  no_noise_settings.receiver_pseudorange_noise_mean_m = 0.0;
  no_noise_settings.receiver_pseudorange_noise_sigma_m = 0.0;
  no_noise_settings.simulate_ionosphere = false;
  no_noise_settings.simulate_troposphere = false;
  gps_simulator_->setGpsSimulationSettings(no_noise_settings);
  if (kSaveGnssSimulationToFile) {
    // First block of the log contains the simulation settings.
    *logger_ << std::setprecision(15)
             << no_noise_settings.ned_origin_vector_in_llh[0] << " "
             << no_noise_settings.ned_origin_vector_in_llh[1] << " "
             << no_noise_settings.ned_origin_vector_in_llh[2] << " "
             << no_noise_settings.min_elevation_for_visibility_rad << " "
             << no_noise_settings.residual_zenith_ionospheric_delay_mean_s
             << " "
             << no_noise_settings.residual_zenith_ionospheric_delay_sigma_s
             << " " << no_noise_settings.receiver_clock_error_mean_s << " "
             << no_noise_settings.receiver_clock_error_sigma_s << " "
             << no_noise_settings.residual_satellite_clock_error_mean_s << " "
             << no_noise_settings.residual_satellite_clock_error_sigma_s << " "
             << no_noise_settings.receiver_pseudorange_noise_mean_m << " "
             << no_noise_settings.receiver_pseudorange_noise_sigma_m << " "
             << no_noise_settings.receiver_carrierphase_noise_mean_m << " "
             << no_noise_settings.receiver_carrierphase_noise_sigma_m << " "
             << no_noise_settings.rinex_navigation_file_path << " "
             << no_noise_settings.satellite_position_calculation_iterations
             << " "
             << no_noise_settings.satellite_position_calculation_threshold
             << " " << no_noise_settings.simulate_troposphere << " "
             << no_noise_settings.simulate_ionosphere << std::endl;
  }

  Aligned<std::vector, GpsMeasurement> measurements;
  NavigationData navigation_data;
  const Eigen::VectorXd timestamps = path_generator_->getTimestampsInSeconds();
  gps_simulator_->simulateMeasurements(
      timestamps, T_G_Bs, &measurements, &navigation_data);
  Eigen::Vector3d ned_origin_ecef, receiver_ecef;
  common::llhToEcef(
      no_noise_settings.ned_origin_vector_in_llh, &ned_origin_ecef);
  for (size_t i = 0u; i < measurements.size(); ++i) {
    Eigen::Vector4d estimated_position = Eigen::Vector4d::Zero();
    estimateReceiverPositionAndClockErrorFromPseudoRanges(
        measurements[i], navigation_data, &estimated_position);
    common::nedToEcef(T_G_Bs[i].getPosition(), ned_origin_ecef, &receiver_ecef);
    EXPECT_NEAR_EIGEN(receiver_ecef, estimated_position.head(3), 0.5);
    EXPECT_NEAR(
        estimated_position[3], 0.0, 0.01 * GpsSimulator::kSpeedOfLightMPerS);
    if (kSaveGnssSimulationToFile) {
      // Logging format for one measurement.
      // true_time true_receiver_x true_receiver_y true_receiver_z
      // estimated_receiver_x estimated_receiver_y estimated_receiver_z
      // estimated_receiver_clock_error number_of_satellites newline.
      // Number of satellite times: {satellite_id receiver_time
      // carrier_phase_cycles pseudo_range_m doppler_frequency_hz newline}.
      *logger_ << std::setprecision(15) << timestamps[i] << " "
               << receiver_ecef[0] << " " << receiver_ecef[1] << " "
               << receiver_ecef[2] << " " << estimated_position[0] << " "
               << estimated_position[1] << " " << estimated_position[2] << " "
               << estimated_position[3] << " "
               << measurements[i].measurements.size() << std::endl;
      for (std::vector<SatelliteMeasurement>::iterator it =
               measurements[i].measurements.begin();
           it != measurements[i].measurements.end(); ++it) {
        *logger_ << std::setprecision(15) << static_cast<int>(it->satellite_id)
                 << " " << it->time << " "
                 << it->carrier_phase_measurement_cycles[0] << " "
                 << it->pseudo_range_m[0] << " " << it->doppler_frequency_hz[0]
                 << std::endl;
      }
    }
  }
}

TEST_F(GpsSimulatorTest, testTroposphericDelay) {
  // Coordinates of the ETH tramstation in LLH.
  Eigen::Vector3d eth_llh(47.37669, 8.54866, 454.3);
  Eigen::Vector3d eth_ecef;
  common::llhToEcef(eth_llh, &eth_ecef);

  for (double angle_deg = 5.0; angle_deg < 180.0; angle_deg += 5.0) {
    SatelliteMeasurementError error =
        modelTroposphericDelays(eth_ecef, angle_deg * M_PI / 180.0);
    EXPECT_LT(error.carrier_phase_error_m, 30.0);
    EXPECT_LT(error.pseudo_range_error_m, 30.0);
  }
}

TEST_F(GpsSimulatorTest, testIonosphericDelay) {
  // Coordinates of the ETH tramstation in LLH.
  Eigen::Vector3d eth_llh(47.37669, 8.54866, 454.3);
  Eigen::Vector3d eth_ecef;
  common::llhToEcef(eth_llh, &eth_ecef);
  NavigationData navigation_data = gps_simulator_->getNavigationData();
  ros::Time receiver_gps_time =
      navigation_data.ephemeris[0].ephemeris_data_reference_time;

  for (std::vector<EphemerisData>::const_iterator it =
           navigation_data.ephemeris.begin();
       it != navigation_data.ephemeris.end(); ++it) {
    Eigen::Vector3d sat_ecef;
    getSatellitePositionAndVelocityEcef(
        *it, receiver_gps_time, &sat_ecef, nullptr, nullptr);
    // Check visibility.
    double elevation = getSatelliteElevationRad(sat_ecef, eth_ecef);
    if (elevation > 5.0 * M_PI / 180.0) {
      SatelliteMeasurementError error = modelL1IonosphericDelays(
          eth_ecef, sat_ecef, navigation_data, receiver_gps_time, 0.0);
      EXPECT_LT(fabs(error.carrier_phase_error_m), 15.0);
      EXPECT_LT(fabs(error.pseudo_range_error_m), 15.0);
    }
  }
}

}  // namespace gps_simulator
}  // namespace simulation

MAPLAB_UNITTEST_ENTRYPOINT
