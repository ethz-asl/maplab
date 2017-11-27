#include "simulation/gps-simulator.h"

#include <array>
#include <functional>
#include <limits>

#include <glog/logging.h>
#include <maplab-common/global-coordinate-tools.h>
#include <maplab-common/parallel-process.h>

namespace simulation {

namespace gps_simulator {

void GpsSimulator::simulateMeasurements(
    const Eigen::VectorXd& timestamps_seconds,
    const aslam::TransformationVector& T_G_Bs,
    Aligned<std::vector, GpsMeasurement>* measurements,
    NavigationData* navigation_data) const {
  CHECK_EQ(timestamps_seconds.rows(), static_cast<int>(T_G_Bs.size()));
  CHECK_NOTNULL(measurements);
  CHECK_GT(navigation_data_.ephemeris.size(), 0u);
  if (navigation_data != nullptr) {
    *navigation_data = navigation_data_;
  }
  const size_t num_samples = static_cast<size_t>(timestamps_seconds.rows());
  std::normal_distribution<double> pseudorange_noise_distribution(
      settings_.receiver_pseudorange_noise_mean_m,
      settings_.receiver_pseudorange_noise_sigma_m);
  std::normal_distribution<double> carrier_phase_noise_distribution(
      settings_.receiver_carrierphase_noise_mean_m,
      settings_.receiver_carrierphase_noise_sigma_m);
  Eigen::Vector3d ned_origin_ecef;
  common::llhToEcef(settings_.ned_origin_vector_in_llh, &ned_origin_ecef);

  measurements->resize(num_samples);
  std::function<void(const std::vector<size_t>&)>
      createRawGpsMeasurementForPose = [&](
          const std::vector<size_t>& sample_idx_range) {
        for (size_t sample_idx : sample_idx_range) {
          CHECK_LT(sample_idx, num_samples);
          // Since the timestamps start at 0 we say the walltime is the
          // ephemeris
          // epoch time plus the timestamp.
          CHECK(navigation_data_.ephemeris.empty() == false);
          CHECK_LT(
              navigation_data_.ephemeris[0].ephemeris_data_reference_time.sec +
                  static_cast<uint32_t>(timestamps_seconds[sample_idx]),
              std::numeric_limits<uint32_t>::max());
          ros::Time receiver_gps_time =
              navigation_data_.ephemeris[0].ephemeris_data_reference_time +
              ros::Duration(timestamps_seconds[sample_idx]);
          // Antenna position ECEF.
          Eigen::Vector3d receiver_ecef, sat_ecef;
          common::nedToEcef(
              T_G_Bs[sample_idx].getPosition(), ned_origin_ecef,
              &receiver_ecef);
          // Receiver velocity for doppler observable:
          Eigen::Vector3d receiver_velocity;
          if (sample_idx != num_samples - 1u) {
            Eigen::Vector3d next_receiver_pos;
            common::nedToEcef(
                T_G_Bs[sample_idx + 1].getPosition(), ned_origin_ecef,
                &next_receiver_pos);
            CHECK_GT(
                timestamps_seconds[sample_idx + 1] -
                    timestamps_seconds[sample_idx],
                0.0);
            receiver_velocity = (next_receiver_pos - receiver_ecef) /
                                (timestamps_seconds[sample_idx + 1] -
                                 timestamps_seconds[sample_idx]);
          } else {
            Eigen::Vector3d prev_receiver_pos;
            common::nedToEcef(
                T_G_Bs[sample_idx - 1].getPosition(), ned_origin_ecef,
                &prev_receiver_pos);
            CHECK_GT(
                timestamps_seconds[sample_idx] -
                    timestamps_seconds[sample_idx - 1],
                0.0);
            receiver_velocity = (receiver_ecef - prev_receiver_pos) /
                                (timestamps_seconds[sample_idx] -
                                 timestamps_seconds[sample_idx - 1]);
          }

          for (std::vector<EphemerisData>::const_iterator it =
                   navigation_data_.ephemeris.begin();
               it != navigation_data_.ephemeris.end(); ++it) {
            // Since we do not know the time of transmission for each satellite
            // yet
            // we calculate the position of each satellite at receiver sampling
            // time.
            getSatellitePositionAndVelocityEcef(
                *it, receiver_gps_time, &sat_ecef, nullptr, nullptr);
            // Check visibility.
            double elevation =
                getSatelliteElevationRad(sat_ecef, receiver_ecef);
            if (elevation > settings_.min_elevation_for_visibility_rad) {
              // Now we iterate to get the true satellite position.
              size_t iterations = 0u;
              Eigen::Vector3d old_sat_position_ecef = receiver_ecef;
              Eigen::Vector3d satellite_velocity_ECEF;
              SatelliteMeasurementError ionosphere_errors, troposphere_errors;
              double eccentricity_anomaly;
              ros::Time satellite_transmission_time;
              while (iterations <
                         settings_.satellite_position_calculation_iterations &&
                     (old_sat_position_ecef - sat_ecef).norm() >
                         settings_.satellite_position_calculation_threshold) {
                old_sat_position_ecef = sat_ecef;
                if (settings_.simulate_ionosphere) {
                  ionosphere_errors = modelL1IonosphericDelays(
                      receiver_ecef, sat_ecef, navigation_data_,
                      receiver_gps_time, residual_zenith_ionospheric_delay_s_);
                }
                if (settings_.simulate_troposphere) {
                  troposphere_errors =
                      modelTroposphericDelays(receiver_ecef, elevation);
                }
                double signal_duration =
                    ((receiver_ecef - sat_ecef).norm() +
                     ionosphere_errors.pseudo_range_error_m +
                     troposphere_errors.pseudo_range_error_m) /
                    kSpeedOfLightMPerS;
                satellite_transmission_time =
                    receiver_gps_time - ros::Duration(signal_duration);
                CHECK_GT(signal_duration, 0.0);
                CHECK_GT(satellite_transmission_time.toSec(), 0.0);
                getSatellitePositionAndVelocityEcef(
                    *it, satellite_transmission_time, &sat_ecef,
                    &satellite_velocity_ECEF, &eccentricity_anomaly);
                transformToEcefFrameAtDifferentTime(signal_duration, &sat_ecef);
                transformToEcefFrameAtDifferentTime(
                    signal_duration, &satellite_velocity_ECEF);
                elevation = getSatelliteElevationRad(sat_ecef, receiver_ecef);
                ++iterations;
              }
              // Now get final error values for this position.
              if (settings_.simulate_ionosphere) {
                ionosphere_errors = modelL1IonosphericDelays(
                    receiver_ecef, sat_ecef, navigation_data_,
                    receiver_gps_time, residual_zenith_ionospheric_delay_s_);
              }
              if (settings_.simulate_troposphere) {
                troposphere_errors =
                    modelTroposphericDelays(receiver_ecef, elevation);
              }

              double receiver_clock_error_m =
                  kSpeedOfLightMPerS * receiver_clock_error_s_;
              SatelliteMeasurementError receiver_white_noise(
                  pseudorange_noise_distribution(rng_),
                  carrier_phase_noise_distribution(rng_));

              double time_since_reference =
                  (satellite_transmission_time - it->clock_data_reference_time)
                      .toSec();
              // For satellite clock error calculation see 'Aided Navigation'
              // Appendix C.1.
              double satellite_clock_error_s =
                  kRelativisticCorrection * it->e * sqrt(it->A) *
                      sin(eccentricity_anomaly) +
                  it->af0 + it->af1 * time_since_reference +
                  it->af2 * pow(time_since_reference, 2.0) +
                  residual_satellite_clock_errors_s.at(it->satellite_id);
              // A positive satellite clock error will shorten the pseudorange.
              double satellite_clock_error_m =
                  -satellite_clock_error_s * kSpeedOfLightMPerS;
              double true_distance_m = (receiver_ecef - sat_ecef).norm();
              // TODO(aforster) Add ephemeris and possibly multipath errors.
              SatelliteMeasurement measurement;
              measurement.time =
                  receiver_gps_time + ros::Duration(receiver_clock_error_s_);
              measurement.satellite_id = it->satellite_id;
              measurement.prn_id = it->prn_id;
              measurement.gnss_id = it->gnss_id;
              measurement.SNR << 127u, 0u, 0u;
              measurement.LLI << 0u, 0u, 0u;
              measurement.code << 1u, 0u, 0u;
              measurement.pseudo_range_m[0] =
                  true_distance_m + receiver_clock_error_m +
                  satellite_clock_error_m +
                  ionosphere_errors.pseudo_range_error_m +
                  troposphere_errors.pseudo_range_error_m +
                  receiver_white_noise.pseudo_range_error_m;
              measurement.pseudo_range_m[1] = 0.0;
              measurement.pseudo_range_m[2] = 0.0;
              double carrier_phase_m =
                  true_distance_m + receiver_clock_error_m +
                  satellite_clock_error_m +
                  ionosphere_errors.carrier_phase_error_m +
                  troposphere_errors.carrier_phase_error_m +
                  receiver_white_noise.carrier_phase_error_m;
              constexpr double wavelength =
                  kSpeedOfLightMPerS / kCarrierFrequencyL1;
              measurement.carrier_phase_measurement_cycles[0] =
                  carrier_phase_m / wavelength;
              // Only the fractional part of carrier phase can be observed.
              measurement.carrier_phase_measurement_cycles[0] -=
                  floor(measurement.carrier_phase_measurement_cycles[0]);
              measurement.carrier_phase_measurement_cycles[1] = 0.0;
              measurement.carrier_phase_measurement_cycles[2] = 0.0;

              // Doppler observable.
              // Primary source: Zhang J., Zhang K., Grenfell, R and Dakin, R
              // 2006
              // "On the Relativistic Doppler Effect for Precise Velocity
              // Determination using GPS" , Journal of Geodesy, vol 80 pp.
              // 104-110.
              // TODO(aforster): Calculate change rate of ionosphere /
              // troposphere
              // delay. Implement receiver clock drift and doppler white noise.

              // Receiver to satellite LOS unit vector.
              Eigen::Vector3d n_R_S = (sat_ecef - receiver_ecef).normalized();
              double ionosphere_dot = 0.0;
              double troposphere_dot = 0.0;
              double receiver_clock_drift = 0.0;
              double satellite_clock_drift = it->af1;
              double potential_difference =
                  kGravitationalAcceleration *
                  ((T_G_Bs[sample_idx].getPosition())[2] +
                   settings_.ned_origin_vector_in_llh[2]);
              double sagnac_dot =
                  kEarthRotationRate / kSpeedOfLightMPerS *
                  (satellite_velocity_ECEF[0] * sat_ecef[1] -
                   receiver_velocity[1] * sat_ecef[0] +
                   receiver_ecef[0] * satellite_velocity_ECEF[1] -
                   sat_ecef[1] * satellite_velocity_ECEF[0]);
              double doppler_white_noise = 0.0;

              CHECK_NE(it->A, 0.0);
              CHECK_NE(sat_ecef.norm(), 0.0);
              // This constant is necessary as it is not possible to divide
              // an Eigen object by a constexpr value. It yields an "undefined
              // symbol" error on clang compiler.
              const double kEigenFriendlySpeedOfLightMPerS = kSpeedOfLightMPerS;
              const double doppler_times_wavelength =
                  (n_R_S +
                   satellite_velocity_ECEF / kEigenFriendlySpeedOfLightMPerS)
                          .transpose() *
                      (satellite_velocity_ECEF - receiver_velocity) -
                  ionosphere_dot + troposphere_dot +
                  kSpeedOfLightMPerS * receiver_clock_drift -
                  kSpeedOfLightMPerS * satellite_clock_drift +
                  potential_difference / kSpeedOfLightMPerS +
                  2.0 * kEarthGravitationalConstant / kSpeedOfLightMPerS *
                      (1.0 / it->A - 1.0 / sat_ecef.norm()) +
                  sagnac_dot + doppler_white_noise;

              measurement.doppler_frequency_hz[0] =
                  doppler_times_wavelength / wavelength;
              measurement.doppler_frequency_hz[1] = 0.0;
              measurement.doppler_frequency_hz[2] = 0.0;
              (*measurements)[sample_idx].measurements.push_back(measurement);
            }
          }
        }
      };

  // Iterate over all samples and create the GPS raw data.
  VLOG(1) << "Simulating GPS... ";
  const bool kAlwaysParallelize = true;
  const size_t num_threads = common::getNumHardwareThreads();
  common::ParallelProcess(
      num_samples, createRawGpsMeasurementForPose, kAlwaysParallelize,
      num_threads);
}

void GpsSimulator::initialiseSimulationSettings() {
  // Read RINEX file.
  try {
    rinex_navigation_file_.setPathFilenameMode(
        settings_.rinex_navigation_file_path, std::ios_base::in);
  } catch (NGSrinex::RinexFileException exception) {  // NOLINT
    LOG(FATAL) << "Error opening rinex file "
               << settings_.rinex_navigation_file_path << std::endl
               << "Rinex file exception: " << std::endl
               << exception.getMessage();
  }

  rinex_navigation_file_.readHeader();

  navigation_data_.utc_gps[0] = rinex_navigation_file_.getUtcA0();
  navigation_data_.utc_gps[1] = rinex_navigation_file_.getUtcA1();
  navigation_data_.utc_gps[2] = rinex_navigation_file_.getUtcRefTime();
  navigation_data_.utc_gps[3] = rinex_navigation_file_.getUtcRefWeek();

  navigation_data_.ionosphere_gps[0] = rinex_navigation_file_.getA0();
  navigation_data_.ionosphere_gps[1] = rinex_navigation_file_.getA1();
  navigation_data_.ionosphere_gps[2] = rinex_navigation_file_.getA2();
  navigation_data_.ionosphere_gps[3] = rinex_navigation_file_.getA3();
  navigation_data_.ionosphere_gps[4] = rinex_navigation_file_.getB0();
  navigation_data_.ionosphere_gps[5] = rinex_navigation_file_.getB1();
  navigation_data_.ionosphere_gps[6] = rinex_navigation_file_.getB2();
  navigation_data_.ionosphere_gps[7] = rinex_navigation_file_.getB3();

  navigation_data_.leap_seconds = rinex_navigation_file_.getLeapSec();

  navigation_data_.ephemeris.clear();
  residual_satellite_clock_errors_s.clear();
  std::normal_distribution<double> residual_satellite_clock_error_distribution(
      settings_.residual_satellite_clock_error_mean_s,
      settings_.residual_satellite_clock_error_sigma_s);
  NGSrinex::PRNBlock block;
  while (rinex_navigation_file_.readPRNBlock(block) != 0u) {
    // Sometimes multiple entries for the same satellite are in the same RINEX
    // file. We only keep the newest one.
    std::vector<EphemerisData>::iterator duplicate = std::find_if(
        navigation_data_.ephemeris.begin(), navigation_data_.ephemeris.end(),
        [&](const EphemerisData& e) {
          return e.satellite_id ==
                 static_cast<uint8_t>(block.getSatellitePRN());
        });
    if (duplicate != navigation_data_.ephemeris.end()) {
      if (duplicate->iode > static_cast<uint8_t>(block.getIode())) {
        continue;
      }
      navigation_data_.ephemeris.erase(duplicate);
    }
    EphemerisData ephemeris;
    ephemeris.satellite_id = static_cast<uint8_t>(block.getSatellitePRN());
    ephemeris.prn_id = static_cast<uint8_t>(block.getSatellitePRN());
    ephemeris.gnss_id = 0x01;  // GNSS system ID for the NAVSTAR GPS.
    ephemeris.iode = static_cast<uint8_t>(block.getIode());
    ephemeris.iodc = static_cast<uint16_t>(block.getIodc());
    ephemeris.sat_accuracy_index = static_cast<uint8_t>(block.getSvAccur());
    ephemeris.sat_health = static_cast<uint8_t>(block.getSvHealth());
    ephemeris.week = static_cast<uint32_t>(block.getToeGPSWeek());
    ephemeris.code_on_L2 = static_cast<uint8_t>(block.getCodesOnL2());
    ephemeris.L2_P_data_flag = static_cast<bool>(block.getPDataFlagL2());
    // GPS week starts counting from 6. Jan 1980 00:00.
    struct tm tm_gps_t0;
    tm_gps_t0.tm_year = 80;
    tm_gps_t0.tm_mon = 0;
    tm_gps_t0.tm_mday = 6;
    tm_gps_t0.tm_hour = 0;
    tm_gps_t0.tm_min = 0;
    tm_gps_t0.tm_sec = 0;
    time_t gps_time_0 = mktime(&tm_gps_t0);
    ephemeris.ephemeris_data_reference_time = ros::Time(
        gps_time_0 + static_cast<int>(block.getToeGPSWeek()) * kSecondsPerWeek +
            static_cast<int>(block.getToe()),
        (block.getToe() - std::floor(block.getToe())) * 1e9);
    struct tm tm_toc;
    tm_toc.tm_year = static_cast<int>(block.getTocYear()) - 1900;
    tm_toc.tm_mon = static_cast<int>(block.getTocMonth()) - 1;
    tm_toc.tm_mday = static_cast<int>(block.getTocDay());
    tm_toc.tm_hour = static_cast<int>(block.getTocHour());
    tm_toc.tm_min = static_cast<int>(block.getTocMin());
    tm_toc.tm_sec = static_cast<int>(block.getTocSec());
    time_t toc = mktime(&tm_toc);
    ephemeris.clock_data_reference_time = ros::Time(
        toc, (block.getTocSec() - std::floor(block.getTocSec())) * 1e9);
    ephemeris.transmission_time = ros::Time(
        gps_time_0 + static_cast<int>(block.getToeGPSWeek()) * kSecondsPerWeek +
            static_cast<int>(block.getTransmTime()),
        (block.getTransmTime() - std::floor(block.getTransmTime())) * 1e9);
    ephemeris.A = block.getSqrtA() * block.getSqrtA();
    ephemeris.e = block.getEccen();
    ephemeris.i_0 = block.getIo();
    ephemeris.Omega_0 = block.getBigOmega();
    ephemeris.omega = block.getLilOmega();
    ephemeris.M_0 = block.getMo();
    ephemeris.delta_n = block.getDeltan();
    ephemeris.Omega_dot = block.getBigOmegaDot();
    ephemeris.i_dot = block.getIdot();
    ephemeris.C_rc = block.getCrc();
    ephemeris.C_rs = block.getCrs();
    ephemeris.C_uc = block.getCuc();
    ephemeris.C_us = block.getCus();
    ephemeris.C_ic = block.getCic();
    ephemeris.C_is = block.getCis();
    ephemeris.ephemeris_reference_time_in_week_s = block.getToe();
    ephemeris.fit_interval_hrs = block.getFitInterval();
    ephemeris.af0 = block.getClockBias();
    ephemeris.af1 = block.getClockDrift();
    ephemeris.af2 = block.getClockDriftRate();

    navigation_data_.ephemeris.push_back(ephemeris);

    // The residual satellite clock error stays constant for each satellite
    // during the simulation.
    residual_satellite_clock_errors_s.emplace(
        ephemeris.satellite_id,
        residual_satellite_clock_error_distribution(rng_));
  }

  std::normal_distribution<double> residual_ion_delay_distribution(
      settings_.residual_zenith_ionospheric_delay_mean_s,
      settings_.residual_zenith_ionospheric_delay_sigma_s);
  residual_zenith_ionospheric_delay_s_ = residual_ion_delay_distribution(rng_);

  // Receiver clock error also stays constant over short intervals of time
  // (clock drift not yet simulated). It does not matter much as it is constant
  // for all satellite measurements.
  std::normal_distribution<double> receiver_clock_error_distribution(
      settings_.receiver_clock_error_mean_s,
      settings_.receiver_clock_error_sigma_s);
  receiver_clock_error_s_ = receiver_clock_error_distribution(rng_);
}

SatelliteMeasurementError modelL1IonosphericDelays(
    const Eigen::Vector3d& receiver_ECEF, const Eigen::Vector3d& sat_ECEF,
    const NavigationData& navigation_data, const ros::Time& receiver_gps_time,
    double residual_ionospheric_delay) {
  // To compute the delay according to the Klobuchar model I follow the steps
  // given in Jay A. Farell 'Aided Navigation' Appendix C.5.
  // The magic numbers are all defined there.
  // Angles except for azimuth (only used in sin/cos) all in semi circles.
  Eigen::Vector3d receiver_llh;
  common::ecefToLlh(receiver_ECEF, &receiver_llh);
  const double phi_u = receiver_llh[0] / 180.0;
  const double lambda_u = receiver_llh[1] / 180.0;
  const double azimuth_rad = getSatelliteAzimuthRad(sat_ECEF, receiver_ECEF);
  const double elevation_sc =
      getSatelliteElevationRad(sat_ECEF, receiver_ECEF) / M_PI;
  const double psi = 0.0137 / (elevation_sc + 0.11) - 0.022;
  double phi_i = phi_u + psi * cos(azimuth_rad);
  if (phi_i > 0.416) {
    phi_i = 0.416;
  } else if (phi_i < -0.416) {
    phi_i = -0.416;
  }
  CHECK_NE(cos(phi_i), 0.0);
  const double lambda_i = lambda_u + psi * sin(azimuth_rad) / cos(phi_i * M_PI);
  ros::Time t = receiver_gps_time + ros::Duration(4.32e4 * lambda_i);
  if (t.toSec() >= 86400.0) {
    t -= ros::Duration(std::floor(t.toSec() / 86400.0) * 86400.0);
  } else if (t.toSec() < 0.0) {
    t += ros::Duration(std::floor(-t.toSec() / 86400.0) * 86400.0);
  }
  const double phi_m = phi_i + 0.064 * cos((lambda_i - 1.617) * M_PI);
  double period = navigation_data.ionosphere_gps[4] +
                  navigation_data.ionosphere_gps[5] * phi_m +
                  navigation_data.ionosphere_gps[6] * phi_m * phi_m +
                  navigation_data.ionosphere_gps[7] * phi_m * phi_m * phi_m;
  if (period < 72000.0) {
    period = 72000.0;
  }
  CHECK_NE(period, 0.0);
  const double x = 2.0 * M_PI * (t.toSec() - 50400.0) / period;
  const double F = 1.0 + 16.0 * pow(0.53 - elevation_sc, 3.0);
  double amplitude = navigation_data.ionosphere_gps[0] +
                     navigation_data.ionosphere_gps[1] * phi_m +
                     navigation_data.ionosphere_gps[2] * phi_m * phi_m +
                     navigation_data.ionosphere_gps[3] * phi_m * phi_m * phi_m;
  if (amplitude < 0.0) {
    amplitude = 0.0;
  }
  double delta_s;
  if (fabs(x) < 1.57) {
    delta_s = F * (5e-9 + amplitude * cos(x) + residual_ionospheric_delay);
  } else {
    delta_s = F * 5e-9;
  }
  delta_s *= GpsSimulator::kSpeedOfLightMPerS;
  return SatelliteMeasurementError(delta_s, -delta_s);
}

// "Understanding GPS: Principles and Applications", Elliot D. Kaplan, 2nd
// edition, page 314-319
// UNB Neutral Atmosphere Models - Rodrigo Leandro, Marcelo Santos and Richard
// B. Langley
// A North America Wide Area Neutral Atmosphere Model for GNSS Applications.
// Rodrigo F. Leandro
// The UNB3 Model is used without taking into account the day of the year.
SatelliteMeasurementError modelTroposphericDelays(
    const Eigen::Vector3d& receiver_ecef,
    double satellite_elevation_angle_rad) {
  Eigen::Vector3d receiver_llh;
  common::ecefToLlh(receiver_ecef, &receiver_llh);
  // All numbers are taken from the cited papers and books.
  // k_1, k_2 and k_3 are refractivity constant.
  // R_d is the gas constant for dry air.
  // g_m is a constant used for calculating the acceleration of gravity at the
  // atmospheric column centroid.
  const double k_1 = 77.604;    // [K/mbar]
  const double k_2 = 16.6;      // [K/mbar]
  const double k_3 = 377600.0;  // [K^2/mbar]
  const double R_d = 287.054;   // [J /kg /K]
  const double g_m = 9.784;     // [m/s^2]

  // Interpolation from Table 7.1 p317 and without taking the day of the year
  // into account. Instead of water vapour pressure we use the relative
  // humidity as in the UNB3m model (the model presented in the book is UNB3).
  // See paper "UNB Neutral Atmosphere Models".
  const std::array<double, 5> p0 = {1013.25, 1017.25, 1015.75, 1011.75, 1013.0};
  const std::array<double, 5> T0 = {299.65, 294.15, 283.15, 272.15, 263.65};
  const std::array<double, 5> RH0 = {75.0, 80.0, 76.0, 77.5, 82.5};
  const std::array<double, 5> beta0 = {6.3e-3, 6.05e-3, 5.58e-3, 5.39e-3,
                                       4.53e-3};
  const std::array<double, 5> lambda0 = {2.77, 3.15, 2.57, 1.81, 1.55};
  size_t phi1 = static_cast<size_t>(receiver_llh[0] / 15.0);
  size_t phi0 = phi1 - 1u;
  if (receiver_llh[0] <= 15.0) {
    phi0 = 0u;
  } else if (receiver_llh[0] >= 75.0) {
    phi1 = 4u;
  }
  CHECK_GE(phi0, 0u);
  CHECK_GE(phi1, 0u);
  CHECK_LT(phi0, 5u);
  CHECK_LT(phi1, 5u);
  const double interpolation_factor = (receiver_llh[0] - phi1 * 15.0) / 15.0;
  const double p = p0[phi0] + (p0[phi1] - p0[phi0]) * interpolation_factor;
  const double T = T0[phi0] + (T0[phi1] - T0[phi0]) * interpolation_factor;
  const double RH = RH0[phi0] + (RH0[phi1] - RH0[phi0]) * interpolation_factor;
  const double beta =
      beta0[phi0] + (beta0[phi1] - beta0[phi0]) * interpolation_factor;
  const double lambda =
      lambda0[phi0] + (lambda0[phi1] - lambda0[phi0]) * interpolation_factor;
  // Conversion from relative humidity to water vapor pressure.
  // (IERS Conventions 2003)
  const double es = 0.01 * exp(1.2378847e-5 * (T * T) - 1.9121316e-2 * T +
                               3.393711047e1 - 6.3431645e3 / T);
  const double fw = 1.00062 + 3.14e-6 * p + 5.6e-7 * (pow(T - 273.15, 2.0));
  const double e = (RH / 100.0) * es * fw;

  const double base = 1.0 - beta * receiver_llh[2] / T;
  CHECK_NE(R_d * beta, 0.0);
  CHECK_NE(g_m * (1.0 + lambda), 0.0);
  const double exponent =
      GpsSimulator::kGravitationalAcceleration / (R_d * beta);
  const double T_m = (T - beta * receiver_llh[2]) *
                     (1.0 - beta * R_d / (g_m * (1.0 + lambda)));

  // Caution: The formulas in the book "Understanding GPS.." are pretty wrong.
  // See the other papers on UNB.
  CHECK_NE(g_m * (lambda + 1) - beta * R_d, 0.0);
  CHECK_NE(T, 0.0);
  const double dry_delay = pow(base, exponent) * (1e-6 * k_1 * R_d * p / g_m);
  const double wet_delay = pow(base, (lambda + 1.0) * exponent - 1.0) *
                           (1e-6 * (T_m * k_2 + k_3) * R_d /
                            (g_m * (lambda + 1.0) - beta * R_d) * e / T);
  CHECK_NE(0.002001 + pow(sin(satellite_elevation_angle_rad), 2.0), 0.0);
  // Black and Eisner's mapping function. See "Understanding GPS: Principles
  // and Applications", Elliot D. Kaplan, 2nd edition, page 318.
  const double mapping =
      1.001 / (sqrt(0.002001 + pow(sin(satellite_elevation_angle_rad), 2.0)));
  const double delta = mapping * (dry_delay + wet_delay);
  return SatelliteMeasurementError(delta, delta);
}

// "Integrierte Navigationssysteme", Jan Wendel, 2nd edition, page 100
// "Aided navigation", Jay A. Farrell, page 270, Eq 8.10
void estimateReceiverPositionAndClockErrorFromPseudoRanges(
    const GpsMeasurement& measurement, const NavigationData& navigation_data,
    Eigen::Vector4d* receiver_position_and_clock_bias) {
  const size_t num_satellites = measurement.measurements.size();
  CHECK_GE(num_satellites, 4u);
  CHECK_NOTNULL(receiver_position_and_clock_bias);

  Eigen::Matrix<double, Eigen::Dynamic, 4> H(num_satellites, 4);
  Eigen::VectorXd delta_rho(num_satellites);
  Eigen::Vector3d r_SiR, r_S_ECEF;
  Eigen::VectorXd distances = Eigen::VectorXd::Zero(num_satellites);
  const size_t kNumIters = 10u;
  for (size_t iter = 0u; iter < kNumIters; ++iter) {
    for (size_t i = 0u; i < num_satellites; ++i) {
      // Find relevant ephemeris data.
      std::vector<EphemerisData>::const_iterator ephemeris = std::find_if(
          navigation_data.ephemeris.begin(), navigation_data.ephemeris.end(),
          [&](const EphemerisData& eph) {
            return eph.satellite_id == measurement.measurements[i].satellite_id;
          });
      CHECK(ephemeris != navigation_data.ephemeris.end());
      const double signal_duration =
          distances[i] / GpsSimulator::kSpeedOfLightMPerS;
      const ros::Time satellite_transmission_time_uncorrected =
          measurement.measurements[i].time -
          ros::Duration(
              (*receiver_position_and_clock_bias)[3] /
              GpsSimulator::kSpeedOfLightMPerS) -
          ros::Duration(signal_duration);
      ros::Time satellite_transmission_time_corrected;
      double E_k = 0.0;
      double clock_correction = 0.0;
      double E_k_1 = 1.0;
      const size_t kMaxIter = 10u;
      const double kEpsilon = 1e-5;
      size_t iterations = 0u;
      const double time_since_clock_epoch =
          (satellite_transmission_time_uncorrected -
           ephemeris->clock_data_reference_time)
              .toSec();
      while (fabs(E_k - E_k_1) > kEpsilon && iterations < kMaxIter) {
        ++iterations;
        E_k_1 = E_k;
        clock_correction = ephemeris->af0 +
                           ephemeris->af1 * time_since_clock_epoch +
                           ephemeris->af2 * pow(time_since_clock_epoch, 2.0) +
                           GpsSimulator::kRelativisticCorrection *
                               ephemeris->e * sqrt(ephemeris->A) * sin(E_k);
        satellite_transmission_time_corrected =
            satellite_transmission_time_uncorrected -
            ros::Duration(clock_correction);
        getSatellitePositionAndVelocityEcef(
            *ephemeris, satellite_transmission_time_corrected, &r_S_ECEF,
            nullptr, &E_k);
      }
      transformToEcefFrameAtDifferentTime(signal_duration, &r_S_ECEF);
      r_SiR = receiver_position_and_clock_bias->head(3) - r_S_ECEF;
      distances[i] = r_SiR.norm();
      CHECK_NE(distances[i], 0.0);
      H(i, 0) = r_SiR[0] / distances[i];
      H(i, 1) = r_SiR[1] / distances[i];
      H(i, 2) = r_SiR[2] / distances[i];
      H(i, 3) = 1.0;
      delta_rho[i] = measurement.measurements[i].pseudo_range_m[0] -
                     (distances[i] + (*receiver_position_and_clock_bias)[3] -
                      clock_correction * GpsSimulator::kSpeedOfLightMPerS);
    }
    *receiver_position_and_clock_bias +=
        (H.transpose() * H).inverse() * H.transpose() * delta_rho;
  }
}

// Jay A. Farell "Aided Navigation" Appendix C.2 page 491.
void getSatellitePositionAndVelocityEcef(
    const EphemerisData& ephemeris, const ros::Time& t_sv,
    Eigen::Vector3d* satellite_position, Eigen::Vector3d* satellite_velocity,
    double* eccentricity_anomaly_rad) {
  CHECK(satellite_position);  // The other values are optional.
  // Computed mean motion [radians/s].
  CHECK_NE(ephemeris.A, 0.0);
  const double n_0 =
      sqrt(GpsSimulator::kEarthGravitationalConstant / pow(ephemeris.A, 3.0));
  // Time from reference epoch [s].
  const double t_k = (t_sv - ephemeris.ephemeris_data_reference_time).toSec();
  // Corrected mean motion [radians/s].
  const double n = n_0 + ephemeris.delta_n;
  // Mean anomaly [radian].
  const double M = ephemeris.M_0 + t_k * n;
  // Iteratively calculate E_k (Kepler's eccentricity anomaly equation):
  // E_k = M+e*sin(E_k).
  // Newton's method with
  // f(E_k)  = E_k-M-e*sin(E_k),
  // f'(E_k) = 1-e*cos(E_k).
  double E_k = 0.0;
  const double eps = 1.0e-5;
  double E_k_1 = eps + 1.0;  // E_{k-1}. Just at some value greater than eps.
  size_t iter = 0u;
  const size_t kMaxIter = 10u;
  while (iter < kMaxIter && fabs(E_k - E_k_1) > eps) {
    E_k_1 = E_k;
    CHECK_NE(1.0 - ephemeris.e * cos(E_k_1), 0.0);
    E_k = E_k_1 -
          (E_k_1 - M - ephemeris.e * sin(E_k_1)) /
              (1.0 - ephemeris.e * cos(E_k_1));
    ++iter;
  }
  if (eccentricity_anomaly_rad != nullptr) {
    *eccentricity_anomaly_rad = E_k;
  }
  // True anomaly [rad].
  CHECK_NE(1.0 - ephemeris.e * cos(E_k), 0.0);
  const double v = atan2(
      sqrt(1.0 - pow(ephemeris.e, 2.0)) * sin(E_k), cos(E_k) - ephemeris.e);
  // Argument of latitude [rad].
  const double phi = v + ephemeris.omega;
  // Latitude correction [rad], Radius correction [m], inclination correction
  // [rad].
  const double sin2phi = sin(2.0 * phi);
  const double cos2phi = cos(2.0 * phi);
  const double delta_u = ephemeris.C_us * sin2phi + ephemeris.C_uc * cos2phi;
  const double delta_r = ephemeris.C_rs * sin2phi + ephemeris.C_rc * cos2phi;
  const double delta_i = ephemeris.C_is * sin2phi + ephemeris.C_ic * cos2phi;
  // Corrected argument of latitude, radius and inclination.
  const double u = phi + delta_u;
  const double r = ephemeris.A * (1.0 - ephemeris.e * cos(E_k)) + delta_r;
  const double i = ephemeris.i_0 + delta_i + ephemeris.i_dot * t_k;
  // Orbital plane X,Y position.
  double X = r * cos(u);
  double Y = r * sin(u);
  // Corrected longitude of ascending node.
  const double omega =
      ephemeris.Omega_0 +
      (ephemeris.Omega_dot - GpsSimulator::kEarthRotationRate) * t_k -
      GpsSimulator::kEarthRotationRate *
          ephemeris.ephemeris_reference_time_in_week_s;
  (*satellite_position)[0] = X * cos(omega) - Y * cos(i) * sin(omega);
  (*satellite_position)[1] = X * sin(omega) + Y * cos(i) * cos(omega);
  (*satellite_position)[2] = Y * sin(i);

  if (satellite_velocity != nullptr) {
    const double x = (*satellite_position)[0];
    const double y = (*satellite_position)[1];

    const double E_dot =
        (n_0 + ephemeris.delta_n) / (1.0 - ephemeris.e * cos(E_k));
    const double phi_dot = sqrt(1.0 - ephemeris.e * ephemeris.e) /
                           (1.0 - ephemeris.e * cos(E_k)) * E_dot;
    const double u_dot = (1.0 + 2.0 * ephemeris.C_us * cos2phi -
                          2.0 * ephemeris.C_uc * sin2phi) *
                         phi_dot;
    const double r_dot =
        2.0 * (ephemeris.C_rs * cos2phi - ephemeris.C_rc * sin2phi) * phi_dot +
        ephemeris.A * ephemeris.e * sin(E_k) * E_dot;
    const double X_dot = r_dot * cos(u) - r * sin(u) * u_dot;
    const double Y_dot = r_dot * sin(u) + r * cos(u) * u_dot;
    const double i_dot =
        2.0 * (ephemeris.C_is * cos2phi - ephemeris.C_ic * sin2phi) * phi_dot +
        ephemeris.i_dot;
    double omega_dot = ephemeris.Omega_dot - GpsSimulator::kEarthRotationRate;

    (*satellite_velocity)[0] = X_dot * cos(omega) -
                               Y_dot * cos(i) * sin(omega) +
                               Y * sin(i) * sin(omega) * i_dot - y * omega_dot;
    (*satellite_velocity)[1] = X_dot * sin(omega) +
                               Y_dot * cos(i) * cos(omega) -
                               Y * sin(i) * cos(omega) * i_dot + x * omega_dot;
    (*satellite_velocity)[2] = Y_dot * sin(i) + Y * cos(i) * i_dot;
  }
}

double getSatelliteElevationRad(
    const Eigen::Vector3d& sat_ECEF_m, const Eigen::Vector3d& receiver_ECEF_m) {
  const Eigen::Vector3d AS = sat_ECEF_m - receiver_ECEF_m;
  CHECK_NE(AS.norm() * receiver_ECEF_m.norm(), 0.0);
  return M_PI / 2.0 - acos(
                          1.0 / (AS.norm() * receiver_ECEF_m.norm()) *
                          AS.transpose() * receiver_ECEF_m);
}

double getSatelliteAzimuthRad(
    const Eigen::Vector3d& sat_ECEF_m, const Eigen::Vector3d& receiver_ECEF_m) {
  Eigen::Vector3d sat_NED;
  common::ecefToNed(sat_ECEF_m, receiver_ECEF_m, &sat_NED);
  CHECK(sat_NED[0] != 0.0 && sat_NED[1] != 0.0);
  return atan2(sat_NED[1], sat_NED[0]);
}

void transformToEcefFrameAtDifferentTime(
    double time_difference_s, Eigen::Vector3d* vector) {
  CHECK_NOTNULL(vector);
  double angle = GpsSimulator::kEarthRotationRate * time_difference_s;

  Eigen::Matrix3d R;
  R << cos(angle), sin(angle), 0.0, -sin(angle), cos(angle), 0.0, 0.0, 0.0, 1.0;
  *vector = R * (*vector);
}

}  // namespace gps_simulator
}  // namespace simulation
