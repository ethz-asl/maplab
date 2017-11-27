#ifndef SIMULATION_GPS_SIMULATOR_H_
#define SIMULATION_GPS_SIMULATOR_H_

#include <map>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <ros/package.h>
#include <ros/time.h>

#include "simulation/external/rinex.h"

namespace simulation {

namespace gps_simulator {

// Some sources to start:
//  - Working with GPS raw data:
//    http://web.ics.purdue.edu/~ecalais/teaching/gps_geodesy/
//  - Matlab scripts for working with GPS raw data:
//    http://web.ics.purdue.edu/~ecalais/teaching/gps_geodesy/matlab/
//  - RINEX archive: ftp://igscb.jpl.nasa.gov/igscb/product/1840/

///
/// \brief The SatelliteMeasurement struct contains the raw measurement of one
///        individual GNSS satellite.
///
struct SatelliteMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::Time time;
  uint8_t satellite_id;
  uint8_t prn_id;
  uint8_t gnss_id;
  Eigen::Matrix<uint8_t, 3, 1> SNR;
  Eigen::Matrix<uint8_t, 3, 1> LLI;
  Eigen::Matrix<uint8_t, 3, 1> code;
  Eigen::Vector3d carrier_phase_measurement_cycles;
  Eigen::Vector3d pseudo_range_m;
  Eigen::Vector3d doppler_frequency_hz;
};

///
/// \brief The GPSMeasurement struct contains the raw GPS measurement.
///
struct GpsMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// The individual measurements.
  std::vector<SatelliteMeasurement> measurements;
};

///
/// \brief The GpsMeasurementError struct contains errors for the different
///        GPS observables. The carrier phase measurement error is multiplied
///        with its wavelength and therefore in meters.
///
struct SatelliteMeasurementError {
  double pseudo_range_error_m;
  double carrier_phase_error_m;

  SatelliteMeasurementError()
      : pseudo_range_error_m(0.0), carrier_phase_error_m(0.0) {}
  SatelliteMeasurementError(double pseudo_error, double carrier_error_m)
      : pseudo_range_error_m(pseudo_error),
        carrier_phase_error_m(carrier_error_m) {}
};

///
/// \brief The EphemerisData struct emulates the Ephemeris.msg from the
///        gps_raw_data_driver package. It includes all information to
///        calculate satellite position and velocity.
///
struct EphemerisData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// Satellite number. Unique over all GNSS systems.
  uint8_t satellite_id;
  /// prn / slot ID of the satellite. Unique within a system.
  uint8_t prn_id;
  /// GNSS system ID. 0x01 for GPS, 0x04 GLONASS.
  uint8_t gnss_id;
  /// Issue of data, ephemeris.
  uint8_t iode;
  /// Issue of data, clock.
  uint16_t iodc;
  /// User range accuracy index (URA).
  uint8_t sat_accuracy_index;
  /// Satellite health. 0: healthy.
  uint8_t sat_health;
  /// GPS week.
  uint32_t week;
  /// Code on L2: 01 = P code. 2 C/A code.
  uint8_t code_on_L2;
  /// Flag for whether NAV data stream is off on the L2 channel.
  bool L2_P_data_flag;
  /// Ephemeris reference time. t_oe.
  ros::Time ephemeris_data_reference_time;
  /// Clock data reference time. t_oc.
  ros::Time clock_data_reference_time;
  /// Transmission time of the ephemeris data.
  ros::Time transmission_time;
  // Satellite orbit parameters. See for example Jay. A. Farrel 'Aided
  // Navigation'Appendix C.2 for further information.
  /// Semi-major axis [m].
  double A;
  /// Eccentricity [-].
  double e;
  /// Inclination angle at reference time [rad].
  double i_0;
  /// Longitude of the ascending node at reference time [rad].
  double Omega_0;
  /// Argument of perigee [rad].
  double omega;
  /// Mean anomaly at reference time [rad].
  double M_0;
  /// Mean motion difference from the computed value [rad / s].
  double delta_n;
  /// Rate of right ascension [rad / s].
  double Omega_dot;
  /// Rate of inclination angle [rad / s].
  double i_dot;
  /// Amplitude of the cosine harmonic correction to the orbit radius [m].
  double C_rc;
  /// Amplitude of the sine harmonic correction to the orbit radius [m].
  double C_rs;
  /// Amplitude of the cosine harmonic correction to the argument of latitude
  /// [rad].
  double C_uc;
  /// Amplitude of the sine harmonic correction to the arguemnt of latitude
  /// [rad].
  double C_us;
  /// Amplitude of the cosine harmonic correction to the angle of inclination
  /// [rad].
  double C_ic;
  /// Amplitude of the sine harmonic correction to the angle of inclination
  /// [rad].
  double C_is;
  /// Ephemeris reference time in current GPS week [s].
  double ephemeris_reference_time_in_week_s;
  /// Fit interval time used to determine the ephemeris parameters [hrs].
  double fit_interval_hrs;
  /// Constant correction to satellite clock [s].
  double af0;
  /// First order correction to satellite clock [s/s].
  double af1;
  /// Second order correction to satellite clock [sec/sec^2].
  double af2;
  /// Group delay parameters. See Section 20.3.3.3.3.2 of IS-GPS-200H
  /// specifications.
  Eigen::Vector4d tgd;
};

///
/// \brief The NavigationData struct emulates the NavigationData.msg from the
///        gps_raw_data_driver package. It contains necessary information to
///        calculate satellite positions, clock correction parameters and
///        ionosphere parameters.
///
struct NavigationData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<EphemerisData> ephemeris;
  Eigen::Vector4d utc_gps;
  Eigen::Matrix<double, 8, 1> ionosphere_gps;
  uint32_t leap_seconds;
};

///
/// \brief Models the ionospheric delay of the GPS observables according to
///        the Klobuchar model with given ephemeris data. Only valid for the
///        L1 carrier frequency. Since this model only removes around 50% of
///        the delay we also add a residual delay to it. For a description see
///        for example Jay. A. Farell 'Aided Navigation' Appendix C.5.
/// \param receiver_ECEF True receiver position in ECEF coordinates [m].
/// \param sat_ECEF The satellite position in ECEF coordinates [m].
/// \param navigation_data The navigation data containing ionospheric correction
///                        data.
/// \param receiver_gps_time The true GPS time of the receiver.
/// \param residual_ionospheric_delay The residual delay that is not modelled
///                                   with the Klobuchar model.
/// \return The error due to the ionosphere.
///
SatelliteMeasurementError modelL1IonosphericDelays(
    const Eigen::Vector3d& receiver_ECEF, const Eigen::Vector3d& sat_ECEF,
    const NavigationData& navigation_data, const ros::Time& receiver_gps_time,
    double residual_ionospheric_delay);

///
/// \brief Models the tropospheric delay of the GPS observables according to
///        the UNB3 model.
///        Sources:
///        - "Understanding GPS: Principles and Applications",
///          Elliot D. Kaplan, 2nd edition, page 314-319.
///        - UNB Neutral Atmosphere Models - Rodrigo Leandro, Marcelo Santos
///          and Richard B. Langley.
///        - A North America Wide Area Neutral Atmosphere Model for GNSS
///          Applications. Rodrigo F. Leandro.
///        Note: The UNB3 Model is used without taking into account the day of
///        the year.
/// \param receiver_ecef True receiver position in ECEF coordinates [m].
/// \return The The error due to the troposphere.
///
SatelliteMeasurementError modelTroposphericDelays(
    const Eigen::Vector3d& receiver_ecef, double satellite_elevation_angle_rad);

///
/// \brief Estimates the receiver position and clock error from pseudo range
///        measurements.
///        http://web.ics.purdue.edu/~ecalais/teaching/gps_geodesy/matlab/get_pos.m.
///        http://web.ics.purdue.edu/~ecalais/teaching/gps_geodesy/matlab/solve_PR.m.
///        "Integrierte Navigationssysteme", Jan Wendel, 2nd edition, page 100.
/// \param[in] measurement The GPS measurements.
/// \param[in] navigation_data The navigation data for the GPS measurements.
/// \param[in, out] receiver_position_and_clock_bias Input the a priori
///                 receiver position and clock bias. Will be updated to
///                 contain the estimated position and clock bias. Position
///                 in ECEF coordinates [m] and the clock bias is multiplied
///                 by the speed of light [m].
///
void estimateReceiverPositionAndClockErrorFromPseudoRanges(
    const GpsMeasurement& measurement, const NavigationData& navigation_data,
    Eigen::Vector4d* receiver_position_and_clock_bias);

///
/// \brief Calculates the position of a satellite according to given ephemeris
///        data in ECEF coordinates. Note that the ECEF coordinate system is
///        rotating. The coordinates are expressed in the ECEF frame at
///        satellite broadcast time. See for example Jay A. Farell "Aided
///        Navigation" Appendix C.2 page 491. To convert it into a ECEF
///        frame at a different time see Appendix C.3.
/// \param ephemeris The broadcast ephemeris data.
/// \param t_sv Already corrected, i.e. true GPS time.
/// \param satellite_position Position of the satellite in ECEF coordinates.
///                           Provide a nullpointer if not interested in the
///                           position.
/// \param satellite_velocity Velocity of the satellite in ECEF coordinates.
///                           Provide a nullpointer if not interested in the
///                           velocity.
/// \param eccentricity_anomaly_rad Return the iteratively found eccentricity
///                                 anomaly E_k. Provide a nullpointer if not
///                                 interested in the value.
///
void getSatellitePositionAndVelocityEcef(
    const EphemerisData& ephemeris, const ros::Time& t_sv,
    Eigen::Vector3d* satellite_position, Eigen::Vector3d* satellite_velocity,
    double* eccentricity_anomaly_rad);

///
/// \brief Calculates the satellite elevation angle in radian.
/// \param sat_ECEF_m The position of the satellite in ECEF coordinates.
/// \param receiver_ECEF_m The position of the receiver in ECEF coordinates.
/// \return The elevation angle of the satellite in radian.
///
double getSatelliteElevationRad(
    const Eigen::Vector3d& sat_ECEF_m, const Eigen::Vector3d& receiver_ECEF_m);

///
/// \brief Calculates the satellite azimuth angle in radian.
/// \param sat_ECEF_m The position of the satellite in ECEF coordinates.
/// \param rceiver_ECEF_m The position of the receiver in ECEF coordinates.
/// \return The azimuth angle from true north in radian.
///
double getSatelliteAzimuthRad(
    const Eigen::Vector3d& sat_ECEF_m, const Eigen::Vector3d& receiver_ECEF_m);

///
/// \brief Since the ECEF frame is rotating with the earth it is distinct at
///        each timestamp with respect to some inertial frame. This function
///        rotates a vector from the ECEF frame at t0 to the frame at
///        t0 + time_difference_s.
/// \param[in] time_difference_s The time difference between the old ECEF system
///                              and the new one.
/// \param[in, out] vector The vector to be transformed.
///
void transformToEcefFrameAtDifferentTime(
    double time_difference_s, Eigen::Vector3d* vector);

class GpsSimulator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr double kSpeedOfLightMPerS = 299792458.0;
  /// GPS L1 carrier frequency [Hz].
  static constexpr double kCarrierFrequencyL1 = 1575.42e6;
  static constexpr uint32_t kSecondsPerDay = 86400u;
  static constexpr uint32_t kSecondsPerWeek = 604800u;
  static constexpr double kEarthGravitationalConstant =
      3.986005e14;  // mu [m^3/s^2]
  static constexpr double kRelativisticCorrection =
      -4.442807633e-10;  // F [s/sqrt(m)]
  static constexpr double kEarthRotationRate = 7.2921151467e-5;  // [rad/sec]
  static constexpr double kGravitationalAcceleration = 9.80665;  // [m/s^2]

  struct GpsSimulationSettings {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d ned_origin_vector_in_llh;

    /// The minimum elevation angle in order for the satellite to be visible.
    double min_elevation_for_visibility_rad;

    // We use the Klobuchar ionospheric model with the data given in the
    // RINEX file. However this only removes around 50% of the error.
    // The rest we model as a constant bias (over the simulation).
    double residual_zenith_ionospheric_delay_mean_s;
    double residual_zenith_ionospheric_delay_sigma_s;

    double receiver_clock_error_mean_s;
    double receiver_clock_error_sigma_s;

    double residual_satellite_clock_error_mean_s;
    double residual_satellite_clock_error_sigma_s;

    double receiver_pseudorange_noise_mean_m;
    double receiver_pseudorange_noise_sigma_m;
    double receiver_carrierphase_noise_mean_m;
    double receiver_carrierphase_noise_sigma_m;

    std::string rinex_navigation_file_path;

    size_t satellite_position_calculation_iterations;
    double satellite_position_calculation_threshold;

    bool simulate_troposphere;
    bool simulate_ionosphere;

    GpsSimulationSettings()
        : ned_origin_vector_in_llh(
              47.37669, 8.54866,
              454.3),  // Position of ETH.
          min_elevation_for_visibility_rad(10.0 / 180.0 * M_PI),
          residual_zenith_ionospheric_delay_mean_s(6.0 / kSpeedOfLightMPerS),
          residual_zenith_ionospheric_delay_sigma_s(4.0 / kSpeedOfLightMPerS),
          receiver_clock_error_mean_s(0.0),
          receiver_clock_error_sigma_s(10.0),
          residual_satellite_clock_error_mean_s(0.0),
          residual_satellite_clock_error_sigma_s(1.0 / kSpeedOfLightMPerS),
          receiver_pseudorange_noise_mean_m(0.0),
          receiver_pseudorange_noise_sigma_m(0.5),
          receiver_carrierphase_noise_mean_m(0.0),
          receiver_carrierphase_noise_sigma_m(5e-3),
          rinex_navigation_file_path(
              ros::package::getPath("simulation") + "/data/polv326w.15n"),
          satellite_position_calculation_iterations(10u),
          satellite_position_calculation_threshold(0.1),
          simulate_troposphere(true),
          simulate_ionosphere(true) {}
  };

  GpsSimulator() {
    initialiseSimulationSettings();
  }

  explicit GpsSimulator(GpsSimulationSettings settings) {
    setGpsSimulationSettings(settings);
  }

  virtual ~GpsSimulator() {}

  GpsSimulationSettings getSimulationSettings() const {
    return settings_;
  }

  NavigationData getNavigationData() const {
    return navigation_data_;
  }

  double getReceiverClockError() const {
    return receiver_clock_error_s_;
  }

  void setGpsSimulationSettings(const GpsSimulationSettings& settings) {
    settings_ = settings;
    initialiseSimulationSettings();
  }

  void setNavigationData(const NavigationData& navigation_data) {
    navigation_data_ = navigation_data;
  }

  ///
  /// \brief Simulate GPS measurements.
  /// \param[in] timestamps_seconds The timestamps for the given poses. Time
  ///                               starts counting from the ephemeris epoch
  ///                               time given in the RINEX file specified in
  ///                               the simulation settings.
  /// \param[in] T_G_Bs T_G_Bs Groundtruth poses. Position in a north east down
  ///                          coordinate system. The LLH position of the origin
  ///                          can be specified in the settings.
  /// \param[out] measurements Returns the simulated GPS measurements.
  /// \param[out] navigation_data Returns the navigation data for the simulated
  ///             measurements. (Optional)
  ///
  void simulateMeasurements(
      const Eigen::VectorXd& timestamps_seconds,
      const aslam::TransformationVector& T_G_Bs,
      Aligned<std::vector, GpsMeasurement>* measurements,
      NavigationData* navigation_data) const;

 private:
  ///
  /// \brief Initialise the settings. This includes for example parsing the
  ///        given RINEX file and generating constant (over a simulation)
  ///        random values such as receiver clock error.
  ///
  void initialiseSimulationSettings();

  GpsSimulationSettings settings_;
  NGSrinex::RinexNavFile rinex_navigation_file_;
  NavigationData navigation_data_;
  double residual_zenith_ionospheric_delay_s_;
  double receiver_clock_error_s_;
  std::map<uint8_t, double> residual_satellite_clock_errors_s;
  mutable std::default_random_engine rng_;
};
}  // namespace gps_simulator
}  // namespace simulation
#endif  // SIMULATION_GPS_SIMULATOR_H_
