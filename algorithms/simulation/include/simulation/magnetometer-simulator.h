#ifndef SIMULATION_MAGNETOMETER_SIMULATOR_H_
#define SIMULATION_MAGNETOMETER_SIMULATOR_H_

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/memory.h>
#include <aslam/frames/visual-nframe.h>

namespace simulation {

class WorldMagneticModel;
struct MagnetometerData;

struct YearMonthDay {
  size_t year;
  size_t month;
  size_t day;
};

struct MagnetometerNoiseSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d misalignment_matrix;
  Eigen::Vector3d bias_vector;
  double power_spectral_density_noise;
};

class MagnetometerSimulator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Simulate the magnetometer without noise.
  /// @param[in]  utc_date  Struct containing year, month and day in utc time.
  explicit MagnetometerSimulator(const YearMonthDay& utc_date);

  /// \brief Simulate the magnetometer with specific noise values.
  /// @param[in]  utc_date        Struct containing year, month and day in utc
  /// time.
  /// @param[in]  noise_settings  Struct containing misalignment matrix, bias
  /// vector and noise.
  MagnetometerSimulator(
      const YearMonthDay& utc_date,
      const MagnetometerNoiseSettings& noise_settings);

  /// \brief Simulate magnetic field from global transformation vector.
  /// @param[in]  timestamp_nanoseconds  Timestamps in nanoseconds.
  /// @param[in]  T_G_Bs                 Transformation vector in global
  /// coordinates.
  ///                                    The position is in geodetic latitude,
  ///                                    longitude and height.
  /// @param[out] magnetic_fields_tesla  Vector containing rotated magnetic
  /// fields of the trajectory
  ///                                    in tesla.
  void simulateMeasurements(
      const Eigen::VectorXd& timestamps_nanoseconds,
      const aslam::TransformationVector& T_G_Bs,
      Aligned<std::vector, Eigen::Vector3d>* magnetic_fields_tesla);

  /// \brief Calculate magnetic field from geodetic coordinates and global
  /// rotation.
  /// @param[in]  llh                   Geodetic latitude in degree, geodetic
  /// longitude in degree,
  ///                                   height above
  ///                                   WGS-84 ellipsoid in meter.
  /// @param[in]  R_B_G                 Rotation from global to body coordinate
  /// frame.
  /// @param[out] magnetic_field_tesla  Vector containing the rotated magnetic
  /// field in tesla.
  void calculateMagneticField(
      const Eigen::Vector3d& llh, const Eigen::Matrix3d& R_B_G,
      Eigen::Vector3d* magnetic_field_tesla);

 private:
  std::unique_ptr<WorldMagneticModel> world_magnetic_model_;

  bool add_noise_;

  MagnetometerNoiseSettings noise_settings_;
};

/// \brief  Wrapper class for the official world magnetic model code.
///         See http://www.ngdc.noaa.gov/geomag/WMM.
class WorldMagneticModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// brief Constructor that sets the utc date and loads the magnetic field
  /// parameters.
  /// @param[in]  utc_date  Struct containing year, month and day in utc time.
  explicit WorldMagneticModel(const YearMonthDay& utc_date);

  ~WorldMagneticModel();

  /// \brief Calculate earth magnetic field from geodetic coordinates.
  /// @param[in]   llh                   Geodetic latitude in degree, geodetic
  /// longitude in degree,
  ///                                    height above WGS-84 ellipsoid in meter.
  /// @param[out]  magnetic_field_tesla  The earth magnetic field in tesla.
  void calculateMagneticField(
      const Eigen::Vector3d& llh, Eigen::Vector3d* magnetic_field_tesla);

 private:
  /// \brief Load the official parameters for the earth magnetic field.
  void loadWorldMagneticModel();

  std::unique_ptr<MagnetometerData> data_;

  YearMonthDay utc_date_;

  const int kNumberTerms = 91;
  /// Maximum degree of spherical harmonic model.
  const int kMaximumDegreeModel = 12;
  const double kNanoTeslaToTesla = 1.0e-9;
};

}  // namespace simulation

#endif  // SIMULATION_MAGNETOMETER_SIMULATOR_H_
