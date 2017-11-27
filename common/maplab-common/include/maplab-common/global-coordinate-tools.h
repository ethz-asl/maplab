#ifndef MAPLAB_COMMON_GLOBAL_COORDINATE_TOOLS_H_
#define MAPLAB_COMMON_GLOBAL_COORDINATE_TOOLS_H_

#include <string>

#include <Eigen/Core>
#include <glog/logging.h>

#include <aslam/common/memory.h>

namespace common {

  /// \mu: WGS 84 value of the earth's gravitational constant for GPS user
  /// [m^3 / sec^2].
constexpr double kWGS84EarthGravitationalConstant = 3.986005e14;
  /// \dot{\Omega}_e: WGS 84 value of the earth's rotation rate.
constexpr double kWGS84EarthRotationRate = 7.2921151467e-5;
// Parameters of the WGS84 ellipsoid:
/// Semi-major axis of the earth in meters.
constexpr double kWGS84A = 6378137.0;

/// Inverse flattening of the earth.
constexpr double kWGS84InverseFlattening = 298.257223563;

/// The flattening of the earth.
constexpr double kWGS84Flattening = 1.0 / kWGS84InverseFlattening;

/// Semi-minor axis of the earth in meters.
constexpr double kWGS84B = kWGS84A * (1.0 - kWGS84Flattening);

/// Eccentricity of the earth.
const double kWGS84Eccentricity =
    sqrt(2.0 * kWGS84Flattening - kWGS84Flattening * kWGS84Flattening);

  /// \brief  Converts from WGS84 geodetic coordinates (latitude, longitude and
  /// height)
  ///         into WGS84 earth centered, earth fixed cartesian (ECEF)
  ///         coordinates (x, y, z).
  /// @param[in]  latitude_longitude_height   Geodetic coordinates to be
  /// converted,
  ///                                         passed as (latitude, longitude,
  ///                                         height)
  ///                                         in [decimal degree, decimal
  ///                                         degree, meter].
  /// @param[out] earth_centered_earth_fixed  Converted cartesian coordinates
  /// (x, y, z) in meter.
void llhToEcef(const Eigen::Vector3d& latitude_longitude_height,
               Eigen::Vector3d* earth_centered_earth_fixed);

  /// \brief  Converts from WGS84 earth centered, earth fixed cartesian (ECEF)
  /// coordinates (x, y, z)
  ///         into WGS84 geodetic coordinates (latitude, longitude and height).
  ///         Analytically closed
  ///         approach. Reference: "Integrierte Navigationssysteme:
  ///         Sensordatenfusion, GPS und
  ///         Inertiale Navigation", Jan Wendel, 2007, Oldenbourg.
  /// @param[in]   earth_centered_earth_fixed  Cartesian coordinates (x, y, z)
  /// in meter to
  ///                                          be converted.
  /// @param[out]  latitude_longitude_height   Converted geodetic coordinates,
  ///                                          passed as (latitude, longitude,
  ///                                          height)
  ///                                          in [decimal degree, decimal
  ///                                          degree, meter].
void ecefToLlh(const Eigen::Vector3d& earth_centered_fixed,
               Eigen::Vector3d* latitude_longitude_height);

  /// \brief  Converts from WGS84 earth centered, earth fixed cartesian (ECEF)
  /// coordinates (x, y, z)
  ///         into WGS84 geodetic coordinates (latitude, longitude and height).
  ///         Iterative approach.
  /// @param[in]   earth_centered_earth_fixed  Cartesian coordinates (x, y, z)
  /// in meter to
  ///                                          be converted.
  /// @param[out]  latitude_longitude_height   Converted geodetic coordinates,
  ///                                          passed as (latitude, longitude,
  ///                                          height)
  ///                                          in [decimal degree, decimal
  ///                                          degree, meter].
void ecefToLlhIterative(const Eigen::Vector3d& earth_centered_earth_fixed,
                        Eigen::Vector3d* latitude_longitude_height);

  /// \brief  Converts from WGS84 earth centered, earth fixed cartesian (ECEF)
  /// coordinates (x, y, z)
  ///         into local tangent plane where the origin is the origin in ECEF
  ///         coordinates.
  /// @param[in]   earth_centered_earth_fixed         Cartesian coordinates (x,
  /// y, z) in meter to
  ///                                                 be converted.
  /// @param[in]   origin_earth_centered_earth_fixed  Origin in cartesian
  /// coordinates (x0, y0, z0)
  ///                                                 in meter.
  /// @param[out]  north_east_down                    Converted coordinates in
  /// (north, east, down).
void ecefToNed(const Eigen::Vector3d& earth_centered_earth_fixed,
               const Eigen::Vector3d& origin_earth_centered_earth_fixed,
               Eigen::Vector3d* north_east_down);

  /// \brief  Converts north-east-down local tangent plane coordinates into
  /// earth centered, earth
  ///         fixed coordinates where the origin is the earth centered, earth
  ///         fixed point of
  ///         tangency.
  /// @param[in]   north_east_down                    North, east, down
  /// coordiantes in meter to
  ///                                                 be converted.
  /// @param[in]   origin_earth_centered_earth_fixed  Origin in earth centered,
  /// earth fixed
  ///                                                 cartesian coordinates (x0,
  ///                                                 y0, z0) in meter.
  /// @param[out]  earth_centered_earth_fixed         Converted earth centered,
  /// earth fixed
  ///                                                 cartesian coordinates in
  ///                                                 meter.
void nedToEcef(const Eigen::Vector3d& north_east_down,
               const Eigen::Vector3d& origin_earth_centered_earth_fixed,
               Eigen::Vector3d* earth_centered_earth_fixed);

  /// \brief  Calculate rotation matrix from north-east-down coordinates to
  /// earth centered, earth
  ///         fixed coordiantes. Reference: E.g. European Space Agency
  ///         http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates
  /// @param[in]   lat_rad     Latitude in radian.
  /// @param[in]   lon_rad     Longitude in radian.
  /// @param[out]  R_NED_ECEF  Rotation matrix from ECEF to NED coordinates.
void getRotationMatrixEcefToNed(double lat_rad, double lon_rad,
                                Eigen::Matrix3d* R_NED_ECEF);

  /// \brief  Calculate rotation matrix from earth centered, earth fixed
  /// coordiantes to
  ///         north-east-down coordinates. Reference: E.g. European Space Agency
  ///         http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates
  /// @param[in]   lat_rad     Latitude in radian.
  /// @param[in]   lon_rad     Longitude in radian.
  /// @param[out]  R_NED_ECEF  Rotation matrix from NED to ECEF coordinates.
void getRotationMatrixNedToEcef(double lat_rad, double lon_rad,
                                Eigen::Matrix3d* R_ECEF_NED);

  /// \brief  Write a path in geodetic coordinates to a KML file, e.g. for
  /// visualizing in
  ///         google earth.
  /// @param[in]  latitude_longitude_height  A vector representing a path in
  /// geodetic coordinates,
  ///                                        with latitude in decimal degree,
  ///                                        longitude in decimal
  ///                                        degree and height above WGS84
  ///                                        ellipsoid in meter.
  /// @param[in]  filename                   Name of the KML-file.
void writeGlobalCoordinatesToKml(
    const Aligned<std::vector, Eigen::Vector3d>& latitude_longitude_height,
    const std::string& filename);

/// \brief  Simple scalar conversion from degree to radian.
/// @param[in]      degree  Value in degree to be converted.
/// @param[return]          Converted value in radian.
constexpr double degreeToRadian(double degree);

/// \brief  Simple scalar conversion from radian to degree.
/// @param[in]      radian  Value in radian to be converted.
/// @param[return]          Converted value in degree.
constexpr double radianToDegree(double radian);
}  // namespace common

#endif  // MAPLAB_COMMON_GLOBAL_COORDINATE_TOOLS_H_
