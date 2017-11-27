#include "maplab-common/global-coordinate-tools.h"

#include <fstream>  // NOLINT

namespace common {

void llhToEcef(const Eigen::Vector3d& latitude_longitude_height,
               Eigen::Vector3d* earth_centered_earth_fixed) {
  CHECK_NOTNULL(earth_centered_earth_fixed);
  // Conversion from degree to rad.
  double latitude_rad = degreeToRadian(latitude_longitude_height(0));
  double longitude_rad = degreeToRadian(latitude_longitude_height(1));
  double height_meter = latitude_longitude_height(2);

  const double d = kWGS84Eccentricity * sin(latitude_rad);
  // N is the radius of curvature.
  const double N = kWGS84A / sqrt(1.0 - d * d);
  *earth_centered_earth_fixed
      << (N + height_meter) * cos(latitude_rad) * cos(longitude_rad),
      (N + height_meter) * cos(latitude_rad) * sin(longitude_rad),
      (N * (1.0 - kWGS84Eccentricity * kWGS84Eccentricity) + height_meter) *
          sin(latitude_rad);
}

void ecefToLlh(const Eigen::Vector3d& earth_centered_earth_fixed,
               Eigen::Vector3d* latitude_longitude_height) {
  CHECK_NOTNULL(latitude_longitude_height);
  const double x = earth_centered_earth_fixed(0);
  const double y = earth_centered_earth_fixed(1);
  const double z = earth_centered_earth_fixed(2);

  // Note: The variable names stick to the referenced notation.
  static const double kWGS84ASquared = kWGS84A * kWGS84A;
  static const double kWGS84BSquared = kWGS84B * kWGS84B;
  static const double kWGS84EccentricitySquared =
      kWGS84Eccentricity * kWGS84Eccentricity;
  static const double kWGS84EccentricityCubed =
      kWGS84EccentricitySquared * kWGS84Eccentricity;
  static const double kWGS84EccentricityFourth =
      kWGS84EccentricityCubed * kWGS84Eccentricity;
  static const double E = kWGS84ASquared - kWGS84BSquared;

  const double p = std::sqrt(x * x + y * y);
  const double z_squared = z * z;
  const double p_squared = p * p;
  const double F = 54.0 * kWGS84BSquared * z_squared;
  const double G = p_squared + (1.0 - kWGS84EccentricitySquared) * z_squared -
                   kWGS84EccentricitySquared * E;
  const double G_squared = G * G;
  const double c = kWGS84EccentricityFourth * F * p_squared / (G_squared * G);
  const double s = std::exp(
      std::log(std::fabs(1.0 + c + std::sqrt(c * c + 2.0 * c))) * 1.0 / 3.0);
  const double P =
      F / (3.0 * (s + 1.0 / s + 1.0) * (s + 1.0 / s + 1.0) * G_squared);
  const double Q = std::sqrt(1.0 + 2.0 * kWGS84EccentricityFourth * P);
  const double r_0 =
      -(P * kWGS84EccentricitySquared * p) / (1.0 + Q) +
      std::sqrt(
          0.5 * kWGS84ASquared * (1.0 + 1.0 / Q) -
          P * (1.0 - kWGS84EccentricitySquared) * z_squared / (Q * (1.0 + Q)) -
          0.5 * P * p_squared);
  const double U = std::sqrt(
      (p - kWGS84EccentricitySquared * r_0) *
          (p - kWGS84EccentricitySquared * r_0) +
      z_squared);
  const double V = std::sqrt(
      (p - kWGS84EccentricitySquared * r_0) *
          (p - kWGS84EccentricitySquared * r_0) +
      (1.0 - kWGS84EccentricitySquared) * z_squared);
  const double z_0 = kWGS84BSquared * z / (kWGS84A * V);
  const double e_prime = kWGS84Eccentricity * kWGS84A / kWGS84B;

  // Return latitude in decimal degree, longitude in decimal degree, height in
  // meter.
  *latitude_longitude_height
      << radianToDegree(std::atan2(z + e_prime * e_prime * z_0, p)),
      radianToDegree(std::atan2(y, x)),
      -U * (kWGS84BSquared / (kWGS84A * V) - 1.0);
}

void ecefToLlhIterative(const Eigen::Vector3d& earth_centered_earth_fixed,
                        Eigen::Vector3d* latitude_longitude_height) {
  CHECK_NOTNULL(latitude_longitude_height);
  // Simplify input.
  double x = earth_centered_earth_fixed(0);
  double y = earth_centered_earth_fixed(1);
  double z = earth_centered_earth_fixed(2);

  // Calculate longitude in radian.
  double longitude_rad = std::atan2(y, x);

  // Initialize the variables to caculate latitude and height.
  double height_meter = 0.0;
  double N = kWGS84A;

  double rho = sqrt(x * x + y * y);
  double latitude_rad = 0.0;
  double previous_latitude_rad = 90.0;

  // Iterate until tolerance is reached.
  double kPrecision = 1.0e-12;
  while (std::abs(latitude_rad - previous_latitude_rad) >= kPrecision) {
    previous_latitude_rad = latitude_rad;
    double sin_latitude =
        z /
        (N * (1.0 - kWGS84Eccentricity * kWGS84Eccentricity) + height_meter);
    latitude_rad = atan(
        (z + kWGS84Eccentricity * kWGS84Eccentricity * N * sin_latitude) / rho);
    N = kWGS84A / sqrt(
                      1.0 -
                      kWGS84Eccentricity * sin_latitude * kWGS84Eccentricity *
                          sin_latitude);
    height_meter = rho / cos(latitude_rad) - N;
  }

  *latitude_longitude_height << radianToDegree(latitude_rad),
      radianToDegree(longitude_rad), height_meter;
}

void ecefToNed(const Eigen::Vector3d& earth_centered_earth_fixed,
               const Eigen::Vector3d& origin_earth_centered_earth_fixed,
               Eigen::Vector3d* north_east_down) {
  CHECK_NOTNULL(north_east_down);
  Eigen::Vector3d origin_latitude_longitude_height;
  ecefToLlh(
      origin_earth_centered_earth_fixed, &origin_latitude_longitude_height);

  // Conversion from degree to radian.
  double origin_latitude_rad =
      degreeToRadian(origin_latitude_longitude_height(0));
  double origin_longitude_rad =
      degreeToRadian(origin_latitude_longitude_height(1));

  // Build rotation matrix to local tangent plane.
  Eigen::Matrix3d R_NED_ECEF;
  getRotationMatrixEcefToNed(
      origin_latitude_rad, origin_longitude_rad, &R_NED_ECEF);
  *north_east_down = R_NED_ECEF * (earth_centered_earth_fixed -
                                   origin_earth_centered_earth_fixed);
}

void nedToEcef(const Eigen::Vector3d& north_east_down,
               const Eigen::Vector3d& origin_earth_centered_earth_fixed,
               Eigen::Vector3d* earth_centered_earth_fixed) {
  CHECK_NOTNULL(earth_centered_earth_fixed);
  Eigen::Vector3d origin_latitude_longitude_height;
  ecefToLlh(
      origin_earth_centered_earth_fixed, &origin_latitude_longitude_height);

  // Conversion from degree to radian.
  double origin_latitude_rad =
      degreeToRadian(origin_latitude_longitude_height(0));
  double origin_longitude_rad =
      degreeToRadian(origin_latitude_longitude_height(1));

  // Build rotation matrix from local tangent plane.
  Eigen::Matrix3d R_ECEF_NED;
  getRotationMatrixNedToEcef(
      origin_latitude_rad, origin_longitude_rad, &R_ECEF_NED);
  *earth_centered_earth_fixed =
      R_ECEF_NED * north_east_down + origin_earth_centered_earth_fixed;
}

void getRotationMatrixEcefToNed(double lat_rad, double lon_rad,
                                Eigen::Matrix3d* R_NED_ECEF) {
  CHECK_NOTNULL(R_NED_ECEF);
  *R_NED_ECEF << -sin(lat_rad) * cos(lon_rad), -sin(lat_rad) * sin(lon_rad),
      cos(lat_rad), -sin(lon_rad), cos(lon_rad), 0.0,
      -cos(lat_rad) * cos(lon_rad), -cos(lat_rad) * sin(lon_rad), -sin(lat_rad);
}

void getRotationMatrixNedToEcef(double lat_rad, double lon_rad,
                                Eigen::Matrix3d* R_ECEF_NED) {
  CHECK_NOTNULL(R_ECEF_NED);
  *R_ECEF_NED << -sin(lat_rad) * cos(lon_rad), -sin(lon_rad),
      -cos(lat_rad) * cos(lon_rad), -sin(lat_rad) * sin(lon_rad), cos(lon_rad),
      -cos(lat_rad) * sin(lon_rad), cos(lat_rad), 0.0, -sin(lat_rad);
}

void writeGlobalCoordinatesToKml(
    const Aligned<std::vector, Eigen::Vector3d>& latitude_longitude_height,
    const std::string& filename) {
  std::ofstream logger(filename);

  logger << "<?xml version='1.0' encoding='UTF-8'?>\n";
  logger << "<kml xmlns='http://earth.google.com/kml/2.1'>\n";
  logger << "<Document>\n";
  logger << "   <name>Path</name>\n";
  logger << "   <Placemark>\n";
  logger << "       <Snippet maxLines='0'> </Snippet>\n";
  logger << "       <description> </description>\n";
  logger << "       <name>Line 1</name>\n";
  logger << "       <Style>\n";
  logger << "          <IconStyle>\n";
  logger << "             <color>ff00ffff</color>\n";
  logger << "          </IconStyle>\n";
  logger << "          <LineStyle>\n";
  logger << "             <color>ff00ffff</color>\n";
  logger << "             <width>3</width>\n";
  logger << "          </LineStyle>\n";
  logger << "       </Style>\n";
  logger << "       <LineString>\n";
  logger << "          <altitudeMode>absolute</altitudeMode>\n";
  logger << "          <coordinates>";
  for (size_t i = 0u; i < latitude_longitude_height.size(); ++i) {
    // KML is in longitude, latitude, height.
    logger << std::to_string(latitude_longitude_height[i](1)) + "," +
                  std::to_string(latitude_longitude_height[i](0)) + "," +
                  std::to_string(latitude_longitude_height[i](2)) + " ";
  }
  logger << "          </coordinates>\n";
  logger << "       </LineString>\n";
  logger << "   </Placemark>\n";
  logger << "</Document>\n";
  logger << "</kml>\n";
}

constexpr double degreeToRadian(double degree) { return degree * M_PI / 180.0; }

constexpr double radianToDegree(double radian) { return radian * 180.0 / M_PI; }
}  // namespace common
