#include <maplab-common/gravity-provider.h>

namespace common {

GravityProvider::GravityProvider(
    const double altitude_meters, const double latitude_degrees) {
  setLocation(altitude_meters, latitude_degrees);
}

void GravityProvider::setLocation(
    const double altitude_meters, const double latitude_degrees) {
  CHECK_GE(latitude_degrees, -90.0);
  CHECK_LE(latitude_degrees, 90.0);
  const double phi = latitude_degrees * kDegreesToRadians;

  const double sin_phi_squared = sin(phi) * sin(phi);
  const double sin_two_phi_squared = sin(2 * phi) * sin(2 * phi);

  gravity_magnitude_ = 9.780327 * (1 + 0.0053024 * sin_phi_squared -
                                   0.0000058 * sin_two_phi_squared) -
                       3.155 * 1e-7 * altitude_meters;
}

}  // namespace common
