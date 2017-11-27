#ifndef MAPLAB_COMMON_GRAVITY_PROVIDER_H_
#define MAPLAB_COMMON_GRAVITY_PROVIDER_H_

#include <cmath>

#include <glog/logging.h>

namespace common {

namespace locations {
const double kAltitudeZurichMeters = 392;
const double kLatitudeZurichDegrees = 47.22;
}

class GravityProvider {
 public:
  GravityProvider(const double altitude_meters, const double latitude_degrees);

  void setLocation(const double altitude_meters, const double latitude_degrees);

  double getGravityMagnitude() {
    return gravity_magnitude_;
  }

 private:
  double gravity_magnitude_;

  static constexpr double kDegreesToRadians = M_PI / 180.0;
};

}  // namespace common

#endif  // MAPLAB_COMMON_GRAVITY_PROVIDER_H_
