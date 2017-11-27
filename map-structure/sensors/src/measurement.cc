#include "sensors/measurement.h"

#include <glog/logging.h>

namespace vi_map {

void Measurement::serialize(
    measurements::proto::Measurement* proto_measurement) const {
  CHECK_NOTNULL(proto_measurement);
  CHECK(sensor_id_.isValid());
  sensor_id_.serialize(proto_measurement->mutable_sensor_id());
  CHECK_NOTNULL(proto_measurement);
  proto_measurement->set_timestamp_nanoseconds(timestamp_nanoseconds_);
}

void Measurement::deserialize(
    const measurements::proto::Measurement& proto_measurement) {
  sensor_id_.deserialize(proto_measurement.sensor_id());
  CHECK(sensor_id_.isValid());
  timestamp_nanoseconds_ = proto_measurement.timestamp_nanoseconds();
}
}  // namespace vi_map
