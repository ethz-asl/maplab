#ifndef SENSORS_MEASUREMENT_H_
#define SENSORS_MEASUREMENT_H_

#include <utility>

#include <Eigen/Core>
#include <aslam/common/time.h>
#include <maplab-common/temporal-buffer.h>

#include "sensors/measurements.pb.h"
#include "sensors/sensor.h"

#define DEFINE_MEASUREMENT_HASH(measurement)                         \
  namespace std {                                                    \
  template <>                                                        \
  struct hash<vi_map::measurement> {                                 \
    std::size_t operator()(const vi_map::measurement& value) const { \
      return std::hash<vi_map::Measurement>()(value);                \
    }                                                                \
  };                                                                 \
  }  // namespace std

namespace vi_map {

#define DEFINE_MEAUREMENT_CONTAINERS(measurement)              \
  typedef AlignedUnorderedSet<measurement> measurement##Set;   \
  typedef Aligned<std::vector, measurement> measurement##List; \
  typedef MeasurementBuffer<measurement> measurement##Buffer;

template <class DerivedMeasurement>
SensorType measurementToSensorType();

class Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Measurement() : timestamp_nanoseconds_(0) {}
  Measurement(const SensorId& sensor_id, const int64_t timestamp_nanoseconds)
      : sensor_id_(sensor_id), timestamp_nanoseconds_(timestamp_nanoseconds) {
    CHECK(sensor_id_.isValid());
    CHECK_GE(timestamp_nanoseconds, 0);
  }
  virtual ~Measurement() = default;

  inline bool operator==(const Measurement& other) const {
    return sensor_id_ == other.sensor_id_ &&
           timestamp_nanoseconds_ == other.timestamp_nanoseconds_;
  }

  inline int64_t getTimestampNanoseconds() const {
    return timestamp_nanoseconds_;
  }

  inline void setTimestampNanoseconds(const int64_t timestamp_nanoseconds) {
    timestamp_nanoseconds_ = timestamp_nanoseconds;
  }

  inline const SensorId& getSensorId() const {
    return sensor_id_;
  }

  void serialize(measurements::proto::Measurement* proto_measurement) const;
  void deserialize(const measurements::proto::Measurement& proto_measurement);

  bool isValid() const {
    if (!sensor_id_.isValid()) {
      LOG(ERROR) << "Invalid sensor id.";
      return false;
    }
    if (timestamp_nanoseconds_ < 0) {
      LOG(ERROR) << "Negative timestamp: " << timestamp_nanoseconds_;
      return false;
    }
    return isValidImpl();
  }

  void setRandom() {
    CHECK(sensor_id_.isValid());
    std::random_device random_device;
    std::default_random_engine random_engine(random_device());
    constexpr int64_t kMinTimestampNanoseconds = 0;
    constexpr int64_t kMaxTimestampNanoseconds = static_cast<int64_t>(1e14);
    std::uniform_int_distribution<int64_t> timestamp_distribution(
        kMinTimestampNanoseconds, kMaxTimestampNanoseconds);
    timestamp_nanoseconds_ = timestamp_distribution(random_engine);
    setRandomImpl();
  }

 protected:
  explicit Measurement(const SensorId& sensor_id)
      : sensor_id_(sensor_id), timestamp_nanoseconds_(0) {
    CHECK(sensor_id_.isValid());
  }

 private:
  virtual bool isValidImpl() const = 0;
  virtual void setRandomImpl() = 0;

  SensorId sensor_id_;
  int64_t timestamp_nanoseconds_;
};
template <class MeasurementType>
using MeasurementBuffer = common::TemporalBuffer<
    MeasurementType,
    Eigen::aligned_allocator<std::pair<const int64_t, MeasurementType>>>;

}  // namespace vi_map

namespace std {
template <>
struct hash<vi_map::Measurement> {
  std::size_t operator()(const vi_map::Measurement& value) const {
    std::size_t h0(std::hash<vi_map::SensorId>()(value.getSensorId()));
    std::size_t h1(std::hash<int64_t>()(value.getTimestampNanoseconds()));
    return h0 ^ h1;
  }
};
}  // namespace std

#endif  // SENSORS_MEASUREMENT_H_
