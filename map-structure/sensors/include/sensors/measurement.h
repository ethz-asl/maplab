#ifndef SENSORS_MEASUREMENT_H_
#define SENSORS_MEASUREMENT_H_

#include <utility>

#include <Eigen/Core>
#include <aslam/common/time.h>
#include <aslam/common/unique-id.h>
#include <maplab-common/threadsafe-temporal-buffer.h>

#include "sensors/sensor-types.h"

#define DEFINE_MEASUREMENT_HASH(measurement)                         \
  namespace std {                                                    \
  template <>                                                        \
  struct hash<vi_map::measurement> {                                 \
    std::size_t operator()(const vi_map::measurement& value) const { \
      return std::hash<vi_map::Measurement>()(value);                \
    }                                                                \
  };                                                                 \
  }  // namespace std

#define DEFINE_MEASUREMENT_CONTAINERS(measurement)              \
  typedef AlignedUnorderedSet<measurement> measurement##Set;   \
  typedef Aligned<std::vector, measurement> measurement##List; \
  typedef MeasurementBuffer<measurement> measurement##Buffer;  \
  typedef ThreadsafeMeasurementBuffer<measurement>             \
      measurement##ThreadsafeBuffer;

namespace vi_map {

class Measurement {
 public:
  Measurement() : timestamp_nanoseconds_(0) {}

  explicit Measurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds)
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

  inline int64_t* getTimestampNanosecondsMutable() {
    return &timestamp_nanoseconds_;
  }

  inline const aslam::SensorId& getSensorId() const noexcept {
    return sensor_id_;
  }

  inline void setSensorId(const aslam::SensorId& sensor_id) {
    sensor_id_ = sensor_id;
  }

  inline bool isValid() const {
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

  inline void setRandom() {
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
  explicit Measurement(const aslam::SensorId& sensor_id)
      : sensor_id_(sensor_id), timestamp_nanoseconds_(0) {
    CHECK(sensor_id_.isValid());
  }

 private:
  virtual bool isValidImpl() const = 0;
  virtual void setRandomImpl() = 0;

  aslam::SensorId sensor_id_;
  int64_t timestamp_nanoseconds_;
};

template <class MeasurementType>
using MeasurementBuffer = common::TemporalBuffer<
    MeasurementType,
    Eigen::aligned_allocator<std::pair<const int64_t, MeasurementType>>>;
template <class MeasurementType>
using ThreadsafeMeasurementBuffer = common::ThreadsafeTemporalBuffer<
    MeasurementType,
    Eigen::aligned_allocator<std::pair<const int64_t, MeasurementType>>>;
}  // namespace vi_map

namespace std {
template <>
struct hash<vi_map::Measurement> {
  std::size_t operator()(const vi_map::Measurement& value) const {
    std::size_t h0(std::hash<aslam::SensorId>()(value.getSensorId()));
    std::size_t h1(std::hash<int64_t>()(value.getTimestampNanoseconds()));
    return h0 ^ h1;
  }
};
}  // namespace std

#endif  // SENSORS_MEASUREMENT_H_
