#ifndef VI_MAP_OPTIONAL_SENSOR_DATA_H_
#define VI_MAP_OPTIONAL_SENSOR_DATA_H_

#include <mutex>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <maplab-common/macros.h>
#include <maplab-common/temporal-buffer.h>
#include <sensors/gps-utm.h>
#include <sensors/gps-wgs.h>
#include <sensors/measurement.h>
#include <sensors/sensor.h>

#include "vi-map/unique-id.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

class OptionalSensorData final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OptionalSensorData() = default;
  explicit OptionalSensorData(const OptionalSensorData& other);
  explicit OptionalSensorData(
      const proto::OptionalSensorData& proto_optional_sensor_data);
  ~OptionalSensorData() = default;

  template<class MeasurementType>
  const MeasurementBuffer<MeasurementType>& getMeasurements(
      const SensorId& sensor_id) const;

  template <class MeasurementType>
  bool hasMeasurements(const SensorId& sensor_id) const;

  template <class MeasurementType>
  void addMeasurement(const MeasurementType& measurement);

  void getAllSensorIds(SensorIdSet* sensor_ids) const;

  template<class MeasurementType>
  void clear(const SensorId& sensor_id) {
    getMeasurementsMutable<MeasurementType>(sensor_id).clear();
  }

  void lock() const {
    mutex_.lock();
  }

  void unlock() const {
    mutex_.unlock();
  }

  void serialize(proto::OptionalSensorData* proto_optional_sensor_data) const;
  void deserialize(const proto::OptionalSensorData& proto_optional_sensor_data);

  bool operator==(const OptionalSensorData& other) const {
    return sensor_id_to_gps_utm_measurements_ ==
        other.sensor_id_to_gps_utm_measurements_ &&
        sensor_id_to_gps_wgs_measurements_ ==
            other.sensor_id_to_gps_wgs_measurements_;
  }

 private:
  template <class MeasurementType>
  MeasurementBuffer<MeasurementType>& getMeasurementsMutable(
      const SensorId& sensor_id);

  template<class MeasurementType>
  using SensorIdToMeasurementsMap =
      AlignedUnorderedMap<SensorId, MeasurementBuffer<MeasurementType>>;
  SensorIdToMeasurementsMap<GpsUtmMeasurement>
      sensor_id_to_gps_utm_measurements_;
  SensorIdToMeasurementsMap<GpsWgsMeasurement>
      sensor_id_to_gps_wgs_measurements_;

  mutable std::recursive_mutex mutex_;
};

typedef AlignedUnorderedMap<MissionId, OptionalSensorData>
    OptionalSensorDataMap;

}  // namespace vi_map

#endif  // VI_MAP_OPTIONAL_SENSOR_DATA_H_
