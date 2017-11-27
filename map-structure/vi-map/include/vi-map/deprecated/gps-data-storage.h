#ifndef VI_MAP_DEPRECATED_GPS_DATA_STORAGE_H_
#define VI_MAP_DEPRECATED_GPS_DATA_STORAGE_H_

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <glog/logging.h>
#include <maplab-common/temporal-buffer.h>
#include <sensors/sensor.h>

#include "vi-map/unique-id.h"
#include "vi-map/vi_map.pb.h"
#include "vi-map/vi_map_deprecated.pb.h"

namespace vi_map {

struct GPSMeasurementBase {
  GPSMeasurementBase() : timestamp(-1) {
    sensor_id.setInvalid();
  }
  GPSMeasurementBase(
      const int64_t timestamp_nanoseconds_, const SensorId& sensor_id_)
      : timestamp(timestamp_nanoseconds_), sensor_id(sensor_id_) {
    CHECK(sensor_id.isValid());
  }
  virtual ~GPSMeasurementBase() = default;
  bool operator==(const GPSMeasurementBase& other) const {
    return timestamp == other.timestamp && sensor_id == other.sensor_id;
  }
  // Timestamp in nanoseconds.
  int64_t timestamp;
  SensorId sensor_id;
};

// GPS measurement following the WGS (World Geodetic System) 84 standard
// coordinate system.
struct GPSMeasurementWGS : public GPSMeasurementBase {
  GPSMeasurementWGS()
      : latitude_deg(0.0), longitude_deg(0.0), altitude_meters(0.0) {}
  GPSMeasurementWGS(
      const int64_t timestamp_nanoseconds_, const SensorId& sensor_id_,
      const double latitude_deg_, const double longitude_deg_,
      const double altitude_meters_)
      : GPSMeasurementBase(timestamp_nanoseconds_, sensor_id_),
        latitude_deg(latitude_deg_),
        longitude_deg(longitude_deg_),
        altitude_meters(altitude_meters_) {}
  virtual ~GPSMeasurementWGS() = default;
  bool operator==(const GPSMeasurementWGS& other) const {
    return GPSMeasurementBase::operator==(other) &&
           latitude_deg == other.latitude_deg &&
           longitude_deg == other.longitude_deg &&
           altitude_meters == other.altitude_meters;
  }
  double latitude_deg;
  double longitude_deg;
  double altitude_meters;
};
typedef std::unordered_set<GPSMeasurementWGS> GPSMeasurementWGSSet;
typedef std::vector<GPSMeasurementWGS> GPSMeasurementWGSList;

// GPS measurement following the  UTM (Universal Transverse Mercator)
// standard coordinate system.
struct GPSMeasurementUTM : public GPSMeasurementBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GPSMeasurementUTM() {
    T_R_S.setIdentity();
  }
  GPSMeasurementUTM(
      const int64_t timestamp_nanoseconds_, const SensorId& sensor_id_,
      const aslam::Transformation& T_R_S_)
      : GPSMeasurementBase(timestamp_nanoseconds_, sensor_id_), T_R_S(T_R_S_) {}
  virtual ~GPSMeasurementUTM() = default;

  bool operator==(const GPSMeasurementUTM& other) const {
    return GPSMeasurementBase::operator==(other) && T_R_S == other.T_R_S;
  }
  // Coordinate frames used:
  // R: Absolute pose measurement frame of reference.
  // S: Absolute pose sensor frame.
  aslam::Transformation T_R_S;
};
typedef AlignedUnorderedSet<GPSMeasurementUTM> GPSMeasurementUTMSet;
typedef Aligned<std::vector, GPSMeasurementUTM> GPSMeasurementUTMList;

template <typename GpsMeasurement>
struct GPSMeasurementMissionPair {
  GPSMeasurementMissionPair() {
    mission_id.setInvalid();
  }
  GPSMeasurementMissionPair(
      const GpsMeasurement& gps_measurement_, const MissionId& mission_id_)
      : gps_measurement(gps_measurement_), mission_id(mission_id_) {
    CHECK(mission_id_.isValid());
  }
  virtual ~GPSMeasurementMissionPair() = default;

  bool operator==(const GPSMeasurementMissionPair& other) const {
    return gps_measurement == other.gps_measurement &&
           mission_id == other.mission_id;
  }

  GpsMeasurement gps_measurement;
  MissionId mission_id;
};
typedef GPSMeasurementMissionPair<GPSMeasurementUTM>
    GPSMeasurementUTMMissionPair;
typedef GPSMeasurementMissionPair<GPSMeasurementWGS>
    GPSMeasurementWGSMissionPair;
typedef AlignedUnorderedSet<GPSMeasurementUTMMissionPair>
    GPSMeasurementUTMMissionPairSet;

}  // namespace vi_map

namespace std {
template <>
struct hash<vi_map::GPSMeasurementUTM> {
  std::size_t operator()(const vi_map::GPSMeasurementUTM& value) const {
    std::size_t h0(std::hash<size_t>()(value.sensor_id.hashToSizeT()));
    std::size_t h1(std::hash<int64_t>()(value.timestamp));
    std::size_t h2(std::hash<double>()(value.T_R_S.getPosition()[0]));
    std::size_t h3(std::hash<double>()(value.T_R_S.getPosition()[1]));
    std::size_t h4(std::hash<double>()(value.T_R_S.getPosition()[2]));
    std::size_t h5(std::hash<double>()(value.T_R_S.getRotation().vector()[0]));
    std::size_t h6(std::hash<double>()(value.T_R_S.getRotation().vector()[1]));
    std::size_t h7(std::hash<double>()(value.T_R_S.getRotation().vector()[2]));
    return h0 ^ h1 ^ h2 ^ h3 ^ h4 ^ h5 ^ h6 ^ h7;
  }
};

template <>
struct hash<vi_map::GPSMeasurementUTMMissionPair> {
  std::size_t operator()(
      const vi_map::GPSMeasurementUTMMissionPair& value) const {
    std::size_t h0(std::hash<vi_map::MissionId>()(value.mission_id));
    std::size_t h1(
        std::hash<vi_map::GPSMeasurementUTM>()(value.gps_measurement));
    return h0 ^ h1;
  }
};

template <>
struct hash<vi_map::GPSMeasurementWGS> {
  std::size_t operator()(const vi_map::GPSMeasurementWGS& value) const {
    std::size_t h0(std::hash<size_t>()(value.sensor_id.hashToSizeT()));
    std::size_t h1(std::hash<int64_t>()(value.timestamp));
    std::size_t h2(std::hash<double>()(value.latitude_deg));
    std::size_t h3(std::hash<double>()(value.longitude_deg));
    std::size_t h4(std::hash<double>()(value.altitude_meters));
    return h0 ^ h1 ^ h2 ^ h3 ^ h4;
  }
};
}  // namespace std

namespace vi_map {

class GPSDataStorage {
 public:
  template <typename T>
  using GPSBuffer = common::TemporalBuffer<
      T, Eigen::aligned_allocator<std::pair<const int64_t, T>>>;
  template <typename GPSMeasurementType>
  using GPSContainer =
      std::unordered_map<MissionId, GPSBuffer<GPSMeasurementType>>;

  typedef GPSContainer<GPSMeasurementWGS> WGSContainer;
  typedef GPSContainer<GPSMeasurementUTM> UTMContainer;

  typedef GPSBuffer<GPSMeasurementWGS> WGSBuffer;
  typedef GPSBuffer<GPSMeasurementUTM> UTMBuffer;

  GPSDataStorage() = default;
  GPSDataStorage& operator=(const GPSDataStorage&) = default;

  template <typename T>
  void addMeasurement(const T& measurement, const MissionId& mission_id);

  void clearWGSData(void);
  void clearUTMData(void);

  template <typename T>
  inline bool getClosestGPSMeasurementInTimeForMission(
      const MissionId& mission_id, int64_t timestamp, T* gps_measurement) const;

  template <typename T>
  inline bool hasData() const;
  inline bool hasWGSData() const;
  inline bool hasUTMData() const;

  size_t getNumWGSMeasurements() const;
  size_t getNumUTMMeasurements() const;

  inline size_t getNumMissionsWithWGSMeasurements() const;
  inline size_t getNumMissionsWithUTMMeasurements() const;

  const WGSContainer& getWGSContainer() const;
  const UTMContainer& getUTMContainer() const;

  inline bool hasWGSBufferForMission(const MissionId& mission_id) const;
  const WGSBuffer& getWGSBufferForMission(const MissionId& mission_id) const;

  inline bool hasUTMBufferForMission(const MissionId& mission_id) const;
  const UTMBuffer& getUTMBufferForMission(const MissionId& mission_id) const;

  void importGPSData(const GPSDataStorage& source_gps_storage);
  void duplicateGpsDataOfMission(
      const MissionId& source_mission_id,
      const MissionId& destination_mission_id);

  void serialize(
      vi_map_deprecated::proto::GPSDataStorage* proto_gps_data_storage) const;
  void deserialize(
      const vi_map_deprecated::proto::GPSDataStorage& proto_gps_data_storage);

  void swap(GPSDataStorage* other);  // NOLINT

 private:
  WGSContainer wgs_data_;
  UTMContainer utm_data_;
};

}  // namespace vi_map

#include "./gps-data-storage-inl.h"

#endif  // VI_MAP_DEPRECATED_GPS_DATA_STORAGE_H_
