#ifndef VI_MAP_HELPERS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_INL_H_
#define VI_MAP_HELPERS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_INL_H_

#include <functional>
#include <limits>
#include <unordered_set>

#include <glog/logging.h>

namespace vi_map_helpers {

// VIMap wrapper class that provides a nearest-neighbor vertex query.
template <typename QueryType, typename DataType>
VIMapNearestNeighborLookup<QueryType, DataType>::VIMapNearestNeighborLookup(
    const vi_map::VIMap& map)
    : map_(map) {
  buildIndex();
  CHECK_EQ(nn_index_data_.cols(), data_items_.size());
  CHECK_EQ(size(), data_items_.size());
}

template <typename QueryType, typename DataType>
bool VIMapNearestNeighborLookup<QueryType, DataType>::empty() const {
  return size() == 0u;
}

template <typename QueryType, typename DataType>
size_t VIMapNearestNeighborLookup<QueryType, DataType>::size() const {
  CHECK_GE(nn_index_data_.cols(), 0u);
  return static_cast<size_t>(nn_index_data_.cols());
}

template <typename QueryType, typename DataType>
bool VIMapNearestNeighborLookup<QueryType, DataType>::getClosestDataItem(
    const QueryType& query, DataType* closest_data_item) const {
  CHECK_NOTNULL(closest_data_item);
  if (empty()) {
    LOG(WARNING) << "The nearest-neighbor index is empty. "
                 << "Can't look for closest data item.";
    return false;
  }
  CHECK(nn_index_);
  CHECK_EQ(size(), data_items_.size());

  constexpr int kNumNeighbors = 1;

  Eigen::VectorXi index = Eigen::VectorXi::Constant(kNumNeighbors, -1);
  Eigen::VectorXd distance_squared = Eigen::VectorXd::Constant(
      kNumNeighbors, std::numeric_limits<double>::infinity());
  constexpr double kSearchNNEpsilon = 0.0;
  const int kOptionFlags = Nabo::NNSearchD::ALLOW_SELF_MATCH;
  const Eigen::VectorXd query_vector = queryTypeToVector(query);
  CHECK_EQ(query_vector.rows(), nn_index_data_.rows());
  nn_index_->knn(query_vector, index, distance_squared, kNumNeighbors,
                 kSearchNNEpsilon, kOptionFlags);
  const double nn_distance_squared = distance_squared(0);
  const int nn_index = index(0);

  if (nn_index >= 0 &&
      nn_distance_squared != std::numeric_limits<double>::infinity()) {
    CHECK_LT(nn_index, data_items_.size());
    *closest_data_item = data_items_[nn_index];
    return true;
  }
  return false;
}

template <typename QueryType, typename DataType>
template <typename Allocator>
void
VIMapNearestNeighborLookup<QueryType, DataType>::getAllDataItemsWithinRadius(
    const QueryType& query, const double search_radius,
    std::unordered_set<DataType, std::hash<DataType>, std::equal_to<DataType>,
                       Allocator>* data_items_within_search_radius) const {
  CHECK_NOTNULL(data_items_within_search_radius)->clear();
  if (empty()) {
    LOG(WARNING) << "The nearest-neighbor index is empty. "
                 << "Can't look for data items within a radius.";
    return;
  }
  CHECK(nn_index_);
  CHECK_EQ(size(), data_items_.size());

  const int num_neighbors = size();
  CHECK_GT(num_neighbors, 0);

  Eigen::VectorXi index = Eigen::VectorXi::Constant(num_neighbors, -1);

  Eigen::VectorXd distance_squared = Eigen::VectorXd::Constant(
      num_neighbors, std::numeric_limits<double>::infinity());
  constexpr double kSearchNNEpsilon = 0.0;
  const int kOptionFlags = Nabo::NNSearchD::ALLOW_SELF_MATCH;
  const Eigen::VectorXd query_vector = queryTypeToVector(query);
  CHECK_EQ(query_vector.rows(), nn_index_data_.rows());
  nn_index_->knn(query_vector, index, distance_squared, num_neighbors,
                 kSearchNNEpsilon, kOptionFlags, search_radius);

  size_t result_idx = 0u;
  while (result_idx < num_neighbors &&
         distance_squared[result_idx] <
             std::numeric_limits<double>::infinity()) {
    const int nn_index = index(result_idx);
    CHECK_LT(nn_index, data_items_.size());
    data_items_within_search_radius->emplace(data_items_[nn_index]);
    ++result_idx;
  }
}

template <>
inline size_t getQueryTypeDimension<vi_map::GpsUtmMeasurement>() {
  return 3u;
}

template <>
inline size_t getQueryTypeDimension<vi_map::GpsWgsMeasurement>() {
  return 2u;
}

template <>
inline Eigen::VectorXd queryTypeToVector(
    const vi_map::GpsWgsMeasurement& wgs_measurement) {
  return Eigen::Vector2d(
      wgs_measurement.getLatitudeDeg(), wgs_measurement.getLongitudeDeg());
}

template <>
inline Eigen::VectorXd queryTypeToVector(
    const vi_map::GpsUtmMeasurement& utm_measurement) {
  return utm_measurement.get_T_UTM_S().getPosition();
}

template <>
inline Eigen::VectorXd queryTypeToVector(const aslam::Position3D& p_G_I) {
  return p_G_I;
}

template <>
inline vi_map::GpsWgsMeasurement
createDataItem<vi_map::GpsWgsMeasurement, vi_map::GpsWgsMeasurement>(
    const vi_map::GpsWgsMeasurement& gps_measuremnt,
    const vi_map::MissionId& /*mission_id*/) {
  return gps_measuremnt;
}

template <>
inline vi_map::GpsWgsMeasurementMissionPair
createDataItem<vi_map::GpsWgsMeasurement, vi_map::GpsWgsMeasurementMissionPair>(
    const vi_map::GpsWgsMeasurement& gps_measuremnt,
    const vi_map::MissionId& mission_id) {
  CHECK(mission_id.isValid());
  return vi_map::GpsWgsMeasurementMissionPair(gps_measuremnt, mission_id);
}

template <>
inline vi_map::GpsUtmMeasurement
createDataItem<vi_map::GpsUtmMeasurement, vi_map::GpsUtmMeasurement>(
    const vi_map::GpsUtmMeasurement& gps_measuremnt,
    const vi_map::MissionId& /*mission_id*/) {
  return gps_measuremnt;
}

template <>
inline vi_map::GpsUtmMeasurementMissionPair
createDataItem<vi_map::GpsUtmMeasurement, vi_map::GpsUtmMeasurementMissionPair>(
    const vi_map::GpsUtmMeasurement& gps_measuremnt,
    const vi_map::MissionId& mission_id) {
  CHECK(mission_id.isValid());
  return vi_map::GpsUtmMeasurementMissionPair(gps_measuremnt, mission_id);
}

template <>
void VIMapNearestNeighborLookup<aslam::Position3D,
                                pose_graph::VertexId>::buildIndex();

template <class GpsQueryType, class GpsDataType>
void VIMapNearestNeighborLookup<GpsQueryType, GpsDataType>::buildIndex() {
  LOG_IF(WARNING, nn_index_)
      << "The already existing k-d tree index will be overwritten.";

  using GpsMeasurementBuffer = vi_map::MeasurementBuffer<GpsQueryType>;

  const vi_map::SensorType kSensorType =
      vi_map::measurementToSensorType<GpsQueryType>();

  const vi_map::SensorManager& sensor_manager = map_.getSensorManager();
  vi_map::SensorIdSet gps_sensor_ids;
  sensor_manager.getAllSensorIdsOfType(kSensorType, &gps_sensor_ids);
  VLOG(2) << "The map has " << gps_sensor_ids.size() << " sensors of type "
          << vi_map::sensorTypeToString(kSensorType);

  vi_map::MissionIdList all_mission_ids;
  map_.getAllMissionIds(&all_mission_ids);

  size_t total_num_gps_measurements = 0u;
  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    CHECK(mission_id.isValid());
    if (!map_.hasOptionalSensorData(mission_id)) {
      continue;
    }

    const vi_map::OptionalSensorData& optional_sensor_data =
        map_.getOptionalSensorData(mission_id);

    optional_sensor_data.lock();

    for (const vi_map::SensorId& gps_sensor_id : gps_sensor_ids) {
      CHECK(gps_sensor_id.isValid());
      const GpsMeasurementBuffer& gps_measurements =
          optional_sensor_data.getMeasurements<GpsQueryType>(gps_sensor_id);
      total_num_gps_measurements += gps_measurements.size();
    }
  }

  if (total_num_gps_measurements > 0u) {
    VLOG(2) << "Building GPS index with " << total_num_gps_measurements
            << " measurements.";

    const size_t kQueryDimVector = getQueryTypeDimension<GpsQueryType>();
    nn_index_data_ =
        Eigen::MatrixXd(kQueryDimVector, total_num_gps_measurements);

    size_t col_idx = 0u;
    for (const vi_map::MissionId& mission_id : all_mission_ids) {
      CHECK(mission_id.isValid());
      const vi_map::OptionalSensorData& optional_sensor_data =
          map_.getOptionalSensorData(mission_id);
      for (const vi_map::SensorId& gps_sensor_id : gps_sensor_ids) {
        CHECK(gps_sensor_id.isValid());
        const GpsMeasurementBuffer& gps_measurements =
            optional_sensor_data.getMeasurements<GpsQueryType>(gps_sensor_id);

        for (const typename GpsMeasurementBuffer::BufferType::value_type&
                 time_gps_pair : gps_measurements.buffered_values()) {
          const GpsDataType& data_item =
              createDataItem<GpsQueryType, GpsDataType>(
                  time_gps_pair.second, mission_id);
          data_items_.emplace_back(data_item);
          const Eigen::VectorXd data_item_as_vector =
              queryTypeToVector(time_gps_pair.second);
          CHECK_EQ(data_item_as_vector.rows(), nn_index_data_.rows());
          nn_index_data_.col(col_idx) = data_item_as_vector;
          ++col_idx;
        }
      }

      optional_sensor_data.unlock();
    }
    nn_index_.reset(
        Nabo::NNSearchD::createKDTreeLinearHeap(
            nn_index_data_, kQueryDimVector));
  }

  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    CHECK(mission_id.isValid());
    if (!map_.hasOptionalSensorData(mission_id)) {
      continue;
    }

    const vi_map::OptionalSensorData& optional_sensor_data =
        map_.getOptionalSensorData(mission_id);

    optional_sensor_data.unlock();
  }
}

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_INL_H_
