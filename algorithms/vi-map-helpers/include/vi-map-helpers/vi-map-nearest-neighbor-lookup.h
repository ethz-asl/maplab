#ifndef VI_MAP_HELPERS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_H_
#define VI_MAP_HELPERS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_H_

#include <functional>
#include <memory>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <gtest/gtest.h>
#include <maplab-common/macros.h>
#include <nabo/nabo.h>
#include <sensors/gps-utm.h>
#include <sensors/gps-wgs.h>
#include <vi-map/vi-map.h>

namespace vi_map {
template <typename GpsMeasurement>
struct GpsMeasurementMissionPair final {
  GpsMeasurementMissionPair() {
    mission_id.setInvalid();
  }
  GpsMeasurementMissionPair(
      const GpsMeasurement& gps_measurement_, const MissionId& mission_id_)
      : gps_measurement(gps_measurement_), mission_id(mission_id_) {
    CHECK(mission_id_.isValid());
  }
  ~GpsMeasurementMissionPair() = default;

  bool operator==(const GpsMeasurementMissionPair& other) const {
    return gps_measurement == other.gps_measurement &&
           mission_id == other.mission_id;
  }

  GpsMeasurement gps_measurement;
  MissionId mission_id;
};
typedef GpsMeasurementMissionPair<GpsUtmMeasurement>
    GpsUtmMeasurementMissionPair;
typedef GpsMeasurementMissionPair<GpsWgsMeasurement>
    GpsWgsMeasurementMissionPair;
typedef AlignedUnorderedSet<GpsUtmMeasurementMissionPair>
    GpsUtmMeasurementMissionPairSet;
}  // namespace vi_map

namespace std {
template <>
struct hash<vi_map::GpsUtmMeasurementMissionPair> {
  std::size_t operator()(
      const vi_map::GpsUtmMeasurementMissionPair& value) const {
    std::size_t h0(std::hash<vi_map::MissionId>()(value.mission_id));
    std::size_t h1(
        std::hash<vi_map::GpsUtmMeasurement>()(value.gps_measurement));
    return h0 ^ h1;
  }
};
}  // namespace std

namespace vi_map_helpers {

// VIMap wrapper class that provides a nearest-neighbor vertex query.
template <typename QueryType, typename DataType>
class VIMapNearestNeighborLookup {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(VIMapNearestNeighborLookup);
  FRIEND_TEST(VIMapNearestNeighborLookupTest, MultipleVertexMap);
  FRIEND_TEST(VIMapNearestNeighborLookupTest, MultipleVertexRadiusLookup);
  FRIEND_TEST(VIMapNearestNeighborLookupTest,
              MultipleGPSWGSMeasurementsNNLookup);
  FRIEND_TEST(VIMapNearestNeighborLookupTest,
              MultipleWGSMeasurementRadiusLookup);
  FRIEND_TEST(VIMapNearestNeighborLookupTest,
              MultipleGPSUTMMeasuremnentsNNLookup);
  FRIEND_TEST(VIMapNearestNeighborLookupTest,
              MultipleUTMMeasurementsRadiusLookup);

  VIMapNearestNeighborLookup() = delete;
  explicit VIMapNearestNeighborLookup(const vi_map::VIMap& map);
  virtual ~VIMapNearestNeighborLookup() = default;

  // Returns the data item in the map that is closest (L2 norm) to the given
  // query object.
  bool getClosestDataItem(const QueryType& query,
                          DataType* closest_data_item) const;
  // Returns all items found to be within the given search radius (L2 norm) of
  // the given query object.
  template <typename Allocator>
  void getAllDataItemsWithinRadius(
      const QueryType& query, const double search_radius,
      std::unordered_set<DataType, std::hash<DataType>, std::equal_to<DataType>,
                         Allocator>* data_items_within_search_radius) const;

  inline size_t size() const;
  inline bool empty() const;

 private:
  inline void buildIndex();

  const vi_map::VIMap& map_;
  std::unique_ptr<Nabo::NNSearchD> nn_index_;
  Eigen::MatrixXd nn_index_data_;
  std::vector<DataType> data_items_;
};

typedef VIMapNearestNeighborLookup<aslam::Position3D, pose_graph::VertexId>
    VIMapNearestNeighborLookupVertexId;
typedef VIMapNearestNeighborLookup<vi_map::GpsWgsMeasurement,
                                   vi_map::GpsWgsMeasurement>
    VIMapNearestNeighborLookupGpsWgs;
typedef VIMapNearestNeighborLookup<vi_map::GpsUtmMeasurement,
                                   vi_map::GpsUtmMeasurement>
    VIMapNearestNeighborLookupGpsUtm;
typedef VIMapNearestNeighborLookup<vi_map::GpsUtmMeasurement,
                                   vi_map::GpsUtmMeasurementMissionPair>
    VIMapNearestNeighborLookupGpsUtmWithMissionId;
typedef VIMapNearestNeighborLookup<vi_map::GpsWgsMeasurement,
                                   vi_map::GpsWgsMeasurementMissionPair>
    VIMapNearestNeighborLookupGpsWgsWithMissionId;

template <class QueryType>
inline Eigen::VectorXd queryTypeToVector(const QueryType& query);

template <class QueryType>
inline size_t getQueryTypeDimension();

template <class GpsMeasurement, class GpsDataType>
GpsDataType createDataItem(
    const GpsMeasurement& gps_measuremnt, const vi_map::MissionId& mission_id);

}  // namespace vi_map_helpers

#include "./vi-map-nearest-neighbor-lookup-inl.h"

#endif  // VI_MAP_HELPERS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_H_
