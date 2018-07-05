#ifndef VI_MAP_HELPERS_GPS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_H_
#define VI_MAP_HELPERS_GPS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_H_

#include <functional>
#include <type_traits>

#include <sensors/gps-utm.h>
#include <sensors/gps-wgs.h>

#include "vi-map-helpers/vi-map-nearest-neighbor-lookup.h"

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

template <typename GpsQueryType, typename GpsDataType>
class GpsVIMapNearestNeighborLookup final
    : public VIMapNearestNeighborLookupBase<GpsQueryType, GpsDataType> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(GpsVIMapNearestNeighborLookup);
  FRIEND_TEST(
      VIMapNearestNeighborLookupTest, MultipleGPSWGSMeasurementsNNLookup);
  FRIEND_TEST(
      VIMapNearestNeighborLookupTest, MultipleWGSMeasurementRadiusLookup);
  FRIEND_TEST(
      VIMapNearestNeighborLookupTest, MultipleGPSUTMMeasuremnentsNNLookup);
  FRIEND_TEST(
      VIMapNearestNeighborLookupTest, MultipleUTMMeasurementsRadiusLookup);
  explicit GpsVIMapNearestNeighborLookup(const vi_map::VIMap& map) {
    static_assert(
        std::is_same<GpsQueryType, vi_map::GpsUtmMeasurement>::value ||
            std::is_same<GpsQueryType, vi_map::GpsWgsMeasurement>::value,
        "Invalid data type. The query type must be a GPS measurement type.");
    VIMapNearestNeighborLookupBase<GpsQueryType, GpsDataType>::buildIndex(map);
  }
  ~GpsVIMapNearestNeighborLookup() = default;

 private:
  typedef VIMapNearestNeighborLookupBase<GpsQueryType, GpsDataType> BaseType;
  void buildIndexImpl(const vi_map::VIMap& map) override;
};

typedef GpsVIMapNearestNeighborLookup<vi_map::GpsWgsMeasurement,
                                      vi_map::GpsWgsMeasurement>
    VIMapNearestNeighborLookupGpsWgs;
typedef GpsVIMapNearestNeighborLookup<vi_map::GpsUtmMeasurement,
                                      vi_map::GpsUtmMeasurement>
    VIMapNearestNeighborLookupGpsUtm;
typedef GpsVIMapNearestNeighborLookup<vi_map::GpsUtmMeasurement,
                                      vi_map::GpsUtmMeasurementMissionPair>
    VIMapNearestNeighborLookupGpsUtmWithMissionId;
typedef GpsVIMapNearestNeighborLookup<vi_map::GpsWgsMeasurement,
                                      vi_map::GpsWgsMeasurementMissionPair>
    VIMapNearestNeighborLookupGpsWgsWithMissionId;

template <class GpsMeasurement, class GpsDataType>
GpsDataType createDataItem(
    const GpsMeasurement& gps_measuremnt, const vi_map::MissionId& mission_id);

}  // namespace vi_map_helpers

#include "./gps-vi-map-nearest-neighbor-lookup-inl.h"

#endif  // VI_MAP_HELPERS_GPS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_H_
