#ifndef VI_MAP_TEST_VI_MAP_TEST_HELPERS_H_
#define VI_MAP_TEST_VI_MAP_TEST_HELPERS_H_

#include <string>

#include "vi-map/vi-map.h"

namespace vi_map {
namespace test {

// Generates a map with a given number of vertices. Note: some minimum number of
// vertices is always created, independent of this setting.
template <typename EdgeType>
void generateMap(const size_t number_of_vertices, vi_map::VIMap* map);
// Uses the VIMapGenerator to create a small test map with a few vertices and
// landmarks.
template <typename EdgeType>
void generateMap(vi_map::VIMap* map) {
  constexpr size_t kNumberOfAdditionalVertices = 0u;
  generateMap<EdgeType>(kNumberOfAdditionalVertices, map);
}

// Generates and adds some optional resources for each mission and adds
// them to the map. In this process, the resources are serialized to disk.
void generateOptionalSensorResourceAndAddToMap(vi_map::VIMap* map);

// Generates and adds some optional resource ids for each mission.
// Does not create the resources themselves and hence also does not
// add any resources to the map itself.
void generateOptionalSensorResourceIdsAndAddToAllMissions(vi_map::VIMap* map);

// Generates and adds random optional sensor data for each mission based on a
// deterministic seed.
void generateOptionalSensorDataAndAddToMap(vi_map::VIMap* map);

// Checks if the two given maps are equal.
bool compareVIMap(const vi_map::VIMap& map_a, const vi_map::VIMap& map_b);

}  // namespace test
}  // namespace vi_map

#include "./vi-map-test-helpers-inl.h"

#endif  // VI_MAP_TEST_VI_MAP_TEST_HELPERS_H_
