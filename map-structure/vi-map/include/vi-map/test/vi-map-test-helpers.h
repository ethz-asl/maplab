#ifndef VI_MAP_VI_MAP_TEST_HELPERS_H_
#define VI_MAP_VI_MAP_TEST_HELPERS_H_

#include <string>

#include "vi-map/vi-map.h"

namespace vi_map {
namespace test {

// Uses the VIMapGenerator to create a small test map with a few vertices and
// landmarks.
void generateMap(vi_map::VIMap* map);
// Generates a map with a given number of vertices. Note: some minimum number of
// vertices is always
// created, independent of this setting.
void generateMap(const size_t number_of_vertices, vi_map::VIMap* map);
// Same as generateMap but also adds some dummy resources.
void generateMapWithOptionalCameraResources(
    const size_t number_of_vertices, const std::string& map_folder,
    vi_map::VIMap* map);

// Checks if the two given maps are equal. This may not be a complete check.
bool compareVIMap(const vi_map::VIMap& map_a, const vi_map::VIMap& map_b);

// Generates random test data based on a deterministic seed.
void generateOptionalSensorDataAndAddToMap(vi_map::VIMap* map);

}  // namespace test
}  // namespace vi_map

#endif  // VI_MAP_VI_MAP_TEST_HELPERS_H_
