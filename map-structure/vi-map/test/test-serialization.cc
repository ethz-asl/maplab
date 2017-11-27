#include <maplab-common/network-common.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "vi-map/test/vi-map-generator.h"
#include "vi-map/test/vi-map-test-helpers.h"
#include "vi-map/vi-map-serialization.h"
#include "vi-map/vi-map.h"

void deleteRawData(const network::RawMessageDataList& raw_data) {
  for (const network::RawMessageData& raw_data_part : raw_data) {
    delete[] static_cast<uint8_t*>(raw_data_part.first);
  }
}

TEST(Serialization, SerializeMapToRawArray) {
  vi_map::VIMap test_map, deserialized_map;
  vi_map::test::generateMap(&test_map);

  network::RawMessageDataList raw_data;
  vi_map::serialization::serializeToRawArray(test_map, &raw_data);
  constexpr size_t kStartIndex = 0u;
  vi_map::serialization::deserializeFromRawArray(
      raw_data, kStartIndex, &deserialized_map);

  vi_map::test::compareVIMap(test_map, deserialized_map);
  deleteRawData(raw_data);
}

TEST(Serialization, SerializeMapWithOptionalCameraResources) {
  const std::string test_folder = "SerializeMapWithOptionalCameraResources";
  const std::string map_folder = test_folder + "/" + "test_map";
  common::removeIfExistsAndCreatePath(map_folder);

  vi_map::VIMap test_map, deserialized_map;
  vi_map::test::generateMapWithOptionalCameraResources(
      100, map_folder, &test_map);

  // Serialize and deserialize.
  network::RawMessageDataList raw_data;
  vi_map::serialization::serializeToRawArray(test_map, &raw_data);
  constexpr size_t kStartIndex = 0u;
  vi_map::serialization::deserializeFromRawArray(
      raw_data, kStartIndex, &deserialized_map);

  // Compare maps.
  vi_map::test::compareVIMap(test_map, deserialized_map);
  deleteRawData(raw_data);
}

MAPLAB_UNITTEST_ENTRYPOINT
