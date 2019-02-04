#include <unordered_map>

#include <maplab-common/test/testing-entrypoint.h>

#include "vi-map/vi-map-metadata.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {
namespace serialization {

class MetadataSerializationTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // Create some metadata.
    metadata_.emplace(VIMapFileType::kMissions, "mission");
    metadata_.emplace(VIMapFileType::kVertices, "vertices0");
    metadata_.emplace(VIMapFileType::kVertices, "vertices1");
    metadata_.emplace(VIMapFileType::kVertices, "vertices2");
    metadata_.emplace(VIMapFileType::kVertices, "vertices3");
    metadata_.emplace(VIMapFileType::kVertices, "vertices4");
    metadata_.emplace(VIMapFileType::kVertices, "vertices5");
    metadata_.emplace(VIMapFileType::kEdges, "edges0");
    metadata_.emplace(VIMapFileType::kEdges, "edges1");
    metadata_.emplace(VIMapFileType::kEdges, "edges2");

    // Create inverse map.
    for (const VIMapMetadata::value_type& entry : metadata_) {
      ASSERT_TRUE(inverse_map_.emplace(entry.second, entry.first).second);
    }

    ASSERT_EQ(metadata_.size(), inverse_map_.size());
  }

  VIMapMetadata metadata_;
  typedef std::unordered_map<std::string, VIMapFileType> InverseMap;
  InverseMap inverse_map_;
};

TEST_F(MetadataSerializationTest, Serialization) {
  proto::VIMapMetadata metadata_proto;
  serializeMetadata(metadata_, &metadata_proto);

  VIMapMetadata deserialized_metadata;
  deserializeMetadata(metadata_proto, &deserialized_metadata);

  EXPECT_EQ(metadata_.size(), deserialized_metadata.size());
  for (const VIMapMetadata::value_type& entry : deserialized_metadata) {
    const InverseMap::const_iterator inverse_map_iterator =
        inverse_map_.find(entry.second);
    EXPECT_TRUE(inverse_map_iterator != inverse_map_.cend());
    EXPECT_EQ(entry.second, inverse_map_iterator->first);
    EXPECT_EQ(entry.first, inverse_map_iterator->second);
  }
}

// Check that the ordering stays consistent between serialization as some
// functions depend on this.
TEST_F(MetadataSerializationTest, Ordering) {
  proto::VIMapMetadata metadata_proto;
  serializeMetadata(metadata_, &metadata_proto);

  VIMapMetadata deserialized_metadata;
  deserializeMetadata(metadata_proto, &deserialized_metadata);

  ASSERT_EQ(metadata_.size(), deserialized_metadata.size());
  for (VIMapMetadata::const_iterator
           it_original = metadata_.cbegin(),
           it_deserialized = deserialized_metadata.cbegin();
       it_original != metadata_.cend() &&
       it_deserialized != deserialized_metadata.cend();
       ++it_original, ++it_deserialized) {
    EXPECT_EQ(it_original->first, it_deserialized->first);
    // Consistent ordering of name is not needed and may not be guaranteed.
  }
}

}  // namespace serialization
}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
