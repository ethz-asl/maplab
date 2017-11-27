#include <glog/logging.h>
#include <gtest/gtest.h>

#include <maplab-common/test/testing-entrypoint.h>
#include <posegraph/unique-id.h>

template <typename IdType>
class AslamPosegraph : public ::testing::Test {
 public:
  aslam::HashId base_id_;
  IdType posegraph_id_;
};

typedef ::testing::Types<pose_graph::VertexId, pose_graph::EdgeId> IdTypes;
TYPED_TEST_CASE(AslamPosegraph, IdTypes);

TYPED_TEST(AslamPosegraph, IdToStringToId) {
  common::generateId(&this->posegraph_id_);
  std::string id_string = this->posegraph_id_.hexString();

  TypeParam id_from_string;
  id_from_string.fromHexString(id_string);

  EXPECT_EQ(id_from_string, this->posegraph_id_);
  EXPECT_EQ(id_string, id_from_string.hexString());
}

TYPED_TEST(AslamPosegraph, IdToHashIdToId) {
  common::generateId(&this->posegraph_id_);
  this->posegraph_id_.toHashId(&this->base_id_);

  TypeParam id_from_hash_id;
  id_from_hash_id.fromHashId(this->base_id_);

  EXPECT_EQ(this->posegraph_id_, id_from_hash_id);
  EXPECT_EQ(this->posegraph_id_.hexString(), id_from_hash_id.hexString());
  EXPECT_EQ(this->posegraph_id_.hexString(), this->base_id_.hexString());
}

MAPLAB_UNITTEST_ENTRYPOINT
