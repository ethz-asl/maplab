#include <maplab-common/test/testing-entrypoint.h>

#include <aslam/common/memory.h>
#include <aslam/frames/visual-nframe.h>
#include <gtest/gtest.h>
#include <map-resources/resource-common.h>

#include "vi-map/unique-id.h"
#include "vi-map/vertex.h"

namespace vi_map {

class VertexResourcesTest : public ::testing::Test {
 public:
  VertexResourcesTest() {}

  uint kFrameIdx0 = 0;
  uint kFrameIdx1 = 1;
  uint kFrameIdx2 = 2;

  uint kInvalidFrameIdx = 99;

  backend::ResourceType kType0 = backend::ResourceType::kRawImage;
  backend::ResourceType kType1 = backend::ResourceType::kRawDepthMap;
  backend::ResourceType kType2 = backend::ResourceType::kImageForDepthMap;

  backend::ResourceType kUnusedType = backend::ResourceType::kOptimizedDepthMap;

  backend::ResourceType kInvalidType = backend::ResourceType::kCount;

  backend::ResourceId kResourceId0;
  backend::ResourceId kResourceId1;
  backend::ResourceId kResourceId2;
  backend::ResourceId kResourceId3;
  backend::ResourceId kResourceId4;

  backend::ResourceId kUnusedResourceId;

  Vertex::UniquePtr vertex_;

  virtual void SetUp() {
    vertex_ = aligned_unique<Vertex>();
    aslam::NFramesId id;
    vertex_->n_frame_.reset(new aslam::VisualNFrame(id, 3));

    common::generateId(&kResourceId0);
    common::generateId(&kResourceId1);
    common::generateId(&kResourceId2);
    common::generateId(&kResourceId3);
    common::generateId(&kResourceId4);

    common::generateId(&kUnusedResourceId);

    vertex_->resource_map_.resize(3);
    vertex_->resource_map_.at(kFrameIdx0)[kType0].insert(kResourceId0);
    vertex_->resource_map_.at(kFrameIdx0)[kType0].insert(kResourceId1);
    vertex_->resource_map_.at(kFrameIdx1)[kType1].insert(kResourceId2);
    vertex_->resource_map_.at(kFrameIdx2)[kType1].insert(kResourceId3);
    vertex_->resource_map_.at(kFrameIdx2)[kType2].insert(kResourceId4);
  }
};

TEST_F(VertexResourcesTest, TestHasFrameResourceWithId) {
  EXPECT_FALSE(vertex_->hasFrameResourceWithId(kFrameIdx0, kUnusedResourceId));
  EXPECT_FALSE(vertex_->hasFrameResourceWithId(kFrameIdx0, kResourceId3));

  EXPECT_TRUE(vertex_->hasFrameResourceWithId(kFrameIdx0, kResourceId0));
  EXPECT_TRUE(vertex_->hasFrameResourceWithId(kFrameIdx0, kResourceId1));
  EXPECT_TRUE(vertex_->hasFrameResourceWithId(kFrameIdx1, kResourceId2));
  EXPECT_TRUE(vertex_->hasFrameResourceWithId(kFrameIdx2, kResourceId3));
  EXPECT_TRUE(vertex_->hasFrameResourceWithId(kFrameIdx2, kResourceId4));
}

TEST_F(VertexResourcesTest, TestHasFrameResourceOfType) {
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx0, kUnusedType));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx0, kType1));

  EXPECT_TRUE(vertex_->hasFrameResourceOfType(kFrameIdx0, kType0));
  EXPECT_TRUE(vertex_->hasFrameResourceOfType(kFrameIdx1, kType1));
  EXPECT_TRUE(vertex_->hasFrameResourceOfType(kFrameIdx2, kType1));
  EXPECT_TRUE(vertex_->hasFrameResourceOfType(kFrameIdx2, kType2));
}

TEST_F(VertexResourcesTest, TestGetNumFrameResourcesOfType) {
  EXPECT_EQ(0u, vertex_->getNumFrameResourcesOfType(kFrameIdx0, kUnusedType));
  EXPECT_EQ(0u, vertex_->getNumFrameResourcesOfType(kFrameIdx0, kType1));

  EXPECT_EQ(2u, vertex_->getNumFrameResourcesOfType(kFrameIdx0, kType0));
  EXPECT_EQ(1u, vertex_->getNumFrameResourcesOfType(kFrameIdx1, kType1));
  EXPECT_EQ(1u, vertex_->getNumFrameResourcesOfType(kFrameIdx2, kType1));
  EXPECT_EQ(1u, vertex_->getNumFrameResourcesOfType(kFrameIdx2, kType2));
}

TEST_F(VertexResourcesTest, TestGetFrameResourceIdsOfType) {
  backend::ResourceIdSet resource_ids_0;
  backend::ResourceIdSet resource_ids_1;
  vertex_->getFrameResourceIdsOfType(kFrameIdx0, kType0, &resource_ids_0);
  EXPECT_EQ(2u, resource_ids_0.size());
  EXPECT_EQ(1u, resource_ids_0.count(kResourceId0));
  EXPECT_EQ(1u, resource_ids_0.count(kResourceId1));

  vertex_->getFrameResourceIdsOfType(kFrameIdx1, kType0, &resource_ids_1);
  EXPECT_EQ(0u, resource_ids_1.size());
}

TEST_F(VertexResourcesTest, TestAddFrameResourceIdOfType) {
  backend::ResourceIdSet resource_ids_before;
  vertex_->getFrameResourceIdsOfType(
      kFrameIdx0, kUnusedType, &resource_ids_before);
  EXPECT_EQ(0u, resource_ids_before.size());
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx0, kUnusedType));
  vertex_->addFrameResourceIdOfType(kFrameIdx0, kUnusedType, kUnusedResourceId);
  EXPECT_TRUE(vertex_->hasFrameResourceOfType(kFrameIdx0, kUnusedType));
  backend::ResourceIdSet resource_ids_after;
  vertex_->getFrameResourceIdsOfType(
      kFrameIdx0, kUnusedType, &resource_ids_after);
  EXPECT_EQ(1u, resource_ids_after.size());
  EXPECT_EQ(1u, resource_ids_after.count(kUnusedResourceId));
}

TEST_F(VertexResourcesTest, TestDeleteFrameResourceIdsOfType) {
  vertex_->deleteFrameResourceIdsOfType(kFrameIdx2, kType1);

  EXPECT_TRUE(vertex_->hasFrameResourceOfType(kFrameIdx0, kType0));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx0, kType1));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx0, kType2));

  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx1, kType0));
  EXPECT_TRUE(vertex_->hasFrameResourceOfType(kFrameIdx1, kType1));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx1, kType2));

  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx2, kType0));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx2, kType1));
  EXPECT_TRUE(vertex_->hasFrameResourceOfType(kFrameIdx2, kType2));
}

TEST_F(VertexResourcesTest, TestDeleteAllFrameResourceInfoForOneFrame) {
  vertex_->deleteAllFrameResourceInfo(kFrameIdx2);

  EXPECT_TRUE(vertex_->hasFrameResourceOfType(kFrameIdx0, kType0));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx0, kType1));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx0, kType2));

  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx1, kType0));
  EXPECT_TRUE(vertex_->hasFrameResourceOfType(kFrameIdx1, kType1));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx1, kType2));

  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx2, kType0));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx2, kType1));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx2, kType2));
}

TEST_F(VertexResourcesTest, TestDeleteAllFrameResourceInfo) {
  vertex_->deleteAllFrameResourceInfo();

  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx0, kType0));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx0, kType1));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx0, kType2));

  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx1, kType0));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx1, kType1));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx1, kType2));

  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx2, kType0));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx2, kType1));
  EXPECT_FALSE(vertex_->hasFrameResourceOfType(kFrameIdx2, kType2));
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
