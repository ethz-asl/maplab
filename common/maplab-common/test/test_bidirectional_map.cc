#include "maplab-common/bidirectional-map.h"
#include "maplab-common/test/testing-entrypoint.h"

namespace common {
class BidirectionalMapTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // Setup the map with the following paris:
    // (0,-10), (1,-9), (2,-8), ...
    for (size_t id = 0; id < kNumElements; ++id) {
      size_t element_left = createLeftFromId(id);
      int element_right = createRightFromId(id);
      ASSERT_TRUE(map_.insert(element_left, element_right));
    }
  }
  virtual void TearDown() {}

  static size_t createLeftFromId(size_t id) {
    return id;
  }
  static int createRightFromId(size_t id) {
    return static_cast<int>(id) - kNumElements;
  }

  bool checkConsistency() const {
    return (map_.left_to_right_.size() == map_.right_to_left_.size());
  }

 protected:
  common::BidirectionalMap<size_t, int> map_;
  static constexpr size_t kNumElements = 10u;
};

TEST_F(BidirectionalMapTest, SuccessfulLookup) {
  for (size_t id = 0; id < kNumElements; ++id) {
    // Left to right.
    const int* element_right = map_.getRight(createLeftFromId(id));
    ASSERT_TRUE(element_right != nullptr);
    EXPECT_EQ(*element_right, createRightFromId(id));

    // Right to left.
    const size_t* element_left = map_.getLeft(createRightFromId(id));
    ASSERT_TRUE(element_left != nullptr);
    EXPECT_EQ(*element_left, createLeftFromId(id));
  }

  EXPECT_TRUE(checkConsistency());
}

TEST_F(BidirectionalMapTest, UnsuccesfullLookup) {
  const int* element_right = map_.getRight(createLeftFromId(kNumElements + 1u));
  EXPECT_EQ(element_right, nullptr);

  const size_t* element_left =
      map_.getLeft(createRightFromId(kNumElements + 1u));
  EXPECT_EQ(element_left, nullptr);

  EXPECT_TRUE(checkConsistency());
}

TEST_F(BidirectionalMapTest, TryReplaceElement) {
  // Try to replace an element.
  EXPECT_FALSE(
      map_.insert(createLeftFromId(kNumElements + 1), createRightFromId(1)));
  EXPECT_FALSE(
      map_.insert(createLeftFromId(1), createRightFromId(kNumElements + 1)));

  // Make the map is still okay.
  for (size_t id = 0; id < kNumElements; ++id) {
    // Left to right.
    const int* element_right = map_.getRight(createLeftFromId(id));
    ASSERT_TRUE(element_right != nullptr);
    EXPECT_EQ(*element_right, createRightFromId(id));

    // Right to left.
    const size_t* element_left = map_.getLeft(createRightFromId(id));
    ASSERT_TRUE(element_left != nullptr);
    EXPECT_EQ(*element_left, createLeftFromId(id));
  }

  EXPECT_TRUE(checkConsistency());
}
}  // namespace common
MAPLAB_UNITTEST_ENTRYPOINT
