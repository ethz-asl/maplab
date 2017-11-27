#include <glog/logging.h>
#include <gtest/gtest.h>

#include "maplab-common/fixed-size-queue.h"
#include "maplab-common/test/testing-entrypoint.h"

namespace common {

TEST(TestFixedSizeQueue, Add) {
  const size_t kBufferSize = 3;
  common::FixedSizeQueue<size_t> buffer(kBufferSize);
  EXPECT_EQ(buffer.size(), 0u);
  EXPECT_FALSE(buffer.isFull());

  buffer.insert(0);
  buffer.insert(1);
  EXPECT_EQ(buffer.size(), 2u);
  EXPECT_FALSE(buffer.isFull());

  buffer.insert(2);
  EXPECT_EQ(buffer.size(), 3u);
  EXPECT_TRUE(buffer.isFull());

  buffer.insert(3);
  EXPECT_EQ(buffer.size(), 3u);
  EXPECT_TRUE(buffer.isFull());

  // There should be 1, 2, 3 in the buffer (0 was discarded).
  ASSERT_EQ(buffer.size(), 3u);
  EXPECT_EQ(buffer.buffer()[0], 1u);
  EXPECT_EQ(buffer.buffer()[1], 2u);
  EXPECT_EQ(buffer.buffer()[2], 3u);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
