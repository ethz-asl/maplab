#include "maplab-common/test/testing-entrypoint.h"

#include <maplab-common/progress-bar.h>

namespace common {

TEST(Standard, ProgressBarTest) {
  const size_t kNumElements = 1000u;
  common::ProgressBar progress_bar(kNumElements);
  for (size_t i = 0; i <= kNumElements; ++i) {
    progress_bar.update(i);
    usleep(1000);
  }
}

TEST(Boundaries, ProgressBarTest) {
  const size_t kNumElements = 1000u;
  common::ProgressBar progress_bar(kNumElements);
  progress_bar.update(kNumElements);
  EXPECT_DEATH(progress_bar.update(kNumElements + 1), "");
}

TEST(IncrementAndReset, ProgressBarTest) {
  const size_t kNumElements = 500u;
  common::ProgressBar progress_bar(kNumElements);
  for (size_t i = 0; i < kNumElements; ++i) {
    progress_bar.increment();
    usleep(1000);
  }
  EXPECT_DEATH(progress_bar.increment(), "");
}

TEST(Reset, ProgressBarTest) {
  const size_t kNumElements = 1000u;
  common::ProgressBar progress_bar(kNumElements);
  for (size_t i = 0; i <= kNumElements; ++i) {
    progress_bar.update(i);
    usleep(1000);
  }
  const int kNewNumElements = 47;
  progress_bar.reset(kNewNumElements);
  // Go backwards, just for fun.
  for (int i = kNewNumElements; i >= 0; --i) {
    progress_bar.update(i);
    usleep(1000);
  }
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
