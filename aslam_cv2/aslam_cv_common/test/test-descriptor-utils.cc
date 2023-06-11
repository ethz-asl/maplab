#include <vector>

#include <aslam/common/entrypoint.h>
#include <Eigen/Core>

#include "aslam/common/descriptor-utils.h"

namespace aslam {
namespace common {
namespace descriptor_utils {

TEST(ViwlsGraph, DescriptorMedianTestSingleDescriptor) {
  DescriptorsType descriptors(48, 1);
  descriptors.setRandom();
  DescriptorType median;

  descriptorMeanRoundedToBinaryValue(descriptors, &median);
  EXPECT_EQ(descriptors, median);
}

TEST(ViwlsGraph, DescriptorMedianTestTwoZeroDescriptors) {
  DescriptorType descriptor0(48, 1);
  DescriptorType descriptor1(48, 1);
  descriptor0.setZero();
  descriptor1.setZero();
  DescriptorType median;

  DescriptorsType descriptors(48, 2);
  descriptors.col(0) = descriptor0;
  descriptors.col(1) = descriptor1;

  descriptorMeanRoundedToBinaryValue(descriptors, &median);
  EXPECT_EQ(descriptor0, median);
}

TEST(ViwlsGraph, DescriptorMedianTestThreeDescriptors) {
  DescriptorsType descriptors(48, 3);
  descriptors.setZero();

  descriptors(0, 0) = 7;
  descriptors(0, 1) = 3;
  descriptors(0, 2) = 1;

  DescriptorType median(48, 1);

  descriptorMeanRoundedToBinaryValue(descriptors, &median);
  // Median of {1, 1, 1}
  EXPECT_TRUE(getBit(0, median));
  // Median of {1, 1, 0}
  EXPECT_TRUE(getBit(1, median));
  // Median of {1, 0, 0}
  EXPECT_FALSE(getBit(2, median));
  // Median of {0, 0, 0}
  EXPECT_FALSE(getBit(3, median));
}

TEST(ViwlsGraph, DescriptorMedianAbsDeviationTest) {
  DescriptorsType descriptors(48, 3);
  descriptors.setZero();

  descriptors(0, 0) = 7;
  descriptors(0, 1) = 3;
  descriptors(0, 2) = 1;

  EXPECT_EQ(descriptorMeanAbsoluteDeviation(descriptors),
            2.0 / 3.0);
}

TEST(ViwlsGraph, DescriptorMeanTestThreeDescriptors) {
  DescriptorsType descriptors(48, 3);
  descriptors.setZero();

  descriptors(0, 0) = 7;
  descriptors(0, 1) = 3;
  descriptors(0, 2) = 1;

  Eigen::Matrix<double, Eigen::Dynamic, 1> mean(48 * kBitsPerByte, 1);

  floatDescriptorMean(descriptors, &mean);
  // Mean of {1, 1, 1}
  EXPECT_DOUBLE_EQ(1.0, mean(0));
  // Mean of {1, 1, 0}
  EXPECT_DOUBLE_EQ(2.0 / 3.0, mean(1));
  // Mean of {1, 0, 0}
  EXPECT_DOUBLE_EQ(1.0 / 3.0, mean(2));
  // Mean of {0, 0, 0}
  EXPECT_DOUBLE_EQ(0.0, mean(3));
}

TEST(ViwlsGraph, DescriptorMeanStdDeviationTest) {
  DescriptorsType descriptors(48, 3);
  descriptors.setZero();

  descriptors(0, 0) = 7;
  descriptors(0, 1) = 3;
  descriptors(0, 2) = 1;

  EXPECT_DOUBLE_EQ(descriptorMeanStandardDeviation(descriptors),
                   2.0 / 3.0);
}

TEST(ViwlsGraph, DescriptorMeanStdDeviationTestZeroDescriptors) {
  DescriptorsType descriptors(48, 1);
  EXPECT_DOUBLE_EQ(descriptorMeanStandardDeviation(descriptors),
                   0.0);
}

TEST(ViwlsGraph, DescriptorMeanStdDeviationTestOneDescriptor) {
  DescriptorsType descriptors(48, 1);
  descriptors.setRandom();
  EXPECT_DOUBLE_EQ(descriptorMeanStandardDeviation(descriptors),
                   0.0);
}

TEST(ViwlsGraph, DescriptorMeanStdDeviationTestSameDescriptors) {
  DescriptorsType descriptors(48, 3);
  descriptors.setZero();

  descriptors(0, 0) = 56;
  descriptors(0, 1) = 56;
  descriptors(0, 2) = 56;

  EXPECT_DOUBLE_EQ(descriptorMeanStandardDeviation(descriptors),
                   0.0);
}

TEST(ViwlsGraph, DescriptorClosestToMedianTestZeroDescriptors) {
  DescriptorsType descriptors(48, 0);
  descriptors.setZero();

  size_t closest_to_median_descriptor_index = 0u;
  EXPECT_DEATH(
      getIndexOfDescriptorClosestToMedian(
          descriptors, &closest_to_median_descriptor_index), "");
}

TEST(ViwlsGraph, DescriptorClosestToMedianTestOneDescriptors) {
  DescriptorsType descriptors(48, 1);
  descriptors.setZero();

  descriptors(0, 0) = 7;

  size_t closest_to_median_descriptor_index = 0u;
  getIndexOfDescriptorClosestToMedian(
      descriptors, &closest_to_median_descriptor_index);

  EXPECT_EQ(0u, closest_to_median_descriptor_index);
}


TEST(ViwlsGraph, DescriptorClosestToMedianTestThreeDescriptors) {
  DescriptorsType descriptors(48, 3);
  descriptors.setZero();

  descriptors(0, 0) = 7;
  descriptors(0, 1) = 3;
  descriptors(0, 2) = 1;

  size_t closest_to_median_descriptor_index = 0u;
  getIndexOfDescriptorClosestToMedian(
      descriptors, &closest_to_median_descriptor_index);

  EXPECT_EQ(1u, closest_to_median_descriptor_index);

  descriptors(0, 1) = 255;

  getIndexOfDescriptorClosestToMedian(
      descriptors, &closest_to_median_descriptor_index);

  EXPECT_EQ(0u, closest_to_median_descriptor_index);

  descriptors(0, 2) = 127;

  getIndexOfDescriptorClosestToMedian(
      descriptors, &closest_to_median_descriptor_index);

  EXPECT_EQ(2u, closest_to_median_descriptor_index);
}

}  // namespace descriptor_utils
}  // namespace common
}  // namespace aslam

ASLAM_UNITTEST_ENTRYPOINT
