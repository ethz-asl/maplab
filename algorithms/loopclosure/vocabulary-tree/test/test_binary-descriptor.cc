#include <loopclosure-common/types.h>
#include <maplab-common/feature-descriptor-ref.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "vocabulary-tree/distance.h"
#include "vocabulary-tree/types.h"

enum { kDescriptorSize = 64 };
TEST(VocabularyTree, BinaryFeatureDescriptor_Distance) {
  using loop_closure::DescriptorType;
  DescriptorType descr1(kDescriptorSize);
  descr1.SetZero();
  aslam::common::SetBit(5, &descr1);
  aslam::common::SetBit(6, &descr1);
  aslam::common::SetBit(7, &descr1);
  aslam::common::SetBit(8, &descr1);

  DescriptorType descr2(kDescriptorSize);
  descr2.SetZero();
  aslam::common::SetBit(1, &descr2);
  aslam::common::SetBit(2, &descr2);
  aslam::common::SetBit(3, &descr2);
  aslam::common::SetBit(4, &descr2);

  DescriptorType descr3(kDescriptorSize);
  descr3.SetZero();
  aslam::common::SetBit(1, &descr3);
  aslam::common::SetBit(2, &descr3);
  aslam::common::SetBit(3, &descr3);
  aslam::common::SetBit(4, &descr3);

  DescriptorType descr4(kDescriptorSize);
  descr4.SetZero();
  aslam::common::SetBit(9, &descr4);
  aslam::common::SetBit(9, &descr4);
  aslam::common::SetBit(3, &descr4);
  aslam::common::SetBit(4, &descr4);

  loop_closure::distance::Hamming<DescriptorType> hamming_distance;

  EXPECT_EQ(hamming_distance(descr1, descr2), static_cast<size_t>(8));
  EXPECT_EQ(hamming_distance(descr2, descr3), static_cast<size_t>(0));
  EXPECT_EQ(hamming_distance(descr2, descr4), static_cast<size_t>(3));
}

TEST(VocabularyTree, BinaryFeatureDescriptor_Mean) {
  using loop_closure::DescriptorType;
  DescriptorType descr1(kDescriptorSize);
  descr1.SetZero();
  aslam::common::SetBit(5, &descr1);
  aslam::common::SetBit(6, &descr1);
  aslam::common::SetBit(7, &descr1);
  aslam::common::SetBit(8, &descr1);

  DescriptorType descr2(kDescriptorSize);
  descr2.SetZero();
  aslam::common::SetBit(1, &descr2);
  aslam::common::SetBit(2, &descr2);
  aslam::common::SetBit(3, &descr2);
  aslam::common::SetBit(9, &descr2);

  DescriptorType descr3(kDescriptorSize);
  descr3.SetZero();
  aslam::common::SetBit(1, &descr3);
  aslam::common::SetBit(2, &descr3);
  aslam::common::SetBit(3, &descr3);
  aslam::common::SetBit(9, &descr3);

  std::vector<DescriptorType*> descriptors;
  descriptors.push_back(&descr1);
  descriptors.push_back(&descr2);
  descriptors.push_back(&descr3);

  DescriptorType descr_mean(kDescriptorSize);
  aslam::common::DescriptorMean(descriptors, &descr_mean);

  DescriptorType expected_mean(kDescriptorSize);
  expected_mean.SetZero();
  aslam::common::SetBit(1, &expected_mean);
  aslam::common::SetBit(2, &expected_mean);
  aslam::common::SetBit(3, &expected_mean);
  aslam::common::SetBit(9, &expected_mean);

  loop_closure::distance::Hamming<DescriptorType> hamming_distance;
  ASSERT_EQ(hamming_distance(descr_mean, expected_mean), 0u);
}

MAPLAB_UNITTEST_ENTRYPOINT
