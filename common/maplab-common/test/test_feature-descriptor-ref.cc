#include <glog/logging.h>
#include <gtest/gtest.h>

#include <aslam/common/feature-descriptor-ref.h>
#include <maplab-common/feature-descriptor-ref.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

static const int kDescriptorSize = 64;

TEST(MaplabCommon, BinaryFeatureDescriptor_ZeroInit) {
  aslam::common::FeatureDescriptorRef descr1(kDescriptorSize);
  descr1.SetZero();
  for (int i = 0; i < kDescriptorSize * 8; ++i) {
    EXPECT_FALSE(aslam::common::GetBit(i, descr1.ToConstRef()));
  }
}

TEST(MaplabCommon, BinaryFeatureDescriptor_Set) {
  aslam::common::FeatureDescriptorRef descr1(kDescriptorSize);
  descr1.SetZero();
  aslam::common::SetBit(5, &descr1);
  EXPECT_EQ(descr1[0], 1 << 5);

  aslam::common::SetBit(134, &descr1);
  EXPECT_EQ(descr1[16], 1 << 6);

  aslam::common::SetBit(410, &descr1);
  EXPECT_EQ(descr1[51], 1 << 2);
}

TEST(MaplabCommon, BinaryFeatureDescriptor_Get) {
  aslam::common::FeatureDescriptorRef descr1(kDescriptorSize);
  descr1.SetZero();

  descr1[0] = 1 << 5;
  EXPECT_TRUE(aslam::common::GetBit(5, descr1.ToConstRef()));

  descr1[16] = 1 << 6;
  EXPECT_TRUE(aslam::common::GetBit(134, descr1.ToConstRef()));

  descr1[51] = 1 << 2;
  EXPECT_TRUE(aslam::common::GetBit(410, descr1.ToConstRef()));
}

TEST(MaplabCommon, BinaryFeatureDescriptor_Flip) {
  aslam::common::FeatureDescriptorRef descr1(kDescriptorSize);
  descr1.SetRandom(10);

  bool bit = aslam::common::GetBit(5, descr1.ToConstRef());
  aslam::common::FlipBit(5u, &descr1);
  EXPECT_TRUE(aslam::common::GetBit(5, descr1.ToConstRef()) != bit);

  bit = aslam::common::GetBit(143, descr1.ToConstRef());
  aslam::common::FlipBit(143u, &descr1);
  EXPECT_TRUE(aslam::common::GetBit(143, descr1.ToConstRef()) != bit);

  bit = aslam::common::GetBit(410, descr1.ToConstRef());
  aslam::common::FlipBit(410u, &descr1);
  EXPECT_TRUE(aslam::common::GetBit(410, descr1.ToConstRef()) != bit);
}

TEST(MaplabCommon, BinaryFeatureDescriptor_GetNumberOfFlippedBits) {
  aslam::common::FeatureDescriptorRef descr1(kDescriptorSize);
  aslam::common::FeatureDescriptorRef descr2(kDescriptorSize);
  descr1.SetRandom(10);
  descr2 = descr1;

  aslam::common::FlipBit(5u, &descr1);
  EXPECT_EQ(1u, aslam::common::GetNumBitsDifferent(descr1, descr2));

  aslam::common::FlipBit(120u, &descr1);
  EXPECT_EQ(2u, aslam::common::GetNumBitsDifferent(descr1, descr2));

  aslam::common::FlipBit(341u, &descr1);
  EXPECT_EQ(3u, aslam::common::GetNumBitsDifferent(descr1, descr2));
}

TEST(MaplabCommon, BinaryFeatureDescriptor_RandomFlip) {
  aslam::common::FeatureDescriptorRef descr1(kDescriptorSize);
  aslam::common::FeatureDescriptorRef descr2(kDescriptorSize);
  aslam::common::FeatureDescriptorRef descr3(kDescriptorSize + 1);

  descr1.SetRandom(10);
  descr2 = descr1;
  descr3.SetRandom(10);

  ASSERT_DEATH(
      aslam::common::GetNumBitsDifferent(descr1, descr3),
      "Cannot compare descriptors of unequal size.");

  aslam::common::FlipNRandomBits(60u, &descr1);
  EXPECT_EQ(60u, aslam::common::GetNumBitsDifferent(descr1, descr2));

  size_t num_bits = kDescriptorSize * 8;
  ASSERT_DEATH(
      aslam::common::FlipNRandomBits(num_bits, &descr1),
      "Cannot flip more than everything.");

  ASSERT_DEATH(aslam::common::FlipNRandomBits(num_bits, nullptr), "");
}

MAPLAB_UNITTEST_ENTRYPOINT
