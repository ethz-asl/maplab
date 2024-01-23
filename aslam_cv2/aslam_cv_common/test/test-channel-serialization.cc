#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include <aslam/common/entrypoint.h>
#include <aslam/common/opencv-predicates.h>
#include <aslam/common/channel-definitions.h>
#include <aslam/common/channel-serialization.h>

template <typename SCALAR>
class CvMatSerializationTest : public ::testing::Test {
 public:
  typedef SCALAR Scalar;
  static const int num_rows = 15;
  static const int num_cols = 20;
  static const int num_channels = 4;

  Scalar getValue(int row, int col, int channel) {
    return row + num_cols * col + num_cols * num_rows * channel;
  }

  void fill(cv::Mat * image) {
    CHECK_NOTNULL(image);
    for(int r = 0; r < image->rows; ++r) {
      for( int c = 0; c < image->cols; ++c) {
        for( int ch = 0; ch < image->channels(); ++ch) {
          (&image->at<Scalar>(r,c))[ch] = getValue(r, c, ch);
        }
      }
    }
  }

  virtual void SetUp() {
    for(int i = 0; i < num_channels; ++i) {
      int type = CV_MAKETYPE(cv::DataType<SCALAR>::type, i);
      // http://docs.opencv.org/modules/core/doc/basic_structures.html#mat-create
      // Create should only allocate if necessary.
      imagesA_[i].value_.create(num_rows, num_cols, type);
      fill(&(imagesA_[i].value_));
    }
  }
  aslam::channels::Channel<cv::Mat> imagesA_[num_channels];
  aslam::channels::Channel<cv::Mat> imagesB_[num_channels];
};

template <typename SCALAR>
class SimpleTypeTestHarness  {
 public:
  SimpleTypeTestHarness(SCALAR value) : value_(value) {
    channelA_.value_ = value;
   }

  void test() {
    char* buffer;
    size_t size = 0;
    channelA_.serializeToBuffer(&buffer, &size);
    SCALAR check = *(reinterpret_cast<SCALAR*>(buffer));
    ASSERT_EQ(size, sizeof(SCALAR));
    ASSERT_EQ(check, value_);
    std::string serialized_string;
    channelA_.serializeToString(&serialized_string);
    check = static_cast<SCALAR>(std::stod(serialized_string));
    ASSERT_EQ(check, value_);
    channelB_.deSerializeFromBuffer(buffer, sizeof(SCALAR));
    ASSERT_EQ(channelB_.value_, value_);
    channelB_.value_ = 0;
    channelB_.deSerializeFromString(serialized_string);
    ASSERT_EQ(channelB_.value_, value_);
  }

  aslam::channels::Channel<SCALAR> channelA_;
  aslam::channels::Channel<SCALAR> channelB_;
  SCALAR value_;
};

typedef ::testing::Types< int32_t, uint8_t,  int8_t, uint16_t, int16_t,
    float, double> TypeTests;
TYPED_TEST_CASE(CvMatSerializationTest, TypeTests);

template <typename TYPE>
class ChannelSerializationTest : public ::testing::Test {
 public:
  typedef TYPE MatrixType;
  typedef typename MatrixType::Scalar Scalar;
  static const int num_rows = 15;
  static const int num_cols = 20;

  virtual void SetUp() {
    if (MatrixType::RowsAtCompileTime == Eigen::Dynamic &&
        MatrixType::ColsAtCompileTime == Eigen::Dynamic) {
      value_a.value_.resize(num_rows, num_cols);
    } else if (MatrixType::RowsAtCompileTime == Eigen::Dynamic &&
        MatrixType::ColsAtCompileTime != Eigen::Dynamic) {
      value_a.value_.resize(num_rows, Eigen::NoChange);
    }  else if (MatrixType::RowsAtCompileTime != Eigen::Dynamic &&
        MatrixType::ColsAtCompileTime == Eigen::Dynamic) {
      value_a.value_.resize(Eigen::NoChange, num_cols);
    }
    value_a.value_.setRandom();
  }
  aslam::channels::Channel<MatrixType> value_a;
  aslam::channels::Channel<MatrixType> value_b;
};

#define MAKE_TYPE_LIST(Scalar) \
      Eigen::Matrix<Scalar, 20, 15>, \
      Eigen::Matrix<Scalar, Eigen::Dynamic, 10>, \
      Eigen::Matrix<Scalar, 25, Eigen::Dynamic>, \
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>

typedef ::testing::Types<MAKE_TYPE_LIST(double),
                         MAKE_TYPE_LIST(float),
                         MAKE_TYPE_LIST(int),
                         MAKE_TYPE_LIST(unsigned char)> DoubleTests;
TYPED_TEST_CASE(ChannelSerializationTest, DoubleTests);

TYPED_TEST(ChannelSerializationTest, SerializeDeserializeString) {
  typedef typename TypeParam::Scalar Scalar;
  aslam::internal::HeaderInformation header_info;
  std::string serialized_value;
  EXPECT_TRUE(this->value_a.serializeToString(&serialized_value));
  EXPECT_EQ(16u, header_info.size());
  ASSERT_EQ(header_info.size() + this->value_a.value_.rows() *
            this->value_a.value_.cols() *
            sizeof(typename TypeParam::Scalar), serialized_value.size());
  EXPECT_FALSE(EIGEN_MATRIX_NEAR(this->value_a.value_, this->value_b.value_, static_cast<Scalar>(1e-4)));
  EXPECT_TRUE(this->value_b.deSerializeFromString(serialized_value));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(this->value_a.value_, this->value_b.value_, static_cast<Scalar>(1e-4)));
}

TYPED_TEST(ChannelSerializationTest, SerializeDeserializeBuffer) {
  typedef typename TypeParam::Scalar Scalar;
  aslam::internal::HeaderInformation header_info;
  char* buffer;
  size_t size;
  EXPECT_TRUE(this->value_a.serializeToBuffer(&buffer, &size));
  EXPECT_EQ(16u, header_info.size());
  ASSERT_EQ(header_info.size() + this->value_a.value_.rows() *
            this->value_a.value_.cols() *
            sizeof(typename TypeParam::Scalar), size);
  EXPECT_FALSE(EIGEN_MATRIX_NEAR(this->value_a.value_, this->value_b.value_, static_cast<Scalar>(1e-4)));
  EXPECT_TRUE(this->value_b.deSerializeFromBuffer(buffer, size));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(this->value_a.value_, this->value_b.value_, static_cast<Scalar>(1e-4)));
}

TEST(MatrixVectorChannelSerialization, SerializeDeserializeString) {
  typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> DescriptorsT;
  aslam::channels::Channel<std::vector<DescriptorsT>> value_a;
  aslam::channels::Channel<std::vector<DescriptorsT>> value_b;

  for (size_t i = 0; i < 5; i++) {
    value_a.value_.emplace_back(i*2+10, i*3+5);
    value_a.value_[i].setRandom();
  }

  char* buffer;
  size_t size;
  EXPECT_TRUE(value_a.serializeToBuffer(&buffer, &size));
  EXPECT_TRUE(value_b.deSerializeFromBuffer(buffer, size));
  ASSERT_EQ(value_a.value_.size(), value_b.value_.size());
  for (size_t i = 0; i < value_a.value_.size(); i++) {
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
        value_a.value_[i], value_b.value_[i], static_cast<unsigned char>(1e-4)));
  }
}

TEST(MatrixVectorChannelSerialization, SerializeDeserializeBuffer) {
  typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> DescriptorsT;
  aslam::channels::Channel<std::vector<DescriptorsT>> value_a;
  aslam::channels::Channel<std::vector<DescriptorsT>> value_b;

  for (size_t i = 0; i < 5; i++) {
    value_a.value_.emplace_back(i*2+10, i*3+5);
    value_a.value_[i].setRandom();
  }

  std::string serialized_value;
  EXPECT_TRUE(value_a.serializeToString(&serialized_value));
  EXPECT_TRUE(value_b.deSerializeFromString(serialized_value));
  ASSERT_EQ(value_a.value_.size(), value_b.value_.size());
  for (size_t i = 0; i < value_a.value_.size(); i++) {
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
        value_a.value_[i], value_b.value_[i], static_cast<unsigned char>(1e-4)));
  }
}

TEST(ChannelSerialization, HeaderInfoSize) {
  aslam::internal::HeaderInformation header_info;
  header_info.cols = 12;
  header_info.rows = 10;
  header_info.depth = 4;
  header_info.channels = 13;
  EXPECT_EQ(16u, header_info.size());
  std::string header_serialized;
  header_serialized.resize(12u);
  header_info.serializeToBuffer(&header_serialized[0], 0);

  aslam::internal::HeaderInformation header_info2;
  header_info2.deSerializeFromBuffer(&header_serialized[0], 0);
  EXPECT_EQ(12u, header_info2.cols);
  EXPECT_EQ(10u, header_info2.rows);
  EXPECT_EQ(4u, header_info2.depth);
  ASSERT_EQ(13u, header_info2.channels);
}

TEST(ChannelSerialization, SerializeDeserializeNamedChannelFromString) {
  static const int numKeypoints = 5;
  aslam::channels::VISUAL_KEYPOINT_MEASUREMENTS keypoints_a;
  keypoints_a.value_.resize(Eigen::NoChange, numKeypoints);
  keypoints_a.value_.setRandom();
  std::string serialized_value;
  EXPECT_TRUE(keypoints_a.serializeToString(&serialized_value));

  aslam::internal::HeaderInformation header_info;
  ASSERT_EQ(serialized_value.size(), header_info.size() + 2 * numKeypoints *
            sizeof(aslam::channels::VISUAL_KEYPOINT_MEASUREMENTS::Type::Scalar));

  aslam::channels::VISUAL_KEYPOINT_MEASUREMENTS keypoints_b;

  EXPECT_FALSE(EIGEN_MATRIX_NEAR(keypoints_a.value_, keypoints_b.value_, 1e-4));
  EXPECT_TRUE(keypoints_b.deSerializeFromString(serialized_value));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(keypoints_a.value_, keypoints_b.value_, 1e-4));
}

TYPED_TEST(CvMatSerializationTest, SerializeDeserializeString) {
  for(int ch = 0; ch < this->num_channels; ++ch) {
    std::string serialized_value;
    EXPECT_TRUE(this->imagesA_[ch].serializeToString(&serialized_value));
    EXPECT_FALSE(gtest_catkin::ImagesEqual(this->imagesA_[ch].value_,
                                           this->imagesB_[ch].value_, 1e-4));
    EXPECT_TRUE(this->imagesB_[ch].deSerializeFromString(serialized_value));
    EXPECT_TRUE(gtest_catkin::ImagesEqual(this->imagesA_[ch].value_,
                                          this->imagesB_[ch].value_, 1e-4));
    typedef TypeParam Scalar;
    // Unit test the comparison function
    this->imagesB_[ch].value_.template at<Scalar>(1,1) = this->imagesB_[ch].value_.template at<Scalar>(1,1) + 1;
    EXPECT_FALSE(gtest_catkin::ImagesEqual(this->imagesA_[ch].value_,
                                          this->imagesB_[ch].value_, 1e-4));
  }
}

TYPED_TEST(CvMatSerializationTest, SerializeDeserializeBuffer) {
  for(int ch = 0; ch < this->num_channels; ++ch) {
    aslam::internal::HeaderInformation header_info;
    char* buffer;
    size_t size;
    EXPECT_TRUE(this->imagesA_[ch].serializeToBuffer(&buffer, &size));
    EXPECT_EQ(16u, header_info.size());
    ASSERT_EQ(header_info.size() + this->imagesA_[ch].value_.rows *
              this->imagesA_[ch].value_.cols * this->imagesA_[ch].value_.channels() *
              sizeof(TypeParam), size);
    EXPECT_FALSE(gtest_catkin::ImagesEqual(this->imagesA_[ch].value_,
                                           this->imagesB_[ch].value_, 1e-4));
    EXPECT_TRUE(this->imagesB_[ch].deSerializeFromBuffer(buffer, size));
    EXPECT_TRUE(gtest_catkin::ImagesEqual(this->imagesA_[ch].value_,
                                          this->imagesB_[ch].value_, 1e-4));
  }
}

TEST(SimpleSerializationTest, SerializeDeserializeSimpleTypes) {
  SimpleTypeTestHarness<int>(45678).test();
  SimpleTypeTestHarness<size_t>(10546548).test();
  SimpleTypeTestHarness<double>(0.457 * 1e12).test();
  SimpleTypeTestHarness<float> float_test(456.54578);
  SimpleTypeTestHarness<long> long_test(-9415);
  SimpleTypeTestHarness<long long> long_long_test(65465461321487);
}

ASLAM_UNITTEST_ENTRYPOINT
