#include <iostream>  // NOLINT
#include <string>

#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/distortion.h>
#include <glog/logging.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "map-resources/resource-common.h"
#include "map-resources/resource-conversion.h"

namespace backend {

const std::string kTestDataBaseFolder = "./map_resources_test_data/";
static constexpr size_t kNumValidDepthEntries = 10457u;

class ResourceConversionTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    depth_map_openni_ = cv::imread(
        kTestDataBaseFolder + "/depth_map_OpenNI.pgm", CV_LOAD_IMAGE_UNCHANGED);
    CHECK_EQ(CV_MAT_TYPE(depth_map_openni_.type()), CV_16U);

    image_ = cv::imread(
        kTestDataBaseFolder + "/intensities_OpenNI.pgm",
        CV_LOAD_IMAGE_GRAYSCALE);
    CHECK_EQ(CV_MAT_TYPE(image_.type()), CV_8UC1);

    fake_rgb_ = cv::Mat(image_.rows, image_.cols, CV_8UC3);
    std::vector<cv::Mat> array_to_merge(3, image_);
    cv::merge(array_to_merge, fake_rgb_);
    ASSERT_EQ(fake_rgb_.rows, image_.rows);
    ASSERT_EQ(fake_rgb_.cols, image_.cols);

    Eigen::VectorXd intrinsics(4);
    intrinsics << 256.4159254034679, 256.3049478392699, 327.0080939570888,
        227.8040711242157;

    Eigen::VectorXd distortion_params(1);
    distortion_params << 0.9297168325597613;

    camera_with_distortion_ =
        aslam::createCamera<aslam::PinholeCamera, aslam::FisheyeDistortion>(
            intrinsics, image_.cols, image_.rows, distortion_params);

    camera_without_distortion_ = aslam::createCamera<aslam::PinholeCamera>(
        intrinsics, image_.cols, image_.rows);
  }

  cv::Mat depth_map_openni_;
  cv::Mat image_;
  cv::Mat fake_rgb_;
  aslam::Camera::Ptr camera_with_distortion_;
  aslam::Camera::Ptr camera_without_distortion_;
};

TEST_F(ResourceConversionTest, TestStripCameraDistortion) {
  aslam::Camera::Ptr camera_without_distortion;
  createCameraWithoutDistortion(
      *camera_with_distortion_, &camera_without_distortion);
}

TEST_F(ResourceConversionTest, TestPointcloudNoImageConversion) {
  resources::PointCloud point_cloud;

  EXPECT_TRUE(
      convertDepthMapToPointCloud(
          depth_map_openni_, *camera_without_distortion_, &point_cloud));

  EXPECT_EQ(point_cloud.size(), kNumValidDepthEntries);
}

TEST_F(ResourceConversionTest, TestPointcloudWithImageConversion) {
  resources::PointCloud point_cloud;

  EXPECT_TRUE(
      convertDepthMapWithImageToPointCloud(
          depth_map_openni_, image_, *camera_without_distortion_,
          &point_cloud));
  EXPECT_EQ(point_cloud.size(), kNumValidDepthEntries);
}

TEST_F(ResourceConversionTest, TestPointcloudWithRgbImageConversion) {
  resources::PointCloud point_cloud;

  EXPECT_TRUE(
      convertDepthMapWithImageToPointCloud(
          depth_map_openni_, fake_rgb_, *camera_without_distortion_,
          &point_cloud));
  EXPECT_EQ(point_cloud.size(), kNumValidDepthEntries);
}

TEST_F(ResourceConversionTest, TestVoxbloxNoImageConversion) {
  pose::Position3DVector point_cloud;

  EXPECT_TRUE(
      convertDepthMapToPointCloud(
          depth_map_openni_, *camera_without_distortion_, &point_cloud));
  EXPECT_EQ(point_cloud.size(), kNumValidDepthEntries);
}

TEST_F(ResourceConversionTest, TestVoxbloxWithImageConversion) {
  pose::Position3DVector point_cloud;
  voxblox::Colors colors;

  EXPECT_TRUE(
      convertDepthMapWithImageToPointCloud(
          depth_map_openni_, image_, *camera_without_distortion_, &point_cloud,
          &colors));
  EXPECT_EQ(point_cloud.size(), kNumValidDepthEntries);
  EXPECT_EQ(colors.size(), kNumValidDepthEntries);
}

TEST_F(ResourceConversionTest, TestVoxbloxWithRgbImageConversion) {
  pose::Position3DVector point_cloud;
  voxblox::Colors colors;

  EXPECT_TRUE(
      convertDepthMapWithImageToPointCloud(
          depth_map_openni_, fake_rgb_, *camera_without_distortion_,
          &point_cloud, &colors));
  EXPECT_EQ(point_cloud.size(), kNumValidDepthEntries);
  EXPECT_EQ(colors.size(), kNumValidDepthEntries);
}

}  // namespace backend

MAPLAB_UNITTEST_ENTRYPOINT
