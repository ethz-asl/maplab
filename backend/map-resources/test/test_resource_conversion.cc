#include <iostream>  // NOLINT
#include <string>

#include <aslam/cameras/camera-3d-lidar.h>
#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/distortion.h>
#include <aslam/common/timer.h>
#include <glog/logging.h>

#include <maplab-common/test/testing-entrypoint.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "map-resources/resource-common.h"
#include "map-resources/resource-conversion.h"

namespace backend {

static constexpr bool kEnableVisualization = false;

const std::string kTestDataBaseFolder = "./map_resources_test_data/";
static constexpr size_t kNumValidDepthEntries = 10457u;

class ResourceConversionTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    depth_map_openni_ = cv::imread(
        kTestDataBaseFolder + "/depth_map_OpenNI.pgm", cv::IMREAD_UNCHANGED);
    CHECK_EQ(CV_MAT_TYPE(depth_map_openni_.type()), CV_16U);

    image_ = cv::imread(
        kTestDataBaseFolder + "/intensities_depth_map.pgm",
        cv::IMREAD_GRAYSCALE);
    CHECK_EQ(CV_MAT_TYPE(image_.type()), CV_8UC1);
    CHECK_GT(image_.rows, 0);
    CHECK_GT(image_.cols, 0);

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

    ASSERT_TRUE(os_1_point_cloud_S_.loadFromFile(
        kTestDataBaseFolder + "/ouster_os_1_64_scan_2.ply"));
    ASSERT_TRUE(euroc_point_cloud_S_.loadFromFile(
        kTestDataBaseFolder + "/euroc_ground_truth_lidar.ply"));
    ASSERT_TRUE(os_1_point_cloud_S_.checkConsistency(true));
    ASSERT_TRUE(euroc_point_cloud_S_.checkConsistency(true));

    {  // Spherical camera.
      Eigen::VectorXd lidar_cam_intrinsics(4, 1);
      lidar_cam_intrinsics
          [aslam::Camera3DLidar::Parameters::kHorizontalResolutionRad] =
              2. * M_PI / 1024.;
      lidar_cam_intrinsics
          [aslam::Camera3DLidar::Parameters::kVerticalResolutionRad] =
              M_PI / 1023.;
      lidar_cam_intrinsics
          [aslam::Camera3DLidar::Parameters::kVerticalCenterRad] = M_PI / 2.;
      lidar_cam_intrinsics
          [aslam::Camera3DLidar::Parameters::kHorizontalCenterRad] = 0;
      spherical_camera_lidar_3d_ = aligned_shared<aslam::Camera3DLidar>(
          lidar_cam_intrinsics, 1024u /*width*/, 1024u /*height*/);
      aslam::CameraId id;
      aslam::generateId(&id);
      spherical_camera_lidar_3d_->setId(id);
    }

    {  // Lidar OS-1
      Eigen::VectorXd lidar_cam_intrinsics(4, 1);
      lidar_cam_intrinsics
          [aslam::Camera3DLidar::Parameters::kHorizontalResolutionRad] =
              2. * M_PI / 1024.;
      lidar_cam_intrinsics
          [aslam::Camera3DLidar::Parameters::kVerticalResolutionRad] =
              (0.289916642 * 2.) / 63.;
      lidar_cam_intrinsics
          [aslam::Camera3DLidar::Parameters::kVerticalCenterRad] =
              0.289916642;  // 16.611 deg
      lidar_cam_intrinsics
          [aslam::Camera3DLidar::Parameters::kHorizontalCenterRad] = 0;
      os_1_camera_lidar_3d_ = aligned_shared<aslam::Camera3DLidar>(
          lidar_cam_intrinsics, 1024u /*width*/, 64u /*height*/);
      aslam::CameraId id;
      aslam::generateId(&id);
      os_1_camera_lidar_3d_->setId(id);
    }

    // Rotation between OS-1 lidar frame and the associated camera frame.
    // clang-format off
    Eigen::Matrix4d T_C_S_mat;
    T_C_S_mat << 0.0, -1.0,  0.0,  0.0,
                 0.0,  0.0, -1.0,  0.0,
                 1.0,  0.0,  0.0,  0.0,
                 0.0,  0.0,  0.0,  1.0;
    // clang-format on

    T_C_S_ = aslam::Transformation(T_C_S_mat);
  }

  cv::Mat depth_map_openni_;
  cv::Mat image_;
  cv::Mat fake_rgb_;

  aslam::Camera::Ptr camera_with_distortion_;
  aslam::Camera::Ptr camera_without_distortion_;

  resources::PointCloud euroc_point_cloud_S_;
  resources::PointCloud os_1_point_cloud_S_;
  aslam::Transformation T_C_S_;
  aslam::Camera::Ptr spherical_camera_lidar_3d_;
  aslam::Camera::Ptr os_1_camera_lidar_3d_;
};

TEST_F(ResourceConversionTest, TestPointCloudToDepthMapConversionSpherical) {
  resources::PointCloud point_cloud_C = euroc_point_cloud_S_;
  ASSERT_TRUE(point_cloud_C.checkConsistency(true));
  point_cloud_C.applyTransformation(T_C_S_);
  ASSERT_TRUE(point_cloud_C.checkConsistency(true));
  point_cloud_C.removeInvalidPoints();

  cv::Mat range_image, image;
  for (size_t it = 0u; it < 10u; ++it) {
    timing::TimerImpl timer("convert_point_cloud_spherical");
    ASSERT_TRUE(backend::convertPointCloudToDepthMap(
        point_cloud_C, *spherical_camera_lidar_3d_, false /*use_openni_format*/,
        true /*create_range_image*/, &range_image, &image));
    timer.Stop();
  }
  LOG(INFO) << timing::Timing::Print();

  EXPECT_NEAR(static_cast<float>(cv::countNonZero(range_image)), 343461., 10.);

  if (!range_image.empty() && kEnableVisualization) {
    cv::namedWindow("depth");

    double min;
    double max;
    cv::minMaxIdx(range_image, &min, &max, 0, 0, range_image > 1e-6);
    max = std::min(10.0, max);
    cv::Mat scaled_range_image;
    float scale = 255 / (max - min);
    range_image.convertTo(scaled_range_image, CV_8UC1, scale, -min * scale);
    cv::Mat color_image;
    cv::applyColorMap(scaled_range_image, color_image, cv::COLORMAP_JET);
    color_image.setTo(cv::Scalar(0u, 0u, 0u), range_image < 1e-6);
    cv::imshow("depth", color_image);
  }

  if (!image.empty() && kEnableVisualization) {
    cv::namedWindow("intensity");
    cv::imshow("intensity", image);
  }

  if (kEnableVisualization) {
    cv::waitKey(0);
  }
}

TEST_F(ResourceConversionTest, TestPointCloudToDepthMapConversionOS1_OpenNI) {
  resources::PointCloud point_cloud_C = os_1_point_cloud_S_;
  ASSERT_TRUE(point_cloud_C.checkConsistency(true));
  point_cloud_C.applyTransformation(T_C_S_);
  ASSERT_TRUE(point_cloud_C.checkConsistency(true));
  point_cloud_C.removeInvalidPoints();

  cv::Mat range_image, image;
  for (size_t it = 0u; it < 10u; ++it) {
    timing::TimerImpl timer("convert_point_cloud_OS1_OpenNI");
    ASSERT_TRUE(backend::convertPointCloudToDepthMap(
        point_cloud_C, *os_1_camera_lidar_3d_, true /*use_openni_format*/,
        true /*create_range_image*/, &range_image, &image));
    timer.Stop();
  }
  LOG(INFO) << timing::Timing::Print();

  EXPECT_NEAR(static_cast<float>(cv::countNonZero(range_image)), 48049., 10.);

  if (!range_image.empty() && kEnableVisualization) {
    cv::namedWindow("depth");

    double min;
    double max;
    cv::minMaxIdx(range_image, &min, &max, 0, 0, range_image > 1e-6);
    max = std::min(10000., max);

    cv::Mat scaled_range_image;
    float scale = 255 / (max - min);
    range_image.convertTo(scaled_range_image, CV_8UC1, scale, -min * scale);
    cv::Mat color_image;
    cv::applyColorMap(scaled_range_image, color_image, cv::COLORMAP_JET);
    color_image.setTo(cv::Scalar(0u, 0u, 0u), range_image < 1e-6);
    cv::imshow("depth", color_image);
  }

  if (!image.empty() && kEnableVisualization) {
    cv::namedWindow("intensity");

    cv::imshow("intensity", image);
  }

  resources::PointCloud point_cloud_reprojected_S;
  backend::convertDepthMapToPointCloud(
      range_image, image, *os_1_camera_lidar_3d_, &point_cloud_reprojected_S);

  ASSERT_TRUE(point_cloud_reprojected_S.checkConsistency(true));
  point_cloud_reprojected_S.applyTransformation(T_C_S_.inverse());

  if (kEnableVisualization) {
    point_cloud_reprojected_S.writeToFile("/tmp/after.ply");
    os_1_point_cloud_S_.writeToFile("/tmp/before.ply");
  }

  if (kEnableVisualization) {
    cv::waitKey(0);
  }
}

TEST_F(ResourceConversionTest, TestPointCloudToDepthMapConversionOS1_Float) {
  resources::PointCloud point_cloud_C = os_1_point_cloud_S_;
  ASSERT_TRUE(point_cloud_C.checkConsistency(true));
  point_cloud_C.applyTransformation(T_C_S_);
  ASSERT_TRUE(point_cloud_C.checkConsistency(true));
  point_cloud_C.removeInvalidPoints();

  cv::Mat range_image, image;
  for (size_t it = 0u; it < 10u; ++it) {
    timing::TimerImpl timer("convert_point_cloud_OS1_Float");
    ASSERT_TRUE(backend::convertPointCloudToDepthMap(
        point_cloud_C, *os_1_camera_lidar_3d_, false /*use_openni_format*/,
        true /*create_range_image*/, &range_image, &image));
    timer.Stop();
  }
  LOG(INFO) << timing::Timing::Print();

  EXPECT_NEAR(static_cast<float>(cv::countNonZero(range_image)), 48049., 10.);

  if (!range_image.empty() && kEnableVisualization) {
    cv::namedWindow("depth");

    double min;
    double max;
    cv::minMaxIdx(range_image, &min, &max, 0, 0, range_image > 1e-6);
    max = std::min(10., max);

    cv::Mat scaled_range_image;
    float scale = 255 / (max - min);
    range_image.convertTo(scaled_range_image, CV_8UC1, scale, -min * scale);
    cv::Mat color_image;
    cv::applyColorMap(scaled_range_image, color_image, cv::COLORMAP_JET);
    color_image.setTo(cv::Scalar(0u, 0u, 0u), range_image < 1e-6);
    cv::imshow("depth", color_image);
  }

  if (!image.empty() && kEnableVisualization) {
    cv::namedWindow("intensity");
    cv::imshow("intensity", image);
  }

  if (kEnableVisualization) {
    cv::waitKey(0);
  }
}

TEST_F(ResourceConversionTest, TestStripCameraDistortion) {
  aslam::Camera::Ptr camera_without_distortion;
  createCameraWithoutDistortion(
      *camera_with_distortion_, &camera_without_distortion);
}

TEST_F(ResourceConversionTest, TestPointcloudNoImageConversion) {
  resources::PointCloud point_cloud;

  EXPECT_TRUE(convertDepthMapToPointCloud(
      depth_map_openni_, *camera_without_distortion_, &point_cloud));

  EXPECT_EQ(point_cloud.size(), kNumValidDepthEntries);
}

TEST_F(ResourceConversionTest, TestPointcloudWithImageConversion) {
  resources::PointCloud point_cloud;

  EXPECT_TRUE(convertDepthMapWithImageToPointCloud(
      depth_map_openni_, image_, *camera_without_distortion_, &point_cloud));
  EXPECT_EQ(point_cloud.size(), kNumValidDepthEntries);
}

TEST_F(ResourceConversionTest, TestPointcloudWithRgbImageConversion) {
  resources::PointCloud point_cloud;

  EXPECT_TRUE(convertDepthMapWithImageToPointCloud(
      depth_map_openni_, fake_rgb_, *camera_without_distortion_, &point_cloud));
  EXPECT_EQ(point_cloud.size(), kNumValidDepthEntries);
}

TEST_F(ResourceConversionTest, TestVoxbloxNoImageConversion) {
  voxblox::Pointcloud point_cloud;

  EXPECT_TRUE(convertDepthMapToPointCloud(
      depth_map_openni_, *camera_without_distortion_, &point_cloud));
  EXPECT_EQ(point_cloud.size(), kNumValidDepthEntries);
}

TEST_F(ResourceConversionTest, TestVoxbloxWithImageConversion) {
  voxblox::Pointcloud point_cloud;
  voxblox::Colors colors;

  EXPECT_TRUE(convertDepthMapWithImageToPointCloud(
      depth_map_openni_, image_, *camera_without_distortion_, &point_cloud,
      &colors));
  EXPECT_EQ(point_cloud.size(), kNumValidDepthEntries);
  EXPECT_EQ(colors.size(), kNumValidDepthEntries);
}

TEST_F(ResourceConversionTest, TestVoxbloxWithRgbImageConversion) {
  voxblox::Pointcloud point_cloud;
  voxblox::Colors colors;

  EXPECT_TRUE(convertDepthMapWithImageToPointCloud(
      depth_map_openni_, fake_rgb_, *camera_without_distortion_, &point_cloud,
      &colors));
  EXPECT_EQ(point_cloud.size(), kNumValidDepthEntries);
  EXPECT_EQ(colors.size(), kNumValidDepthEntries);
}

}  // namespace backend

MAPLAB_UNITTEST_ENTRYPOINT
