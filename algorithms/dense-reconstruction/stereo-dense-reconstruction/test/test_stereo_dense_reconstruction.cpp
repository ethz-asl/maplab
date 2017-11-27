#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <map-resources/resource-common.h>
#include <map-resources/resource-loader.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "dense-reconstruction/disparity-conversion-utils.h"
#include "dense-reconstruction/resource-utils.h"
#include "dense-reconstruction/stereo-camera-utils.h"
#include "dense-reconstruction/stereo-matcher.h"

namespace dense_reconstruction {
namespace stereo {

constexpr bool kDebugVisualize = false;
constexpr bool kRecomputeComparisonResults = false;

class StereoDenseReconstructionTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

  void initStereoMatcher() {
    if (config_.use_sgbm) {
      LOG(INFO) << "Stereo matching algorithm used: SGBM";
      stereo_matcher_ = cv::StereoSGBM::create(
          config_.sgbm_min_disparity, config_.sgbm_num_disparities,
          config_.sgbm_sad_window_size, config_.sgbm_p1, config_.sgbm_p2,
          config_.sgbm_disp12_max_diff, config_.sgbm_pre_filter_cap,
          config_.sgbm_uniqueness_ratio, config_.sgbm_speckle_window_size,
          config_.sgbm_speckle_range, config_.sgbm_mode);

      LOG(INFO) << " sgbm_pre_filter_cap: " << config_.sgbm_pre_filter_cap;
      LOG(INFO) << " sgbm_min_disparity: " << config_.sgbm_min_disparity;
      LOG(INFO) << " sgbm_p1: " << config_.sgbm_p1;
      LOG(INFO) << " sgbm_p2: " << config_.sgbm_p2;
      LOG(INFO) << " sgbm_uniqueness_ratio: " << config_.sgbm_uniqueness_ratio;
      LOG(INFO) << " sgbm_speckle_range: " << config_.sgbm_speckle_range;
      LOG(INFO) << " sgbm_speckle_window_size: "
                << config_.sgbm_speckle_window_size;
      LOG(INFO) << " sgbm_disp12_max_diff: " << config_.sgbm_disp12_max_diff;
      LOG(INFO) << " sgbm_num_disparities: " << config_.sgbm_num_disparities;
      LOG(INFO) << " sgbm_sad_window_size: " << config_.sgbm_sad_window_size;
      LOG(INFO) << " sgbm_mode: " << config_.sgbm_mode;

      min_disparity_ = config_.sgbm_min_disparity;
      num_disparities_ = config_.sgbm_num_disparities;
      sad_window_size_ = config_.sgbm_sad_window_size;

    } else {
      LOG(INFO) << "Stereo matching algorithm used: BM";
      stereo_matcher_ = cv::StereoBM::create(
          config_.bm_num_disparities, config_.bm_sad_window_size);

      cv::StereoBM* bm_ptr = static_cast<cv::StereoBM*>(stereo_matcher_.get());
      bm_ptr->setPreFilterCap(config_.bm_pre_filter_cap);
      bm_ptr->setPreFilterSize(config_.bm_pre_filter_size);
      bm_ptr->setMinDisparity(config_.bm_min_disparity);
      bm_ptr->setTextureThreshold(config_.bm_texture_threshold);
      bm_ptr->setUniquenessRatio(config_.bm_uniqueness_ratio);
      bm_ptr->setSpeckleRange(config_.bm_speckle_range);
      bm_ptr->setSpeckleWindowSize(config_.bm_speckle_window_size);
      bm_ptr->setDisp12MaxDiff(config_.bm_disp12_max_diff);

      LOG(INFO) << " bm_pre_filter_cap: " << config_.bm_pre_filter_cap;
      LOG(INFO) << " bm_pre_filter_size: " << config_.bm_pre_filter_size;
      LOG(INFO) << " bm_min_disparity: " << config_.bm_min_disparity;
      LOG(INFO) << " bm_texture_threshold: " << config_.bm_texture_threshold;
      LOG(INFO) << " bm_uniqueness_ratio: " << config_.bm_uniqueness_ratio;
      LOG(INFO) << " bm_speckle_range: " << config_.bm_speckle_range;
      LOG(INFO) << " bm_speckle_window_size: "
                << config_.bm_speckle_window_size;
      LOG(INFO) << " bm_disp12_max_diff: " << config_.bm_disp12_max_diff;
      LOG(INFO) << " bm_num_disparities: " << config_.bm_num_disparities;
      LOG(INFO) << " bm_sad_window_size: " << config_.bm_sad_window_size;

      min_disparity_ = config_.bm_min_disparity;
      num_disparities_ = config_.bm_num_disparities;
      sad_window_size_ = config_.bm_sad_window_size;
    }
  }

  void loadSolution(const std::string& name) {
    img_disparity_solution_ = cv::imread(
        "data/" + name + "_disparity_comparison.png", cv::IMREAD_UNCHANGED);
    CHECK(!img_disparity_solution_.empty());
    CHECK_EQ(img_disparity_solution_.type(), CV_16UC1);
  }

  void setupBikeStereoDataset() {
    resolution_ = cv::Size(2820, 1920);

    // Custom stereo matcher parameter for this dataset:
    config_.adaptParamsBasedOnImageSize(resolution_.width);

    T_C1_G_ = Eigen::Matrix4d::Identity();
    T_C2_G_ = Eigen::Matrix4d::Identity();

    D_left_ = std::vector<double>(5, 0.0);
    D_right_ = std::vector<double>(5, 0.0);

    K_left_ << 2826.171, 0, 1292.2, 0, 2826.171, 965.806, 0, 0, 1;
    K_right_ << 2826.171, 0, 1415.97, 0, 2826.171, 965.806, 0, 0, 1;

    cv::Mat img_left_color = cv::imread("data/img_left.png", cv::IMREAD_COLOR);
    CHECK(!img_left_color.empty());
    CHECK_EQ(img_left_color.type(), CV_8UC3);

    cv::cvtColor(img_left_color, img_left_, cv::COLOR_BGR2GRAY);
    CHECK(!img_left_.empty());
    CHECK_EQ(img_left_.type(), CV_8UC1);

    cv::Mat img_right_color =
        cv::imread("data/img_right.png", cv::IMREAD_COLOR);
    CHECK(!img_right_color.empty());
    CHECK_EQ(img_right_color.type(), CV_8UC3);

    cv::cvtColor(img_right_color, img_right_, cv::COLOR_BGR2GRAY);
    CHECK(!img_right_.empty());
    CHECK_EQ(img_right_.type(), CV_8UC1);

    if (!kRecomputeComparisonResults) {
      if (!kRecomputeComparisonResults) {
        loadSolution("bike");
      }
    }

    T_C2_G_(0, 3) = -0.178089;

    LOG(INFO) << "Baseline in x direction: " << T_C2_G_(0, 3);

    scale_ = config_.downscaling_factor;

    initStereoMatcher();
  }

  void setupKittiStereoDataset() {
    resolution_ = cv::Size(1392, 512);

    // Custom stereo matcher parameter for this dataset:
    config_.adaptParamsBasedOnImageSize(resolution_.width);

    T_C1_G_ = Eigen::Matrix4d::Identity();
    T_C2_G_ = Eigen::Matrix4d::Identity();

    D_left_ = std::vector<double>(5, 0.0);
    D_left_[0] = -3.728755e-01;
    D_left_[0] = 2.037299e-01;
    D_left_[0] = 2.219027e-03;
    D_left_[0] = 1.383707e-03;
    D_left_[0] = -7.233722e-02;
    D_right_ = std::vector<double>(5, 0.0);
    D_right_[0] = -3.644661e-01;
    D_right_[0] = 1.790019e-01;
    D_right_[0] = 1.148107e-03;
    D_right_[0] = -6.298563e-04;
    D_right_[0] = -5.314062e-02;

    K_left_ << 9.842439e+02, 0.000000e+00, 6.900000e+02, 0.000000e+00,
        9.808141e+02, 2.331966e+02, 0.000000e+00, 0.000000e+00, 1.000000e+00;
    K_right_ << 9.895267e+02, 0.000000e+00, 7.020000e+02, 0.000000e+00,
        9.878386e+02, 2.455590e+02, 0.000000e+00, 0.000000e+00, 1.000000e+00;

    img_left_ = cv::imread("data/kitti_img_left.png", cv::IMREAD_UNCHANGED);
    CHECK(!img_left_.empty());
    CHECK_EQ(img_left_.type(), CV_8UC1);

    img_right_ = cv::imread("data/kitti_img_right.png", cv::IMREAD_UNCHANGED);
    CHECK(!img_right_.empty());
    CHECK_EQ(img_right_.type(), CV_8UC1);

    if (!kRecomputeComparisonResults) {
      loadSolution("kitti");
    }

    Eigen::Matrix3d R_C2_G;  // = R01 from kitti
    R_C2_G << 9.993513e-01, 1.860866e-02, -3.083487e-02, -1.887662e-02,
        9.997863e-01, -8.421873e-03, 3.067156e-02, 8.998467e-03, 9.994890e-01;
    Eigen::Vector3d t_C2_G;  // = T01 from kitti
    t_C2_G << -5.370000e-01, 4.822061e-03, -1.252488e-02;
    Eigen::Matrix4d T_C2_G = Eigen::Matrix4d::Identity();
    T_C2_G.block<3, 3>(0, 0) = R_C2_G;
    T_C2_G.block<3, 1>(0, 3) = t_C2_G;
    T_C2_G_ = T_C2_G;

    LOG(INFO) << "Baseline in x direction: " << T_C2_G_(0, 3);

    scale_ = config_.downscaling_factor;

    initStereoMatcher();
  }

  void computeStereoReconstruction(
      const std::string& name, const size_t point_cloud_size) {
    StereoCameraParameters stereo_camera_parameters(scale_);

    stereo_camera_parameters.setInputCameraParameters(
        resolution_, T_C1_G_, K_left_, D_left_, DistortionModel::RADTAN,
        CameraSide::FIRST);
    stereo_camera_parameters.setInputCameraParameters(
        resolution_, T_C2_G_, K_right_, D_right_, DistortionModel::RADTAN,
        CameraSide::SECOND);

    Undistorter undistorter_left(stereo_camera_parameters.getFirst());
    Undistorter undistorter_right(stereo_camera_parameters.getSecond());

    cv::Mat img_left_undistorted;
    undistorter_left.undistortImage(img_left_, &img_left_undistorted);

    cv::Mat img_right_undistorted;
    undistorter_right.undistortImage(img_right_, &img_right_undistorted);

    cv::Mat img_disparity;
    stereo_matcher_->compute(
        img_left_undistorted, img_right_undistorted, img_disparity);
    CHECK_EQ(img_disparity.type(), CV_16SC1);

    if (kDebugVisualize) {
      if (!kRecomputeComparisonResults) {
        cv::namedWindow("solution", cv::WINDOW_NORMAL);
        cv::Mat color_map_disparity;
        generateColorMap(img_disparity_solution_, &color_map_disparity);
        cv::imshow("solution", color_map_disparity);
      }
      cv::namedWindow("left", cv::WINDOW_NORMAL);
      cv::imshow("left", img_left_undistorted);
      cv::namedWindow("right", cv::WINDOW_NORMAL);
      cv::imshow("right", img_right_undistorted);
      cv::namedWindow("computation", cv::WINDOW_NORMAL);
      cv::Mat color_map_disparity;
      generateColorMap(img_disparity, &color_map_disparity);
      cv::imshow("computation", color_map_disparity);
      cv::waitKey(0);
    }

    cv::Mat unsigned_disparity_map;
    img_disparity.convertTo(unsigned_disparity_map, CV_16UC1);
    CHECK_EQ(unsigned_disparity_map.type(), CV_16UC1);
    if (kRecomputeComparisonResults) {
      cv::imwrite(
          "data/" + name + "_disparity_comparison.png", unsigned_disparity_map);
    } else {
      EXPECT_TRUE(
          compareImages(unsigned_disparity_map, img_disparity_solution_));
    }

    const std::shared_ptr<OutputCameraParameters> left_params =
        undistorter_left.getCameraParametersPair().getOutputPtr();
    const std::shared_ptr<OutputCameraParameters> right_params =
        undistorter_right.getCameraParametersPair().getOutputPtr();

    const double focal_length = left_params->P()(0, 0);
    const double baseline = (right_params->P()(0, 3) - left_params->P()(0, 3)) /
                            left_params->P()(0, 0);
    ASSERT_NE(baseline, 0.0);
    const double cx_left = left_params->P()(0, 2);
    const double cx_right = right_params->P()(0, 2);
    CHECK_EQ(cx_left, cx_right)
        << "The undistortion should have made cx_left and cx_right identical!";
    const double cy = left_params->P()(1, 2);

    resources::PointCloud pointcloud;
    convertDisparityMapToPointCloud(
        img_disparity, img_left_undistorted, baseline, focal_length, cx_left,
        cy, sad_window_size_, min_disparity_, num_disparities_, &pointcloud);

    EXPECT_EQ(pointcloud.size(), point_cloud_size);

    common::deleteFile("data/" + name + ".ply");

    backend::ResourceLoader resource_loader;
    resource_loader.saveResourceToFile(
        "data/" + name + ".ply", backend::ResourceType::kPointCloudXYZRGBN,
        pointcloud);
  }

  bool compareImages(const cv::Mat& image_A, const cv::Mat& image_B) const {
    // NOTE(mfehr): This would be supported with opencv 3.3 img_hash module
    // cv::Ptr<cv::img_hash::ImgHashBase> hash_function =
    //     cv::img_hash::AverageHash::create();
    //
    // cv::Mat hash_A, hash_B;
    // hash_function->compute(image_A, hash_A);
    // hash_function->compute(image_B, hash_B);
    //
    // const double comparison_result = hash_function->compare(hash_A,
    // hash_B);
    //
    // LOG(INFO) << "AverageHash difference: " << comparison_result;

    CHECK_EQ(image_A.type(), image_B.type())
        << "Incompatible disparity map type!";
    CHECK_EQ(image_A.cols, image_B.cols) << "Incompatible disparity map width!";
    CHECK_EQ(image_A.rows, image_B.rows)
        << "Incompatible disparity map height!";
    cv::Mat difference = image_A == image_B;

    const int number_of_correct_pixels = cv::countNonZero(difference);

    return number_of_correct_pixels == (difference.rows * difference.cols);
  }

  cv::Size resolution_;
  Eigen::Matrix4d T_C1_G_;
  Eigen::Matrix4d T_C2_G_;
  Eigen::Matrix<double, 3, 3> K_left_;
  Eigen::Matrix<double, 3, 3> K_right_;
  std::vector<double> D_left_;
  std::vector<double> D_right_;
  DistortionModel distortion_model_;

  cv::Mat img_left_;
  cv::Mat img_right_;
  cv::Mat img_disparity_solution_;

  double scale_;

  StereoMatcherConfig config_;
  cv::Ptr<cv::StereoMatcher> stereo_matcher_;

  int min_disparity_;
  int num_disparities_;
  int sad_window_size_;
};

TEST_F(StereoDenseReconstructionTest, TestStereoDenseReconstructionBike) {
  setupBikeStereoDataset();
  computeStereoReconstruction("bike", 3618464u);
}

TEST_F(StereoDenseReconstructionTest, TestStereoDenseReconstructionKitti) {
  setupKittiStereoDataset();
  computeStereoReconstruction("kitti", 388692u);
}

}  // namespace stereo
}  // namespace dense_reconstruction

MAPLAB_UNITTEST_ENTRYPOINT
