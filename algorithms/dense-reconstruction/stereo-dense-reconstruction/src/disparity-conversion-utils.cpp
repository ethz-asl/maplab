#include "dense-reconstruction/disparity-conversion-utils.h"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace dense_reconstruction {
namespace stereo {

void convertDisparityMapToPointCloud(
    const cv::Mat& input_disparity, const cv::Mat& left_image,
    const double baseline, const double focal_length, const double cx,
    const double cy, const int sad_window_size, const int min_disparity,
    const int num_disparities, resources::PointCloud* pointcloud) {
  CHECK_NOTNULL(pointcloud);
  CHECK(pointcloud->empty());
  CHECK_GT(focal_length, 0.0);

  if (left_image.type() != CV_8U) {
    LOG(ERROR)
        << "Pointcloud generation is currently only supported on 8 bit images";
    return;
  }

  const int side_bound = sad_window_size / 2;

  const int max_size = input_disparity.rows * input_disparity.cols;

  pointcloud->xyz.reserve(3 * max_size);
  pointcloud->colors.reserve(3 * max_size);

  cv::Mat input_valid =
      cv::Mat(input_disparity.rows, input_disparity.cols, CV_8U);

  const int lower_x_border = side_bound + min_disparity + num_disparities;
  const int lower_y_border = side_bound;
  const int upper_x_border = input_disparity.cols - side_bound;
  const int upper_y_border = input_disparity.rows - side_bound;

  const int disparity_threshold = (min_disparity + num_disparities - 1) * 16;

  for (int y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
    for (int x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
      // The last check is because the sky has a bad habit of having a disparity
      // at just less than the max disparity.
      const int16_t disparity_value =
          input_disparity.at<int16_t>(y_pixels, x_pixels);
      if ((x_pixels < lower_x_border) || (y_pixels < lower_y_border) ||
          (x_pixels > upper_x_border) || (y_pixels > upper_y_border) ||
          (disparity_value < 0) || (disparity_value >= disparity_threshold)) {
        input_valid.at<uint8_t>(y_pixels, x_pixels) = 0;
      } else {
        input_valid.at<uint8_t>(y_pixels, x_pixels) = 1;
      }
    }
  }

  // Build pointcloud.
  for (int y_pixels = lower_y_border; y_pixels < upper_y_border; ++y_pixels) {
    for (int x_pixels = lower_x_border; x_pixels < upper_x_border; ++x_pixels) {
      const bool is_valid = input_valid.at<uint8_t>(y_pixels, x_pixels) > 0u;
      const int16_t input_value =
          input_disparity.at<int16_t>(y_pixels, x_pixels);

      double disparity_value;

      // If the filled disparity is valid it must be a freespace ray.
      if (is_valid) {
        disparity_value = static_cast<double>(input_value);
      } else {
        continue;
      }

      if (disparity_value < 1e-6) {
        continue;
      }

      // The 16x is needed as opencv stores disparity maps as 16 * the true
      // values.

      // NOTE(mfehr): This is the reprojection formula based on:
      // Source: http://answers.opencv.org/upfiles/13535653931527438.jpg
      // We undistort such that cx_left and cx_right are identical, therefore
      // the formula can be simplified to:
      // X = -b(x-Cx)/d
      // Y = -b(y - Cy) / d
      // Z = -(f*b)/d
      // with t = -b/d we get:
      // X = t * (x-Cx)
      // Y = t * (y - Cy)
      // Z = t * f

      const double t = -16.0 * baseline / disparity_value;
      const double z = t * focal_length;
      const double x = t * (x_pixels - cx);
      const double y = t * (y_pixels - cy);

      LOG_IF(WARNING, z < 0.0) << "Z is negative, which is weird!";

      uint8_t r, g, b;
      if (left_image.channels() == 3) {
        const cv::Vec3b& color = left_image.at<cv::Vec3b>(y_pixels, x_pixels);
        b = color[0];
        g = color[1];
        r = color[2];
      } else if (left_image.channels() == 4) {
        const cv::Vec4b& color = left_image.at<cv::Vec4b>(y_pixels, x_pixels);
        b = color[0];
        g = color[1];
        r = color[2];
      } else {
        b = left_image.at<uint8_t>(y_pixels, x_pixels);
        g = b;
        r = b;
      }

      pointcloud->xyz.push_back(x);
      pointcloud->xyz.push_back(y);
      pointcloud->xyz.push_back(z);

      pointcloud->colors.push_back(r);
      pointcloud->colors.push_back(g);
      pointcloud->colors.push_back(b);
    }
  }
  CHECK_LE(pointcloud->size(), 3 * max_size);
}

// Convert disparity map to a depth map in the target camera frame.
// NOTE(mfehr): This is definitely not the most efficient way to do this, but it
// uses the available infrastructure. If speed is an issue, this needs to
// be improved.
void convertDisparityMapToDepthMap(
    const cv::Mat& disparity_map, const cv::Mat& first_image_undistorted,
    const double baseline, const double focal_length, const double cx,
    const double cy, const int sad_window_size, const int min_disparity,
    const int num_disparities, const aslam::Camera& target_camera,
    cv::Mat* depth_map) {
  CHECK_NOTNULL(depth_map);
  CHECK(!disparity_map.empty());
  CHECK(!first_image_undistorted.empty());

  // Convert disparity to point cloud.
  resources::PointCloud point_cloud;
  convertDisparityMapToPointCloud(
      disparity_map, first_image_undistorted, baseline, focal_length, cx, cy,
      sad_window_size, min_disparity, num_disparities, &point_cloud);

  // Convert point cloud format to Eigen matrix.
  const size_t point_cloud_size = point_cloud.size();
  const size_t max_index = point_cloud_size * 3u;
  Eigen::Matrix3Xd p_C1_mat(3, point_cloud_size);
  size_t point_idx = 0u;
  for (size_t idx = 0u; idx < max_index; idx += 3u, ++point_idx) {
    CHECK_LT(point_idx, point_cloud_size);
    p_C1_mat.col(point_idx) = Eigen::Vector3d(
        point_cloud.xyz[idx], point_cloud.xyz[idx + 1u],
        point_cloud.xyz[idx + 2u]);
  }

  // Init depth map.
  const size_t height = target_camera.imageHeight();
  const size_t width = target_camera.imageWidth();
  *depth_map = cv::Mat(height, width, CV_16UC1, cv::Scalar(0u));

  // Project all 3D points into the image.
  Eigen::Matrix2Xd img_points_px;
  std::vector<aslam::ProjectionResult> projection_results;
  target_camera.project3Vectorized(
      p_C1_mat, &img_points_px, &projection_results);

  // Convert Z coordinate of points to millimeters.
  constexpr double kMetersToMillimeters = 1e3;

  // Statistics.
  for (point_idx = 0u; point_idx < point_cloud_size; ++point_idx) {
    if (!projection_results[point_idx].isKeypointVisible()) {
      continue;
    }

    const Eigen::Ref<const Eigen::Vector2d>& img_point =
        img_points_px.col(point_idx);

    const int depth_in_mm = std::min(
        static_cast<int>(std::numeric_limits<uint16_t>::max()),
        static_cast<int>(p_C1_mat(2, point_idx) * kMetersToMillimeters));
    CHECK_GE(depth_in_mm, 0);

    const int img_u = static_cast<int>(std::round(img_point[0]));
    const int img_v = static_cast<int>(std::round(img_point[1]));
    depth_map->at<uint16_t>(img_v, img_u) = static_cast<uint16_t>(depth_in_mm);
  }
}

}  // namespace stereo
}  // namespace dense_reconstruction
