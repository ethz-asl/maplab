#ifndef FEATURE_TRACKING_GRIDDED_DETECTOR_H_
#define FEATURE_TRACKING_GRIDDED_DETECTOR_H_
#include <algorithm>
#include <array>
#include <vector>

#include <aslam/frames/visual-frame.h>
#include <aslam/tracker/tracking-helpers.h>
#include <maplab-common/parallel-process.h>
#include <maplab-common/threading-helpers.h>
#include <opencv2/core/version.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace feature_tracking {

// TODO(magehrig): Local non-maximum suppression only on octave level for
//                 improved scale invariance.
inline void localNonMaximumSuppression(
    size_t image_height, const float radius, const float ratio_threshold,
    std::vector<cv::KeyPoint>* keypoints) {
  CHECK_NOTNULL(keypoints);
  CHECK_GT(radius, 0.0f);
  CHECK_GT(ratio_threshold, 0.0f);
  CHECK_LE(ratio_threshold, 1.0f);
  CHECK_GT(image_height, 0u);

  if (keypoints->empty()) {
    return;
  }

  const float radius_sq = radius * radius;
  const size_t num_keypoints = keypoints->size();

  struct KeyPointData {
    KeyPointData(const cv::KeyPoint& keypoint, const size_t _keypoint_index)
        : coordinates{keypoint.pt.x, keypoint.pt.y},  // NOLINT
          response(keypoint.response),
          keypoint_index(_keypoint_index) {}
    std::array<float, 2> coordinates;
    float response;
    size_t keypoint_index;
  };

  typedef std::vector<KeyPointData>::const_iterator KeyPointDataIterator;

  std::function<bool(const KeyPointData&, KeyPointDataIterator)>  // NOLINT
      IsInsideCircle = [radius_sq](
          const KeyPointData& keypoint_1,
          KeyPointDataIterator keypoint_iterator_2) -> bool {
    const float x_diff =
        keypoint_1.coordinates[0] - keypoint_iterator_2->coordinates[0];
    const float y_diff =
        keypoint_1.coordinates[1] - keypoint_iterator_2->coordinates[1];
    return (x_diff * x_diff + y_diff * y_diff) < radius_sq;
  };

  std::function<size_t(const int, const int, const int)> Clamp =
      [](  // NOLINT
          const int lower, const int upper, const int in) -> size_t {
    return std::min<int>(std::max<int>(in, lower), upper);
  };

  std::vector<KeyPointData> keypoint_data_vector;
  keypoint_data_vector.reserve(num_keypoints);
  for (size_t i = 0u; i < num_keypoints; ++i) {
    keypoint_data_vector.emplace_back((*keypoints)[i], i);
  }

  // Create LUT of keypoints in y axis.
  std::sort(
      keypoint_data_vector.begin(), keypoint_data_vector.end(),
      [](const KeyPointData& lhs, const KeyPointData& rhs) -> bool {
        return lhs.coordinates[1] < rhs.coordinates[1];
      });

  std::vector<size_t> corner_row_LUT;
  corner_row_LUT.reserve(image_height);

  size_t num_kpts_below_y = 0u;
  for (size_t y = 0u; y < image_height; ++y) {
    while (num_kpts_below_y < num_keypoints &&
           y > keypoint_data_vector[num_kpts_below_y].coordinates[1]) {
      ++num_kpts_below_y;
    }
    corner_row_LUT.push_back(num_kpts_below_y);
  }
  CHECK_EQ(num_kpts_below_y, keypoint_data_vector.size());

  // Create a list of keypoints to reject.
  std::vector<bool> erase_keypoints(num_keypoints, false);

  for (size_t i = 0u; i < num_keypoints; ++i) {
    const KeyPointData& current_keypoint_data = keypoint_data_vector[i];
    const size_t y_top = Clamp(
        0, static_cast<int>(image_height - 1),
        std::floor(current_keypoint_data.coordinates[1] - radius));
    const size_t y_bottom = Clamp(
        0, static_cast<int>(image_height - 1),
        std::ceil(current_keypoint_data.coordinates[1] + radius));
    CHECK_LT(y_top, image_height);
    CHECK_LE(y_bottom, image_height);

    CHECK_LT(corner_row_LUT[y_top], keypoint_data_vector.size());
    CHECK_LE(corner_row_LUT[y_bottom], keypoint_data_vector.size());
    KeyPointDataIterator nearest_corners_begin =
        keypoint_data_vector.begin() + corner_row_LUT[y_top];
    KeyPointDataIterator nearest_corners_end =
        keypoint_data_vector.begin() + corner_row_LUT[y_bottom];

    for (KeyPointDataIterator it = nearest_corners_begin;
         it != nearest_corners_end; ++it) {
      if (it->keypoint_index == current_keypoint_data.keypoint_index ||
          erase_keypoints[it->keypoint_index] ||
          !IsInsideCircle(current_keypoint_data, it)) {
        continue;
      }
      const float response_threshold =
          ratio_threshold * current_keypoint_data.response;
      if (response_threshold > it->response) {
        erase_keypoints[it->keypoint_index] = true;
      }
    }
  }

  // Remove the flaged non-maximum keypoints.
  std::vector<bool>::iterator it_erase = erase_keypoints.begin();

  std::vector<cv::KeyPoint>::iterator it_erase_from = std::remove_if(
      keypoints->begin(), keypoints->end(),
      [&it_erase](const cv::KeyPoint & /*keypoint*/) -> bool {
        return *it_erase++ == true;
      });
  keypoints->erase(it_erase_from, keypoints->end());
}

inline void detectKeypointsGridded(
    const cv::Ptr<cv::FeatureDetector>& detector, const cv::Mat& image,
    const cv::Mat& detection_mask, const bool detector_use_nonmaxsuppression,
    const float detector_nonmaxsuppression_radius,
    const float detector_nonmaxsuppression_ratio_threshold,
    const size_t orb_detector_number_features, const size_t max_feature_count,
    const size_t gridded_detector_cell_num_features,
    const size_t gridded_detector_num_grid_cols,
    const size_t gridded_detector_num_grid_rows,
    const size_t gridded_detector_num_threads_per_image,
    std::vector<cv::KeyPoint>* keypoints) {
  CHECK_NOTNULL(keypoints)->clear();
  CHECK_GT(gridded_detector_num_grid_cols, 0u);
  CHECK_GT(gridded_detector_num_grid_rows, 0u);
  CHECK_GT(gridded_detector_cell_num_features, 0u);

  if (image.empty() ||
      max_feature_count <
          gridded_detector_num_grid_rows * gridded_detector_num_grid_cols) {
    keypoints->clear();
    return;
  }
  keypoints->reserve(orb_detector_number_features);

  std::mutex m_keypoints;
  auto detectFeaturesOfGridCells = [&](const std::vector<size_t>& range) {
    for (const int cell_idx : range) {
      const int celly = cell_idx / gridded_detector_num_grid_cols;
      const int cellx = cell_idx - celly * gridded_detector_num_grid_cols;

      const cv::Range row_range(
          (celly * image.rows) / gridded_detector_num_grid_rows,
          ((celly + 1) * image.rows) / gridded_detector_num_grid_rows);
      const cv::Range col_range(
          (cellx * image.cols) / gridded_detector_num_grid_cols,
          ((cellx + 1) * image.cols) / gridded_detector_num_grid_cols);

      cv::Mat sub_image = image(row_range, col_range);
      cv::Mat sub_mask;
      if (!detection_mask.empty()) {
        sub_mask = detection_mask(row_range, col_range);
      }

      std::vector<cv::KeyPoint> sub_keypoints;
      sub_keypoints.reserve(gridded_detector_cell_num_features);
      detector->detect(sub_image, sub_keypoints, sub_mask);

      std::vector<cv::KeyPoint>::iterator it = sub_keypoints.begin(),
                                          end = sub_keypoints.end();
      for (; it != end; ++it) {
        it->pt.x += col_range.start;
        it->pt.y += row_range.start;
      }

      if (detector_use_nonmaxsuppression) {
        localNonMaximumSuppression(
            image.rows, detector_nonmaxsuppression_radius,
            detector_nonmaxsuppression_ratio_threshold, &sub_keypoints);
      }

      std::unique_lock<std::mutex> lock(m_keypoints);
      keypoints->insert(
          keypoints->end(), sub_keypoints.begin(), sub_keypoints.end());
    }
  };

  size_t num_threads;
  if (gridded_detector_num_threads_per_image != 0)
    num_threads = gridded_detector_num_threads_per_image;
  else
    num_threads =
        gridded_detector_num_grid_cols * gridded_detector_num_grid_rows / 2;
  CHECK_GT(num_threads, 0u);
  common::ParallelProcess(
      gridded_detector_num_grid_cols * gridded_detector_num_grid_rows,
      detectFeaturesOfGridCells, /*kAlwaysParallelize=*/true, num_threads);
  cv::KeyPointsFilter::retainBest(*keypoints, max_feature_count);
}

}  // namespace feature_tracking

#endif  // FEATURE_TRACKING_GRIDDED_DETECTOR_H_
