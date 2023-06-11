#include <algorithm>
#include <memory>
#include <vector>

#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>
#include <aslam/common/yaml-serialization.h>
#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "aslam/calibration/target-aprilgrid.h"
#include "aslam/calibration/target-observation.h"

namespace aslam {
namespace calibration {

TargetAprilGrid::TargetConfiguration
TargetAprilGrid::TargetConfiguration::fromYaml(const std::string& yaml_file) {
  TargetConfiguration target_config;
  try {
    const YAML::Node yaml_node = YAML::LoadFile(yaml_file.c_str());
    std::string target_type;
    YAML::safeGet(yaml_node, "target_type", &target_type);
    CHECK_EQ(target_type, "aprilgrid") << "Wrong target type.";
    YAML::safeGet(yaml_node, "tagCols", &target_config.num_tag_cols);
    CHECK_GT(target_config.num_tag_cols, 0u);
    YAML::safeGet(yaml_node, "tagRows", &target_config.num_tag_rows);
    CHECK_GT(target_config.num_tag_rows, 0u);
    YAML::safeGet(yaml_node, "tagSize", &target_config.tag_size_meter);
    CHECK_GT(target_config.tag_size_meter, 0.0);
    double relative_tag_spacing;
    YAML::safeGet(yaml_node, "tagSpacing", &relative_tag_spacing);
    target_config.tag_inbetween_space_meter =
        relative_tag_spacing * target_config.tag_size_meter;
    CHECK_GT(target_config.tag_inbetween_space_meter, 0.0);
  } catch (const YAML::Exception& ex) {
    LOG(FATAL) << "Failed to load yaml file " << yaml_file
               << " with the error: \n " << ex.what() << ".";
  }
  return target_config;
}

TargetAprilGrid::TargetAprilGrid(const TargetAprilGrid::TargetConfiguration& target_config)
    : TargetBase(2u * target_config.num_tag_rows,
                 2u * target_config.num_tag_cols, // 4 points per tag.
                 createAprilGridPoints(target_config)),
                 target_config_(target_config) {
  CHECK_GT(target_config.tag_size_meter, 0.0);
  CHECK_GT(target_config.tag_inbetween_space_meter, 0.0);
}

Eigen::Matrix3Xd createAprilGridPoints(
    const TargetAprilGrid::TargetConfiguration& target_config) {
  // Point ordering (e.g. 2x2 grid):
  //
  //          12-----13  14-----15
  //          | TAG 2 |  | TAG 3 |
  //          8-------9  10-----11
  //          4-------5  6-------7
  //    y     | TAG 0 |  | TAG 1 |
  //   ^      0-------1  2-------3
  //   |-->x
  const double tag_size = target_config.tag_size_meter;
  const double tag_spacing = target_config.tag_inbetween_space_meter;
  const size_t num_point_rows = 2u * target_config.num_tag_rows;
  const size_t num_point_cols = 2u * target_config.num_tag_cols;
  CHECK_GT(tag_size, 0.0);
  CHECK_GT(tag_spacing, 0.0);
  Eigen::Matrix3Xd grid_points_meters =
      Eigen::Matrix3Xd(3, num_point_rows * num_point_cols);
  for (size_t row_idx = 0u; row_idx < num_point_rows; ++row_idx) {
    for (size_t col_idx = 0u; col_idx < num_point_cols; ++col_idx) {
      Eigen::Vector3d point;
      point(0) = static_cast<double>(col_idx / 2u) * (tag_size + tag_spacing) +
                 static_cast<double>(col_idx % 2u) * tag_size;
      point(1) = static_cast<double>(row_idx / 2u) * (tag_size + tag_spacing) +
                 static_cast<double>(row_idx % 2u) * tag_size;
      point(2) = 0.0;

      grid_points_meters.col(row_idx * num_point_cols + col_idx) = point;
    }
  }
  return grid_points_meters;
}

DetectorAprilGrid::DetectorAprilGrid(
    const TargetAprilGrid::Ptr& target,
    const DetectorAprilGrid::DetectorConfiguration& detector_config)
    : target_(target),
      detector_config_(detector_config),
      tag_codes_(AprilTags::tagCodes36h11) {
  CHECK(target);
  tag_detector_.reset(
      new AprilTags::TagDetector(tag_codes_, target_->getConfig().black_tag_border_bits));
}

TargetObservation::Ptr DetectorAprilGrid::detectTargetInImage(const cv::Mat& image) const {
  // Detect all Apriltags in the image.
  std::vector<AprilTags::TagDetection> detections = tag_detector_->extractTags(image);

  // Remove bad tags.
  std::vector<AprilTags::TagDetection>::iterator iter = detections.begin();
  for (iter = detections.begin(); iter != detections.end();) {
    bool remove = false;

    // Enforce min. distance of corners to the image border (tag removed if violated).
    for (int tag_corner_idx = 0; tag_corner_idx < 4; ++tag_corner_idx) {
      remove |= iter->p[tag_corner_idx].first < detector_config_.min_border_distance_px;
      remove |= iter->p[tag_corner_idx].first >
        static_cast<double>(image.cols) - detector_config_.min_border_distance_px;
      remove |= iter->p[tag_corner_idx].second < detector_config_.min_border_distance_px;
      remove |= iter->p[tag_corner_idx].second >
        static_cast<double>(image.rows) - detector_config_.min_border_distance_px;
    }

    // Flag for removal if tag detection is marked as bad.
    if (iter->good != 1) {
      remove |= true;
    }

    // Flag for removal if the tag ID is out-of-range for this grid (faulty detection or wild tag).
    if (iter->id >= static_cast<int>(target_->size() / 4.0)) {
      remove |= true;
    }

    // Remove tag from the observation list.
    if (remove) {
      VLOG(200) << "Tag with ID " << iter->id << " is only partially in image (corners outside) "
                << "and will be removed from the TargetObservation.\n";

      // delete the tag and advance in list
      iter = detections.erase(iter);
    } else {
      //advance in list
      ++iter;
    }
  }

  // Check if enough tags have been found.
  if (detections.size() < detector_config_.min_visible_tags_for_valid_obs) {
    // Detection failed; return nullptr.
    return TargetObservation::Ptr();
  }

  // Sort detections by tagId.
  std::sort(detections.begin(), detections.end(), AprilTags::TagDetection::sortByIdCompare);

  // Check for duplicate tag ids that would indicate Apriltags not belonging to calibration target.
  if (detections.size() > 1) {
    for (size_t tag_idx = 0; tag_idx < detections.size() - 1; ++tag_idx)
      if (detections[tag_idx].id == detections[tag_idx + 1].id) {
        // Show image of duplicate Apriltag.
        cv::destroyAllWindows();
        cv::namedWindow("Wild Apriltag detected. Hide them!");
        cvStartWindowThread();

        cv::Mat image_copy = image.clone();
        cv::cvtColor(image_copy, image_copy, CV_GRAY2RGB);

        // Mark all duplicate tags in the image.
        for (size_t inner_tag_idx = 0; inner_tag_idx < detections.size() - 1; ++inner_tag_idx) {
          if (detections[inner_tag_idx].id == detections[inner_tag_idx + 1].id) {
            detections[inner_tag_idx].draw(image_copy);
            detections[inner_tag_idx + 1].draw(image_copy);
          }
        }

        cv::putText(image_copy, "Duplicate Apriltags detected. Hide them.", cv::Point(50, 50),
                    CV_FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255, 0, 0), 2, 8, false);
        cv::putText(image_copy, "Press enter to exit...", cv::Point(50, 80),
                    CV_FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255, 0, 0), 2, 8, false);
        cv::imshow("Duplicate Apriltags detected. Hide them", image_copy);
        cv::waitKey();

        LOG(WARNING) << "Found apriltag not belonging to calibration board. Check the image for "
                     << "the tag and hide it.";
        return TargetObservation::Ptr();
      }
  }

  // Convert corners to cv::Mat (4 consecutive corners form one tag).
  // point ordering here
  //          11-----10  15-----14
  //          | TAG 2 |  | TAG 3 |
  //          8-------9  12-----13
  //          3-------2  7-------6
  //    y     | TAG 0 |  | TAG 1 |
  //   ^      0-------1  4-------5
  //   |-->x
  cv::Mat tag_corners(4u * detections.size(), 2, CV_32F);
  for (size_t tag_idx = 0; tag_idx < detections.size(); tag_idx++) {
    for (size_t tag_corner_idx = 0; tag_corner_idx < 4; tag_corner_idx++) {
      tag_corners.at<float>(4u * tag_idx + tag_corner_idx, 0) =
          detections[tag_idx].p[tag_corner_idx].first;
      tag_corners.at<float>(4u * tag_idx + tag_corner_idx, 1) =
          detections[tag_idx].p[tag_corner_idx].second;
    }
  }

  // Store a copy of the corner list before subpix refinement.
  cv::Mat tag_corners_raw = tag_corners.clone();

  // Perform optional subpixel refinement on all tag corners (four corners each tag).
  if (detector_config_.run_subpixel_refinement) {
    cv::cornerSubPix(image, tag_corners, cv::Size(2, 2), cv::Size(-1, -1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
  }

  // Insert the observed points into the correct location of the grid point array.
  // point ordering
  //          12-----13  14-----15
  //          | TAG 2 |  | TAG 3 |
  //          8-------9  10-----11
  //          4-------5  6-------7
  //    y     | TAG 0 |  | TAG 1 |
  //   ^      0-------1  2-------3
  //   |-->x
  Eigen::VectorXi corner_ids(target_->size());
  Eigen::Matrix2Xd image_corners(2, target_->size());
  size_t out_point_idx = 0u;

  for (size_t tag_idx = 0; tag_idx < detections.size(); ++tag_idx) {
    const unsigned int tag_id = detections[tag_idx].id;

    // Calculate the grid idx for all four tag corners given the tagId and cols.
    const size_t cols = target_->cols();
    const unsigned int base_idx =
        tag_id * 2 + static_cast<int>(tag_id / (cols / 2)) * cols;
    unsigned int point_indices_tag[] = {base_idx,
                                        base_idx + 1,
                                        base_idx + static_cast<unsigned int>(cols + 1),
                                        base_idx + static_cast<unsigned int>(cols)};

    // Add four points per tag
    for (int tag_corner_idx = 0; tag_corner_idx < 4; tag_corner_idx++) {
      const Eigen::Vector2d corner_refined(
          tag_corners.row(4 * tag_idx + tag_corner_idx).at<float>(0),
          tag_corners.row(4 * tag_idx + tag_corner_idx).at<float>(1));
      const Eigen::Vector2d corner_raw(
          tag_corners_raw.row(4 * tag_idx + tag_corner_idx).at<float>(0),
          tag_corners_raw.row(4 * tag_idx + tag_corner_idx).at<float>(1));

      // Add corner points if it has not moved too far in the subpix refinement.
      const double subpix_displacement_squarred = (corner_refined - corner_raw).squaredNorm();
      if (subpix_displacement_squarred <= detector_config_.max_subpixel_refine_displacement_px_sq) {
        corner_ids(out_point_idx) = point_indices_tag[tag_corner_idx];
        image_corners.col(out_point_idx) = corner_refined;
        ++out_point_idx;
      }
    }
  }
  corner_ids.conservativeResize(out_point_idx);
  image_corners.conservativeResize(Eigen::NoChange, out_point_idx);

  return TargetObservation::Ptr(new TargetObservation(target_, image.rows, image.cols,
                                                      corner_ids, image_corners));
}

}  // namespace calibration
}  // namespace aslam
