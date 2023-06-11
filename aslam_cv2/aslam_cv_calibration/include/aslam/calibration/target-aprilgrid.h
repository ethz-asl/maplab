#ifndef ASLAM_CALIBRATION_TARGET_APRILGRID_H
#define ASLAM_CALIBRATION_TARGET_APRILGRID_H

#include <memory>
#include <string>
#include <vector>

#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include "aslam/calibration/target-base.h"
#include "aslam/calibration/target-observation.h"

namespace aslam {
namespace calibration {

/// \class TargetAprilGrid
/// \brief Aprilgrid calibration target.
///        num_tag_rows:              number of tags in y-dir
///        num_tag_cols:              number of tags in x-dir
///        tag_size_meter:            size of a tag [m]
///        tag_inbetween_space_meter: space between tags [m]
///
///        Corner ordering :
///          12-----13  14-----15
///          | TAG 2 |  | TAG 3 |
///          8-------9  10-----11
///          4-------5  6-------7
///    y     | TAG 0 |  | TAG 1 |
///   ^      0-------1  2-------3
///   |-->x
class TargetAprilGrid : public TargetBase {
 public:
  ASLAM_POINTER_TYPEDEFS(TargetAprilGrid);

  struct TargetConfiguration {
    TargetConfiguration()
        : num_tag_rows(6u),
          num_tag_cols(6u),
          tag_size_meter(0.088),
          tag_inbetween_space_meter(0.0264),
          black_tag_border_bits(2u) {};
    /// Number of tags in row direction.
    size_t num_tag_rows;
    /// Number of tags in col direction.
    size_t num_tag_cols;

    /// Size of one tag. [m]
    double tag_size_meter;
    /// Space inbetween tags. [m]
    double tag_inbetween_space_meter;
    /// Size of black border around the tags [bits].
    size_t black_tag_border_bits;

    static TargetConfiguration fromYaml(const std::string& yaml_file);
  };

  TargetAprilGrid(const TargetConfiguration& target_config);
  virtual ~TargetAprilGrid() {};

  const TargetConfiguration& getConfig() const {
    return target_config_;
  }

 private:
  const TargetConfiguration target_config_;
};

Eigen::Matrix3Xd createAprilGridPoints(
    const TargetAprilGrid::TargetConfiguration& target_config);

class DetectorAprilGrid : public DetectorBase {
 public:
  ASLAM_POINTER_TYPEDEFS(DetectorAprilGrid);

  struct DetectorConfiguration {
    DetectorConfiguration()
        : run_subpixel_refinement(true),
          max_subpixel_refine_displacement_px_sq(1.5),
          min_visible_tags_for_valid_obs(4),
          min_border_distance_px(4.0) {};
    /// Perform subpixel refinement of extracted corners.
    bool run_subpixel_refinement;
    /// Max. displacement squared in subpixel refinement. [px^2]
    double max_subpixel_refine_displacement_px_sq;
    /// Min. number of tags for a valid observation.
    size_t min_visible_tags_for_valid_obs;
    /// Min. distance from image border for valid corners. [px]
    double min_border_distance_px;
  };

  DetectorAprilGrid(const TargetAprilGrid::Ptr& target,
                    const DetectorAprilGrid::DetectorConfiguration& detector_config);
  virtual ~DetectorAprilGrid() {};

  virtual TargetObservation::Ptr detectTargetInImage(const cv::Mat& image) const;

 private:
  const TargetAprilGrid::Ptr target_;
  const DetectorConfiguration detector_config_;

  AprilTags::TagCodes tag_codes_;
  std::unique_ptr<AprilTags::TagDetector> tag_detector_;
};

}  // namespace calibration
}  // namespace aslam

#endif  // ASLAM_CALIBRATION_TARGET_APRILGRID_H
