#ifndef DENSE_RECONSTRUCTION_PMVS_CONFIG_H_
#define DENSE_RECONSTRUCTION_PMVS_CONFIG_H_

#include <string>

#include <aslam/common/types.h>
#include <gflags/gflags.h>
#include <map-resources/resource-common.h>

namespace dense_reconstruction {

struct PmvsConfig {
  bool use_cmvs = true;

  const std::string kBundleName = "bundle.rd.out";
  const std::string kOptionFileNameString_ = "option-%04d";
  const std::string kCameraFileNameString_ = "%s/%08d.txt";
  const std::string kModelsFileNameString = "%smodels/%s";

  const std::string kVisualizeFolderName = "/visualize";
  const std::string kTxtFolderName = "/txt";
  const std::string kModelsFolderName = "/models";

  const int64_t kOptionalCameraResourceMatchingToleranceNs = 10 * 1e6;  // 10ms

  const aslam::InterpolationMethod kUndistortionInterpolationMethod =
      aslam::InterpolationMethod::Cubic;

  const backend::ResourceType supported_color_image_types[2] = {
      backend::ResourceType::kUndistortedColorImage,
      backend::ResourceType::kRawColorImage};
  const backend::ResourceType supported_grayscale_image_types[2] = {
      backend::ResourceType::kUndistortedImage,
      backend::ResourceType::kRawImage};

  bool use_only_good_landmarks = true;
  size_t export_data_for_nth_vertex = 1u;
  std::string image_file_name_string = "%s/%08d.ppm";
  bool use_color_images = true;
  double undistortion_alpha = 0.0;
  double undistortion_scale = 1.0;
  std::string reconstruction_folder = "";

  static PmvsConfig getFromGflags();
};

void checkPmvsConfig(const PmvsConfig& config);

}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_PMVS_CONFIG_H_
