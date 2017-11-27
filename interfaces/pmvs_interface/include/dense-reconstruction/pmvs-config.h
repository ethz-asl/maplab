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
  const std::string kImageFileNameString_ = "%s/%08d.ppm";
  const std::string kCameraFileNameString_ = "%s/%08d.txt";
  const std::string kModelsFileNameString = "%smodels/%s";

  const std::string kTxtSuffix = ".txt";
  const std::string kObserverFileName = "observers.txt";

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

  bool cmvs_use_only_good_landmarks = true;
  bool pmvs_use_color_images = true;

  double pmvs_undistortion_alpha = 0.0;
  double pmvs_undistortion_scale = 1.0;

  std::string pmvs_reconstruction_folder = "";
  std::string txt_output_folder = "";
  std::string txt_output_suffix = "invalid";

  static PmvsConfig getFromGflags();
};

}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_PMVS_CONFIG_H_
