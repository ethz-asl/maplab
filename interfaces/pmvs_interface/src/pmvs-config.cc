#include "dense-reconstruction/pmvs-config.h"

#include <unordered_set>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_bool(
    cmvs_use_only_good_landmarks, true,
    "If enabled, only good landmarks are used to cluster the images.");
DEFINE_uint64(
    pmvs_export_data_for_nth_vertex, 1u,
    "Export the images and landmarks for each nth vertex. This is applied for "
    "each mission separately. Set to '1' by default.");
DEFINE_string(
    pmvs_image_format, "ppm",
    "Format used for the image export. Options: ppm (default), jpg.");
DEFINE_bool(
    pmvs_use_color_images, true,
    "If true, PMVS will only use VisualFrames with color images, otherwise "
    "only the grayscale images.");
DEFINE_double(
    pmvs_undistortion_alpha, 0.0,
    "Determines how many invalid pixels are part of the undistorted "
    "image. [0 = none, 1 = all]");
DEFINE_double(pmvs_undistortion_scale, 1.0, "Scale of undistorted image.");
DEFINE_string(
    pmvs_reconstruction_folder, "", "Output folder of the PMVS export.");

namespace dense_reconstruction {

PmvsConfig PmvsConfig::getFromGflags() {
  PmvsConfig settings;
  settings.use_only_good_landmarks = FLAGS_cmvs_use_only_good_landmarks;
  settings.export_data_for_nth_vertex = FLAGS_pmvs_export_data_for_nth_vertex;
  settings.image_file_name_string = "%s/%08d." + FLAGS_pmvs_image_format;
  settings.use_color_images = FLAGS_pmvs_use_color_images;
  settings.undistortion_alpha = FLAGS_pmvs_undistortion_alpha;
  settings.undistortion_scale = FLAGS_pmvs_undistortion_scale;
  settings.reconstruction_folder = FLAGS_pmvs_reconstruction_folder;
  return settings;
}

void checkPmvsConfig(const PmvsConfig& config) {
  CHECK_GT(config.export_data_for_nth_vertex, 0u);
  const std::unordered_set<std::string> kSupportedImageFormats = {
      "%s/%08d.ppm", "%s/%08d.jpg"};
  CHECK(
      kSupportedImageFormats.find(config.image_file_name_string) !=
      kSupportedImageFormats.cend());
  CHECK_GE(config.undistortion_alpha, 0.0);
  CHECK_LE(config.undistortion_alpha, 1.0);
  CHECK_GT(config.undistortion_scale, 0.0);
}

}  // namespace dense_reconstruction
