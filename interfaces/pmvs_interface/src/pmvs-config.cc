#include "dense-reconstruction/pmvs-config.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_bool(
    pmvs_use_color_images, true,
    "If true, PMVS will only use VisualFrames with color images, otherwise "
    "only the grayscale images.");
DEFINE_bool(
    cmvs_use_only_good_landmarks, true,
    "If enabled, only good landmarks are used to cluster the images.");
DEFINE_double(
    pmvs_undistortion_alpha, 0.0,
    "Determines how many invalid pixels are part of the undistorted "
    "image. [0 = none, 1 = all]");
DEFINE_double(pmvs_undistortion_scale, 1.0, "Scale of undistorted image.");
DEFINE_string(
    txt_output_suffix, "invalid", "Suffix for all metric file exports.");
DEFINE_string(
    txt_output_folder, "", "Export all metric output to this folder.");
DEFINE_string(
    pmvs_reconstruction_folder, "", "Output folder of the PMVS export.");

namespace dense_reconstruction {

PmvsConfig PmvsConfig::getFromGflags() {
  PmvsConfig settings;
  settings.cmvs_use_only_good_landmarks = FLAGS_cmvs_use_only_good_landmarks;
  settings.pmvs_use_color_images = FLAGS_pmvs_use_color_images;
  settings.pmvs_undistortion_alpha = FLAGS_pmvs_undistortion_alpha;
  settings.pmvs_undistortion_scale = FLAGS_pmvs_undistortion_scale;
  settings.pmvs_reconstruction_folder = FLAGS_pmvs_reconstruction_folder;
  settings.txt_output_folder = FLAGS_txt_output_folder;
  settings.txt_output_suffix = FLAGS_txt_output_suffix;
  return settings;
}

}  // namespace dense_reconstruction
