#ifndef DENSE_RECONSTRUCTION_PMVS_FILE_UTILS_H_
#define DENSE_RECONSTRUCTION_PMVS_FILE_UTILS_H_

#include <bitset>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <aslam/pipeline/undistorter-mapped.h>
#include <glog/logging.h>
#include <maplab-common/progress-bar.h>
#include <vi-map/unique-id.h>
#include <vi-map/vertex.h>

#include "dense-reconstruction/pmvs-common.h"
#include "dense-reconstruction/pmvs-config.h"

namespace dense_reconstruction {

bool exportAllImagesForCalibration(
    const std::string& export_folder, vi_map::VIMap* vi_map);

void createBundleFileForCmvs(
    const PmvsConfig& config, const std::string& folder_prefix,
    const unsigned int total_num_images, const unsigned int num_landmarks,
    const ObservedLandmarks& observed_landmarks);

void writeObserverPosesAndImagesToFileSystem(
    const vi_map::VIMap& vi_map, const PmvsConfig& config,
    const std::string& image_folder, const std::string& txt_folder,
    const ObserverCameraMap& observer_cameras,
    const ObserverPosesMap& observer_poses);

void createReconstructionFolders(
    const PmvsConfig& config, const std::string& reconstruction_folder,
    std::string* visualize_folder, std::string* txt_folder,
    std::string* models_folder);

}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_PMVS_FILE_UTILS_H_
