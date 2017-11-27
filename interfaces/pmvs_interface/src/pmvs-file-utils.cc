#include "dense-reconstruction/pmvs-file-utils.h"

#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <glog/logging.h>
#include <maplab-common/file-logger.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace dense_reconstruction {

void createBundleFileForCmvs(
    const PmvsConfig& config, const std::string& folder_prefix,
    const unsigned int total_num_images, const unsigned int num_landmarks,
    const ObservedLandmarks& observed_landmarks) {
  common::FileLogger bundle_file(folder_prefix + config.kBundleName);
  CHECK(bundle_file.isOpen()) << "Could not write to bundler file";

  bundle_file << "# Customized Bundle file" << std::endl;
  bundle_file << total_num_images << " " << num_landmarks << std::endl;

  // Fill in dummy camera intrinsics/extrinsics as they are not needed and not
  // parsed by CMVS.
  const std::string kDummyLine = "1.0 1.0 1.0\n";
  for (unsigned int i = 0u; i < total_num_images; ++i) {
    // Dummy camera parameter (focal length, radial distortion 1 and 2).
    bundle_file << kDummyLine;
    // Dummy camera rotation.
    bundle_file << kDummyLine;
    bundle_file << kDummyLine;
    bundle_file << kDummyLine;
    // Dummy camera translation.
    bundle_file << kDummyLine;
  }

  // Fill in landmarks data.
  const std::string kDummyColorLine = "255 255 255\n";
  const std::string kDummyLandmarkDetails = "1000 1.0 1.0";
  for (const ObservedLandmarks::value_type& observed_landmark_with_id :
       observed_landmarks) {
    const ObservedLandmark& observed_landmark =
        observed_landmark_with_id.second;
    bundle_file << observed_landmark.p_G[0] << " " << observed_landmark.p_G[1]
                << " " << observed_landmark.p_G[2] << std::endl;
    // Dummy landmark color (RGB).
    bundle_file << kDummyColorLine;
    bundle_file << observed_landmark.observer_pose_numbers.size();
    for (size_t observer_num : observed_landmark.observer_pose_numbers) {
      bundle_file << " " << observer_num << " ";
      // Dummy landmark detail (SIFT key, image coordinates x and y).
      bundle_file << kDummyLandmarkDetails;
    }
    bundle_file << std::endl;
  }
}

void writeObserverPosesAndImagesToFileSystem(
    const vi_map::VIMap& vi_map, const PmvsConfig& config,
    const std::string& image_folder, const std::string& txt_folder,
    const ObserverCameraMap& observer_cameras,
    const ObserverPosesMap& observer_poses) {
  for (const ObserverPosesMap::value_type& observer_pose_w_vertex_id :
       observer_poses) {
    const ObserverPoseSet& observer_pose_set = observer_pose_w_vertex_id.second;
    for (const ObserverPose& observer_pose : observer_pose_set) {
      const size_t observer_number = observer_pose.camera_number;
      char image_name[1024];
      snprintf(
          image_name, sizeof(image_name), config.kImageFileNameString_.c_str(),
          image_folder.c_str(), observer_number);

      cv::Mat image;
      observer_pose.loadImage(vi_map, &image);

      if (observer_pose.needsUndistortion()) {
        const ObserverCamera& observer_camera =
            common::getChecked(observer_cameras, observer_pose.camera_id);
        cv::Mat undistorted_image;
        observer_camera.undistortImage(image, &undistorted_image);
        image = undistorted_image;
      }

      cv::Mat color_image;
      if (image.channels() == 3 && image.type() == CV_8UC3) {
        color_image = image;
      } else {
        // PMVS expects color images, therefore we convert the grayscale image
        // to a pseudo color image.
        VLOG(2) << "Convert grayscale image to pseudo color image.";
        cv::cvtColor(image, color_image, CV_GRAY2RGB);
      }
      // Save to visualize folder.
      cv::imwrite(std::string(image_name), color_image);

      // Write camera projection matrix to txt folder.
      char camera_file_name_buffer[1024];
      snprintf(
          camera_file_name_buffer, sizeof(camera_file_name_buffer),
          config.kCameraFileNameString_.c_str(), txt_folder.c_str(),
          observer_number);
      std::string camera_file_name(camera_file_name_buffer);
      common::FileLogger camera_file(camera_file_name);
      CHECK(camera_file.isOpen())
          << "Could not write to camera projection matrix file: "
          << camera_file_name;
      camera_file << "CONTOUR" << std::endl;
      camera_file << observer_pose.P_undistorted;
    }
  }
}

void createReconstructionFolders(
    const PmvsConfig& config, const std::string& reconstruction_folder,
    std::string* visualize_folder, std::string* txt_folder,
    std::string* models_folder) {
  CHECK_NOTNULL(visualize_folder);
  CHECK_NOTNULL(txt_folder);
  CHECK_NOTNULL(models_folder);

  *visualize_folder = reconstruction_folder + config.kVisualizeFolderName;
  *txt_folder = reconstruction_folder + config.kTxtFolderName;
  *models_folder = reconstruction_folder + config.kModelsFolderName;

  // Create folders if necessary.
  if (!common::pathExists(reconstruction_folder)) {
    common::createPath(reconstruction_folder);
  }
  if (!common::pathExists(*visualize_folder)) {
    common::createPath(*visualize_folder);
  }
  if (!common::pathExists(*txt_folder)) {
    common::createPath(*txt_folder);
  }
  if (!common::pathExists(*models_folder)) {
    common::createPath(*models_folder);
  }
}

bool exportAllImagesForCalibration(
    const std::string& export_folder, vi_map::VIMap* vi_map) {
  CHECK_NOTNULL(vi_map);
  CHECK(!export_folder.empty());

  const backend::ResourceType cv_mat_resources_array[8] = {
      backend::ResourceType::kRawImage,
      backend::ResourceType::kUndistortedImage,
      backend::ResourceType::kRectifiedImage,
      backend::ResourceType::kImageForDepthMap,
      backend::ResourceType::kRawColorImage,
      backend::ResourceType::kUndistortedColorImage,
      backend::ResourceType::kRectifiedColorImage,
      backend::ResourceType::kColorImageForDepthMap};

  std::vector<backend::ResourceType> cv_mat_resource_types;
  cv_mat_resource_types.assign(
      cv_mat_resources_array, std::end(cv_mat_resources_array));

  backend::ResourceLoader resource_loader;

  vi_map::MissionIdList mission_ids;
  vi_map->getAllMissionIds(&mission_ids);
  for (const vi_map::MissionId& mission_id : mission_ids) {
    pose_graph::VertexIdList vertex_ids;
    vi_map->getAllVertexIdsInMissionAlongGraph(mission_id, &vertex_ids);

    for (const backend::ResourceType& resource_type : cv_mat_resource_types) {
      const std::string resource_type_folder =
          export_folder + "/" + mission_id.hexString() + "/" +
          backend::ResourceTypeNames[static_cast<int>(resource_type)] + "/";

      bool created_path = false;

      for (const pose_graph::VertexId& vertex_id : vertex_ids) {
        const vi_map::Vertex& vertex = vi_map->getVertex(vertex_id);
        const size_t num_frames = vertex.numFrames();

        for (size_t frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
          cv::Mat resource;
          if (!vi_map->getFrameResource(
                  vertex, frame_idx, resource_type, &resource)) {
            continue;
          }

          if (!created_path) {
            CHECK(common::createPath(resource_type_folder));
            created_path = true;
          }

          const int64_t frame_timestamp_ns =
              vertex.getVisualFrame(frame_idx).getTimestampNanoseconds();
          const std::string file_path =
              resource_type_folder + std::to_string(frame_timestamp_ns) +
              backend::ResourceTypeFileSuffix[static_cast<int>(resource_type)];

          resource_loader.saveResourceToFile(
              file_path, resource_type, resource);
        }
      }
    }
  }
  return true;
}

}  // namespace dense_reconstruction
