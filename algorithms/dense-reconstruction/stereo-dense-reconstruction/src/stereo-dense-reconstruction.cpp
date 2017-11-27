#include "dense-reconstruction/stereo-dense-reconstruction.h"

#include <string>
#include <unordered_set>

#include <Eigen/Dense>
#include <aslam/cameras/camera.h>
#include <map-resources/resource-common.h>
#include <maplab-common/progress-bar.h>
#include <vi-map/sensor-manager.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

#include "dense-reconstruction/disparity-conversion-utils.h"
#include "dense-reconstruction/resource-utils.h"
#include "dense-reconstruction/stereo-matcher.h"
#include "dense-reconstruction/stereo-pair-detection.h"

DEFINE_bool(
    dense_stereo_images_and_result_in_ocv_windows, false,
    "Show opencv windows with the images and resulting disparity maps of the "
    "dense reconstruction.");

DEFINE_double(
    dense_stereo_debug_reconstruction_start_fraction_of_trajectory, 0.0,
    "DEBUGGING: This fraction determines the starting point of the "
    "reconstruction along the trajectory, e.g. 0.2 means that the stereo "
    "matcher will start at 20% of the trajectory.");

DEFINE_double(
    dense_stereo_debug_reconstruction_end_fraction_of_trajectory, 1.0,
    "DEBUGGING: This fraction determines the end point of the "
    "reconstruction along the trajectory, e.g. 0.8 means that the stereo "
    "matcher will stop after 80% of the trajectory.");

DEFINE_bool(
    dense_stereo_adapt_params_to_image_size, true,
    "If enabled, the stereo matcher params will be adapted to the image size. "
    "This affects the number of disparities and the p1/p2 parameter for the "
    "SGBM.");

namespace dense_reconstruction {
static std::unordered_set<backend::ResourceType, backend::ResourceTypeHash>
    kSupportedDepthTypes{backend::ResourceType::kRawDepthMap,
                         backend::ResourceType::kPointCloudXYZRGBN};

bool getSuitableGrayscaleImageForFrame(
    const vi_map::VIMap& vi_map, const vi_map::Vertex& vertex,
    const size_t frame_idx, cv::Mat* image) {
  bool found = false;
  if (vi_map.getFrameResource(
          vertex, frame_idx, backend::ResourceType::kRawImage, image)) {
    VLOG(5) << "Found raw grayscale image.";
    found = true;
  } else if (
      vi_map.getFrameResource(
          vertex, frame_idx, backend::ResourceType::kUndistortedImage, image)) {
    VLOG(5) << "Found undistorted grayscale image.";
    found = true;
  } else if (
      vi_map.getFrameResource(
          vertex, frame_idx, backend::ResourceType::kRectifiedImage, image)) {
    VLOG(5) << "Found rectified grayscale image.";
    found = true;
  } else if (
      vi_map.getFrameResource(
          vertex, frame_idx, backend::ResourceType::kImageForDepthMap, image)) {
    VLOG(5) << "Found grayscale image for a depth map.";
    found = true;
  }

  // If we found a grayscale image, return successfully.
  if (found) {
    CHECK_EQ(image->type(), CV_8UC1);
    return true;
  }

  // If no grayscale image has been found, try converting a color image.
  cv::Mat color_image;
  if (vi_map.getFrameResource(
          vertex, frame_idx, backend::ResourceType::kRawColorImage,
          &color_image)) {
    VLOG(5) << "Found raw color image.";
    found = true;
  } else if (
      vi_map.getFrameResource(
          vertex, frame_idx, backend::ResourceType::kUndistortedColorImage,
          &color_image)) {
    VLOG(5) << "Found undistorted color image.";
    found = true;
  } else if (
      vi_map.getFrameResource(
          vertex, frame_idx, backend::ResourceType::kRectifiedColorImage,
          &color_image)) {
    VLOG(5) << "Found rectified color image.";
    found = true;
  } else if (
      vi_map.getFrameResource(
          vertex, frame_idx, backend::ResourceType::kColorImageForDepthMap,
          &color_image)) {
    VLOG(5) << "Found color image for a depth map.";
    found = true;
  }

  // If we found a color image, convert and return successfully.
  if (found) {
    CHECK_EQ(color_image.type(), CV_8UC3);
    cv::cvtColor(color_image, *image, cv::COLOR_RGB2GRAY);
    CHECK_EQ(image->type(), CV_8UC1);
    return true;
  }

  return false;
}

void computeDepthForAllStereoCameras(
    const backend::ResourceType& depth_resource_type, vi_map::VIMap* vi_map) {
  CHECK_NOTNULL(vi_map);
  // With no mission selected, it will process all missions.
  vi_map::MissionIdList no_selected_mission_ids;
  computeDepthForAllStereoCameras(
      depth_resource_type, no_selected_mission_ids, vi_map);
}

void computeDepthForAllStereoCameras(
    const backend::ResourceType& depth_resource_type,
    const vi_map::MissionIdList& selected_mission_ids, vi_map::VIMap* vi_map) {
  CHECK_NOTNULL(vi_map);

  StereoPairsPerMissionMap stereo_camera_ids_per_mission;
  stereo::findAllStereoCameras(*vi_map, &stereo_camera_ids_per_mission);
  constexpr int kVerbosity = 1;
  stereo::printStereoCamerasPerMission(
      stereo_camera_ids_per_mission, kVerbosity);

  for (const StereoPairIdsPerMission& pair_per_mission :
       stereo_camera_ids_per_mission) {
    for (const StereoPairIdentifier& stereo_pair_identifier :
         pair_per_mission.second) {
      const vi_map::MissionId& mission_id = pair_per_mission.first;
      const aslam::CameraId& first_camera_id =
          stereo_pair_identifier.first_camera_id;
      const aslam::CameraId& second_camera_id =
          stereo_pair_identifier.second_camera_id;
      const aslam::Transformation T_C2_C1 = stereo_pair_identifier.T_C2_C1;

      if (!selected_mission_ids.empty()) {
        if (std::find(
                selected_mission_ids.begin(), selected_mission_ids.end(),
                mission_id) == selected_mission_ids.end()) {
          continue;
        }
      }

      computeDepthForStereoCamerasOfMission(
          first_camera_id, second_camera_id, T_C2_C1, mission_id,
          depth_resource_type, vi_map);
    }
  }
}

void computeDepthForStereoCamerasOfMission(
    const aslam::CameraId& first_camera_id,
    const aslam::CameraId& second_camera_id,
    const aslam::Transformation& T_C2_C1, const vi_map::MissionId& mission_id,
    const backend::ResourceType& depth_resource_type, vi_map::VIMap* vi_map) {
  CHECK_NOTNULL(vi_map);
  CHECK_GT(kSupportedDepthTypes.count(depth_resource_type), 0)
      << "This depth type is not supported! type: "
      << backend::ResourceTypeNames[static_cast<int>(depth_resource_type)];

  VLOG(1) << "Computing depth resources of type '"
          << backend::ResourceTypeNames[static_cast<int>(depth_resource_type)]
          << "' for stereo camera pair [" << first_camera_id << "/"
          << second_camera_id << "] of mission " << mission_id;

  const vi_map::SensorManager& sensor_manager = vi_map->getSensorManager();
  const aslam::NCamera& ncamera =
      sensor_manager.getNCameraForMission(mission_id);

  const size_t first_camera_idx = ncamera.getCameraIndex(first_camera_id);
  const aslam::Camera& first_camera = ncamera.getCamera(first_camera_idx);
  const size_t second_camera_idx = ncamera.getCameraIndex(second_camera_id);
  const aslam::Camera& second_camera = ncamera.getCamera(second_camera_idx);

  static const std::string kDisparityMapWindowName = "Disparity Map";
  static const std::string kFirstImageWindowName = "First Image";
  static const std::string kSecondImageWindowName = "Second Image";
  static const std::string kDepthMapWindowName = "Depth Map";

  if (FLAGS_dense_stereo_images_and_result_in_ocv_windows) {
    cv::namedWindow(kDisparityMapWindowName, cv::WINDOW_NORMAL);
    cv::namedWindow(kFirstImageWindowName, cv::WINDOW_NORMAL);
    cv::namedWindow(kSecondImageWindowName, cv::WINDOW_NORMAL);
    cv::namedWindow(kDepthMapWindowName, cv::WINDOW_NORMAL);
  }

  stereo::StereoMatcherConfig config =
      stereo::StereoMatcherConfig::getFromGflags();

  if (FLAGS_dense_stereo_adapt_params_to_image_size) {
    config.adaptParamsBasedOnImageSize(first_camera.imageWidth());
  }

  stereo::StereoMatcher matcher(first_camera, second_camera, T_C2_C1, config);

  pose_graph::VertexIdList all_vertices;
  vi_map->getAllVertexIdsInMissionAlongGraph(mission_id, &all_vertices);

  // Use these flags if you only want to reconstruct parts of the trajectory for
  // tuning or debugging purposes.
  const size_t start = static_cast<size_t>(
      FLAGS_dense_stereo_debug_reconstruction_start_fraction_of_trajectory *
      static_cast<double>(all_vertices.size()));
  const size_t end = static_cast<size_t>(
      FLAGS_dense_stereo_debug_reconstruction_end_fraction_of_trajectory *
      static_cast<double>(all_vertices.size()));
  size_t counter = 0u;

  common::ProgressBar progress_bar(all_vertices.size());
  progress_bar.update(start);
  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    if (counter < start) {
      ++counter;
      continue;
    } else if (counter > end) {
      break;
    }
    ++counter;

    vi_map::Vertex* vertex_ptr = vi_map->getVertexPtr(vertex_id);

    cv::Mat first_image, second_image;
    const bool has_first_image = getSuitableGrayscaleImageForFrame(
        *vi_map, *vertex_ptr, first_camera_idx, &first_image);
    const bool has_second_image = getSuitableGrayscaleImageForFrame(
        *vi_map, *vertex_ptr, second_camera_idx, &second_image);
    if (!(has_first_image && has_second_image)) {
      VLOG(3) << "Skipping vertex " << vertex_ptr->id()
              << " - no suitable image was found.";
      progress_bar.increment();
      continue;
    }

    VLOG(3) << "Computing disparity map for vertex " << vertex_ptr->id();

    CHECK(!first_image.empty());
    CHECK(!second_image.empty());
    CHECK_EQ(first_image.type(), CV_8UC1);
    CHECK_EQ(second_image.type(), CV_8UC1);
    CHECK_EQ(first_image.cols, second_image.cols);
    CHECK_EQ(first_image.rows, second_image.rows);

    if (FLAGS_dense_stereo_images_and_result_in_ocv_windows) {
      cv::imshow(kFirstImageWindowName, first_image);
      cv::imshow(kSecondImageWindowName, second_image);
    }

    cv::Mat disparity_map, first_image_rectified, second_image_rectified;
    matcher.computeDisparityMap(
        first_image, second_image, &disparity_map, &first_image_rectified,
        &second_image_rectified);

    if (FLAGS_dense_stereo_images_and_result_in_ocv_windows) {
      cv::Mat color_map_disparity;
      generateColorMap(disparity_map, &color_map_disparity);
      cv::imshow(kDisparityMapWindowName, color_map_disparity);
    }

    switch (depth_resource_type) {
      case backend::ResourceType::kPointCloudXYZRGBN: {
        resources::PointCloud point_cloud;
        stereo::convertDisparityMapToPointCloud(
            disparity_map, first_image_rectified, matcher.baseline(),
            matcher.focal_length(), matcher.cx(), matcher.cy(),
            matcher.sad_window_size(), matcher.min_disparity(),
            matcher.num_disparities(), &point_cloud);

        if (point_cloud.size() > 0) {
          storeFrameResourceWithOptionalOverwrite(
              point_cloud, first_camera_idx, depth_resource_type, vertex_ptr,
              vi_map);
        } else {
          VLOG(3) << "No 3D points reconstructed.";
        }

        break;
      }
      case backend::ResourceType::kRawDepthMap: {
        cv::Mat depth_map;
        stereo::convertDisparityMapToDepthMap(
            disparity_map, first_image_rectified, matcher.baseline(),
            matcher.focal_length(), matcher.cx(), matcher.cy(),
            matcher.sad_window_size(), matcher.min_disparity(),
            matcher.num_disparities(), first_camera, &depth_map);

        storeFrameResourceWithOptionalOverwrite(
            depth_map, first_camera_idx, depth_resource_type, vertex_ptr,
            vi_map);

        if (FLAGS_dense_stereo_images_and_result_in_ocv_windows) {
          cv::Mat color_map_depth;
          generateColorMap(depth_map, &color_map_depth);
          cv::imshow(kDepthMapWindowName, color_map_depth);
        }
        break;
      }
      default:
        LOG(FATAL)
            << "Resource type '"
            << backend::ResourceTypeNames[static_cast<int>(depth_resource_type)]
            << "' is not supported as output format of the stereo dense "
            << "reconstruction.";
    }

    if (FLAGS_dense_stereo_images_and_result_in_ocv_windows) {
      cv::waitKey(1);
    }
    progress_bar.increment();
  }

  if (FLAGS_dense_stereo_images_and_result_in_ocv_windows) {
    cv::destroyAllWindows();
  }
}

}  // namespace dense_reconstruction
