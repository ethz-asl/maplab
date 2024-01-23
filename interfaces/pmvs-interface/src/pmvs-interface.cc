#include "dense-reconstruction/pmvs-interface.h"

#include <map>
#include <stdlib.h>

#include <Eigen/Core>
#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/camera.h>
#include <aslam/common/timer.h>
#include <aslam/common/undistort-helpers.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <glog/logging.h>
#include <landmark-triangulation/pose-interpolator.h>
#include <maplab-common/accessors.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/progress-bar.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <vi-map/unique-id.h>

#include "dense-reconstruction/pmvs-config.h"
#include "dense-reconstruction/pmvs-file-utils.h"

namespace dense_reconstruction {

bool exportVIMapToPmvsSfmInputData(
    const PmvsConfig& pmvs_settings, const vi_map::VIMap& vi_map) {
  vi_map::MissionIdList mission_ids;
  vi_map.getAllMissionIds(&mission_ids);

  return exportVIMapToPmvsSfmInputData(pmvs_settings, mission_ids, vi_map);
}

bool exportVIMapToPmvsSfmInputData(
    const PmvsConfig& pmvs_settings, const vi_map::MissionIdList& mission_ids,
    const vi_map::VIMap& vi_map) {
  checkPmvsConfig(pmvs_settings);
  if (mission_ids.empty()) {
    LOG(ERROR) << "No missions found!";
    return true;
  }

  // Prepare reconstruction folders.
  const std::string pmvs_reconstruction_folder =
      pmvs_settings.reconstruction_folder + "/pmvs_" +
      common::generateDateStringFromCurrentTime() + "/";
  if (!common::pathExists(pmvs_reconstruction_folder)) {
    CHECK(common::createPath(pmvs_reconstruction_folder));
  }
  std::string image_folder, models_folder, txt_folder;
  createReconstructionFolders(
      pmvs_settings, pmvs_reconstruction_folder, &image_folder, &txt_folder,
      &models_folder);

  VLOG(1) << "Get all unique cameras...";
  ObserverCameraMap observer_cameras;
  getAllObserverCameras(vi_map, pmvs_settings, mission_ids, &observer_cameras);
  VLOG(1) << "Found " << observer_cameras.size() << " unique cameras in total.";

  if (observer_cameras.empty()) {
    LOG(ERROR) << "No cameras found!";
    return true;
  }

  VLOG(1) << "Get all VI map vertices whose resources will be exported...";
  pose_graph::VertexIdList export_vertex_ids;
  getExportVertexIds(vi_map, mission_ids, pmvs_settings, &export_vertex_ids);
  ObserverPosesMap observer_poses;
  size_t num_observers = 0u;

  VLOG(1) << "Get all images from optional camera resources...";
  getObserverPosesFromOptionalCameras(
      vi_map, pmvs_settings, export_vertex_ids, observer_cameras,
      &observer_poses, &num_observers);
  VLOG(1) << "Found " << num_observers << " images.";
  const size_t num_observer_after_opt_resources = num_observers;

  VLOG(1) << "Get all images from nframe camera resources...";
  getObserverPosesFromNCamera(
      vi_map, pmvs_settings, export_vertex_ids, observer_cameras,
      &observer_poses, &num_observers);
  VLOG(1) << "Found " << num_observers - num_observer_after_opt_resources
          << " images.";
  VLOG(1) << "Total: found " << num_observers << " images.";

  VLOG(1) << "Get covisibility information from landmarks...";
  ObservedLandmarks observed_landmarks;
  getObservedLandmarksAndCovisibilityInformation(
      vi_map, pmvs_settings, export_vertex_ids, observer_cameras,
      observer_poses, &observed_landmarks);
  VLOG(1) << "Found " << observed_landmarks.size() << " observable landmarks.";

  VLOG(2) << "Creating bundle file for CMVS...";
  const size_t num_landmarks = observed_landmarks.size();
  createBundleFileForCmvs(
      pmvs_settings, pmvs_reconstruction_folder, num_observers, num_landmarks,
      observed_landmarks);

  VLOG(1) << "Writing projection matrices and images to the file system...";
  writeObserverPosesAndImagesToFileSystem(
      vi_map, pmvs_settings, image_folder, txt_folder, observer_cameras,
      observer_poses);

  VLOG(1) << "Export successful!";

  LOG(INFO)
      << "\n"
      << "================================================================\n"
      << "Run the PMVS reconstruction using the cmvs_pmvs_catkin package\n"
      << "Github: https://github.com/ethz-asl/cmvs_pmvs_catkin\n"
      << "\n"
      << "rosrun cmvs_catkin cmvs " << pmvs_reconstruction_folder << " \n"
      << "rosrun pmvs_catkin genOption " << pmvs_reconstruction_folder << " \n"
      << "sh " << pmvs_reconstruction_folder << "pmvs.sh \n"
      << "================================================================\n";

  return true;
}

void getAllObserverCameras(
    const vi_map::VIMap& vi_map, const PmvsConfig& pmvs_settings,
    const vi_map::MissionIdList& mission_ids,
    ObserverCameraMap* observer_cameras) {
  CHECK_NOTNULL(observer_cameras)->clear();

  // Get all main cameras.
  constexpr bool kIsOptionalCamera = false;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    if (!vi_map.getMission(mission_id).hasNCamera()) {
      continue;
    }
    const aslam::SensorId ncamera_id =
        vi_map.getMission(mission_id).getNCameraId();
    const aslam::NCamera& ncamera =
        vi_map.getSensorManager().getSensor<aslam::NCamera>(ncamera_id);

    const size_t num_cameras = ncamera.getNumCameras();
    for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
      const aslam::Camera& camera = ncamera.getCamera(camera_idx);
      const aslam::CameraId& camera_id = camera.getId();
      CHECK(observer_cameras
                ->emplace(
                    camera_id, ObserverCamera(
                                   vi_map, pmvs_settings, mission_id, camera_id,
                                   camera_idx, kIsOptionalCamera))
                .second);
    }
  }

  // Get all optional cameras.
  for (const vi_map::MissionId& mission_id : mission_ids) {
    backend::SensorIdToResourcesMap camera_to_img_resources;

    const backend::SensorIdToResourcesMap& sensor_to_color_img_resources =
        *vi_map.getMission(mission_id)
             .getAllSensorResourceIdsOfType(
                 backend::ResourceType::kRawColorImage);
    const backend::SensorIdToResourcesMap& sensor_to_grayscale_img_resources =
        *vi_map.getMission(mission_id)
             .getAllSensorResourceIdsOfType(backend::ResourceType::kRawImage);
    camera_to_img_resources.insert(
        sensor_to_color_img_resources.begin(),
        sensor_to_color_img_resources.begin());
    camera_to_img_resources.insert(
        sensor_to_grayscale_img_resources.begin(),
        sensor_to_grayscale_img_resources.begin());

    for (const backend::SensorIdToResourcesMap::value_type&
             resources_per_sensor : camera_to_img_resources) {
      const aslam::SensorId& sensor_id = resources_per_sensor.first;
      if (vi_map.getSensorManager().getSensorType(sensor_id) !=
          vi_map::SensorType::kCamera) {
        LOG(WARNING)
            << "This VIMap has images associated with another sensor than a "
            << "camera, we cannot process them in the PMVS-interface!";
        continue;
      }

      observer_cameras->emplace(
          sensor_id,
          ObserverCamera(vi_map, pmvs_settings, mission_id, sensor_id));
    }
  }
}

void getExportVertexIds(
    const vi_map::VIMap& vi_map, const vi_map::MissionIdList& mission_ids,
    const PmvsConfig& pmvs_settings,
    pose_graph::VertexIdList* export_vertex_ids) {
  CHECK(!mission_ids.empty());
  CHECK_GT(pmvs_settings.export_data_for_nth_vertex, 0u);
  CHECK_NOTNULL(export_vertex_ids)->clear();
  size_t total_num_vertices = 0u;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    pose_graph::VertexIdList mission_vertex_ids;
    vi_map.getAllVertexIdsInMissionAlongGraph(mission_id, &mission_vertex_ids);
    CHECK(!mission_vertex_ids.empty());
    total_num_vertices += mission_vertex_ids.size();
    size_t vertex_counter = 0u;
    for (const pose_graph::VertexId& vertex_id : mission_vertex_ids) {
      if (vertex_counter++ % pmvs_settings.export_data_for_nth_vertex != 0u) {
        continue;
      }
      export_vertex_ids->push_back(vertex_id);
    }
  }
  VLOG(1) << "Using images and landmarks of " << export_vertex_ids->size()
          << "/" << total_num_vertices << " vertices.";
}

void getObserverPosesFromOptionalCameras(
    const vi_map::VIMap& vi_map, const PmvsConfig& pmvs_settings,
    const pose_graph::VertexIdList& export_vertex_ids,
    const ObserverCameraMap& observer_camera_map,
    ObserverPosesMap* observer_poses, size_t* number_of_observers) {
  CHECK(!export_vertex_ids.empty());
  CHECK_NOTNULL(observer_poses);
  CHECK_NOTNULL(number_of_observers);

  std::vector<backend::ResourceType> supported_image_types;
  if (pmvs_settings.use_color_images) {
    supported_image_types.assign(
        pmvs_settings.supported_color_image_types,
        std::end(pmvs_settings.supported_color_image_types));
  } else {
    supported_image_types.assign(
        pmvs_settings.supported_grayscale_image_types,
        std::end(pmvs_settings.supported_grayscale_image_types));
  }

  for (const pose_graph::VertexId& vertex_id : export_vertex_ids) {
    const vi_map::Vertex& vertex = vi_map.getVertex(vertex_id);
    const int64_t vertex_timestamp_ns = vertex.getMinTimestampNanoseconds();
    const vi_map::VIMission& mission = vi_map.getMissionForVertex(vertex_id);

    // Loop over all supported resource types and try to find a optional
    // camera resource that fits.
    aslam::CameraIdSet optional_cameras;
    for (const backend::ResourceType& resource_type : supported_image_types) {
      aslam::CameraIdList opt_camera_ids;
      std::vector<int64_t> closest_timestamps_ns;
      mission.findAllCloseSensorResources(
          resource_type, vertex_timestamp_ns,
          pmvs_settings.kOptionalCameraResourceMatchingToleranceNs,
          &opt_camera_ids, &closest_timestamps_ns);
      const size_t num_opt_cameras = opt_camera_ids.size();
      CHECK_EQ(num_opt_cameras, closest_timestamps_ns.size());
      for (size_t idx = 0u; idx < num_opt_cameras; ++idx) {
        const aslam::CameraId& optional_camera_id = opt_camera_ids[idx];
        // If we already have a resource of this camera we discard all
        // subsequent resource types, because the supported image types are
        // sorted by priority, i.e. if we found an undistorted color image
        // there is no need to use the distorted image of the same camera.
        if (optional_cameras.count(optional_camera_id) > 0u) {
          continue;
        }

        const int64_t observer_timestamp_ns = closest_timestamps_ns[idx];
        const size_t observer_number = (*number_of_observers)++;
        const ObserverCamera& observer_camera =
            common::getChecked(observer_camera_map, optional_camera_id);

        ObserverPoseSet& observer_pose_set = (*observer_poses)[vertex_id];

        observer_pose_set.emplace(
            vi_map, observer_number, mission.id(), vertex_id,
            observer_timestamp_ns, observer_camera, resource_type);

        optional_cameras.insert(optional_camera_id);
      }
    }
  }
}

void getObserverPosesFromNCamera(
    const vi_map::VIMap& vi_map, const PmvsConfig& pmvs_settings,
    const pose_graph::VertexIdList& export_vertex_ids,
    const ObserverCameraMap& observer_camera_map,
    ObserverPosesMap* observer_poses, size_t* number_of_observers) {
  CHECK(!export_vertex_ids.empty());
  CHECK_NOTNULL(observer_poses);
  CHECK_NOTNULL(number_of_observers);

  std::vector<backend::ResourceType> supported_image_types;
  if (pmvs_settings.use_color_images) {
    supported_image_types.assign(
        pmvs_settings.supported_color_image_types,
        std::end(pmvs_settings.supported_color_image_types));
  } else {
    supported_image_types.assign(
        pmvs_settings.supported_grayscale_image_types,
        std::end(pmvs_settings.supported_grayscale_image_types));
  }

  for (const pose_graph::VertexId& vertex_id : export_vertex_ids) {
    const vi_map::Vertex& vertex = vi_map.getVertex(vertex_id);
    const size_t num_frames = vertex.numFrames();
    const vi_map::MissionId& mission_id =
        vi_map.getMissionIdForVertex(vertex_id);

    for (size_t frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
      const aslam::VisualFrame frame = vertex.getVisualFrame(frame_idx);
      const int64_t frame_timestamp_ns = frame.getTimestampNanoseconds();
      const aslam::CameraId& nframe_camera_id =
          vertex.getCamera(frame_idx)->getId();

      // Loop over all supported resource types and try to find a nframe
      // camera resource that fits.
      aslam::CameraIdSet nframe_cameras;
      for (const backend::ResourceType& resource_type : supported_image_types) {
        if (!vi_map.hasFrameResource(vertex, frame_idx, resource_type)) {
          continue;
        }

        // If we already have a resource of this camera we discard all
        // subsequent resource types, because the supported image types are
        // sorted by priority, i.e. if we found an undistorted color image
        // there is no need to use the distorted image of the same camera.
        if (nframe_cameras.count(nframe_camera_id) > 0u) {
          continue;
        }

        const size_t observer_number = (*number_of_observers)++;
        const ObserverCamera& observer_camera =
            common::getChecked(observer_camera_map, nframe_camera_id);

        ObserverPoseSet& observer_pose_set = (*observer_poses)[vertex_id];

        observer_pose_set.emplace(
            vi_map, observer_number, mission_id, vertex_id, frame_timestamp_ns,
            observer_camera, resource_type);

        nframe_cameras.insert(nframe_camera_id);
      }
    }
  }
}

void getObservedLandmarksAndCovisibilityInformation(
    const vi_map::VIMap& vi_map, const PmvsConfig& config,
    const pose_graph::VertexIdList& export_vertex_ids,
    const ObserverCameraMap& observer_cameras,
    const ObserverPosesMap& observer_poses,
    ObservedLandmarks* observed_landmarks) {
  CHECK(!export_vertex_ids.empty());
  CHECK_NOTNULL(observed_landmarks);

  size_t landmark_number = 0u;
  vi_map::LandmarkIdSet actually_observed_landmarks_ids;

  for (const pose_graph::VertexId& vertex_id : export_vertex_ids) {
    const vi_map::Vertex& vertex = vi_map.getVertex(vertex_id);

    vi_map::LandmarkIdList observed_landmark_ids;
    vertex.getAllObservedLandmarkIds(&observed_landmark_ids);

    const ObserverPosesMap::const_iterator it = observer_poses.find(vertex_id);
    if (it == observer_poses.cend()) {
      VLOG(3) << "No observer poses found for vertex " << vertex_id;
      return;
    }
    const ObserverPoseSet& observer_poses_for_vertex = it->second;
    CHECK(!observer_poses_for_vertex.empty());

    VLOG(3) << "Found " << observer_poses_for_vertex.size()
            << " observers for vertex " << vertex_id;

    for (const vi_map::LandmarkId& landmark_id : observed_landmark_ids) {
      if (!landmark_id.isValid()) {
        VLOG(3) << "Discard invalid landmark!";
        continue;
      }

      if (config.use_only_good_landmarks) {
        if (vi_map.getLandmark(landmark_id).getQuality() !=
            vi_map::Landmark::Quality::kGood) {
          continue;
        }
      }

      const Eigen::Vector3d p_G = vi_map.getLandmark_G_p_fi(landmark_id);

      // Find out which observer poses see this landmark.
      std::unordered_set<size_t> observer_camera_numbers;
      for (const ObserverPose& observer_pose : observer_poses_for_vertex) {
        const ObserverCamera& observer_camera =
            common::getChecked(observer_cameras, observer_pose.camera_id);
        if (isLandmarkVisibleForObserverCamera(
                vi_map, vertex, landmark_id, p_G, observer_camera,
                observer_pose)) {
          observer_camera_numbers.insert(observer_pose.camera_number);
          VLOG(4) << "Landmark: " << landmark_id
                  << " is visible from observer number "
                  << observer_pose.camera_number;
        } else {
          VLOG(4) << "Landmark: " << landmark_id
                  << " is NOT visible from observer number "
                  << observer_pose.camera_number;
        }
      }

      if (observer_camera_numbers.empty()) {
        VLOG(3) << "Landmark " << landmark_id << " has no observers!";
        continue;
      }

      ObservedLandmarks::iterator it = observed_landmarks->find(landmark_id);
      // Initialize the observed landmark if it doesnt exists already.
      ObservedLandmark* observed_landmark = nullptr;
      if (it == observed_landmarks->end()) {
        observed_landmark = &((*observed_landmarks)[landmark_id]);
        CHECK_NOTNULL(observed_landmark);

        observed_landmark->p_G = vi_map.getLandmark_G_p_fi(landmark_id);
        observed_landmark->landmark_id = landmark_id;
        observed_landmark->landmark_number = landmark_number++;
        VLOG(4) << "New observed landmark created: " << landmark_id;
      } else {
        observed_landmark = &(it->second);
        CHECK_NOTNULL(observed_landmark);
        CHECK_EQ(observed_landmark->landmark_id, landmark_id);

        VLOG(4) << "Retrieved observed landmark: " << landmark_id;
      }

      observed_landmark->observer_pose_numbers.insert(
          observer_camera_numbers.cbegin(), observer_camera_numbers.cend());

      VLOG(3) << "Added " << observer_camera_numbers.size()
              << " observers for landmark " << landmark_id
              << ". Total number of observers are now "
              << observed_landmark->observer_pose_numbers.size();
    }
  }
}

bool isLandmarkVisibleForObserverCamera(
    const vi_map::VIMap& vi_map, const vi_map::Vertex& vertex,
    const vi_map::LandmarkId& landmark_id, const Eigen::Vector3d& p_G,
    const ObserverCamera& observer_camera, const ObserverPose& observer_pose) {
  // Retrieve aslam::Camera from either optional cameras or NCamera.
  const aslam::Camera* camera = nullptr;
  if (observer_camera.is_optional_camera) {
    camera = &vi_map.getSensorManager().getSensor<aslam::Camera>(
        observer_camera.camera_id);
  } else {
    CHECK_LT(observer_camera.frame_idx, vertex.numFrames());
    camera = vertex.getCamera(observer_camera.frame_idx).get();
  }
  CHECK_NOTNULL(camera);

  Eigen::Vector3d p_C = observer_pose.T_G_C.inverse() * p_G;

  if (p_C.z() < 0.0) {
    VLOG(4) << "Landmark " << landmark_id << " is behind camera "
            << observer_camera.camera_id;
    return false;
  }

  Eigen::Vector2d keypoint_out;
  const aslam::ProjectionResult& projection_result =
      camera->project3(p_C, &keypoint_out);

  if (projection_result) {
    VLOG(4) << "Landmark " << landmark_id << " is visible in camera "
            << observer_camera.camera_id;
    return true;
  } else {
    VLOG(4) << "Landmark " << landmark_id << " is NOT visible in camera "
            << observer_camera.camera_id;
  }
  return false;
}

}  // namespace dense_reconstruction
