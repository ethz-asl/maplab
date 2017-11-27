#include "voxblox-interface/integration.h"

#include <type_traits>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <map-resources/resource-conversion.h>
#include <maplab-common/progress-bar.h>
#include <posegraph/unique-id.h>
#include <vi-map/landmark.h>
#include <vi-map/unique-id.h>
#include <vi-map/vertex.h>

namespace voxblox_interface {

void integrateAllLandmarks(
    const vi_map::VIMap& vi_map,
    const voxblox::TsdfIntegratorBase::Config& integrator_config,
    voxblox::TsdfMap* tsdf_map) {
  CHECK_GT(vi_map.numMissions(), 0u) << "No missions in database.";
  CHECK_NOTNULL(tsdf_map);

  if (!integrator_config.use_sparsity_compensation_factor) {
    LOG(WARNING) << "Sparsity compensation factor not used for landmarks TSDF "
                    "integration.";
  }

  voxblox::SimpleTsdfIntegrator tsdf_integrator(
      integrator_config, tsdf_map->getTsdfLayerPtr());

  VLOG(1) << "Collecting all vertices in posegraph...";
  pose_graph::VertexIdList all_vertex_ids;
  vi_map.getAllVertexIdsAlongGraphsSortedByTimestamp(&all_vertex_ids);

  VLOG(1) << "Building the TSDF map...";
  common::ProgressBar tsdf_progress_bar(all_vertex_ids.size());
  size_t num_landmarks = 0u;
  for (const pose_graph::VertexId& vertex_id : all_vertex_ids) {
    tsdf_progress_bar.increment();

    const vi_map::Vertex& vertex = vi_map.getVertex(vertex_id);
    const pose::Transformation T_G_I = vi_map.getVertex_T_G_I(vertex_id);

    // Iterate through all frames.
    for (unsigned int frame_idx = 0u; frame_idx < vertex.numFrames();
         ++frame_idx) {
      const vi_map::LandmarkIdList& landmark_id_list =
          vertex.getFrameObservedLandmarkIds(frame_idx);
      const pose::Transformation& T_C_I =
          vertex.getVisualNFrame().get_T_C_B(frame_idx);
      const pose::Transformation T_G_C = T_G_I * T_C_I.inverse();

      // Add all landmarks to the voxblox pointcloud.
      pose::Position3DVector landmarks_C;
      for (const vi_map::LandmarkId& landmark_id : landmark_id_list) {
        if (!landmark_id.isValid()) {
          continue;
        }
        const vi_map::Landmark& landmark = vi_map.getLandmark(landmark_id);
        const vi_map::Landmark::Quality landmark_quality =
            landmark.getQuality();
        CHECK(landmark_quality != vi_map::Landmark::Quality::kUnknown)
            << "Retriangulate the landmarks before creating the TSDF map.";
        if (landmark_quality != vi_map::Landmark::Quality::kGood) {
          continue;
        }
        pose::Position3D landmark_C = static_cast<pose::Position3D>(
            T_C_I * vi_map.getLandmark_p_I_fi(landmark_id, vertex));
        landmarks_C.emplace_back(landmark_C);
      }

      // Integrate the landmarks.
      integratePointCloud(T_G_C, landmarks_C, &tsdf_integrator);
      num_landmarks += landmarks_C.size();
    }
  }
  VLOG(1) << "Integrated " << num_landmarks << " landmarks in TSDF map.";
}

void integratePointCloud(
    const pose::Transformation& T_G_C, const pose::Position3DVector& points_C,
    voxblox::TsdfIntegratorBase* tsdf_integrator) {
  CHECK_NOTNULL(tsdf_integrator);
  voxblox::Colors empty_colors;
  empty_colors.resize(points_C.size());
  integrateColorPointCloud(T_G_C, points_C, empty_colors, tsdf_integrator);
}

void integratePointCloud(
    const pose::Transformation& T_G_C, const resources::PointCloud& points_C,
    voxblox::TsdfIntegratorBase* tsdf_integrator) {
  CHECK_NOTNULL(tsdf_integrator);

  pose::Position3DVector tmp_points_C;
  voxblox::Colors tmp_colors;

  resources::VoxbloxColorPointCloud voxblox_point_cloud;
  voxblox_point_cloud.points_C = &tmp_points_C;
  voxblox_point_cloud.colors = &tmp_colors;
  CHECK(backend::convertPointCloudType(points_C, &voxblox_point_cloud));

  integrateColorPointCloud(T_G_C, tmp_points_C, tmp_colors, tsdf_integrator);
}

void integrateColorPointCloud(
    const pose::Transformation& T_G_C, const pose::Position3DVector& points_C,
    const voxblox::Colors& colors,
    voxblox::TsdfIntegratorBase* tsdf_integrator) {
  CHECK_NOTNULL(tsdf_integrator);
  tsdf_integrator->integratePointCloud(
      static_cast<voxblox::Transformation>(T_G_C),
      static_cast<voxblox::Pointcloud>(points_C), colors);
}

void integrateDepthMap(
    const pose::Transformation& T_G_C, const cv::Mat& depth_map,
    const aslam::Camera& camera, voxblox::TsdfIntegratorBase* tsdf_integrator) {
  CHECK_NOTNULL(tsdf_integrator);

  pose::Position3DVector point_cloud;
  backend::convertDepthMapToPointCloud(depth_map, camera, &point_cloud);

  voxblox::Colors empty_colors;
  empty_colors.resize(point_cloud.size());

  tsdf_integrator->integratePointCloud(
      static_cast<voxblox::Transformation>(T_G_C),
      static_cast<voxblox::Pointcloud>(point_cloud), empty_colors);
}

void integrateDepthMap(
    const pose::Transformation& T_G_C, const cv::Mat& depth_map,
    const cv::Mat& image, const aslam::Camera& camera,
    voxblox::TsdfIntegratorBase* tsdf_integrator) {
  CHECK_NOTNULL(tsdf_integrator);

  pose::Position3DVector point_cloud;
  voxblox::Colors colors;
  backend::convertDepthMapWithImageToPointCloud(
      depth_map, image, camera, &point_cloud, &colors);

  tsdf_integrator->integratePointCloud(
      static_cast<voxblox::Transformation>(T_G_C),
      static_cast<voxblox::Pointcloud>(point_cloud), colors);
}

static std::unordered_set<backend::ResourceType, backend::ResourceTypeHash>
    kSupportedDepthInputTypes{backend::ResourceType::kRawDepthMap,
                              backend::ResourceType::kOptimizedDepthMap,
                              backend::ResourceType::kPointCloudXYZRGBN};

bool integrateAllDepthResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const bool use_distorted_camera,
    const voxblox::TsdfIntegratorBase::Config& integrator_config,
    vi_map::VIMap* vi_map, voxblox::TsdfMap* tsdf_map) {
  CHECK_NOTNULL(vi_map);
  CHECK_NOTNULL(tsdf_map);
  CHECK_GT(kSupportedDepthInputTypes.count(input_resource_type), 0)
      << "This depth type is not supported! type: "
      << backend::ResourceTypeNames[static_cast<int>(input_resource_type)];

  // Init Voxblox map and integrator.
  voxblox::MergedTsdfIntegrator tsdf_integrator(
      integrator_config, tsdf_map->getTsdfLayerPtr());

  // Start integration.
  for (const vi_map::MissionId& mission_id : mission_ids) {
    VLOG(1) << "Integrating mission " << mission_id;

    const aslam::NCamera& n_camera =
        vi_map->getSensorManager().getNCameraForMission(mission_id);

    // Get cameras for depth map reprojection if necessary. If the flag is set
    // we use the camera without distortion.
    const size_t num_cameras = n_camera.getNumCameras();
    std::vector<aslam::Camera::ConstPtr> cameras(num_cameras);
    if (input_resource_type == backend::ResourceType::kRawDepthMap ||
        input_resource_type == backend::ResourceType::kOptimizedDepthMap) {
      for (size_t frame_idx = 0u; frame_idx < num_cameras; ++frame_idx) {
        if (use_distorted_camera) {
          cameras[frame_idx] = n_camera.getCameraShared(frame_idx);
        } else {
          aslam::Camera::Ptr camera_no_distortion;
          backend::createCameraWithoutDistortion(
              n_camera.getCamera(frame_idx), &camera_no_distortion);
          CHECK(camera_no_distortion);
          cameras[frame_idx] = camera_no_distortion;
        }
      }
    }
    const aslam::Transformation& T_G_M =
        vi_map->getMissionBaseFrameForMission(mission_id).get_T_G_M();

    pose_graph::VertexIdList vertex_ids;
    vi_map->getAllVertexIdsInMissionAlongGraph(mission_id, &vertex_ids);

    common::ProgressBar tsdf_progress_bar(vertex_ids.size());
    size_t vertex_counter = 0u;
    constexpr size_t kUpdateEveryNthVertex = 20u;
    for (const pose_graph::VertexId& vertex_id : vertex_ids) {
      if (vertex_counter % kUpdateEveryNthVertex == 0u) {
        tsdf_progress_bar.update(vertex_counter);
      }
      ++vertex_counter;

      vi_map::Vertex& vertex = vi_map->getVertex(vertex_id);

      const aslam::Transformation T_G_I = T_G_M * vertex.get_T_M_I();

      // Get number of frames for this vertex
      const size_t num_frames = vertex.numFrames();
      for (size_t frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
        VLOG(3) << "Vertex " << vertex_id << " / Frame " << frame_idx;

        // Compute complete transformation.
        const aslam::Transformation T_I_C =
            n_camera.get_T_C_B(frame_idx).inverse();
        const aslam::Transformation T_G_C = T_G_I * T_I_C;

        switch (input_resource_type) {
          case backend::ResourceType::kRawDepthMap:
          // Fall through intended.
          case backend::ResourceType::kOptimizedDepthMap: {
            // Check if a depth map resource is available.
            CHECK_LT(frame_idx, num_cameras);
            CHECK(cameras[frame_idx]);
            cv::Mat depth_map;
            if (!vi_map->getFrameResource(
                    vertex, frame_idx, input_resource_type, &depth_map)) {
              VLOG(3) << "Nothing to integrate.";
              continue;
            }
            // Check if there is a dedicated image for this depth map. If not,
            // use the normal grayscale image.
            cv::Mat image;
            bool has_image = false;
            if (vi_map->getImageForDepthMap(vertex, frame_idx, &image)) {
              VLOG(3) << "Found depth map with intensity information "
                         "from the dedicated grayscale image.";
              has_image = true;
            } else if (vi_map->getRawImage(vertex, frame_idx, &image)) {
              VLOG(3) << "Found depth map with intensity information "
                         "from the raw grayscale image.";
              has_image = true;
            } else {
              VLOG(3) << "Found depth map without intensity information.";
            }

            // Integrate with or without intensity information.
            if (has_image) {
              integrateDepthMap(
                  T_G_C, depth_map, image, *cameras[frame_idx],
                  &tsdf_integrator);
            } else {
              integrateDepthMap(
                  T_G_C, depth_map, *cameras[frame_idx], &tsdf_integrator);
            }
            continue;
          }
          case backend::ResourceType::kPointCloudXYZRGBN: {
            // Check if a point cloud is available.
            resources::PointCloud point_cloud;
            if (!vi_map->getFrameResource(
                    vertex, frame_idx, input_resource_type, &point_cloud)) {
              VLOG(3) << "Nothing to integrate.";
              continue;
            }

            VLOG(3) << "Found point cloud.";
            integratePointCloud(T_G_C, point_cloud, &tsdf_integrator);
            continue;
          }
          default:
            LOG(FATAL) << "This depth type is not supported! type: "
                       << backend::ResourceTypeNames[static_cast<int>(
                              input_resource_type)];
        }
      }
    }
  }
  return true;
}

}  // namespace voxblox_interface

void staticTypeAssertions() {
  static_assert(
      std::is_same<voxblox::Point, pose::Position3D>::value,
      "Voxblox 3D points and maplab 3D points are not the same!");
  static_assert(
      std::is_same<voxblox::Pointcloud, pose::Position3DVector>::value,
      "Voxblox 3D point vectors and maplab 3D point vectors are not the same!");
  static_assert(
      std::is_same<voxblox::Transformation, pose::Transformation>::value,
      "Voxblox transformations and maplab transformations are not the same!");
}
