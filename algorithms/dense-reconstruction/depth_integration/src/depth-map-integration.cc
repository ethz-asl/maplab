#include "depth-integration/depth-map-integration.h"

#include <algorithm>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <landmark-triangulation/pose-interpolator.h>
#include <map-resources/resource-conversion.h>
#include <maplab-common/progress-bar.h>
#include <maplab-common/sigint-breaker.h>
#include <posegraph/unique-id.h>
#include <vi-map/landmark.h>
#include <vi-map/unique-id.h>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>

#include "depth-integration/depth-integration.h"

namespace depth_integration {

void integrateAllDepthMapResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const vi_map::VIMap& vi_map,
    DepthMapIntegrationFunction integration_function) {
  CHECK(integration_function);

  // Integrate all frame resources.
  integrateAllFrameDepthMapResourcesOfType(
      mission_ids, input_resource_type, vi_map, integration_function);

  // Integrate all optional sensor resources.
  integrateAllOptionalSensorDepthMapResourcesOfType(
      mission_ids, input_resource_type, vi_map, integration_function);
}

void integrateAllFrameDepthMapResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const vi_map::VIMap& vi_map,
    DepthMapIntegrationFunction integration_function) {
  CHECK(integration_function);
  CHECK_GT(kSupportedDepthMapInputTypes.count(input_resource_type), 0u)
      << "This depth type is not supported! type: "
      << backend::ResourceTypeNames[static_cast<int>(input_resource_type)];

  std::unique_ptr<common::SigintBreaker> sigint_breaker;
  if (FLAGS_dense_depth_integrator_enable_sigint_breaker) {
    sigint_breaker.reset(new common::SigintBreaker);
  }

  // Start integration.
  for (const vi_map::MissionId& mission_id : mission_ids) {
    const vi_map::VIMission& mission = vi_map.getMission(mission_id);

    // If this flags is set, we skip this mission if its baseframe is not known.
    if (FLAGS_dense_depth_integrator_visualize_only_with_known_baseframe) {
      const vi_map::MissionBaseFrameId& mission_baseframe_id =
          mission.getBaseFrameId();
      if (!vi_map.getMissionBaseFrame(mission_baseframe_id).is_T_G_M_known()) {
        continue;
      }
    }

    if (!mission.hasNCamera()) {
      VLOG(1) << "Mission " << mission_id
              << " has no NCamera, hence no such resources!";
      continue;
    }
    VLOG(1) << "Integrating mission " << mission_id;

    aslam::SensorId ncamera_id = mission.getNCameraId();
    const aslam::NCamera& n_camera =
        vi_map.getSensorManager().getSensor<aslam::NCamera>(ncamera_id);
    const aslam::Transformation T_B_Cn =
        vi_map.getSensorManager().getSensor_T_B_S(ncamera_id);

    const aslam::Transformation& T_G_M =
        vi_map.getMissionBaseFrameForMission(mission_id).get_T_G_M();

    pose_graph::VertexIdList vertex_ids;
    vi_map.getAllVertexIdsInMissionAlongGraph(mission_id, &vertex_ids);

    common::ProgressBar progress_bar(vertex_ids.size());
    size_t vertex_counter = 0u;
    constexpr size_t kUpdateEveryNthVertex = 20u;
    for (const pose_graph::VertexId& vertex_id : vertex_ids) {
      if (vertex_counter % kUpdateEveryNthVertex == 0u) {
        progress_bar.update(vertex_counter);
      }
      ++vertex_counter;

      if (FLAGS_dense_depth_integrator_enable_sigint_breaker) {
        CHECK(sigint_breaker);
        if (sigint_breaker->isBreakRequested()) {
          LOG(WARNING) << "Depth integration has been aborted by the user!";
          return;
        }
      }

      const vi_map::Vertex& vertex = vi_map.getVertex(vertex_id);

      const aslam::Transformation T_G_B = T_G_M * vertex.get_T_M_I();

      // Get number of frames for this vertex
      const size_t num_frames = vertex.numFrames();
      for (size_t frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
        VLOG(3) << "Vertex " << vertex_id << " / Frame " << frame_idx;

        // Compute complete transformation.
        const aslam::Transformation T_Cn_C =
            n_camera.get_T_C_B(frame_idx).inverse();
        const aslam::Transformation T_B_C = T_B_Cn * T_Cn_C;
        const aslam::Transformation T_G_C = T_G_B * T_B_C;

        switch (input_resource_type) {
          case backend::ResourceType::kRawDepthMap:
          // Fall through intended.
          case backend::ResourceType::kOptimizedDepthMap: {
            // Check if a depth map resource is available.
            cv::Mat depth_map;
            if (!vi_map.getFrameResource(
                    vertex, frame_idx, input_resource_type, &depth_map)) {
              VLOG(3) << "Nothing to integrate.";
              continue;
            }
            // Check if there is a dedicated image for this depth map. If not,
            // use the normal grayscale image.
            cv::Mat image;
            if (vi_map.getFrameResource(
                    vertex, frame_idx, backend::ResourceType::kImageForDepthMap,
                    &image)) {
              VLOG(3) << "Found depth map with intensity information "
                         "from the dedicated grayscale image.";
            } else if (vi_map.getFrameResource(
                           vertex, frame_idx, backend::ResourceType::kRawImage,
                           &image)) {
              VLOG(3) << "Found depth map with intensity information "
                         "from the raw grayscale image.";
            } else {
              VLOG(3) << "Found depth map without intensity information.";
            }

            // Integrate with or without intensity information.
            integration_function(
                T_G_C, depth_map, image, n_camera.getCamera(frame_idx));
          } break;
          default:
            LOG(FATAL) << "This depth type is not supported! type: "
                       << backend::ResourceTypeNames[static_cast<int>(
                              input_resource_type)];
        }
      }
    }
  }
}

void integrateAllOptionalSensorDepthMapResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const vi_map::VIMap& vi_map,
    DepthMapIntegrationFunction integration_function) {
  CHECK(integration_function);
  CHECK_GT(kSupportedDepthMapInputTypes.count(input_resource_type), 0u)
      << "This depth type is not supported! Type: "
      << backend::ResourceTypeNames[static_cast<int>(input_resource_type)];

  const vi_map::SensorManager& sensor_manager = vi_map.getSensorManager();

  std::unique_ptr<common::SigintBreaker> sigint_breaker;
  if (FLAGS_dense_depth_integrator_enable_sigint_breaker) {
    sigint_breaker.reset(new common::SigintBreaker);
  }

  // Start integration.
  for (const vi_map::MissionId& mission_id : mission_ids) {
    VLOG(1) << "Integrating mission " << mission_id;
    const vi_map::VIMission& mission = vi_map.getMission(mission_id);

    const aslam::Transformation& T_G_M =
        vi_map.getMissionBaseFrameForMission(mission_id).get_T_G_M();

    // Check if there is IMU data to interpolate the optional sensor poses.
    landmark_triangulation::VertexToTimeStampMap vertex_to_time_map;
    int64_t min_timestamp_ns;
    int64_t max_timestamp_ns;
    const landmark_triangulation::PoseInterpolator pose_interpolator;
    pose_interpolator.getVertexToTimeStampMap(
        vi_map, mission_id, &vertex_to_time_map, &min_timestamp_ns,
        &max_timestamp_ns);
    if (vertex_to_time_map.empty()) {
      VLOG(2) << "Couldn't find any IMU data to interpolate exact optional "
              << "sensor position in mission " << mission_id;
      continue;
    }

    LOG(INFO) << "All resources within this time range will be integrated: ["
              << min_timestamp_ns << "ns," << max_timestamp_ns << "ns]";

    // Retrieve sensor id to resource id mapping.
    typedef std::unordered_map<
        aslam::SensorId, backend::TemporalResourceIdBuffer>
        SensorsToResourceMap;
    const SensorsToResourceMap* sensor_id_to_res_id_map;
    sensor_id_to_res_id_map =
        mission.getAllSensorResourceIdsOfType(input_resource_type);

    if (sensor_id_to_res_id_map == nullptr) {
      continue;
    }
    VLOG(1) << "Found " << sensor_id_to_res_id_map->size()
            << " sensors that have resources of this depth type.";

    // Integrate them one sensor at a time.
    for (const typename SensorsToResourceMap::value_type& sensor_to_res_ids :
         *sensor_id_to_res_id_map) {
      const backend::TemporalResourceIdBuffer& resource_buffer =
          sensor_to_res_ids.second;

      const aslam::SensorId& sensor_id = sensor_to_res_ids.first;

      // Get transformation between reference (e.g. IMU) and sensor.
      aslam::Transformation T_B_Cn = sensor_manager.getSensor_T_B_S(sensor_id);

      // Get the camera.
      aslam::NCamera::Ptr ncamera_ptr;
      aslam::Camera::Ptr camera_ptr;
      CHECK(
          sensor_manager.getSensorType(sensor_id) ==
          vi_map::SensorType::kNCamera)
          << "The sensor (" << sensor_id
          << ") associated with this depth map resource is not a "
          << "camera!";
      ncamera_ptr = sensor_manager.getSensorPtr<aslam::NCamera>(sensor_id);
      CHECK(ncamera_ptr);
      CHECK_EQ(ncamera_ptr->getNumCameras(), 1u);
      camera_ptr = ncamera_ptr->getCameraShared(0);

      // Need to update the sensor extrinsics, since ncameras have an
      // additiona extrinsics between ncamera frame and camera frame.
      const aslam::Transformation T_Cn_C = ncamera_ptr->get_T_C_B(0).inverse();
      const aslam::Transformation T_B_C = T_B_Cn * T_Cn_C;

      const size_t num_resources = resource_buffer.size();
      VLOG(1) << "Sensor " << sensor_id.shortHex() << " has " << num_resources
              << " such resources.";

      // Collect all timestamps that need to be interpolated.
      Eigen::Matrix<int64_t, 1, Eigen::Dynamic> resource_timestamps(
          num_resources);
      size_t idx = 0u;

      const int64_t timestamp_shift_ns =
          FLAGS_dense_depth_integrator_timeshift_resource_to_imu_ns;

      for (const std::pair<int64_t, backend::ResourceId>& stamped_resource_id :
           resource_buffer) {
        const int64_t timestamp_resource_ns =
            stamped_resource_id.first + timestamp_shift_ns;

        // If the resource timestamp does not lie within the min and max
        // timestamp of the vertices, we cannot interpolate the position. To
        // keep this efficient, we simply replace timestamps outside the range
        // with the min or max. Since their transformation will not be used
        // later, that's fine.
        resource_timestamps[idx] = std::max(
            min_timestamp_ns,
            std::min(max_timestamp_ns, timestamp_resource_ns));

        ++idx;
      }

      // Interpolate poses for every resource.
      aslam::TransformationVector poses_M_B;
      pose_interpolator.getPosesAtTime(
          vi_map, mission_id, resource_timestamps, &poses_M_B);
      CHECK_EQ(static_cast<int>(poses_M_B.size()), resource_timestamps.size());
      CHECK_EQ(poses_M_B.size(), resource_buffer.size());

      // Retrieve and integrate all resources.
      idx = 0u;
      common::ProgressBar progress_bar(resource_buffer.size());
      for (const std::pair<int64_t, backend::ResourceId>& stamped_resource_id :
           resource_buffer) {
        progress_bar.increment();

        if (FLAGS_dense_depth_integrator_enable_sigint_breaker) {
          CHECK(sigint_breaker);
          if (sigint_breaker->isBreakRequested()) {
            LOG(WARNING) << "Depth integration has been aborted by the user!";
            return;
          }
        }

        const aslam::Transformation& T_M_B = poses_M_B[idx];
        const aslam::Transformation T_G_S = T_G_M * T_M_B * T_B_C;
        ++idx;

        const int64_t timestamp_ns = stamped_resource_id.first;

        // If the resource timestamp does not lie within the min and max
        // timestamp of the vertices, we cannot interpolate the position.
        if (timestamp_ns < min_timestamp_ns ||
            timestamp_ns > max_timestamp_ns) {
          LOG(WARNING) << "The optional depth resource at " << timestamp_ns
                       << "ns is outside of the time range of the pose graph, "
                       << "skipping.";
          continue;
        }

        switch (input_resource_type) {
          case backend::ResourceType::kRawDepthMap:
          // Fall through intended.
          case backend::ResourceType::kOptimizedDepthMap: {
            CHECK(camera_ptr);
            const aslam::Camera& camera = *camera_ptr;

            cv::Mat depth_map;
            if (!vi_map.getSensorResource(
                    mission, input_resource_type, sensor_id, timestamp_ns,
                    &depth_map)) {
              LOG(FATAL) << "Cannot retrieve depth map resource at "
                         << "timestamp " << timestamp_ns << "ns!";
            }

            // Check if there is a dedicated grayscale or color image for this
            // depth map.
            cv::Mat image;
            if (vi_map.getSensorResource(
                    mission, backend::ResourceType::kImageForDepthMap,
                    sensor_id, timestamp_ns, &image)) {
              VLOG(3) << "Found depth map with intensity information "
                         "from the dedicated grayscale image.";
            } else if (vi_map.getSensorResource(
                           mission,
                           backend::ResourceType::kColorImageForDepthMap,
                           sensor_id, timestamp_ns, &image)) {
              VLOG(3) << "Found depth map with RGB information "
                      << "from the dedicated color image.";
            } else {
              VLOG(3)
                  << "Found depth map without any color/intensity information.";
            }

            // Integrate with or without intensity information.
            integration_function(T_G_S, depth_map, image, camera);
          } break;
          default:
            LOG(FATAL) << "This depth type is not supported! type: "
                       << backend::ResourceTypeNames[static_cast<int>(
                              input_resource_type)];
        }
      }
    }
  }
}

}  // namespace depth_integration
