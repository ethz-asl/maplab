#ifndef DEPTH_INTEGRATION_DEPTH_INTEGRATION_INL_H_
#define DEPTH_INTEGRATION_DEPTH_INTEGRATION_INL_H_

#include <algorithm>
#include <functional>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <landmark-triangulation/pose-interpolator.h>
#include <map-resources/resource-conversion.h>
#include <map-resources/temporal-resource-id-buffer.h>
#include <maplab-common/progress-bar.h>
#include <maplab-common/sigint-breaker.h>
#include <posegraph/unique-id.h>
#include <vi-map/landmark.h>
#include <vi-map/unique-id.h>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>
#include <voxblox/core/common.h>

#include "depth-integration/depth-integration.h"

namespace depth_integration {

template <typename IntegrationFunctionType>
void integrateAllFrameDepthResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const bool use_undistorted_camera_for_depth_maps,
    const vi_map::VIMap& vi_map, IntegrationFunctionType integration_function,
    ResourceSelectionFunction resource_selection_function) {
  CHECK(integration_function);

  if (!isSupportedResourceType<IntegrationFunctionType>(input_resource_type)) {
    LOG(ERROR)
        << "Resource type "
        << backend::ResourceTypeNames[static_cast<int>(input_resource_type)]
        << "(" << static_cast<int>(input_resource_type)
        << ")' is not supported by this depth integration function.";
    return;
  }

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

    // Get cameras for depth map reprojection if necessary. If the flag is set
    // we use the camera without distortion.
    const size_t num_cameras = n_camera.getNumCameras();
    std::vector<aslam::Camera::ConstPtr> cameras(num_cameras);
    if (input_resource_type == backend::ResourceType::kRawDepthMap ||
        input_resource_type == backend::ResourceType::kOptimizedDepthMap) {
      for (size_t frame_idx = 0u; frame_idx < num_cameras; ++frame_idx) {
        if (use_undistorted_camera_for_depth_maps) {
          aslam::Camera::Ptr camera_no_distortion;
          backend::createCameraWithoutDistortion(
              n_camera.getCamera(frame_idx), &camera_no_distortion);
          CHECK(camera_no_distortion);
          cameras[frame_idx] = camera_no_distortion;
        } else {
          cameras[frame_idx] = n_camera.getCameraShared(frame_idx);
        }
      }
    }

    pose_graph::VertexIdList vertex_ids;
    vi_map.getAllVertexIdsInMissionAlongGraph(mission_id, &vertex_ids);
    size_t vertex_counter = 0u;
    constexpr size_t kUpdateEveryNthVertex = 20u;
    common::ProgressBar progress_bar(vertex_ids.size());
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

        if (!vi_map.hasFrameResource(vertex, frame_idx, input_resource_type)) {
          continue;
        }

        const int64_t timestamp_ns =
            vertex.getVisualFrame(frame_idx).getTimestampNanoseconds();

        // Compute complete transformation.
        const aslam::Transformation T_Cn_C =
            n_camera.get_T_C_B(frame_idx).inverse();
        const aslam::SensorId& sensor_id =
            n_camera.getCamera(frame_idx).getId();
        const aslam::Transformation T_B_C = T_B_Cn * T_Cn_C;
        const aslam::Transformation T_G_C = T_G_B * T_B_C;

        // If we have a resource selection function, use it to abort here.
        if (resource_selection_function != nullptr) {
          if (!resource_selection_function(timestamp_ns, T_G_C)) {
            continue;
          }
        }

        switch (input_resource_type) {
          case backend::ResourceType::kRawDepthMap:
          // Fall through intended.
          case backend::ResourceType::kOptimizedDepthMap: {
            // Check if a depth map resource is available.
            CHECK_LT(frame_idx, num_cameras);
            CHECK(cameras[frame_idx]);
            cv::Mat depth_map;
            if (!vi_map.getFrameResource(
                    vertex, frame_idx, input_resource_type, &depth_map)) {
              VLOG(3) << "Nothing to integrate.";
              continue;
            }
            CHECK_EQ(CV_MAT_TYPE(depth_map.type()), CV_16UC1);
            CHECK(!depth_map.empty())
                << "Depth map at vertex " << vertex_id << " frame " << frame_idx
                << " is invalid/empty!";

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
            integrateDepthMap(
                mission_id, timestamp_ns, T_G_C, depth_map, image,
                *cameras[frame_idx], integration_function);

            continue;
          }
          case backend::ResourceType::kPointCloudXYZI:
          // Fall through intended.
          case backend::ResourceType::kPointCloudXYZ:
          // Fall through intended.
          case backend::ResourceType::kPointCloudXYZRGBN: {
            // Check if a point cloud is available.
            resources::PointCloud point_cloud;
            if (!vi_map.getFrameResource(
                    vertex, frame_idx, input_resource_type, &point_cloud)) {
              VLOG(3) << "Nothing to integrate.";
              continue;
            }

            VLOG(3) << "Found point cloud.";
            integratePointCloud(
                mission_id, timestamp_ns, sensor_id, T_G_C, point_cloud,
                integration_function);
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
}

template <typename IntegrationFunctionType>
void integrateAllSensorDepthResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const bool use_undistorted_camera_for_depth_maps,
    const vi_map::VIMap& vi_map, IntegrationFunctionType integration_function,
    ResourceSelectionFunction resource_selection_function) {
  CHECK(integration_function);

  if (!isSupportedResourceType<IntegrationFunctionType>(input_resource_type)) {
    LOG(ERROR)
        << "Resource type "
        << backend::ResourceTypeNames[static_cast<int>(input_resource_type)]
        << "(" << static_cast<int>(input_resource_type)
        << ")' is not supported by this depth integration function.";
    return;
  }

  const vi_map::SensorManager& sensor_manager = vi_map.getSensorManager();

  const int64_t timestamp_shift_ns =
      FLAGS_dense_depth_integrator_timeshift_resource_to_imu_ns;

  std::unique_ptr<common::SigintBreaker> sigint_breaker;
  if (FLAGS_dense_depth_integrator_enable_sigint_breaker) {
    sigint_breaker.reset(new common::SigintBreaker);
  }

  // Start integration.
  for (const vi_map::MissionId& mission_id : mission_ids) {
    VLOG(1) << "Integrating mission " << mission_id;
    const vi_map::VIMission& mission = vi_map.getMission(mission_id);

    // If this flags is set, we skip this mission if its baseframe is not known.
    if (FLAGS_dense_depth_integrator_visualize_only_with_known_baseframe) {
      const vi_map::MissionBaseFrameId& mission_baseframe_id =
          mission.getBaseFrameId();
      if (!vi_map.getMissionBaseFrame(mission_baseframe_id).is_T_G_M_known()) {
        continue;
      }
    }

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

    std::vector<int64_t> vertex_timestamps_nanoseconds;
    if (FLAGS_dense_depth_integrator_use_closest_to_vertex) {
      vertex_timestamps_nanoseconds.reserve(vertex_to_time_map.size());
      pose_interpolator.getVertexTimeStampVector(
          vi_map, mission_id, &vertex_timestamps_nanoseconds);
      // There is no need to sort the timestamps as they are already in correct
      // order.
    }
    VLOG(1) << "Got timestamps for vertices: "
            << vertex_timestamps_nanoseconds.size();

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
      // Mission does not have any sensor resources.
      continue;
    }
    VLOG(1) << "Found " << sensor_id_to_res_id_map->size()
            << " sensors that have resources of this depth type.";

    // Integrate them one sensor at a time.
    for (const typename SensorsToResourceMap::value_type& sensor_to_res_ids :
         *sensor_id_to_res_id_map) {
      const backend::TemporalResourceIdBuffer* resource_buffer_ptr =
          &(sensor_to_res_ids.second);

      const aslam::SensorId& sensor_id = sensor_to_res_ids.first;

      // Get transformation between reference (e.g. IMU) and sensor.
      aslam::Transformation T_B_S;
      if (!sensor_manager.hasSensor(sensor_id)) {
        T_B_S = sensor_manager.getCamera_T_B_C(sensor_id);
      } else {
        T_B_S = sensor_manager.getSensor_T_B_S(sensor_id);
      }

      // If the sensor is a camera and the resource type is a depth map we will
      // need the camera model to reproject them.
      aslam::NCamera::Ptr ncamera_ptr;
      aslam::Camera::Ptr camera_ptr;
      if (sensor_manager.hasSensor(sensor_id) &&
          sensor_manager.getSensorType(sensor_id) ==
              vi_map::SensorType::kNCamera) {
        ncamera_ptr = sensor_manager.getSensorPtr<aslam::NCamera>(sensor_id);
        CHECK(ncamera_ptr);
        CHECK_EQ(ncamera_ptr->getNumCameras(), 1u);
        camera_ptr = ncamera_ptr->getCameraShared(0);
        // Need to update the sensor extrinsics, since ncameras have an
        // additiona extrinsics between ncamera frame and camera frame.
        T_B_S = T_B_S * ncamera_ptr->get_T_C_B(0).inverse();
      }
      // Optionally, remove distortion from camera.
      if (use_undistorted_camera_for_depth_maps && camera_ptr) {
        aslam::Camera::Ptr camera_no_distortion;
        backend::createCameraWithoutDistortion(
            *camera_ptr, &camera_no_distortion);
        CHECK(camera_no_distortion);
        camera_ptr = camera_no_distortion;
      }

      size_t num_resources = resource_buffer_ptr->size();
      VLOG(1) << "Sensor " << sensor_id.shortHex() << " has " << num_resources
              << " such resources.";

      // If we only want to integrate a subset of the resource, fill this buffer
      // accordingly and then reassign the resource_buffer_ptr to point to this
      // subset buffer.
      backend::TemporalResourceIdBuffer resource_buffer_subset;
      if (FLAGS_dense_depth_integrator_use_closest_to_vertex) {
        CHECK_GT(vertex_timestamps_nanoseconds.size(), 0u);
        backend::StampedResourceId stamped_resource_id;
        for (const int64_t& vertex_timestamp_nanoseconds :
             vertex_timestamps_nanoseconds) {
          VLOG(2) << "Find closest resource for "
                  << vertex_timestamp_nanoseconds;
          if (resource_buffer_ptr->getClosestResourceId(
                  vertex_timestamp_nanoseconds,
                  std::numeric_limits<int64_t>::max(), &stamped_resource_id)) {
            resource_buffer_subset.addResourceId(
                stamped_resource_id.first, stamped_resource_id.second);
          }
        }
        VLOG(1) << "Selected a subset of resources to integrate: "
                << resource_buffer_subset.size() << "/" << num_resources;

        // Switch the pointer to this object.
        resource_buffer_ptr = &resource_buffer_subset;
        num_resources = resource_buffer_ptr->size();
      }
      CHECK_NOTNULL(resource_buffer_ptr);

      // Collect all timestamps that need to be interpolated.
      Eigen::Matrix<int64_t, 1, Eigen::Dynamic> resource_timestamps(
          num_resources);
      size_t idx = 0u;
      for (const std::pair<const int64_t, backend::ResourceId>&
               stamped_resource_id : *resource_buffer_ptr) {
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
      CHECK_EQ(poses_M_B.size(), num_resources);

      // Retrieve and integrate all resources.
      idx = 0u;
      common::ProgressBar progress_bar(num_resources);
      for (const std::pair<const int64_t, backend::ResourceId>&
               stamped_resource_id : *resource_buffer_ptr) {
        progress_bar.increment();

        if (FLAGS_dense_depth_integrator_enable_sigint_breaker) {
          CHECK(sigint_breaker);
          if (sigint_breaker->isBreakRequested()) {
            LOG(WARNING) << "Depth integration has been aborted by the user!";
            return;
          }
        }

        const aslam::Transformation& T_M_B = poses_M_B[idx];
        const aslam::Transformation T_G_S = T_G_M * T_M_B * T_B_S;
        ++idx;

        const int64_t timestamp_ns = stamped_resource_id.first;

        // If we have a resource selection function, use it to abort here.
        if (resource_selection_function != nullptr) {
          if (!resource_selection_function(timestamp_ns, T_G_S)) {
            continue;
          }
        }

        // If the resource timestamp does not lie within the min and max
        // timestamp of the vertices, we cannot interpolate the position.
        if (timestamp_ns < min_timestamp_ns ||
            timestamp_ns > max_timestamp_ns) {
          VLOG(3) << "The optional depth resource at " << timestamp_ns
                  << "ns is outside of the time range of the pose graph, "
                  << "skipping.";
          continue;
        }

        switch (input_resource_type) {
          case backend::ResourceType::kRawDepthMap:
          // Fall through intended.
          case backend::ResourceType::kOptimizedDepthMap: {
            CHECK(camera_ptr)
                << "For depth maps we need a camera for reprojection, "
                << "but the associated sensor is not a camera!";
            const aslam::Camera& camera = *camera_ptr;

            cv::Mat depth_map;
            if (!vi_map.getSensorResource(
                    mission, input_resource_type, sensor_id, timestamp_ns,
                    &depth_map)) {
              LOG(FATAL) << "Cannot retrieve depth map resource at "
                         << "timestamp " << timestamp_ns << "ns!";
            }
            CHECK_EQ(CV_MAT_TYPE(depth_map.type()), CV_16UC1);
            CHECK(!depth_map.empty())
                << "Depth map at time " << timestamp_ns << " of sensor "
                << sensor_id << " is invalid/empty!";

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
            integrateDepthMap(
                mission_id, timestamp_ns, T_G_S, depth_map, image, camera,
                integration_function);

            continue;
          }
          case backend::ResourceType::kPointCloudXYZI:
          // Fall through intended.
          case backend::ResourceType::kPointCloudXYZ:
          // Fall through intended.
          case backend::ResourceType::kPointCloudXYZRGBN: {
            // Check if a point cloud is available.
            resources::PointCloud point_cloud;
            if (!vi_map.getSensorResource(
                    mission, input_resource_type, sensor_id, timestamp_ns,
                    &point_cloud)) {
              LOG(FATAL) << "Cannot retrieve point cloud resource at "
                         << "timestamp " << timestamp_ns << "ns!";
            }

            VLOG(3) << "Found point cloud at timestamp " << timestamp_ns
                    << "ns";
            integratePointCloud(
                mission_id, timestamp_ns, sensor_id, T_G_S, point_cloud,
                integration_function);
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
}

template <typename IntegrationFunctionType>
void integrateAllDepthResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const bool use_undistorted_camera_for_depth_maps,
    const vi_map::VIMap& vi_map, IntegrationFunctionType integration_function,
    ResourceSelectionFunction resource_selection_function) {
  CHECK(integration_function);

  if (!isSupportedResourceType<IntegrationFunctionType>(input_resource_type)) {
    LOG(ERROR)
        << "Resource type "
        << backend::ResourceTypeNames[static_cast<int>(input_resource_type)]
        << "(" << static_cast<int>(input_resource_type)
        << ")' is not supported by this depth integration function.";
    return;
  }

  // Integrate all depth resources associated with a visual frame.
  integrateAllFrameDepthResourcesOfType(
      mission_ids, input_resource_type, use_undistorted_camera_for_depth_maps,
      vi_map, integration_function, resource_selection_function);

  // Integrate all depth resources associated with an arbitrary sensor and
  // timestamp.
  integrateAllSensorDepthResourcesOfType(
      mission_ids, input_resource_type, use_undistorted_camera_for_depth_maps,
      vi_map, integration_function, resource_selection_function);
}

}  // namespace depth_integration

#endif  // DEPTH_INTEGRATION_DEPTH_INTEGRATION_INL_H_
