#ifndef VOXBLOX_INTERFACE_INTEGRATION_INL_H_
#define VOXBLOX_INTERFACE_INTEGRATION_INL_H_

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
#include <posegraph/unique-id.h>
#include <vi-map/landmark.h>
#include <vi-map/unique-id.h>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>
#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>

namespace voxblox_interface {

// Convenience function that returns a pointer to a camera if called with a
// valid optional camera id, or a nullptr if called with a sensor id.
template <typename SensorOrCameraId>
aslam::Camera::Ptr getOptionalCameraIfCameraId(
    const vi_map::SensorManager& sensor_manager,
    const bool use_undistorted_camera_for_depth_maps,
    const SensorOrCameraId& sensor_or_camera_id);
template <>
aslam::Camera::Ptr getOptionalCameraIfCameraId(
    const vi_map::SensorManager& /*sensor_manager*/,
    const bool /*use_undistorted_camera_for_depth_maps*/,
    const vi_map::SensorId& /*sensor_id*/) {
  return aslam::Camera::Ptr();
}
template <>
aslam::Camera::Ptr getOptionalCameraIfCameraId(
    const vi_map::SensorManager& sensor_manager,
    const bool use_undistorted_camera_for_depth_maps,
    const aslam::CameraId& camera_id) {
  if (use_undistorted_camera_for_depth_maps) {
    const aslam::Camera::Ptr camera_with_distortion =
        sensor_manager.getOptionalCameraWithExtrinsics(camera_id).second;
    CHECK(camera_with_distortion);

    aslam::Camera::Ptr camera_no_distortion;
    backend::createCameraWithoutDistortion(
        *camera_with_distortion, &camera_no_distortion);
    CHECK(camera_no_distortion);
    return camera_no_distortion;
  } else {
    return sensor_manager.getOptionalCameraWithExtrinsics(camera_id).second;
  }
}

template <typename SensorOrCameraId>
bool integrateAllOptionalSensorDepthResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const bool use_undistorted_camera_for_depth_maps,
    const voxblox::TsdfIntegratorBase::Config& integrator_config,
    const vi_map::VIMap& vi_map, voxblox::TsdfMap* tsdf_map) {
  CHECK_NOTNULL(tsdf_map);
  CHECK_GT(kSupportedDepthInputTypes.count(input_resource_type), 0u)
      << "This depth type is not supported! Type: "
      << backend::ResourceTypeNames[static_cast<int>(input_resource_type)];

  // Init Voxblox map and integrator.
  voxblox::MergedTsdfIntegrator tsdf_integrator(
      integrator_config, tsdf_map->getTsdfLayerPtr());

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
              << min_timestamp_ns << "," << max_timestamp_ns << "]";

    // Retrieve sensor id to resource id mapping.
    typedef std::unordered_map<SensorOrCameraId,
                               backend::OptionalSensorResources>
        SensorsToResourceMap;

    const SensorsToResourceMap* sensor_id_to_res_id_map;
    sensor_id_to_res_id_map =
        mission.getAllOptionalSensorResourceIdsOfType<SensorOrCameraId>(
            input_resource_type);

    if (sensor_id_to_res_id_map == nullptr) {
      continue;
    }
    VLOG(1) << "Found " << sensor_id_to_res_id_map->size()
            << " optional sensors with this depth type.";

    // Integrate them one sensor at a time.
    for (const typename SensorsToResourceMap::value_type& sensor_to_res_ids :
         *sensor_id_to_res_id_map) {
      const backend::OptionalSensorResources& resource_buffer =
          sensor_to_res_ids.second;

      const SensorOrCameraId& sensor_or_camera_id = sensor_to_res_ids.first;

      // Get transformation between reference (e.g. IMU) and sensor.
      aslam::Transformation T_I_S;
      vi_map.getSensorManager().getSensorOrCamera_T_R_S(
          sensor_or_camera_id, &T_I_S);

      // Retrieve the camera as well if this function is called using an
      // aslam::CameraId. This is needed for depth maps to reproject them.
      const aslam::Camera::Ptr camera_ptr = getOptionalCameraIfCameraId(
          vi_map.getSensorManager(), use_undistorted_camera_for_depth_maps,
          sensor_or_camera_id);

      const size_t num_resources = resource_buffer.size();
      VLOG(1) << "Sensor " << sensor_or_camera_id.shortHex() << " has "
              << num_resources << " such resources.";

      // Collect all timestamps that need to be interpolated.
      Eigen::Matrix<int64_t, 1, Eigen::Dynamic> resource_timestamps(
          num_resources);
      size_t idx = 0u;
      for (const std::pair<int64_t, backend::ResourceId>& stamped_resource_id :
           resource_buffer.buffered_values()) {
        // If the resource timestamp does not lie within the min and max
        // timestamp of the vertices, we cannot interpolate the position. To
        // keep this efficient, we simply replace timestamps outside the range
        // with the min or max. Since their transformation will not be used
        // later, that's fine.
        resource_timestamps[idx] = std::max(
            min_timestamp_ns,
            std::min(max_timestamp_ns, stamped_resource_id.first));

        ++idx;
      }

      // Interpolate poses at resource timestamp.
      aslam::TransformationVector poses_M_I;
      pose_interpolator.getPosesAtTime(
          vi_map, mission_id, resource_timestamps, &poses_M_I);

      CHECK_EQ(poses_M_I.size(), resource_timestamps.size());
      CHECK_EQ(poses_M_I.size(), resource_buffer.size());

      // Retrieve and integrate all resources.
      idx = 0u;
      common::ProgressBar tsdf_progress_bar(resource_buffer.size());
      for (const std::pair<int64_t, backend::ResourceId>& stamped_resource_id :
           resource_buffer.buffered_values()) {
        tsdf_progress_bar.increment();

        // We assume the frame of reference for the sensor system is the IMU
        // frame.
        const aslam::Transformation& T_M_I = poses_M_I[idx];
        const aslam::Transformation T_G_S = T_G_M * T_M_I * T_I_S;
        ++idx;

        const int64_t timestamp_ns = stamped_resource_id.first;

        // If the resource timestamp does not lie within the min and max
        // timestamp of the vertices, we cannot interpolate the position.
        if (timestamp_ns < min_timestamp_ns ||
            timestamp_ns > max_timestamp_ns) {
          LOG(WARNING) << "The optional depth resource at " << timestamp_ns
                       << " is outside of the time range of the pose graph, "
                       << "skipping.";
          continue;
        }

        switch (input_resource_type) {
          case backend::ResourceType::kRawDepthMap:
          // Fall through intended.
          case backend::ResourceType::kOptimizedDepthMap: {
            CHECK(camera_ptr) << "For depth maps we should have "
                                 "retrieved a camera for reprojection, "
                                 "but the camera is not available!";
            const aslam::Camera& camera = *camera_ptr;

            cv::Mat depth_map;
            if (!vi_map.getOptionalSensorResource(
                    mission, input_resource_type, sensor_or_camera_id,
                    timestamp_ns, &depth_map)) {
              LOG(FATAL) << "Cannot retrieve optional depth map resources at "
                         << "timestamp " << timestamp_ns << "!";
            }

            // Check if there is a dedicated grayscale or color image for this
            // depth map.
            cv::Mat image;
            bool has_image = false;
            if (!vi_map.getOptionalSensorResource(
                    mission, backend::ResourceType::kImageForDepthMap,
                    sensor_or_camera_id, timestamp_ns, &depth_map)) {
              VLOG(3) << "Found depth map with intensity information "
                         "from the dedicated grayscale image.";
              has_image = true;
            } else if (
                !vi_map.getOptionalSensorResource(
                    mission, backend::ResourceType::kColorImageForDepthMap,
                    sensor_or_camera_id, timestamp_ns, &depth_map)) {
              VLOG(3) << "Found depth map with RGB information "
                         "from the dedicated color image.";
              has_image = true;
            } else {
              VLOG(3)
                  << "Found depth map without any color/intensity information.";
            }

            // Integrate with or without intensity information.
            if (has_image) {
              integrateDepthMap(
                  T_G_S, depth_map, image, camera, &tsdf_integrator);
            } else {
              integrateDepthMap(T_G_S, depth_map, camera, &tsdf_integrator);
            }
            continue;
          }
          case backend::ResourceType::kPointCloudXYZI:
          // Fall through intended.
          case backend::ResourceType::kPointCloudXYZ:
          // Fall through intended.
          case backend::ResourceType::kPointCloudXYZRGBN: {
            // Check if a point cloud is available.
            resources::PointCloud point_cloud;
            if (!vi_map.getOptionalSensorResource(
                    mission, input_resource_type, sensor_or_camera_id,
                    timestamp_ns, &point_cloud)) {
              LOG(ERROR) << "Cannot retrieve optional point cloud resources at "
                         << "timestamp " << timestamp_ns << "!";
                         continue;
            }

            VLOG(3) << "Found point cloud at timestamp " << timestamp_ns;
            integratePointCloud(T_G_S, point_cloud, &tsdf_integrator);
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

#endif  // VOXBLOX_INTERFACE_INTEGRATION_INL_H_
