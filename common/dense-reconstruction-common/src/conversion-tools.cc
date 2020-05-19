#include "dense-reconstruction/conversion-tools.h"

#include <memory>
#include <string>

#include <depth-integration/depth-integration.h>
#include <glog/logging.h>
#include <map-resources/resource-conversion.h>

namespace dense_reconstruction {

bool convertAllDepthMapsToPointClouds(vi_map::VIMap* vi_map) {
  CHECK_NOTNULL(vi_map);

  size_t num_conversions = 0u;

  depth_integration::IntegrationFunctionPointCloudMaplab integration_function =
      [&vi_map, &num_conversions](
          const vi_map::MissionId& mission_id, const int64_t timestamp_ns,
          const aslam::SensorId& sensor_id, const aslam::Transformation& T_G_S,
          const resources::PointCloud& points_S) {
        const backend::ResourceType point_cloud_type =
            backend::getResourceTypeForPointCloud(points_S);

        vi_map->addSensorResource(
            point_cloud_type, sensor_id, timestamp_ns, points_S,
            &vi_map->getMission(mission_id));

        ++num_conversions;
      };

  vi_map::MissionIdList mission_ids;
  vi_map->getAllMissionIds(&mission_ids);

  depth_integration::integrateAllDepthResourcesOfType(
      mission_ids, backend::ResourceType::kRawDepthMap, false, *vi_map,
      integration_function);

  depth_integration::integrateAllDepthResourcesOfType(
      mission_ids, backend::ResourceType::kOptimizedDepthMap, false, *vi_map,
      integration_function);

  VLOG(1) << "Done. Converted " << num_conversions << " depth maps.";
  return true;
}

}  // namespace dense_reconstruction
