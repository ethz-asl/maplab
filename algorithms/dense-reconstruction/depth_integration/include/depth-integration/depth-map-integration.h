#ifndef DEPTH_INTEGRATION_DEPTH_MAP_INTEGRATION_H_
#define DEPTH_INTEGRATION_DEPTH_MAP_INTEGRATION_H_

#include <functional>
#include <unordered_set>

#include <vi-map/vi-map.h>
#include <voxblox/core/common.h>

namespace depth_integration {

typedef std::function<void(
    const aslam::Transformation&, const cv::Mat&, const cv::Mat&,
    const aslam::Camera&)>
    DepthMapIntegrationFunction;

static std::unordered_set<backend::ResourceType, backend::ResourceTypeHash>
    kSupportedDepthMapInputTypes{backend::ResourceType::kRawDepthMap,
                                 backend::ResourceType::kOptimizedDepthMap};

// Calls the integration function for all depth (optional and frame) resources.
void integrateAllDepthMapResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const vi_map::VIMap& vi_map,
    DepthMapIntegrationFunction integration_function);

// Calls the integration function for all depth frame resources from the
// selected missions using the integration function
void integrateAllFrameDepthMapResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const vi_map::VIMap& vi_map,
    DepthMapIntegrationFunction integration_function);

// Calls the integration function for all optional depth resources from the
// selected missions using the integration function.
void integrateAllOptionalSensorDepthMapResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const vi_map::VIMap& vi_map,
    DepthMapIntegrationFunction integration_function);

}  // namespace depth_integration

#endif  // DEPTH_INTEGRATION_DEPTH_MAP_INTEGRATION_H_
