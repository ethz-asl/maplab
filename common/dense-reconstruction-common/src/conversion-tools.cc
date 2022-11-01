#include "dense-reconstruction/conversion-tools.h"

#include <memory>
#include <string>

#include <glog/logging.h>
#include <map-resources/resource-conversion.h>

namespace dense_reconstruction {

bool convertAllDepthMapsToPointClouds(vi_map::VIMap* vi_map) {
  CHECK_NOTNULL(vi_map);

  VLOG(1) << "Converting all depth maps to point clouds...";
  size_t num_conversions = 0u;
  vi_map->forEachVertex([&](vi_map::Vertex* vertex) {
    CHECK_NOTNULL(vertex);
    const vi_map::MissionId& mission_id = vertex->getMissionId();
    const aslam::NCamera& n_camera = vi_map->getMissionNCamera(mission_id);

    const pose_graph::VertexId& vertex_id = vertex->id();
    const size_t num_frames = vertex->numFrames();
    const aslam::VisualNFrame& nframe = vertex->getVisualNFrame();

    for (size_t frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
      if (nframe.isFrameSet(frame_idx)) {
        cv::Mat depth_map;
        if (vi_map->getFrameResource(
                *vertex, frame_idx, backend::ResourceType::kOptimizedDepthMap,
                &depth_map)) {
          // Nothing to do here.
        } else if (vi_map->getFrameResource(
                       *vertex, frame_idx, backend::ResourceType::kRawDepthMap,
                       &depth_map)) {
          // Nothing to do here.
        } else {
          continue;
        }
        CHECK(!depth_map.empty()) << "Vertex " << vertex_id << " frame "
                                  << frame_idx << " has an empty depth map!";

        const aslam::Camera& camera = n_camera.getCamera(frame_idx);

        resources::PointCloud point_cloud;
        cv::Mat image_for_depth_map;
        if (vi_map->getFrameResource(
                *vertex, frame_idx, backend::ResourceType::kImageForDepthMap,
                &image_for_depth_map)) {
          CHECK(!image_for_depth_map.empty())
              << "Vertex " << vertex_id << " frame " << frame_idx
              << " has an empty image for the depth map!";
          if (backend::convertDepthMapWithImageToPointCloud(
                  depth_map, image_for_depth_map, camera, &point_cloud)) {
            vi_map->storeFrameResource(
                point_cloud, frame_idx,
                backend::ResourceType::kPointCloudXYZRGBN, vertex);
            ++num_conversions;
          }
        } else {
          if (backend::convertDepthMapToPointCloud(
                  depth_map, camera, &point_cloud)) {
            vi_map->storeFrameResource(
                point_cloud, frame_idx, backend::ResourceType::kPointCloudXYZ,
                vertex);
            ++num_conversions;
          }
        }
      }
    }
  });
  VLOG(1) << "Done. Converted " << num_conversions << " depth maps.";
  return true;
}

}  // namespace dense_reconstruction
