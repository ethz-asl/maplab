#include "visualization/vertex-landmark-visibility-plotter.h"

#include <fstream>  // NOLINT
#include <limits>

#include <aslam/cameras/camera.h>
#include <glog/logging.h>
#include <maplab-common/conversions.h>
#include <maplab-common/quaternion-math.h>
#include <vi-map-helpers/vi-map-geometry.h>
#include <vi-map/landmark.h>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>

#include "visualization/common-rviz-visualization.h"
#include "visualization/viwls-graph-plotter.h"

DECLARE_bool(vis_color_by_mission);
DECLARE_int32(vis_color_salt);

namespace visualization {

VertexLandmarkVisibilityPlotter::VertexLandmarkVisibilityPlotter(
    const visualization::ViwlsGraphRvizPlotter::Ptr& plotter,
    const std::string& mission_id_string, const vi_map::VIMap& map)
    : plotter_(plotter),
      map_(map),
      map_queries_(map),
      vertex_rays_topic_("vertex_rays") {
  if (map_.numMissions() == 0u) {
    LOG(ERROR) << "No missions in database.";
    return;
  }

  vi_map::MissionId mission_id;
  mission_id.fromHexString(mission_id_string);
  if (!mission_id.isValid() || !map_.hasMission(mission_id)) {
    mission_id = map_.getIdOfFirstMission();
    LOG(WARNING) << "Mission ID not set or invalid, taking 1st mission ID: "
                 << mission_id.hexString();
  }

  root_vertex_id_ = map_.getMission(mission_id).getRootVertexId();
  current_vertex_id_ = root_vertex_id_;
  graph_traversal_edge_type_ = map_.getGraphTraversalEdgeType(mission_id);
  CHECK(root_vertex_id_.isValid());
  const vi_map::Vertex& vertex = map_.getVertex(root_vertex_id_);
  CHECK_GT(vertex.numFrames(), 0u);
  root_vertex_timestamp_ =
      vertex.getVisualFrame(kFirstFrame).getTimestampNanoseconds();

  vi_map::MissionIdList all_mission_ids;
  map.getAllMissionIds(&all_mission_ids);
  plotter_->publishBaseFrames(map_, all_mission_ids);
  plotter_->publishVertices(map_, all_mission_ids);
  plotter_->publishEdges(map_, all_mission_ids);
  plotter_->publishLandmarks(map_, all_mission_ids);
}

void VertexLandmarkVisibilityPlotter::publishVertexRaysAndIterate(
    bool absolute_time_scale) {
  const vi_map::Vertex& viwls_vertex = map_.getVertex(current_vertex_id_);

  // Publish the pose to TF:
  plotter_->publishVertexPoseAsTF(map_, current_vertex_id_);

  visualization::Vector3dList p_G_fi;
  std::vector<visualization::Color> segment_colors;

  if (!FLAGS_vis_color_by_mission) {
    int64_t timestamp_max;
    int64_t timestamp_min;
    getKeypointObservationTimeRange(
        viwls_vertex, absolute_time_scale, &timestamp_min, &timestamp_max);
    color_palette_ = visualization::GetPalette(
        visualization::Palette::PaletteTypes::kFalseColor1);
    map_queries_.forEachWellConstrainedObservedLandmark(
        viwls_vertex,
        [&](const size_t frame_idx, const vi_map::LandmarkId& landmark_id,
            const vi_map::Vertex& storing_vertex) {
          p_G_fi.emplace_back(map_.getLandmark_G_p_fi(landmark_id));

          int64_t timestamp = storing_vertex.getVisualFrame(frame_idx)
                                  .getTimestampNanoseconds();

          const double color_idx =
              (static_cast<double>(timestamp - timestamp_min) * 255 /
               timestamp_max);
          segment_colors.push_back(
              visualization::getPaletteColor(color_idx, color_palette_));
        });
  } else {
    map_queries_.forEachWellConstrainedObservedLandmark(
        viwls_vertex, [&](const size_t /*frame_idx*/,
                          const vi_map::LandmarkId& landmark_id,
                          const vi_map::Vertex& storing_vertex) {
          p_G_fi.emplace_back(map_.getLandmark_G_p_fi(landmark_id));

          const size_t index = storing_vertex.getMissionId().hashToSizeT() *
                               FLAGS_vis_color_salt;
          segment_colors.emplace_back(
              visualization::getPaletteColor(index, color_palette_));
        });
  }

  const double kScale = 0.01;
  const double kAlpha = 0.3;
  const size_t marker_id = viwls_vertex.getMissionId().hashToSizeT();
  visualization::publishLines(
      map_.getVertex_G_p_I(current_vertex_id_), p_G_fi, segment_colors, kAlpha,
      kScale, marker_id, visualization::kDefaultMapFrame,
      visualization::kDefaultNamespace, vertex_rays_topic_);

  iterateVertexAlongGraph();
}

void VertexLandmarkVisibilityPlotter::iterateVertexAlongGraph() {
  if (!map_.getNextVertex(
          current_vertex_id_, graph_traversal_edge_type_,
          &current_vertex_id_)) {
    reset();
  }
}

void VertexLandmarkVisibilityPlotter::getKeypointObservationTimeRange(
    const vi_map::Vertex& viwls_vertex, const bool absolute_time_scale,
    int64_t* timestamp_min, int64_t* timestamp_max) const {
  CHECK_NOTNULL(timestamp_min);
  CHECK_NOTNULL(timestamp_max);
  int64_t current_timestamp =
      viwls_vertex.getVisualFrame(kFirstFrame).getTimestampNanoseconds();
  if (absolute_time_scale) {
    *timestamp_min = root_vertex_timestamp_;
    *timestamp_max = current_timestamp;
  } else {
    *timestamp_max = std::numeric_limits<int64_t>::min();
    *timestamp_min = std::numeric_limits<int64_t>::max();

    map_queries_.forEachWellConstrainedObservedLandmark(
        viwls_vertex, [timestamp_min, timestamp_max](
                          const size_t frame_idx,
                          const vi_map::LandmarkId& /*landmark_id*/,
                          const vi_map::Vertex& storing_vertex) {
          int64_t timestamp = storing_vertex.getVisualFrame(frame_idx)
                                  .getTimestampNanoseconds();

          *timestamp_min = std::min(*timestamp_min, timestamp);
          *timestamp_max = std::max(*timestamp_max, timestamp);
        });
  }
  CHECK_GE(*timestamp_max, *timestamp_min);
}

}  // namespace visualization
