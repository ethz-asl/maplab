#include "visualization/viwls-graph-plotter.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include <aslam/common/memory.h>
#include <aslam/common/time.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/parallel-process.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensors/sensor.h>
#include <vi-map/vertex.h>
#include <vi-map/viwls-edge.h>

#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"

DEFINE_bool(
    vis_color_by_mission, true,
    "Color landmarks and pose-graph edges by mission.");
DEFINE_bool(
    vis_color_mission_with_unknown_baseframe_transformation, true,
    "Whether or not to color missions with unknown baseframe transformation.");
DEFINE_int32(vis_color_salt, 1, "Salt to multiply the hash with for coloring.");

DEFINE_double(vis_scale, 1, "Scale for visualizing edges and landmarks.");

DEFINE_int32(vis_landmark_gray_level, 170, "Gray level for landmark color.");
DEFINE_bool(
    vis_check_landmark_constrained, true,
    "Whether to only plot well-constrained landmarks or all landmarks.");
DEFINE_bool(
    vis_color_landmarks_by_number_of_observations, false,
    "Whether to color landmarks by number of observers.");
DEFINE_bool(
    vis_color_landmarks_by_observer_datasets, false,
    "Whether to color landmarks by number of observer_datasets.");
DEFINE_int32(
    vis_landmarks_max_observers, 5,
    "Maximum number of observers to use for coloring scale in "
    " --color_by_observers and --color_by_observer_datasets.");
DEFINE_int32(
    vis_landmarks_min_observers, 1,
    "Minimum number of observers to use for coloring scale in "
    " --color_by_observer_datasets.");
DEFINE_bool(
    vis_color_landmarks_by_time, false, "Whether to color landmarks by time.");
DEFINE_bool(
    vis_color_landmarks_by_first_observer_frame, false,
    "Whether to color landmarks by their first observer.");
DEFINE_double(
    vis_color_landmarks_by_time_period_seconds, 60,
    "The period for time coloring in seconds.");
DEFINE_bool(
    vis_color_landmarks_by_height, false,
    "Whether to color landmarks by height.");
DEFINE_double(
    vis_color_by_height_period_m, 3., "The period for coloring by height.");
DEFINE_double(
    vis_color_by_height_offset_m, 0., "The offset for coloring by height.");

DEFINE_double(vis_offset_x_m, 0.0, "Offset in x for RViz visualization");
DEFINE_double(vis_offset_y_m, 0.0, "Offset in y for RViz visualization");
DEFINE_double(vis_offset_z_m, 0.0, "Offset in z for RViz visualization");

DEFINE_int32(
    vis_landmarks_min_number_of_observer_missions, 1,
    "Minimum number of observer missions for a landmark to be "
    "visualized.");

namespace visualization {
const std::string ViwlsGraphRvizPlotter::kCamPredictionTopic =
    "cam_predictions";
const std::string ViwlsGraphRvizPlotter::kEdgeTopic =
    visualization::kViMapTopicHead + "_edges";
const std::string ViwlsGraphRvizPlotter::kBoundingBoxTopic = "bounding_box";
const std::string ViwlsGraphRvizPlotter::kBaseframeTopic =
    visualization::kViMapTopicHead + "_baseframe";
const std::string ViwlsGraphRvizPlotter::kVertexTopic =
    visualization::kViMapTopicHead + "_vertices";
const std::string ViwlsGraphRvizPlotter::kVertexPartitioningTopic =
    "vertex_partitioning";
const std::string ViwlsGraphRvizPlotter::kBoxTopic = "boxes";
const std::string ViwlsGraphRvizPlotter::kLoopclosureTopic = "loop_closures";
const std::string ViwlsGraphRvizPlotter::kLandmarkPairsTopic = "landmark_pairs";
const std::string ViwlsGraphRvizPlotter::kLandmarkTopic =
    visualization::kViMapTopicHead + "_landmarks";
const std::string ViwlsGraphRvizPlotter::kMeshTopic = "meshes";
const std::string ViwlsGraphRvizPlotter::kLandmarkNormalsTopic =
    "landmark_normals";
const std::string ViwlsGraphRvizPlotter::kSlidingWindowLocalizationResultTopic =
    "sliding_window_localization_result";
const std::string ViwlsGraphRvizPlotter::kUniqueKeyFramesTopic =
    "unique_key_frames";
const std::string ViwlsGraphRvizPlotter::kNcamExtrinsicsTopic =
    "ncam_extrinsics";
const std::string ViwlsGraphRvizPlotter::kSensorExtrinsicsTopic =
    "sensor_extrinsics";

ViwlsGraphRvizPlotter::ViwlsGraphRvizPlotter()
    : origin_(
          FLAGS_vis_offset_x_m, FLAGS_vis_offset_y_m, FLAGS_vis_offset_z_m) {}

void ViwlsGraphRvizPlotter::publishEdges(
    const vi_map::VIMap& map, const vi_map::MissionIdList& missions) const {
  visualization::Color color;
  color.red = 25;
  color.green = 200;
  color.blue = 255;

  for (const vi_map::MissionId& mission_id : missions) {
    vi_map::MissionIdList single_selected_mission = {mission_id};
    publishEdges(
        map, single_selected_mission, map.getGraphTraversalEdgeType(mission_id),
        color);
  }
  visualization::Color color_green;
  color_green.red = 25;
  color_green.green = 255;
  color_green.blue = 50;
  publishEdges(
      map, missions, pose_graph::Edge::EdgeType::kLoopClosure, color_green);

  visualization::Color color_blue;
  color_blue.red = 50;
  color_blue.green = 25;
  color_blue.blue = 200;
  publishEdges(
      map, missions, pose_graph::Edge::EdgeType::kOdometry, color_blue);
}

void ViwlsGraphRvizPlotter::publishEdges(
    const vi_map::VIMap& map, const vi_map::MissionIdList& missions,
    pose_graph::Edge::EdgeType edge_type,
    const visualization::Color& color_in) const {
  visualization::Color color = color_in;
  visualization::LineSegmentVector line_segments;

  for (const vi_map::MissionId& mission_id : missions) {
    pose_graph::EdgeIdList edges;
    map.getAllEdgeIdsInMissionAlongGraph(mission_id, &edges);

    // Visualize only edges of a defined type. Removes edges of different type.
    std::function<bool(pose_graph::EdgeId&)> edge_type_different =  // NOLINT
        [&](const pose_graph::EdgeId& edge_id) {
          return map.getEdgeType(edge_id) != edge_type;
        };  // NOLINT

    edges.erase(
        std::remove_if(edges.begin(), edges.end(), edge_type_different),
        edges.end());

    const unsigned int marker_id = mission_id.hashToSizeT();

    publishEdges(
        map, edges, color, marker_id,
        pose_graph::Edge::edgeTypeToString(edge_type));
  }
}

void ViwlsGraphRvizPlotter::publishEdges(
    const vi_map::VIMap& map, const pose_graph::EdgeIdList& edges,
    const visualization::Color& color, unsigned int marker_id,
    const std::string& topic_extension) const {
  visualization::LineSegmentVector line_segments;
  visualization::LineSegmentVector lc_transformation_line_segments;

  visualization::Palette palette =
      GetPalette(visualization::Palette::PaletteTypes::kFalseColor1);
  visualization::Color local_color = color;

  for (const pose_graph::EdgeId& edge_id : edges) {
    const pose_graph::Edge* edge_ptr =
        map.getEdgePtrAs<pose_graph::Edge>(edge_id);

    const vi_map::Vertex& vertex_from = map.getVertex(edge_ptr->from());
    const vi_map::Vertex& vertex_to = map.getVertex(edge_ptr->to());

    const Eigen::Vector3d& M_p_I_from = vertex_from.get_p_M_I() - origin_;
    const Eigen::Vector3d& M_p_I_to = vertex_to.get_p_M_I() - origin_;

    // Vertices linked by an edge should come from a single mission, but it's
    // safer to retrieve them separately.
    const vi_map::VIMission& mission_from =
        map.getMission(vertex_from.getMissionId());
    const vi_map::VIMission& mission_to =
        map.getMission(vertex_to.getMissionId());

    const vi_map::MissionBaseFrame& baseframe_from =
        map.getMissionBaseFrame(mission_from.getBaseFrameId());
    const vi_map::MissionBaseFrame& baseframe_to =
        map.getMissionBaseFrame(mission_to.getBaseFrameId());

    visualization::LineSegment line_segment;
    line_segment.from =
        baseframe_from.transformPointInMissionFrameToGlobalFrame(M_p_I_from);
    line_segment.to =
        baseframe_to.transformPointInMissionFrameToGlobalFrame(M_p_I_to);

    constexpr double kEdgeScalingFactor = 0.02;
    line_segment.scale = FLAGS_vis_scale * kEdgeScalingFactor;
    line_segment.alpha = 1.0;

    if (edge_ptr->getType() == pose_graph::Edge::EdgeType::kLoopClosure) {
      line_segment.scale = FLAGS_vis_scale * 0.02;
      const vi_map::LoopClosureEdge& edge =
          edge_ptr->getAs<vi_map::LoopClosureEdge>();
      const double switch_variable = edge.getSwitchVariable();
      CHECK_GE(switch_variable, 0.0);
      CHECK_LE(switch_variable, 1.0);

      local_color.red = static_cast<char>((1.0 - switch_variable) * 255.0);
      local_color.green = static_cast<char>(switch_variable * 255.0);
      local_color.blue = 50;

      visualization::LineSegment lc_line_segment;
      const aslam::Transformation from_T_M_I = vertex_from.get_T_M_I();
      const aslam::Transformation from_T_M_lc = from_T_M_I * edge.getT_A_B();
      lc_line_segment.from =
          baseframe_from.transformPointInMissionFrameToGlobalFrame(
              from_T_M_I.getPosition());
      lc_line_segment.to =
          baseframe_from.transformPointInMissionFrameToGlobalFrame(
              from_T_M_lc.getPosition());

      lc_line_segment.color.red = 200u;
      lc_line_segment.color.green = 50u;
      lc_line_segment.color.blue = 200u;
      lc_line_segment.scale = 0.1;
      lc_line_segment.alpha = 1.0;
      lc_transformation_line_segments.push_back(lc_line_segment);
      line_segment.color = local_color;
    } else {
      if (FLAGS_vis_color_by_mission) {
        const bool is_T_G_M_known =
            map.getMissionBaseFrame(mission_from.getBaseFrameId())
                .is_T_G_M_known();
        if (FLAGS_vis_color_mission_with_unknown_baseframe_transformation ||
            is_T_G_M_known) {
          const size_t index =
              mission_from.id().hashToSizeT() * FLAGS_vis_color_salt;
          line_segment.color = getPaletteColor(index, palette);
        } else {
          line_segment.color.red = line_segment.color.green =
              line_segment.color.blue = FLAGS_vis_landmark_gray_level;
        }
      }
    }
    line_segments.push_back(line_segment);
  }

  if (!line_segments.empty()) {
    visualization::publishLines(
        line_segments, marker_id, visualization::kDefaultMapFrame,
        visualization::kDefaultNamespace, kEdgeTopic + '/' + topic_extension);
  }
  if (!lc_transformation_line_segments.empty()) {
    visualization::publishLines(
        lc_transformation_line_segments, marker_id,
        visualization::kDefaultMapFrame, visualization::kDefaultNamespace,
        kEdgeTopic + "/loop_closure_transformations");
  }
}

void ViwlsGraphRvizPlotter::publishVertices(
    const vi_map::VIMap& map, const vi_map::MissionIdList& missions) const {
  pose_graph::VertexIdList all_vertices;
  for (const vi_map::MissionId& mission_id : missions) {
    pose_graph::VertexIdList mission_vertices;
    map.getAllVertexIdsInMission(mission_id, &mission_vertices);
    all_vertices.insert(
        all_vertices.end(), mission_vertices.begin(), mission_vertices.end());
  }
  publishVertices(map, all_vertices);
}

void ViwlsGraphRvizPlotter::publishVertices(
    const vi_map::VIMap& map, const pose_graph::VertexIdList& vertices) const {
  visualization::PoseVector poses;
  for (const pose_graph::VertexId& vertex_id : vertices) {
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);

    const Eigen::Vector3d M_p_I = vertex.get_p_M_I();
    const Eigen::Quaterniond M_q_I = vertex.get_q_M_I();

    const vi_map::VIMission& mission = map.getMission(vertex.getMissionId());

    const vi_map::MissionBaseFrame& baseframe =
        map.getMissionBaseFrame(mission.getBaseFrameId());

    visualization::Pose pose;
    pose.G_p_B =
        baseframe.transformPointInMissionFrameToGlobalFrame(M_p_I) - origin_;
    pose.G_q_B = baseframe.transformRotationInMissionFrameToGlobalFrame(M_q_I);

    pose.id = vertex_id.hashToSizeT();
    pose.scale = 0.15;
    pose.line_width = 0.01;
    pose.alpha = 0.6;

    poses.push_back(pose);
  }

  const std::string& kNamespace = "vertices";
  visualization::publishVerticesFromPoseVector(
      poses, visualization::kDefaultMapFrame, kNamespace, kVertexTopic);
}

void ViwlsGraphRvizPlotter::publishBaseFrames(
    const vi_map::VIMap& map, const vi_map::MissionIdList& missions) const {
  const size_t num_missions = missions.size();

  vi_map::MissionBaseFrameIdList baseframes_ids;
  baseframes_ids.reserve(num_missions);
  for (const vi_map::MissionId& mission_id : missions) {
    baseframes_ids.push_back(map.getMission(mission_id).getBaseFrameId());
  }
  publishBaseFrames(map, baseframes_ids);
}

void ViwlsGraphRvizPlotter::publishBaseFrames(
    const vi_map::VIMap& map,
    const vi_map::MissionBaseFrameIdList& baseframes) const {
  visualization::PoseVector poses;

  for (const vi_map::MissionBaseFrameId& baseframe_id : baseframes) {
    const vi_map::MissionBaseFrame& baseframe =
        map.getMissionBaseFrame(baseframe_id);

    visualization::Pose pose;
    pose.G_p_B = baseframe.get_p_G_M();
    pose.G_q_B = baseframe.get_q_G_M();
    pose.id = baseframe_id.hashToSizeT();
    pose.scale = 0.3;
    pose.line_width = 0.015;
    pose.alpha = 0.8;

    poses.push_back(pose);
  }

  const std::string kNamespace = "baseframes";
  visualization::publishVerticesFromPoseVector(
      poses, visualization::kDefaultMapFrame, kNamespace, kBaseframeTopic);
}

void ViwlsGraphRvizPlotter::publishLandmarks(
    const vi_map::VIMap& map, const vi_map::MissionIdList& missions) const {
  visualization::SphereVector spheres;
  appendLandmarksToSphereVector(map, missions, &spheres);

  visualization::publishSpheresAsPointCloud(
      spheres, visualization::kDefaultMapFrame, kLandmarkTopic);
}

void ViwlsGraphRvizPlotter::publishLandmarks(
    const Eigen::Matrix3Xd& p_G_landmarks) const {
  visualization::Color color;
  color.red = 200;
  color.green = 200;
  color.blue = 200;

  publishLandmarks(p_G_landmarks, color, kLandmarkTopic);
}

void ViwlsGraphRvizPlotter::publishLandmarks(
    const Eigen::Matrix3Xd& p_G_landmarks, const visualization::Color& color,
    const std::string& topic) const {
  const double kAlpha = 1.;
  if (!topic.empty()) {
    visualization::publish3DPointsAsPointCloud(
        p_G_landmarks, color, kAlpha, visualization::kDefaultMapFrame, topic);
  } else {
    visualization::publish3DPointsAsPointCloud(
        p_G_landmarks, color, kAlpha, visualization::kDefaultMapFrame,
        kLandmarkTopic);
  }
}

void ViwlsGraphRvizPlotter::appendLandmarksToSphereVector(
    const vi_map::VIMap& map, const vi_map::MissionIdList& missions,
    visualization::SphereVector* spheres) const {
  CHECK_NOTNULL(spheres);

  visualization::Color color;
  color.red = FLAGS_vis_landmark_gray_level;
  color.green = FLAGS_vis_landmark_gray_level;
  color.blue = FLAGS_vis_landmark_gray_level;

  for (const vi_map::MissionId& mission_id : missions) {
    pose_graph::VertexIdList vertices;
    map.getAllVertexIdsInMission(mission_id, &vertices);

    appendLandmarksToSphereVector(map, vertices, color, spheres);
  }
}

void ViwlsGraphRvizPlotter::appendLandmarksToSphereVector(
    const vi_map::VIMap& map, const pose_graph::VertexIdList& storing_vertices,
    const visualization::Color& color,
    visualization::SphereVector* spheres) const {
  CHECK_NOTNULL(spheres);

  const int kFrameSalt = 300;

  visualization::Palette palette =
      FLAGS_vis_color_landmarks_by_number_of_observations
          ? visualization::GetPalette(
                visualization::Palette::PaletteTypes::kLinearRed)
          : visualization::GetPalette(
                visualization::Palette::PaletteTypes::kFalseColor1);

  if (storing_vertices.empty()) {
    return;
  }

  const vi_map::MissionId& mission_id =
      map.getVertex(*storing_vertices.begin()).getMissionId();
  const vi_map::VIMission& mission = map.getMission(mission_id);
  const size_t start_timestamp_seconds = aslam::time::nanoSecondsToSeconds(
      map.getVertex(*storing_vertices.begin())
          .getVisualFrame(0)
          .getTimestampNanoseconds());

  VLOG_IF(2, FLAGS_vis_color_landmarks_by_number_of_observations == true)
      << "The more observations a landmark has the more red it is. Make "
      << "sure you have set the "
      << "'-vis_color_landmarks_by_number_of_observations' flag appropriately.";

  for (const pose_graph::VertexId& vertex_id : storing_vertices) {
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);
    CHECK_EQ(vertex.getMissionId(), mission_id)
        << "All vertices should belong "
        << "to the same mission with id " << mission_id.hexString();
    const vi_map::MissionBaseFrame& baseframe =
        map.getMissionBaseFrame(mission.getBaseFrameId());

    const vi_map::LandmarkStore& landmark_store = vertex.getLandmarks();
    for (const vi_map::Landmark& landmark : landmark_store) {
      if (FLAGS_vis_check_landmark_constrained &&
          landmark.getQuality() == vi_map::Landmark::Quality::kBad) {
        continue;
      }
      if (FLAGS_vis_landmarks_min_number_of_observer_missions > 1) {
        vi_map::MissionIdSet observer_missions;
        map.getLandmarkObserverMissions(landmark.id(), &observer_missions);
        if (observer_missions.size() <
            static_cast<size_t>(
                FLAGS_vis_landmarks_min_number_of_observer_missions)) {
          continue;
        }
      }
      visualization::Sphere sphere;
      Eigen::Vector3d LM_p_fi =
          vertex.getLandmark_p_LM_fi(landmark.id()) - origin_;

      sphere.position =
          baseframe.transformPointInMissionFrameToGlobalFrame(LM_p_fi);
      sphere.radius = 0.03;
      sphere.alpha = 0.8;

      if (FLAGS_vis_color_by_mission) {
        const vi_map::VIMission& mission =
            map.getMission(vertex.getMissionId());
        const bool is_T_G_M_known =
            map.getMissionBaseFrame(mission.getBaseFrameId()).is_T_G_M_known();
        if (FLAGS_vis_color_mission_with_unknown_baseframe_transformation ||
            is_T_G_M_known) {
          const size_t index =
              (vertex.getMissionId().hashToSizeT() * FLAGS_vis_color_salt) %
              visualization::kNumColors;
          sphere.color = getPaletteColor(index, palette);
        } else {
          sphere.color.red = sphere.color.green = sphere.color.blue =
              FLAGS_vis_landmark_gray_level;
        }
      } else if (FLAGS_vis_color_landmarks_by_number_of_observations) {
        CHECK_GT(FLAGS_vis_landmarks_max_observers, 0);
        sphere.color = getPaletteColor(
            std::min<double>(
                landmark.numberOfObservations() * 1.0 /
                    FLAGS_vis_landmarks_max_observers,
                1.0),
            palette);
      } else if (FLAGS_vis_color_landmarks_by_observer_datasets) {
        CHECK_GT(FLAGS_vis_landmarks_max_observers, 0);
        CHECK_GT(FLAGS_vis_landmarks_min_observers, 0);
        const size_t num_observer_missions =
            map.numLandmarkObserverMissions(landmark.id());
        if (static_cast<int>(num_observer_missions) <
            FLAGS_vis_landmarks_min_observers) {
          continue;
        }
        sphere.color = getPaletteColor(
            std::min<double>(
                static_cast<double>(num_observer_missions) /
                    FLAGS_vis_landmarks_max_observers,
                1.0),
            palette);
        VLOG_IF(
            2, num_observer_missions >
                   static_cast<size_t>(FLAGS_vis_landmarks_max_observers))
            << "Landmark is observed by more missions ("
            << num_observer_missions << ") than what is specified with the "
            << "'viz_max_observers' flag (" << FLAGS_vis_landmarks_max_observers
            << "). This likely leads to undesired plotting results.";
      } else if (FLAGS_vis_color_landmarks_by_time) {
        const size_t timestamp_seconds = aslam::time::nanoSecondsToSeconds(
            vertex.getVisualFrame(0).getTimestampNanoseconds());
        const size_t color_index =
            FLAGS_vis_color_salt *
            static_cast<size_t>(
                floor(
                    (timestamp_seconds - start_timestamp_seconds) /
                    FLAGS_vis_color_landmarks_by_time_period_seconds));
        sphere.color = getPaletteColor(color_index, palette);
      } else if (FLAGS_vis_color_landmarks_by_first_observer_frame) {
        const vi_map::KeypointIdentifierList& observations =
            landmark.getObservations();
        CHECK(!observations.empty());
        const vi_map::KeypointIdentifier& observation = observations[0];
        sphere.color = getPaletteColor(
            static_cast<size_t>(
                ((observation.frame_id.frame_index + 1) * kFrameSalt *
                 FLAGS_vis_color_salt)),
            palette);
      } else if (FLAGS_vis_color_landmarks_by_height) {
        const size_t color_index =
            FLAGS_vis_color_salt *
            static_cast<size_t>(
                (sphere.position(2) + FLAGS_vis_color_by_height_offset_m) /
                FLAGS_vis_color_by_height_period_m);
        sphere.color = getPaletteColor(color_index, palette);
      } else {
        sphere.color = color;
      }
      spheres->push_back(sphere);
    }
  }
}

void ViwlsGraphRvizPlotter::publishVertexPoseAsTF(
    const vi_map::VIMap& map, pose_graph::VertexId vertex_id) const {
  aslam::Transformation T_M_I = map.getVertex(vertex_id).get_T_M_I();
  publishTF(
      T_M_I, visualization::kDefaultMapFrame,
      "camera_" +
          map.getMissionIdForVertex(vertex_id).hexString().substr(0, 4));
}

void ViwlsGraphRvizPlotter::publishTF(
    aslam::Transformation T_G_I, const std::string& frame_id,
    const std::string& child_frame_id) const {
  visualization::publishTF(T_G_I, frame_id, child_frame_id);
}

void ViwlsGraphRvizPlotter::publishPosesInGlobalFrame(
    const aslam::TransformationVector& transformations) const {
  visualization::PoseVector pose_list;
  uint32_t id = 0;
  for (const aslam::Transformation& T_W_B : transformations) {
    visualization::Pose pose;
    // Convert it to a visualization pose.
    pose.G_p_B = T_W_B.getPosition();
    pose.G_q_B = T_W_B.getRotation().vector();
    pose.id = id++;
    pose.line_width = 0.05;
    pose_list.push_back(pose);
  }
  visualization::publishVerticesFromPoseVector(
      pose_list, visualization::kDefaultMapFrame,
      visualization::kDefaultNamespace, kVertexTopic);
}

void ViwlsGraphRvizPlotter::publishReferenceMap() const {
  CHECK(!reference_edges_line_segments_.empty());

  constexpr size_t kMarkerId = 2345u;
  visualization::publishLines(
      reference_edges_line_segments_, kMarkerId,
      visualization::kDefaultMapFrame, visualization::kDefaultNamespace,
      kEdgeTopic);
}

void ViwlsGraphRvizPlotter::setReferenceMap(const vi_map::VIMap& map) {
  visualization::Color reference_color;

  reference_color.red = 222;
  reference_color.green = 222;
  reference_color.blue = 222;

  pose_graph::EdgeIdList edges;
  map.getAllEdgeIds(&edges);

  for (const pose_graph::EdgeId& edge_id : edges) {
    const pose_graph::Edge* edge_ptr =
        map.getEdgePtrAs<pose_graph::Edge>(edge_id);

    const vi_map::Vertex& vertex_from = map.getVertex(edge_ptr->from());
    const vi_map::Vertex& vertex_to = map.getVertex(edge_ptr->to());

    const Eigen::Vector3d& M_p_I_from = vertex_from.get_p_M_I();
    const Eigen::Vector3d& M_p_I_to = vertex_to.get_p_M_I();

    // Vertices linked by an edge should come from a single mission, but it's
    // safer to retrieve them separately.
    const vi_map::VIMission& mission_from =
        map.getMission(vertex_from.getMissionId());
    const vi_map::VIMission& mission_to =
        map.getMission(vertex_to.getMissionId());

    const vi_map::MissionBaseFrame& baseframe_from =
        map.getMissionBaseFrame(mission_from.getBaseFrameId());
    const vi_map::MissionBaseFrame& baseframe_to =
        map.getMissionBaseFrame(mission_to.getBaseFrameId());

    visualization::LineSegment line_segment;
    line_segment.from =
        baseframe_from.transformPointInMissionFrameToGlobalFrame(M_p_I_from);
    line_segment.to =
        baseframe_to.transformPointInMissionFrameToGlobalFrame(M_p_I_to);
    line_segment.scale = FLAGS_vis_scale * 0.04;
    line_segment.alpha = 0.5;
    line_segment.color = reference_color;
    reference_edges_line_segments_.push_back(line_segment);
  }
}

void ViwlsGraphRvizPlotter::plotPartitioning(
    const vi_map::VIMap& map,
    const std::vector<pose_graph::VertexIdList>& partitioning) const {
  visualization::Palette palette =
      GetPalette(visualization::Palette::PaletteTypes::kFalseColor1);
  std::vector<visualization::Color> colors;

  CHECK(!partitioning.empty());
  const size_t num_partitions = partitioning.size();
  for (size_t cluster_index = 0u; cluster_index < num_partitions;
       ++cluster_index) {
    constexpr size_t kColorSeparationPrime = 4111;
    visualization::Color color =
        getPaletteColor(kColorSeparationPrime * cluster_index, palette);
    colors.push_back(color);
  }
  plotPartitioning(map, partitioning, colors);
}

void ViwlsGraphRvizPlotter::plotPartitioning(
    const vi_map::VIMap& map,
    const std::vector<pose_graph::VertexIdList>& partitioning,
    const std::vector<visualization::Color>& colors) const {
  visualization::SphereVector vertex_spheres;
  visualization::SphereVector landmark_spheres;

  const unsigned int num_vertices = map.numVertices();
  vertex_spheres.reserve(num_vertices);
  unsigned int cluster_index = 0;
  const double kLandmarkRadius = 0.03;
  const double kVertexRadius = 0.1;
  for (const pose_graph::VertexIdList& partition : partitioning) {
    for (const pose_graph::VertexId& vertex_id : partition) {
      const Eigen::Vector3d p_G_I = map.getVertex_G_p_I(vertex_id);

      visualization::Sphere vertex_sphere;
      vertex_sphere.position = p_G_I;
      vertex_sphere.radius = kVertexRadius;

      CHECK_EQ(colors.size(), partitioning.size());
      vertex_sphere.color = colors[cluster_index];
      vertex_spheres.push_back(vertex_sphere);

      VLOG(4) << "Vertex " << vertex_id
              << " belongs to cluster: " << cluster_index;

      const vi_map::LandmarkStore& landmark_store =
          map.getVertex(vertex_id).getLandmarks();
      landmark_spheres.reserve(landmark_store.size());

      for (const vi_map::Landmark& landmark : landmark_store) {
        visualization::Sphere landmark_sphere;
        Eigen::Vector3d G_p_fi = map.getLandmark_G_p_fi(landmark.id());

        landmark_sphere.position = G_p_fi;
        landmark_sphere.radius = kLandmarkRadius;
        landmark_sphere.alpha = 0.8;
        landmark_sphere.color = colors[cluster_index];
        landmark_spheres.push_back(landmark_sphere);
      }
    }
    ++cluster_index;
  }
  const size_t kMarkerId = 1234u;
  visualization::publishSpheres(
      vertex_spheres, kMarkerId, visualization::kDefaultMapFrame,
      visualization::kDefaultNamespace, kVertexPartitioningTopic);
  visualization::publishSpheresAsPointCloud(
      landmark_spheres, visualization::kDefaultMapFrame, kLandmarkTopic);
}

void ViwlsGraphRvizPlotter::plotVisualFramePartitioning(
    const vi_map::VIMap& map,
    const vi_map::FrameIdToFrameIdentifierMap& frame_to_frame_identifier_map,
    const std::vector<aslam::FrameIdList>& partitioning) const {
  visualization::PoseVector vertex_poses;

  visualization::Palette palette =
      GetPalette(visualization::Palette::PaletteTypes::kFalseColor1);

  const unsigned int num_frames = frame_to_frame_identifier_map.size();
  vertex_poses.reserve(num_frames);
  unsigned int cluster_index = 0u;

  CHECK(!partitioning.empty());

  for (const aslam::FrameIdList& partition : partitioning) {
    for (const aslam::FrameId& frame_id : partition) {
      const vi_map::VisualFrameIdentifier& identifier =
          frame_to_frame_identifier_map.at(frame_id);
      const pose_graph::VertexId& vertex_id = identifier.vertex_id;
      const unsigned int frame_idx = identifier.frame_index;
      const aslam::Transformation& T_G_I = map.getVertex_T_G_I(vertex_id);
      const vi_map::Vertex& vertex = map.getVertex(vertex_id);
      const aslam::VisualNFrame& nframe = vertex.getVisualNFrame();
      const aslam::Transformation& T_C_I = nframe.get_T_C_B(frame_idx);
      aslam::Transformation T_G_C = T_G_I * T_C_I.inverse();

      visualization::Pose pose;
      pose.G_p_B = T_G_C.getPosition();
      pose.G_q_B = T_G_C.getRotation().toImplementation();
      pose.scale = 0.3;
      pose.line_width = 0.03;
      pose.alpha = 0.8;
      pose.id = frame_id.hashToSizeT();

      visualization::Color color = getPaletteColor(
          static_cast<double>(cluster_index) / partitioning.size(), palette);
      pose.color.red = color.red;
      pose.color.green = color.green;
      pose.color.blue = color.blue;

      vertex_poses.push_back(pose);
    }
    ++cluster_index;
  }

  visualization::publishVerticesFromPoseVector(
      vertex_poses, visualization::kDefaultMapFrame,
      visualization::kDefaultNamespace, kVertexTopic);
}

void ViwlsGraphRvizPlotter::publishStructureMatches(
    const Eigen::Vector3d& G_vertex_position,
    const Eigen::Matrix3Xd& G_landmarks) const {
  CHECK_GT(G_landmarks.cols(), 0);

  visualization::LineSegmentVector line_segments;
  line_segments.resize(G_landmarks.cols());

  visualization::Color color;
  color.red = 85;
  color.green = 195;
  color.blue = 220;

  for (int i = 0; i < G_landmarks.cols(); ++i) {
    visualization::LineSegment& line_segment = line_segments[i];
    line_segment.from = G_vertex_position;
    line_segment.to = G_landmarks.col(i);

    line_segment.scale = FLAGS_vis_scale * 0.02;
    line_segment.alpha = 0.3;
    line_segment.color = color;
  }

  const size_t kMarkerId = 0u;
  visualization::publishLines(
      line_segments, kMarkerId, visualization::kDefaultMapFrame,
      visualization::kDefaultNamespace, kLoopclosureTopic);
}

void ViwlsGraphRvizPlotter::visualizeMap(const vi_map::VIMap& map) const {
  constexpr bool kPlotBaseframes = true;
  constexpr bool kPlotVertices = true;
  constexpr bool kPlotEdges = true;
  constexpr bool kPlotLandmarks = true;
  visualizeMap(map, kPlotBaseframes, kPlotVertices, kPlotEdges, kPlotLandmarks);
}

void ViwlsGraphRvizPlotter::visualizeMap(
    const vi_map::VIMap& map, bool publish_baseframes, bool publish_vertices,
    bool publish_edges, bool publish_landmarks) const {
  vi_map::MissionIdList all_missions;
  map.getAllMissionIds(&all_missions);
  visualizeMissions(
      map, all_missions, publish_baseframes, publish_vertices, publish_edges,
      publish_landmarks);
}

void ViwlsGraphRvizPlotter::visualizeMissions(
    const vi_map::VIMap& map, const vi_map::MissionIdList& mission_ids,
    bool publish_baseframes, bool publish_vertices, bool publish_edges,
    bool publish_landmarks) const {
  if (mission_ids.empty()) {
    LOG(ERROR) << "No missions in database.";
    return;
  }

  Aligned<std::vector, visualization::SphereVector> mission_spheres(
      mission_ids.size());

  // For memory saving reasons we publish one mission at a time.
  std::function<void(const std::vector<size_t>&)> visualizer =
      [&, this](const std::vector<size_t>& batch) {
        for (size_t item : batch) {
          const vi_map::MissionId& mission_id = mission_ids[item];
          if (publish_baseframes) {
            publishBaseFrames(map, {mission_id});
          }
          if (publish_vertices) {
            publishVertices(map, {mission_id});
          }
          if (publish_edges) {
            publishEdges(map, {mission_id});
          }
          if (publish_landmarks) {
            // The landmarks have to be published together since rviz displays
            // just
            // one point cloud at a time.
            appendLandmarksToSphereVector(
                map, {mission_id}, &mission_spheres[item]);
          }
        }
      };

  static constexpr bool kAlwaysParallelize = true;
  const size_t num_threads = common::getNumHardwareThreads();
  common::ParallelProcess(
      mission_ids.size(), visualizer, kAlwaysParallelize, num_threads);

  visualization::SphereVector all_spheres;
  for (const visualization::SphereVector& spheres : mission_spheres) {
    all_spheres.insert(all_spheres.end(), spheres.begin(), spheres.end());
  }
  visualization::publishSpheresAsPointCloud(
      all_spheres, visualization::kDefaultMapFrame, kLandmarkTopic);
}

void ViwlsGraphRvizPlotter::plotSlidingWindowLocalizationResult(
    const aslam::Transformation& T_G_B, size_t marker_id) const {
  visualization::SphereVector spheres;
  visualization::Sphere sphere;
  sphere.radius = 0.1;
  sphere.position = T_G_B.getPosition();
  sphere.color.red = 0;
  sphere.color.blue = 200;
  sphere.color.green = 50;
  spheres.emplace_back(sphere);

  visualization::publishSpheres(
      spheres, marker_id, visualization::kDefaultMapFrame,
      visualization::kDefaultNamespace, kSlidingWindowLocalizationResultTopic);
}

void ViwlsGraphRvizPlotter::publishCamPredictions(
    const vi_map_helpers::NearCameraPoseSampling& sampling,
    const std::vector<double>& predictions) {
  const size_t num_samples = sampling.size();
  CHECK_EQ(num_samples, predictions.size());

  std::vector<double> temp_predictions(predictions.begin(), predictions.end());
  std::nth_element(
      temp_predictions.begin(),
      temp_predictions.begin() + (3 * temp_predictions.size() / 4),
      temp_predictions.end());
  const double upper_quartile =
      temp_predictions[3 * temp_predictions.size() / 4];
  CHECK_GT(upper_quartile, 0.);

  aslam::TransformationVector T_G_Cs;
  T_G_Cs.insert(T_G_Cs.end(), sampling.begin(), sampling.end());

  std::vector<visualization::Color> colors(num_samples);
  for (size_t sample_idx = 0u; sample_idx < num_samples; ++sample_idx) {
    visualization::Color& color = colors[sample_idx];
    color.red = static_cast<unsigned char>(
        255.0 *
        (1.0 - std::min(predictions[sample_idx] / upper_quartile, 1.0)));
    color.green = static_cast<unsigned char>(
        255.0 * (std::min(predictions[sample_idx] / upper_quartile, 1.0)));
    color.blue = 0u;
  }

  const double kAlpha = 1.0;
  visualization::publishTransformations(
      T_G_Cs, colors, kAlpha, visualization::kDefaultMapFrame,
      visualization::kDefaultNamespace, kCamPredictionTopic);
}

void ViwlsGraphRvizPlotter::visualizeNCameraExtrinsics(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id) const {
  CHECK(map.hasMission(mission_id));
  const vi_map::VIMission& mission = map.getMission(mission_id);
  const aslam::NCamera& ncamera =
      map.getSensorManager().getNCameraForMission(mission_id);
  const aslam::Transformation& T_G_I0 =
      map.getVertex_T_G_I(mission.getRootVertexId());
  const size_t kImuMarkerId = 0u;
  visualization::publishCoordinateFrame(
      T_G_I0, "IMU", kImuMarkerId, kNcamExtrinsicsTopic);
  const size_t num_cameras = ncamera.getNumCameras();
  for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
    const aslam::Transformation& T_C_I = ncamera.get_T_C_B(camera_idx);
    const aslam::Transformation T_G_C = T_G_I0 * T_C_I.inverse();
    visualization::publishCoordinateFrame(
        T_G_C, 'C' + std::to_string(camera_idx), camera_idx + 1u,
        kNcamExtrinsicsTopic);
  }
}

void ViwlsGraphRvizPlotter::visualizeAllOptionalSensorsExtrinsics(
    const vi_map::VIMap& map) {
  // First, get the transformation between the global frame and the first
  // vertex of some mission.
  vi_map::MissionIdList all_mission_ids;
  map.getAllMissionIds(&all_mission_ids);
  if (all_mission_ids.empty()) {
    LOG(ERROR) << "The loaded map does not contain any missions. Aborting.";
    return;
  }
  const aslam::Transformation T_G_I = map.getVertex_T_G_I(
      map.getMission(all_mission_ids.front()).getRootVertexId());

  const vi_map::SensorManager& sensor_manager = map.getSensorManager();
  vi_map::SensorIdSet all_sensors_ids;
  sensor_manager.getAllSensorIds(&all_sensors_ids);

  size_t marker_id = 0u;
  visualization::publishCoordinateFrame(
      T_G_I, "IMU", marker_id++, kSensorExtrinsicsTopic);
  for (const vi_map::SensorId& sensor_id : all_sensors_ids) {
    CHECK(sensor_id.isValid());

    aslam::Transformation T_R_S;
    CHECK(sensor_manager.getSensor_T_R_S(sensor_id, &T_R_S));
    const aslam::Transformation T_G_S = T_G_I * T_R_S;

    const vi_map::Sensor& sensor = sensor_manager.getSensor(sensor_id);
    const std::string& label = sensor.getHardwareId();
    visualization::publishCoordinateFrame(
        T_G_S, label, marker_id++, kSensorExtrinsicsTopic);
  }
}

}  // namespace visualization
