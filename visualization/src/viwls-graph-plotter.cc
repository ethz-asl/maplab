#include "visualization/viwls-graph-plotter.h"

#include <algorithm>
#include <aslam/common/memory.h>
#include <aslam/common/time.h>
#include <functional>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <limits>
#include <maplab-common/parallel-process.h>
#include <sensors/sensor-types.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <unordered_map>
#include <unordered_set>
#include <vi-map/vertex.h>
#include <vi-map/viwls-edge.h>
#include <visualization_msgs/MarkerArray.h>

#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include "visualization/resource-visualization.h"

DEFINE_bool(
    vis_color_by_mission, true,
    "Color landmarks and pose-graph edges by mission.");
DEFINE_bool(
    vis_color_by_landmark_type, false, "Color landmarks by feature type.");
DEFINE_string(
    vis_show_only_landmark_type, "",
    "Only show landmarks with a certain feature type. Empty string means all "
    "landmark types are displayed.");
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
    vis_offset_landmarks_by_first_observer_frame_m, 0,
    "Separates landmarks by first observer frame with the given offset along "
    "the x axis.");
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

DEFINE_bool(
    vis_lc_edge_covariances, false,
    "Whether to visualize the loop closure edge covariances.");
DEFINE_bool(
    vis_abs_6dof_constraint_covariances, false,
    "Whether to visualize the absolute_6dof_constraint covariances.");

DEFINE_double(vis_offset_x_m, 0.0, "Offset in x for RViz visualization");
DEFINE_double(vis_offset_y_m, 0.0, "Offset in y for RViz visualization");
DEFINE_double(vis_offset_z_m, 0.0, "Offset in z for RViz visualization");

DEFINE_int32(
    vis_landmarks_min_number_of_observer_missions, 1,
    "Minimum number of observer missions for a landmark to be "
    "visualized.");

DEFINE_bool(
    vis_wait_for_traversal_edge_subscriber, false,
    "If true, the RViz publisher waits for a subscriber before publishing the "
    "traversal edges of a mission.");

namespace visualization {
const std::string ViwlsGraphRvizPlotter::kBaseframeTopic =
    visualization::kViMapTopicHead + "_baseframe";
const std::string ViwlsGraphRvizPlotter::kBoundingBoxTopic = "bounding_box";
const std::string ViwlsGraphRvizPlotter::kBoxTopic = "boxes";
const std::string ViwlsGraphRvizPlotter::kCamPredictionTopic =
    "cam_predictions";
const std::string ViwlsGraphRvizPlotter::kEdgeTopic =
    visualization::kViMapTopicHead + "_edges";
const std::string ViwlsGraphRvizPlotter::kLandmarkNormalsTopic =
    "landmark_normals";
const std::string ViwlsGraphRvizPlotter::kLandmarkPairsTopic = "landmark_pairs";
const std::string ViwlsGraphRvizPlotter::kLandmarkTopic =
    visualization::kViMapTopicHead + "_landmarks";
const std::string ViwlsGraphRvizPlotter::kLoopclosureTopic = "loop_closures";
const std::string ViwlsGraphRvizPlotter::kMeshTopic = "meshes";
const std::string ViwlsGraphRvizPlotter::kSensorExtrinsicsTopic =
    "sensor_extrinsics";
const std::string ViwlsGraphRvizPlotter::kSlidingWindowLocalizationResultTopic =
    "sliding_window_localization_result";
const std::string ViwlsGraphRvizPlotter::kUniqueKeyFramesTopic =
    "unique_key_frames";
const std::string ViwlsGraphRvizPlotter::kVertexPartitioningTopic =
    "vertex_partitioning";
const std::string ViwlsGraphRvizPlotter::kVertexTopic =
    visualization::kViMapTopicHead + "_vertices";
const std::string ViwlsGraphRvizPlotter::kAbsolute6DoFTopic =
    "absolute_6dof_constraints";

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
        color, FLAGS_vis_wait_for_traversal_edge_subscriber);
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
      map, missions, pose_graph::Edge::EdgeType::kWheelOdometry, color_blue);

  visualization::Color color_red;
  color_red.red = 255;
  color_red.green = 0;
  color_red.blue = 0;
  publishEdges(map, missions, pose_graph::Edge::EdgeType::kOdometry, color_red);
}

void ViwlsGraphRvizPlotter::publishEdges(
    const vi_map::VIMap& map, const vi_map::MissionIdList& missions,
    pose_graph::Edge::EdgeType edge_type, const visualization::Color& color_in,
    const bool wait_for_subscriber) const {
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
        pose_graph::Edge::edgeTypeToString(edge_type), wait_for_subscriber);
  }
}

void ViwlsGraphRvizPlotter::publishEdges(
    const vi_map::VIMap& map, const pose_graph::EdgeIdList& edges,
    const visualization::Color& color, unsigned int marker_id,
    const std::string& topic_extension, const bool wait_for_subscriber) const {
  visualization::LineSegmentVector line_segments;
  visualization::LineSegmentVector lc_transformation_line_segments;
  visualization::SphereVector lc_vertices;
  visualization::ArrowVector wheel_odom_transformation_arrows;
  visualization::ArrowVector odom_6dof_transformation_arrows;

  visualization::Palette palette =
      GetPalette(visualization::Palette::PaletteTypes::kFalseColor1);
  visualization::Color local_color = color;

  std::vector<aslam::Transformation> lc_edges_T_G_B;
  std::vector<aslam::TransformationCovariance> lc_edges_B_cov;

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

      // Visualize loop closure transformation as a line connecting the 'to'
      // vertex with the position relative to the 'from' enforced by the lc
      // edge.
      visualization::LineSegment lc_line_segment;
      const aslam::Transformation from_T_G_I =
          map.getVertex_T_G_I(edge_ptr->from());
      const aslam::Transformation to_T_G_I =
          map.getVertex_T_G_I(edge_ptr->to());
      const aslam::Transformation to_T_G_I_lc = from_T_G_I * edge.get_T_A_B();
      lc_line_segment.from = to_T_G_I.getPosition();
      lc_line_segment.to = to_T_G_I_lc.getPosition();
      lc_line_segment.color.red = 200u;
      lc_line_segment.color.green = 50u;
      lc_line_segment.color.blue = 200u;
      lc_line_segment.scale = 0.05;
      lc_line_segment.alpha = 1.0;
      lc_transformation_line_segments.push_back(lc_line_segment);
      line_segment.color = local_color;

      visualization::Sphere lc_sphere_from, lc_sphere_to;
      lc_sphere_from.position = from_T_G_I.getPosition();
      lc_sphere_from.color.red = 0u;
      lc_sphere_from.color.green = 255u;
      lc_sphere_from.color.blue = 255u;
      lc_sphere_from.radius = 0.3;
      lc_sphere_from.alpha = 0.3;
      lc_vertices.emplace_back(lc_sphere_from);
      lc_sphere_to.position = to_T_G_I.getPosition();
      lc_sphere_to.color.red = 255u;
      lc_sphere_to.color.green = 0u;
      lc_sphere_to.color.blue = 255u;
      lc_sphere_to.radius = 0.3;
      lc_sphere_to.alpha = 0.3;
      lc_vertices.emplace_back(lc_sphere_to);

      // Assemble the transformation and covariance for later visualization.
      lc_edges_T_G_B.emplace_back(to_T_G_I);
      lc_edges_B_cov.emplace_back(edge.get_T_A_B_Covariance());
    } else if (
        edge_ptr->getType() == pose_graph::Edge::EdgeType::kWheelOdometry &&
        map.getMission(vertex_from.getMissionId()).hasWheelOdometrySensor()) {
      line_segment.scale = FLAGS_vis_scale * 0.02;
      line_segment.color = local_color;
      const vi_map::TransformationEdge& edge =
          edge_ptr->getAs<vi_map::TransformationEdge>();
      // Visualize wheel odom transformation as an arrow connecting the 'to'
      // vertex with the position relative to the 'from' enforced by the odom
      // edge.
      aslam::Transformation T_B_S = map.getSensorManager().getSensor_T_B_S(
          mission_from.getWheelOdometrySensor());
      visualization::Arrow wheel_odom_arrow;
      // assuming I == B
      const aslam::Transformation T_G_S_from =
          map.getVertex_T_G_I(edge_ptr->from()) * T_B_S;
      const aslam::Transformation T_G_S_to =
          map.getVertex_T_G_I(edge_ptr->to()) * T_B_S;
      wheel_odom_arrow.from = T_G_S_from.getPosition();
      const aslam::Transformation T_G_measurement =
          T_G_S_from * edge.get_T_A_B();

      wheel_odom_arrow.color.red = 200u;
      wheel_odom_arrow.color.green = 50u;
      wheel_odom_arrow.color.blue = 200u;
      wheel_odom_arrow.scale = 0.1;
      wheel_odom_arrow.alpha = 1.0;
      wheel_odom_arrow.from = T_G_S_from.getPosition();
      wheel_odom_arrow.to = T_G_measurement.getPosition();
      wheel_odom_transformation_arrows.push_back(wheel_odom_arrow);
    } else if (
        edge_ptr->getType() == pose_graph::Edge::EdgeType::kOdometry &&
        map.getMission(vertex_from.getMissionId()).hasOdometry6DoFSensor()) {
      line_segment.scale = FLAGS_vis_scale * 0.02;
      line_segment.color = local_color;
      const vi_map::TransformationEdge& edge =
          edge_ptr->getAs<vi_map::TransformationEdge>();
      // Visualize 6DoF odom transformation as an arrow connecting the 'to'
      // vertex with the position relative to the 'from' enforced by the odom
      // edge.
      aslam::Transformation T_B_S = map.getSensorManager().getSensor_T_B_S(
          mission_from.getOdometry6DoFSensor());
      visualization::Arrow odom_6dof_arrow;
      // assuming I == B
      const aslam::Transformation T_G_S_from =
          map.getVertex_T_G_I(edge_ptr->from()) * T_B_S;
      const aslam::Transformation T_G_S_to =
          map.getVertex_T_G_I(edge_ptr->to()) * T_B_S;
      odom_6dof_arrow.from = T_G_S_from.getPosition();
      const aslam::Transformation T_G_measurement =
          T_G_S_from * edge.get_T_A_B();

      odom_6dof_arrow.color.red = local_color.red;
      odom_6dof_arrow.color.green = local_color.green;
      odom_6dof_arrow.color.blue = local_color.blue;
      odom_6dof_arrow.scale = 0.1;
      odom_6dof_arrow.alpha = 1.0;
      odom_6dof_arrow.from = T_G_S_from.getPosition();
      odom_6dof_arrow.to = T_G_measurement.getPosition();
      odom_6dof_transformation_arrows.push_back(odom_6dof_arrow);
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
        line_segments, marker_id, FLAGS_tf_map_frame,
        FLAGS_vis_default_namespace, kEdgeTopic + '/' + topic_extension,
        wait_for_subscriber);
  }
  if (!lc_transformation_line_segments.empty()) {
    visualization::publishLines(
        lc_transformation_line_segments, marker_id, FLAGS_tf_map_frame,
        FLAGS_vis_default_namespace,
        kEdgeTopic + "/loop_closure_transformations");
  }
  if (!wheel_odom_transformation_arrows.empty()) {
    visualization::publishArrows(
        wheel_odom_transformation_arrows, marker_id, FLAGS_tf_map_frame,
        FLAGS_vis_default_namespace, kEdgeTopic + "/wheel_odometry_arrows");
  }
  if (!odom_6dof_transformation_arrows.empty()) {
    visualization::publishArrows(
        odom_6dof_transformation_arrows, marker_id, FLAGS_tf_map_frame,
        FLAGS_vis_default_namespace, kEdgeTopic + "/6dof_odometry_arrows");
  }

  if (!lc_edges_T_G_B.empty() && !lc_vertices.empty() &&
      FLAGS_vis_lc_edge_covariances) {
    CHECK_EQ(lc_edges_T_G_B.size(), lc_edges_B_cov.size());
    const visualization::Color covariance_color = visualization::kCommonYellow;
    const std::string kEdgeCovTopic = kEdgeTopic + "/loop_closure_covariances";
    const std::string kEdgeCovNamespace = "loop_closure_covariances";
    visualization::publishPoseCovariances(
        lc_edges_T_G_B, lc_edges_B_cov, covariance_color, FLAGS_tf_map_frame,
        kEdgeCovNamespace, kEdgeCovTopic);

    const std::string kEdgeVertexTopic = kEdgeTopic + "/loop_closure_vertices";
    const std::string kEdgeVertexNamespace = "loop_closure_vertices";
    visualization::publishSpheres(
        lc_vertices, marker_id, FLAGS_tf_map_frame, kEdgeVertexNamespace,
        kEdgeVertexTopic);
  }
}

void ViwlsGraphRvizPlotter::publishAbsolute6DoFConstraints(
    const vi_map::VIMap& map, const vi_map::MissionIdList& missions) const {
  pose_graph::VertexIdList all_vertices;
  for (const vi_map::MissionId& mission_id : missions) {
    const vi_map::VIMission& mission = map.getMission(mission_id);
    if (!mission.hasAbsolute6DoFSensor()) {
      continue;
    }

    pose_graph::VertexIdList mission_vertices;
    map.getAllVertexIdsInMission(mission_id, &mission_vertices);
    all_vertices.insert(
        all_vertices.end(), mission_vertices.begin(), mission_vertices.end());
  }
  publishAbsolute6DoFConstraints(map, all_vertices);
}

void ViwlsGraphRvizPlotter::publishAbsolute6DoFConstraints(
    const vi_map::VIMap& map, const pose_graph::VertexIdList& vertices) const {
  Eigen::Matrix3Xd all_lines_start_G = Eigen::Matrix3Xd(3, 0);
  Eigen::Matrix3Xd all_lines_end_G = Eigen::Matrix3Xd(3, 0);
  std::vector<visualization::Color> all_line_colors;

  std::vector<aslam::Transformation> absolute_constraints_T_G_S_vec;
  std::vector<aslam::TransformationCovariance> absolute_constraints_S_cov_vec;

  size_t total_num_constraints = 0u;

  for (const pose_graph::VertexId& vertex_id : vertices) {
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);
    const vi_map::VIMission& mission = map.getMission(vertex.getMissionId());
    if (!mission.hasAbsolute6DoFSensor() ||
        !vertex.hasAbsolute6DoFMeasurements()) {
      continue;
    }

    const aslam::SensorId& sensor_id = mission.getAbsolute6DoFSensor();
    const aslam::Transformation& T_B_S =
        map.getSensorManager().getSensor_T_B_S(sensor_id);

    const aslam::Transformation& T_G_B_actual = map.getVertex_T_G_I(vertex_id);

    const std::vector<vi_map::Absolute6DoFMeasurement>& abs_6dof_measurements =
        vertex.getAbsolute6DoFMeasurements();

    const size_t num_constraints = abs_6dof_measurements.size();
    total_num_constraints += num_constraints;

    Eigen::Matrix3Xd lines_start_G = Eigen::Matrix3Xd(3, 0);
    Eigen::Matrix3Xd lines_end_G = Eigen::Matrix3Xd(3, 0);
    std::vector<visualization::Color> line_colors;
    lines_start_G.conservativeResize(Eigen::NoChange, num_constraints);
    lines_end_G.conservativeResize(Eigen::NoChange, num_constraints);
    line_colors.resize(num_constraints);

    CHECK_EQ(lines_start_G.cols(), lines_end_G.cols());
    CHECK_EQ(lines_start_G.cols(), static_cast<int>(line_colors.size()));

    for (size_t idx = 0u; idx < num_constraints; ++idx) {
      const vi_map::Absolute6DoFMeasurement& abs_6dof_measurement =
          abs_6dof_measurements[idx];
      const aslam::Transformation& T_G_S_meas =
          abs_6dof_measurement.get_T_G_S();
      const aslam::SensorId& sensor_id_meas =
          abs_6dof_measurement.getSensorId();
      CHECK_EQ(sensor_id_meas, sensor_id);
      const aslam::Transformation T_G_B_meas = T_G_S_meas * T_B_S.inverse();

      absolute_constraints_T_G_S_vec.push_back(T_G_S_meas);
      absolute_constraints_S_cov_vec.push_back(
          abs_6dof_measurement.get_T_G_S_covariance());

      CHECK_LT(static_cast<int>(idx), lines_start_G.cols());
      CHECK_LT(static_cast<int>(idx), lines_end_G.cols());
      CHECK_LT(idx, line_colors.size());
      lines_start_G.col(idx) = T_G_B_actual.getPosition();
      lines_end_G.col(idx) = T_G_B_meas.getPosition();
      line_colors[idx] = visualization::kCommonYellow;
    }

    all_lines_start_G.conservativeResize(
        Eigen::NoChange, all_lines_start_G.cols() + lines_start_G.cols());
    all_lines_end_G.conservativeResize(
        Eigen::NoChange, all_lines_end_G.cols() + lines_end_G.cols());
    all_line_colors.reserve(all_line_colors.size() + line_colors.size());

    all_lines_start_G.rightCols(lines_start_G.cols()) = lines_start_G;
    all_lines_end_G.rightCols(lines_end_G.cols()) = lines_end_G;
    all_line_colors.insert(
        all_line_colors.end(), line_colors.begin(), line_colors.end());

    CHECK_EQ(all_lines_start_G.cols(), all_lines_end_G.cols());
    CHECK_EQ(
        all_lines_start_G.cols(), static_cast<int>(all_line_colors.size()));
  }

  CHECK(!all_lines_start_G.hasNaN());
  CHECK(all_lines_start_G.allFinite());
  CHECK(!all_lines_end_G.hasNaN());
  CHECK(all_lines_end_G.allFinite());

  if (total_num_constraints == 0u) {
    return;
  }

  constexpr double kAlpha = 1.0;
  constexpr double kScale = 0.05;
  constexpr size_t id = 42u;
  const std::string& kNamespace = "absolute_6dof_constraints";
  const std::string kFrame = FLAGS_tf_map_frame;
  visualization::publishLines(
      all_lines_start_G, all_lines_end_G, all_line_colors, kAlpha, kScale, id,
      kFrame, kNamespace, kAbsolute6DoFTopic);

  if (FLAGS_vis_abs_6dof_constraint_covariances) {
    CHECK_EQ(
        absolute_constraints_T_G_S_vec.size(),
        absolute_constraints_S_cov_vec.size());
    const visualization::Color covariance_color = visualization::kCommonPink;
    const std::string kAbsolute6DoFCovTopic =
        kAbsolute6DoFTopic + "_covariances";
    visualization::publishPoseCovariances(
        absolute_constraints_T_G_S_vec, absolute_constraints_S_cov_vec,
        covariance_color, kFrame, kNamespace, kAbsolute6DoFCovTopic);
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
      poses, FLAGS_tf_map_frame, kNamespace, kVertexTopic);
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
      poses, FLAGS_tf_map_frame, kNamespace, kBaseframeTopic);
}

void ViwlsGraphRvizPlotter::publishLandmarks(
    const vi_map::VIMap& map, const vi_map::MissionIdList& missions) const {
  visualization::SphereVector spheres;
  appendLandmarksToSphereVector(map, missions, &spheres);

  visualization::publishSpheresAsPointCloud(
      spheres, FLAGS_tf_map_frame, kLandmarkTopic);
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
        p_G_landmarks, color, kAlpha, FLAGS_tf_map_frame, topic);
  } else {
    visualization::publish3DPointsAsPointCloud(
        p_G_landmarks, color, kAlpha, FLAGS_tf_map_frame, kLandmarkTopic);
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
  const size_t start_timestamp_seconds =
      aslam::time::nanoSecondsToSeconds(map.getVertex(*storing_vertices.begin())
                                            .getVisualFrame(0)
                                            .getTimestampNanoseconds());

  VLOG_IF(2, FLAGS_vis_color_landmarks_by_number_of_observations == true)
      << "The more observations a landmark has the more red it is. Make "
      << "sure you have set the "
      << "'-vis_color_landmarks_by_number_of_observations' flag appropriately.";

  vi_map::FeatureType show_feature_type = vi_map::FeatureType::kInvalid;
  if (!FLAGS_vis_show_only_landmark_type.empty()) {
    show_feature_type =
        vi_map::StringToFeatureType(FLAGS_vis_show_only_landmark_type);
  }

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
        map.getObserverMissionsForLandmark(landmark.id(), &observer_missions);
        if (observer_missions.size() <
            static_cast<size_t>(
                FLAGS_vis_landmarks_min_number_of_observer_missions)) {
          continue;
        }
      }
      if (show_feature_type != vi_map::FeatureType::kInvalid &&
          landmark.getFeatureType() != show_feature_type) {
        continue;
      }
      visualization::Sphere sphere;
      Eigen::Vector3d LM_p_fi =
          vertex.getLandmark_p_LM_fi(landmark.id()) - origin_;

      sphere.position =
          baseframe.transformPointInMissionFrameToGlobalFrame(LM_p_fi);
      sphere.radius = 0.03;
      sphere.alpha = 0.8;

      if (FLAGS_vis_offset_landmarks_by_first_observer_frame_m > 0) {
        const vi_map::KeypointIdentifierList& observations =
            landmark.getObservations();
        CHECK(!observations.empty());
        const vi_map::KeypointIdentifier& observation = observations[0];
        sphere.position.x() +=
            observation.frame_id.frame_index *
            FLAGS_vis_offset_landmarks_by_first_observer_frame_m;
      }

      if (FLAGS_vis_color_landmarks_by_number_of_observations) {
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
            static_cast<size_t>(floor(
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
      } else if (FLAGS_vis_color_by_landmark_type) {
        const size_t index =
            FLAGS_vis_color_salt * static_cast<int>(landmark.getFeatureType());
        sphere.color = getPaletteColor(index, palette);
      } else if (FLAGS_vis_color_by_mission) {
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
      T_M_I, FLAGS_tf_map_frame,
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
      pose_list, FLAGS_tf_map_frame, FLAGS_vis_default_namespace, kVertexTopic);
}

void ViwlsGraphRvizPlotter::publishReferenceMap() const {
  CHECK(!reference_edges_line_segments_.empty());

  constexpr size_t kMarkerId = 2345u;
  visualization::publishLines(
      reference_edges_line_segments_, kMarkerId, FLAGS_tf_map_frame,
      FLAGS_vis_default_namespace, kEdgeTopic);
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
      vertex_spheres, kMarkerId, FLAGS_tf_map_frame,
      FLAGS_vis_default_namespace, kVertexPartitioningTopic);
  visualization::publishSpheresAsPointCloud(
      landmark_spheres, FLAGS_tf_map_frame, kLandmarkTopic);
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
      vertex_poses, FLAGS_tf_map_frame, FLAGS_vis_default_namespace,
      kVertexTopic);
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
      line_segments, kMarkerId, FLAGS_tf_map_frame, FLAGS_vis_default_namespace,
      kLoopclosureTopic);
}

void ViwlsGraphRvizPlotter::visualizeMap(const vi_map::VIMap& map) const {
  constexpr bool kPlotBaseframes = true;
  constexpr bool kPlotVertices = true;
  constexpr bool kPlotEdges = true;
  constexpr bool kPlotLandmarks = true;
  constexpr bool kPlotAbsolute6DoFConstraints = true;
  visualizeMap(
      map, kPlotBaseframes, kPlotVertices, kPlotEdges, kPlotLandmarks,
      kPlotAbsolute6DoFConstraints);
}

void ViwlsGraphRvizPlotter::visualizeMap(
    const vi_map::VIMap& map, bool publish_baseframes, bool publish_vertices,
    bool publish_edges, bool publish_landmarks,
    bool publish_absolute_6dof_constraints) const {
  vi_map::MissionIdList all_missions;
  map.getAllMissionIds(&all_missions);
  visualizeMissions(
      map, all_missions, publish_baseframes, publish_vertices, publish_edges,
      publish_landmarks, publish_absolute_6dof_constraints);
}

void ViwlsGraphRvizPlotter::visualizeMissions(
    const vi_map::VIMap& map, const vi_map::MissionIdList& mission_ids,
    bool publish_baseframes, bool publish_vertices, bool publish_edges,
    bool publish_landmarks, bool publish_absolute_6dof_constraints) const {
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
          if (publish_absolute_6dof_constraints) {
            publishAbsolute6DoFConstraints(map, {mission_id});
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
      all_spheres, FLAGS_tf_map_frame, kLandmarkTopic);
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
      spheres, marker_id, FLAGS_tf_map_frame, FLAGS_vis_default_namespace,
      kSlidingWindowLocalizationResultTopic);
}

void ViwlsGraphRvizPlotter::visualizeSensorExtrinsics(
    const vi_map::VIMap& map) {
  const vi_map::SensorManager& sensor_manager = map.getSensorManager();
  aslam::SensorIdSet all_sensors_ids;
  sensor_manager.getAllSensorIds(&all_sensors_ids);

  size_t marker_id = 0u;
  for (const aslam::SensorId& sensor_id : all_sensors_ids) {
    CHECK(sensor_id.isValid());

    LOG(INFO) << "Visualizing extrinsics sensor " << sensor_id << "...";

    const aslam::Transformation& T_B_S =
        sensor_manager.getSensor_T_B_S(sensor_id);
    const aslam::Sensor& sensor =
        sensor_manager.getSensor<aslam::Sensor>(sensor_id);
    const std::string& description = sensor.getDescription();
    visualization::publishCoordinateFrame(
        T_B_S, description, marker_id++, kSensorExtrinsicsTopic);

    if (sensor.getSensorType() == aslam::SensorType::kNCamera) {
      LOG(INFO) << " -> is an NCamera";
      LOG(INFO) << "-> T_B_Cn: \n" << T_B_S;
      const aslam::NCamera& ncamera =
          static_cast<const aslam::NCamera&>(sensor);
      const size_t num_cameras = ncamera.getNumCameras();
      for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
        LOG(INFO) << " -> camera " << camera_idx;
        const aslam::Transformation& T_C_S = ncamera.get_T_C_B(camera_idx);
        const aslam::Transformation T_B_C = T_B_S * T_C_S.inverse();
        visualization::publishCoordinateFrame(
            T_B_C, ncamera.getCamera(camera_idx).getDescription(), marker_id++,
            kSensorExtrinsicsTopic);
        LOG(INFO) << "-> T_Cn_C" << camera_idx << ": \n" << T_B_C;
      }
    } else {
      LOG(INFO) << "-> T_B_S: \n" << T_B_S;
    }
  }
}
}  // namespace visualization
