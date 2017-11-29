#include "visualization/sequential-plotter.h"

#include <maplab-common/frequency-enforcer.h>
#include <vi-map/landmark-quality-metrics.h>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>

#include "visualization/color-palette.h"
#include "visualization/common-rviz-visualization.h"
#include "visualization/viwls-graph-plotter.h"

DECLARE_int32(vis_color_salt);
DECLARE_double(vis_scale);

DEFINE_int32(vis_seq_vertex_frequency, 10, "Publishing frequency in Hz.");
DEFINE_bool(
    vis_seq_landmark_fadeout, true, "Smoothly fade out landmark coloring.");
DEFINE_bool(
    vis_seq_synchronize_missions_on_distance, true,
    "Synchronize missions based on distance traveled.");
DEFINE_bool(
    vis_seq_synchronize_missions_on_distance_percent, true,
    "Synchronize missions based on percent of trajectory.");

namespace visualization {

SequentialPlotter::SequentialPlotter(ViwlsGraphRvizPlotter* plotter)
    : plotter_(CHECK_NOTNULL(plotter)) {}

void SequentialPlotter::publishMissionsSequentially(
    const vi_map::VIMap& map, const vi_map::MissionIdSet& mission_set) {
  if (mission_set.empty()) {
    LOG(ERROR) << "Passed empty mission set to publish map sequentially.";
    return;
  }

  const bool is_only_one_mission = (mission_set.size() == 1u);

  Palette palette = GetPalette(Palette::PaletteTypes::kFalseColorJet);

  vi_map::MissionIdList missions(mission_set.begin(), mission_set.end());
  std::vector<pose_graph::VertexId> vertices;
  for (const vi_map::MissionId& mission_id : missions) {
    const vi_map::VIMission& mission = map.getMission(mission_id);
    vertices.push_back(mission.getRootVertexId());
  }

  resetSmoothers();

  bool more_vertices_exist = true;

  std::unordered_map<vi_map::LandmarkId, int> landmarks_to_publish;
  SphereVector landmark_vector;

  std::unordered_map<vi_map::LandmarkId, int> landmark_to_age;

  const unsigned int kMasterMissionIndex = 0;
  const double kInitialLandmarkColoringAlpha = 0.4;
  const unsigned int kNumKeyframesToVisualizeLandmarksFrom = 9u;

  std::vector<double> mission_distance_traveled;
  mission_distance_traveled.resize(missions.size());
  for (unsigned int i = 0; i < mission_distance_traveled.size(); ++i) {
    mission_distance_traveled[i] = 0;
  }

  std::vector<double> mission_distance_traveled_total;
  mission_distance_traveled_total.resize(missions.size(), 0);
  if (FLAGS_vis_seq_synchronize_missions_on_distance_percent) {
    for (unsigned int i = 0; i < mission_distance_traveled_total.size(); ++i) {
      map.getDistanceTravelledPerMission(
          missions[i], &mission_distance_traveled_total[i]);
    }
  }

  common::FrequencyEnforcer freq_enforcer;

  LineSegmentVector edges_vector;

  constexpr size_t kEdgeCountRvizLimit = 8192 - 1;
  // It is highly unlikely we will deal with more than 1e4 missions.
  size_t edge_marker_id = 10000u;

  while (more_vertices_exist) {
    freq_enforcer.EnforceFrequency(FLAGS_vis_seq_vertex_frequency);
    // Will be set to true if at least one mission has an unpublished vertex.
    more_vertices_exist = false;
    for (size_t mission_index = 0; mission_index < missions.size();
         ++mission_index) {
      double master_mission_distance = 0;

      const vi_map::MissionId& mission_id = missions[mission_index];

      const int mission_color_index =
          (mission_id.hashToSizeT() * FLAGS_vis_color_salt) % 256;
      const int landmark_color_index = ((mission_color_index + 100) % 256);
      const int ray_color_index = ((mission_color_index + 200) % 256);

      if (FLAGS_vis_seq_synchronize_missions_on_distance) {
        if (FLAGS_vis_seq_synchronize_missions_on_distance_percent) {
          // Calculate the percent of traj that the master did yet and
          // set the limit for the current mission to the same fraction from
          // its own total distance.
          CHECK_NE(mission_distance_traveled_total[kMasterMissionIndex], 0.0);
          double master_traveled_percent =
              mission_distance_traveled[kMasterMissionIndex] /
              mission_distance_traveled_total[kMasterMissionIndex];
          master_mission_distance =
              master_traveled_percent *
                  mission_distance_traveled_total[mission_index] +
              0.01;
        } else {
          // Set the limit of the current mission to the distance of the master.
          master_mission_distance =
              mission_distance_traveled[kMasterMissionIndex];
        }
      } else {
        // Set it to its own value s.t. each mission does only a single step
        // in every iteration.
        master_mission_distance = mission_distance_traveled[mission_index];
      }

      while (mission_distance_traveled[mission_index] <=
             master_mission_distance) {
        pose_graph::VertexIdList vertex_ids;
        pose_graph::EdgeIdList edge_ids;

        const vi_map::Vertex& viwls_vertex =
            map.getVertex(vertices[mission_index]);
        vertex_ids.push_back(vertices[mission_index]);
        pose_graph::EdgeIdSet incoming_edges;
        viwls_vertex.getIncomingEdges(&incoming_edges);
        if (!incoming_edges.empty()) {
          edge_ids.push_back(*incoming_edges.begin());
        }

        Eigen::Vector3d vertex_position =
            map.getVertex_G_p_I(vertices[mission_index]);
        Aligned<std::vector, Eigen::Vector3d> landmark_positions;
        std::vector<Color> segment_colors;

        const unsigned int num_frames = viwls_vertex.numFrames();
        for (unsigned int frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
          if (!viwls_vertex.isVisualFrameSet(frame_idx)) {
            continue;
          }
          for (size_t i = 0;
               i < viwls_vertex.observedLandmarkIdsSize(frame_idx); ++i) {
            vi_map::LandmarkId landmark_id =
                viwls_vertex.getObservedLandmarkId(frame_idx, i);

            if (landmark_id.isValid()) {
              if (!vi_map::isLandmarkWellConstrained(
                      map, map.getLandmark(landmark_id))) {
                continue;
              }
              Eigen::Vector3d p_G_fi = map.getLandmark_G_p_fi(landmark_id);

              int landmark_index = 0;
              if (landmarks_to_publish.count(landmark_id) == 0u) {
                landmarks_to_publish.emplace(
                    landmark_id, landmark_vector.size());
                landmark_index = landmark_vector.size();
                landmark_vector.push_back(Sphere());
              } else {
                landmark_index = landmarks_to_publish[landmark_id];
              }

              Sphere& sphere = landmark_vector[landmark_index];
              sphere.position = p_G_fi;
              sphere.radius = 0.05;
              sphere.alpha = kInitialLandmarkColoringAlpha;

              const Color& color = (is_only_one_mission)
                                       ? palette.colors[landmark_color_index]
                                       : palette.colors[mission_color_index];
              sphere.color.red = color.red;
              sphere.color.green = color.green;
              sphere.color.blue = color.blue;
              landmark_to_age[landmark_id] =
                  kNumKeyframesToVisualizeLandmarksFrom;
              landmark_positions.push_back(p_G_fi);

              const Color& color_ray =
                  (is_only_one_mission) ? palette.colors[ray_color_index]
                                        : palette.colors[mission_color_index];
              sphere.color.red = color.red;
              sphere.color.green = color.green;
              sphere.color.blue = color.blue;
              segment_colors.push_back(color_ray);
            }
          }
        }

        if (FLAGS_vis_seq_landmark_fadeout) {
          typedef std::unordered_map<vi_map::LandmarkId, int> LandmarkMap;
          for (LandmarkMap::iterator it = landmark_to_age.begin();
               it != landmark_to_age.end();) {
            const vi_map::LandmarkId& landmark_id = it->first;
            int landmark_index = 0;
            if (landmarks_to_publish.count(landmark_id) == 0u) {
              landmarks_to_publish.emplace(landmark_id, landmark_vector.size());
              landmark_index = landmark_vector.size();
              landmark_vector.push_back(Sphere());
            } else {
              landmark_index = landmarks_to_publish[landmark_id];
            }

            Sphere& sphere = landmark_vector[landmark_index];
            int lifetime = kNumKeyframesToVisualizeLandmarksFrom - it->second;
            sphere.alpha = kInitialLandmarkColoringAlpha * lifetime /
                           (kNumKeyframesToVisualizeLandmarksFrom + 1);
            --it->second;
            // Drop items from the list if they are dead.
            if (it->second == 0) {
              it = landmark_to_age.erase(it);
            } else {
              ++it;
            }
          }
        }

        const double kAlpha = 0.3;
        const double scale = FLAGS_vis_scale * 0.04;
        const size_t kMarkerId = mission_index;
        publishLines(
            vertex_position, landmark_positions, segment_colors, kAlpha, scale,
            kMarkerId, kDefaultMapFrame, kDefaultNamespace,
            ViwlsGraphRvizPlotter::kLoopclosureTopic);

        publishVertexPoseAsTFSmoothed(map, viwls_vertex.id());

        pose_graph::VertexId prev_vertex_id = vertices[mission_index];
        if (map.getNextVertex(
                prev_vertex_id,
                map.getGraphTraversalEdgeType(viwls_vertex.getMissionId()),
                &vertices[mission_index])) {
          double distance =
              (map.getVertex(vertices[mission_index]).get_p_M_I() -
               map.getVertex(prev_vertex_id).get_p_M_I())
                  .norm();

          LineSegment line_segment;
          line_segment.from = map.getVertex_G_p_I(vertices[mission_index]);
          line_segment.to = map.getVertex_G_p_I(prev_vertex_id);
          line_segment.scale = FLAGS_vis_scale * 0.08;
          line_segment.alpha = 0.5;
          Color color_ray;
          line_segment.color.red = palette.colors[mission_color_index].red;
          line_segment.color.green = palette.colors[mission_color_index].green;
          line_segment.color.blue = palette.colors[mission_color_index].blue;
          edges_vector.push_back(line_segment);

          mission_distance_traveled[mission_index] += distance;
          more_vertices_exist = true;
        } else {
          break;
        }
      }
    }

    publishSpheresAsPointCloud(
        landmark_vector, kDefaultMapFrame,
        ViwlsGraphRvizPlotter::kLandmarkTopic);

    bool is_marker_array_too_large;
    do {
      is_marker_array_too_large = false;

      LineSegmentVector temp_edges_vector;
      if (edges_vector.size() > kEdgeCountRvizLimit) {
        // Copy the elements over the vector size limit to a temporary vector.
        temp_edges_vector.insert(
            temp_edges_vector.end(), edges_vector.begin() + kEdgeCountRvizLimit,
            edges_vector.end());
        // Truncate the vector that will get published now.
        edges_vector.resize(kEdgeCountRvizLimit);
        is_marker_array_too_large = true;
      }
      publishLines(
          edges_vector, edge_marker_id, kDefaultMapFrame, kDefaultNamespace,
          ViwlsGraphRvizPlotter::kEdgeTopic);
      if (!temp_edges_vector.empty()) {
        edges_vector.swap(temp_edges_vector);
        ++edge_marker_id;
      }
    } while (is_marker_array_too_large);
  }
}

void SequentialPlotter::publishVertexPoseAsTFSmoothed(
    const vi_map::VIMap& map, const pose_graph::VertexId& vertex_id) const {
  vi_map::Vertex vertex = map.getVertex(vertex_id);
  Eigen::Vector3d G_p = map.getVertex_G_p_I(vertex_id);

  std::string child_frame =
      "camera_" + map.getMissionIdForVertex(vertex_id).hexString().substr(0, 4);

  ConstantVelocitySmoother& smoother = smoothers_[child_frame];
  smoother.addSample(G_p);
  Eigen::Vector3d G_p_smoothed = smoother.getCurrentPosition();

  aslam::Transformation G_camera_pose(vertex.get_q_M_I(), G_p_smoothed);

  publishTF(G_camera_pose, kDefaultMapFrame, child_frame);
}

}  // namespace visualization
