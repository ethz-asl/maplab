#include "vi-map-helpers/vi-map-manipulation.h"

#include <Eigen/Dense>
#include <aslam/common/memory.h>
#include <maplab-common/accessors.h>
#include <maplab-common/conversions.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/progress-bar.h>
#include <vi-map/vi-map.h>

namespace vi_map_helpers {

VIMapManipulation::VIMapManipulation(vi_map::VIMap* map)
    : map_(*CHECK_NOTNULL(map)), geometry_(map_), queries_(map_) {}

void VIMapManipulation::rotate(const size_t dimension, const double degrees) {
  pose::Transformation T_G_old_G_new(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Quaterniond(
          Eigen::AngleAxisd(
              degrees * kDegToRad, Eigen::Vector3d::Unit(dimension)))
          .normalized());
  vi_map::MissionBaseFrameIdList frames;
  map_.getAllMissionBaseFrameIds(&frames);
  for (const vi_map::MissionBaseFrameId& id : frames) {
    vi_map::MissionBaseFrame& frame = map_.getMissionBaseFrame(id);
    frame.set_T_G_M(T_G_old_G_new * frame.get_T_G_M());
  }
}

void VIMapManipulation::alignToXYPlane(const vi_map::MissionId& mission_id) {
  CHECK(map_.hasMission(mission_id));
  // Let bv_M_z be the eigenvector of lowest eigenvalue of the vertex position
  // covariance. We then first rotate along bv_G_R_A x bv_G_z (z unit vector)
  // until bv_M_z x bv_G_z is minimized. We then rotate along the new bv_G_R_A
  // until bv_M_z x bv_G_z is zero.
  vi_map::MissionBaseFrame& base_frame =
      map_.getMissionBaseFrameForMission(mission_id);
  Eigen::Vector3d bv_M_z;
  Eigen::Vector3d bv_G_R_A_normalized;
  // bv_G_R_A x bv_G_z rotation
  {
    Eigen::Vector3d eigenvalues;
    Eigen::Matrix3d eigenvectors;
    geometry_.get_p_G_I_CovarianceEigenValuesAndVectorsAscending(
        mission_id, &eigenvalues, &eigenvectors);
    bv_M_z = eigenvectors.col(0).normalized();
    if (bv_M_z(2) < 0) {
      bv_M_z *= -1.;
    }
    bv_G_R_A_normalized =
        geometry_.get_bv_G_root_average(mission_id).normalized();
    // Angle to rotate along bv_G_R_A x bv_G_z = angle between bv_G_z and the
    // projection of bv_M_z into the plane bv_G_R_A x bv_G_z.
    // atan() intended, want range +- pi/2
    const double pitch_angle_rad =
        atan(bv_M_z.head<2>().dot(bv_G_R_A_normalized.head<2>()) / bv_M_z(2));
    const Eigen::Vector3d R_G_M_old_new =
        bv_G_R_A_normalized.cross(Eigen::Vector3d::UnitZ()).normalized() *
        pitch_angle_rad;
    const pose::Quaternion q_G_M_old_new(R_G_M_old_new);
    base_frame.set_T_G_M(
        pose::Transformation(q_G_M_old_new, Eigen::Vector3d::Zero()) *
        base_frame.get_T_G_M());

    bv_M_z = q_G_M_old_new.rotate(bv_M_z);
  }
  // bv_G_R_A rotation
  {
    const double roll_angle_rad = acos(bv_M_z(2));
    bv_G_R_A_normalized(2) = 0;
    bv_G_R_A_normalized.normalize();
    const pose::Quaternion q_G_M_old_new(
        Eigen::Vector3d(bv_G_R_A_normalized * -roll_angle_rad));
    base_frame.set_T_G_M(
        pose::Transformation(q_G_M_old_new, Eigen::Vector3d::Zero()) *
        base_frame.get_T_G_M());
  }
}

void VIMapManipulation::getViwlsEdgesWithoutImuMeasurements(
    const vi_map::MissionId& mission_id,
    pose_graph::EdgeIdList* corrupt_edge_ids) const {
  CHECK(map_.hasMission(mission_id));
  CHECK_NOTNULL(corrupt_edge_ids)->clear();

  pose_graph::EdgeIdList edge_ids;
  map_.getAllEdgeIdsInMissionAlongGraph(
      mission_id, pose_graph::Edge::EdgeType::kViwls, &edge_ids);

  CHECK(!edge_ids.empty()) << "No Viwls edges found in mission "
                           << mission_id.hexString();

  for (size_t i = 0; i < edge_ids.size(); ++i) {
    const vi_map::ViwlsEdge& edge =
        map_.getEdgeAs<vi_map::ViwlsEdge>(edge_ids[i]);
    if (edge.getImuData().size() == 0) {
      corrupt_edge_ids->push_back(edge_ids[i]);

      VLOG(2) << "Found edge with no IMU data: " << edge_ids[i].hexString();
      VLOG(2) << "Edge index within mission: " << i;
    }
  }
}

void VIMapManipulation::fixViwlsEdgesWithoutImuMeasurements(
    const vi_map::MissionId& mission_id) {
  CHECK(map_.hasMission(mission_id));

  pose_graph::EdgeIdList corrupt_edge_ids;
  getViwlsEdgesWithoutImuMeasurements(mission_id, &corrupt_edge_ids);

  if (corrupt_edge_ids.empty()) {
    VLOG(2) << "No corrupt edges found in mission: " << mission_id.hexString();
    return;
  }
  pose_graph::EdgeIdList all_edge_ids;
  map_.getAllEdgeIdsInMissionAlongGraph(mission_id, &all_edge_ids);
  CHECK_LT(corrupt_edge_ids.size(), all_edge_ids.size()) << "All Viwls edges "
                                                         << "are corrupt.";

  for (const pose_graph::EdgeId& edge_id : corrupt_edge_ids) {
    VLOG(2) << "Fixing edge " << edge_id;
    const vi_map::ViwlsEdge& current_edge =
        map_.getEdgeAs<vi_map::ViwlsEdge>(edge_id);

    pose_graph::EdgeId new_edge_id;
    common::generateId(&new_edge_id);

    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> new_imu_timestamps;
    Eigen::Matrix<double, 6, Eigen::Dynamic> new_imu_data;

    const pose_graph::VertexId& vertex_from = current_edge.from();
    const pose_graph::VertexId& vertex_to = current_edge.to();

    pose_graph::EdgeIdSet incoming_edges_from;
    map_.getVertex(vertex_from).getIncomingEdges(&incoming_edges_from);

    // Get the IMU measurements from the previous edge.
    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps_from;
    Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data_from;
    for (const pose_graph::EdgeId& incoming_edge_id : incoming_edges_from) {
      if (map_.getEdgeType(incoming_edge_id) ==
          pose_graph::Edge::EdgeType::kViwls) {
        const vi_map::ViwlsEdge& incoming_edge =
            map_.getEdgeAs<vi_map::ViwlsEdge>(incoming_edge_id);

        imu_timestamps_from = incoming_edge.getImuTimestamps();
        imu_data_from = incoming_edge.getImuData();
        break;
      }
    }

    // Add the last IMU measurement from the previous edge.
    if (imu_timestamps_from.cols() > 0u) {
      new_imu_timestamps.conservativeResize(
          Eigen::NoChange, new_imu_timestamps.cols() + 1);
      new_imu_timestamps.col(new_imu_timestamps.cols() - 1) =
          imu_timestamps_from.rightCols(1);
      new_imu_data.conservativeResize(Eigen::NoChange, new_imu_data.cols() + 1);
      new_imu_data.col(new_imu_data.cols() - 1) = imu_data_from.rightCols(1);
    } else {
      // Since corrupt edges are in graph traversal order, if the previous
      // edge is corrupt it must be the first edge (otherwise it already
      // crashed), so we need to remove the first vertex in the mission
      // (vertex_from) and make vertex_to the new root vertex for the mission.
      map_.removeVertex(vertex_from);
      vi_map::VIMission& mission = map_.getMission(mission_id);
      mission.setRootVertexId(vertex_to);
      map_.removeEdge(edge_id);
      continue;
    }

    pose_graph::EdgeIdSet outgoing_edges_to;
    map_.getVertex(vertex_to).getOutgoingEdges(&outgoing_edges_to);

    // Get the IMU measurements from the following edge.
    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps_to;
    Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data_to;
    for (const pose_graph::EdgeId& outgoing_edge_id : outgoing_edges_to) {
      if (map_.getEdgeType(outgoing_edge_id) ==
          pose_graph::Edge::EdgeType::kViwls) {
        const vi_map::ViwlsEdge& outgoing_edge =
            map_.getEdgeAs<vi_map::ViwlsEdge>(outgoing_edge_id);

        imu_timestamps_to = outgoing_edge.getImuTimestamps();
        imu_data_to = outgoing_edge.getImuData();
        break;
      }
    }
    if (imu_timestamps_to.cols() > 0u) {
      constexpr unsigned int kNumInterpolationTerms = 3u;
      new_imu_timestamps.conservativeResize(
          Eigen::NoChange,
          new_imu_timestamps.cols() + kNumInterpolationTerms + 1);
      new_imu_data.conservativeResize(
          Eigen::NoChange, new_imu_data.cols() + kNumInterpolationTerms + 1);
      // Add interpolation terms for stability.
      Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps_interp_step =
          (imu_timestamps_to.leftCols(1) - imu_timestamps_from.rightCols(1)) /
          (kNumInterpolationTerms + 1);
      Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data_interp_step =
          (imu_data_to.leftCols(1) + imu_data_from.rightCols(1)) /
          (kNumInterpolationTerms + 1);
      for (unsigned int i = 0u; i < kNumInterpolationTerms; ++i) {
        const unsigned int index =
            new_imu_timestamps.cols() - kNumInterpolationTerms - 1 + i;
        new_imu_timestamps.col(index) =
            new_imu_timestamps.col(index - 1) + imu_timestamps_interp_step;
        new_imu_data.col(index) =
            new_imu_data.col(index - 1) + imu_data_interp_step;
      }
      // Add the first IMU measurement from the following edge.
      new_imu_timestamps.col(new_imu_timestamps.cols() - 1) =
          imu_timestamps_to.leftCols(1);
      new_imu_data.col(new_imu_data.cols() - 1) = imu_data_to.leftCols(1);
    } else {
      const bool kIsVertexToLastVertexInMission = outgoing_edges_to.size() == 0;
      CHECK(kIsVertexToLastVertexInMission) << "Consecutive corrupt edges.";
      // The corrupt edge is the last edge, so it can be safely removed along
      // with the last vertex.
      map_.removeEdge(edge_id);
      map_.removeVertex(vertex_to);
      continue;
    }

    VLOG(3) << "Adding the following IMU timestamps: " << new_imu_timestamps;
    VLOG(3) << "Adding the following IMU data: " << new_imu_data;
    vi_map::Edge* new_edge_ptr(
        new vi_map::ViwlsEdge(
            new_edge_id, vertex_from, vertex_to, new_imu_timestamps,
            new_imu_data));

    // Align the vertex_from and vertex_to to avoid overextending the edge.
    map_.getVertexPtr(vertex_to)->set_T_M_I(
        map_.getVertex(vertex_from).get_T_M_I());

    map_.removeEdge(edge_id);
    map_.addEdge(vi_map::Edge::UniquePtr(new_edge_ptr));
  }
}

void VIMapManipulation::removePosegraphAfter(
    const pose_graph::VertexId& vertex_id,
    pose_graph::VertexIdList* removed_vertex_ids) {
  CHECK_NOTNULL(removed_vertex_ids)->clear();
  CHECK(vertex_id.isValid());

  constexpr bool kIncludeStartingVertex = false;
  queries_.getFollowingVertexIdsAlongGraph(
      vertex_id, kIncludeStartingVertex, removed_vertex_ids);

  removeVerticesAndIncomingEdges(*removed_vertex_ids);
}

void VIMapManipulation::removeVerticesAndIncomingEdges(
    const pose_graph::VertexIdList& vertex_ids) {
  // Get all edges to delete.
  pose_graph::EdgeIdSet edges_to_remove;
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    const vi_map::Vertex& vertex = map_.getVertex(vertex_id);

    pose_graph::EdgeIdSet incoming_edges;
    vertex.getIncomingEdges(&incoming_edges);
    edges_to_remove.insert(incoming_edges.begin(), incoming_edges.end());
  }

  // Remove the elements from the map.
  for (const pose_graph::EdgeId& edge_id : edges_to_remove) {
    map_.removeEdge(edge_id);
  }

  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    map_.removeVertex(vertex_id);
  }
}

size_t VIMapManipulation::initializeLandmarksFromUnusedFeatureTracksOfMission(
    const vi_map::MissionId& mission_id) {
  CHECK(mission_id.isValid());
  pose_graph::VertexIdList all_vertices_in_missions;
  map_.getAllVertexIdsInMissionAlongGraph(
      mission_id, &all_vertices_in_missions);

  TrackIndexToLandmarkIdMap track_id_to_landmark_id;
  track_id_to_landmark_id.reserve(1000u * all_vertices_in_missions.size());
  const size_t num_landmarks_initial = map_.numLandmarks();
  initializeLandmarksFromUnusedFeatureTracksOfOrderedVertices(
      all_vertices_in_missions, &track_id_to_landmark_id);

  const size_t num_new_landmarks = map_.numLandmarks() - num_landmarks_initial;
  return num_new_landmarks;
}

void VIMapManipulation::
    initializeLandmarksFromUnusedFeatureTracksOfOrderedVertices(
        const pose_graph::VertexIdList& ordered_vertex_ids,
        TrackIndexToLandmarkIdMap* trackid_landmarkid_map) {
  for (const pose_graph::VertexId& vertex_id : ordered_vertex_ids) {
    initializeLandmarksFromUnusedFeatureTracksOfVertex(
        vertex_id, trackid_landmarkid_map);
  }
}

void VIMapManipulation::initializeLandmarksFromUnusedFeatureTracksOfVertex(
    const pose_graph::VertexId& vertex_id,
    TrackIndexToLandmarkIdMap* trackid_landmarkid_map) {
  CHECK_NOTNULL(trackid_landmarkid_map);
  const vi_map::Vertex& vertex = map_.getVertex(vertex_id);

  vertex.forEachFrame(
      [this, &vertex, &trackid_landmarkid_map](
          const size_t frame_index, const aslam::VisualFrame& frame) {
        const size_t num_keypoints = frame.getNumKeypointMeasurements();
        if (!frame.hasTrackIds()) {
          VLOG(3) << "Frame has no tracking information. Skipping frame...";
          return;
        }
        const Eigen::VectorXi& track_ids = frame.getTrackIds();

        CHECK_EQ(static_cast<int>(num_keypoints), track_ids.rows());

        vi_map::LandmarkIdList landmark_ids;
        vertex.getFrameObservedLandmarkIds(frame_index, &landmark_ids);

        // Go over all tracks of this frame and add a new landmark if it wasn't
        // observed before, otherwise add an observation backlink.
        for (size_t keypoint_i = 0u; keypoint_i < num_keypoints; ++keypoint_i) {
          const int track_id = track_ids(keypoint_i);

          // Skip non-tracked landmark observation.
          if (track_id < 0 || landmark_ids[keypoint_i].isValid()) {
            continue;
          }

          // Check whether this track has already a global landmark id
          // associated.
          const vi_map::LandmarkId* landmark_id_ptr =
              common::getValuePtr(*trackid_landmarkid_map, track_id);

          if (landmark_id_ptr != nullptr &&
              map_.hasLandmark(*landmark_id_ptr)) {
            map_.associateKeypointWithExistingLandmark(
                vertex.id(), frame_index, keypoint_i, *landmark_id_ptr);
          } else {
            // Assign a new global landmark id to this track if it hasn't
            // been seen before and add a new landmark to the map.
            vi_map::LandmarkId landmark_id =
                common::createRandomId<vi_map::LandmarkId>();
            // operator[] intended as this is either overwriting an old outdated
            // entry or creating a new one.
            (*trackid_landmarkid_map)[track_id] = landmark_id;

            vi_map::KeypointIdentifier keypoint_id;
            keypoint_id.frame_id.frame_index = frame_index;
            keypoint_id.frame_id.vertex_id = vertex.id();
            keypoint_id.keypoint_index = keypoint_i;
            map_.addNewLandmark(landmark_id, keypoint_id);
          }
        }
      });
}

size_t VIMapManipulation::mergeLandmarksBasedOnTrackIds(
    const vi_map::MissionId& mission_id) {
  CHECK(map_.hasMission(mission_id));
  typedef std::unordered_map<int, vi_map::LandmarkId>
      TrackIdToLandmarkIdMap;
  TrackIdToLandmarkIdMap track_id_to_store_landmark_id;
  size_t num_merges = 0u;

  pose_graph::VertexIdList vertex_ids;
  map_.getAllVertexIdsInMission(mission_id, &vertex_ids);

  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    const vi_map::Vertex& vertex = map_.getVertex(vertex_id);

    for (size_t frame_idx = 0u; frame_idx < vertex.numFrames(); ++frame_idx) {
      vi_map::LandmarkIdList landmark_ids;
      vertex.getFrameObservedLandmarkIds(frame_idx, &landmark_ids);

      CHECK(vertex.getVisualFrame(frame_idx).hasTrackIds());
      const Eigen::VectorXi& track_ids =
          vertex.getVisualFrame(frame_idx).getTrackIds();
      CHECK_EQ(
          static_cast<size_t>(track_ids.rows()), landmark_ids.size());

      for (size_t keypoint_idx = 0u; keypoint_idx < landmark_ids.size();
           ++keypoint_idx) {
        if (landmark_ids[keypoint_idx].isValid() &&
            track_ids(keypoint_idx) < 0) {
          const vi_map::LandmarkId landmark_id = landmark_ids[keypoint_idx];
          CHECK(map_.hasLandmark(landmark_id));

          if (track_id_to_store_landmark_id
                  .emplace(track_ids(keypoint_idx), landmark_id).second) {
            // Emplace succeeded so this track ID was not used before.
          } else {
            // Emplace failed so this track ID is already used, we need to
            // merge landmarks.
            TrackIdToLandmarkIdMap::const_iterator it =
                track_id_to_store_landmark_id.find(track_ids(keypoint_idx));
            CHECK(it != track_id_to_store_landmark_id.end());
            CHECK(it->second.isValid());
            CHECK(map_.hasLandmark(it->second)) << "Landmark "
                                                << it->second.hexString()
                                                << " not found in the map.";

            if (it->second != landmark_id) {
              map_.mergeLandmarks(landmark_id, it->second);
              ++num_merges;

              // As the dataset could be loop-closed before (so more than
              // a single track id pointing to the same store landmark, we need
              // to update the track id to store landmark id map.
              for (TrackIdToLandmarkIdMap::value_type& track_id_to_landmark :
                   track_id_to_store_landmark_id) {
                if (track_id_to_landmark.second == landmark_id) {
                  track_id_to_landmark.second = it->second;
                }
              }
            }
          }
        }
      }
    }
  }
  VLOG(2) << "Number of merges " << num_merges;
  return num_merges;
}

void VIMapManipulation::releaseOldVisualFrameImages(
    const pose_graph::VertexId& current_vertex_id,
    const int image_removal_age_threshold) {
  CHECK_GT(image_removal_age_threshold, 0);
  CHECK(map_.hasVertex(current_vertex_id));

  const vi_map::MissionId& mission_id =
      map_.getVertex(current_vertex_id).getMissionId();

  pose_graph::VertexId vertex_id = current_vertex_id;
  for (int i = 0; i < image_removal_age_threshold; ++i) {
    map_.getPreviousVertex(
        vertex_id, map_.getGraphTraversalEdgeType(mission_id), &vertex_id);
  }

  while (map_.getPreviousVertex(
      vertex_id, map_.getGraphTraversalEdgeType(mission_id), &vertex_id)) {
    const vi_map::Vertex& vertex = map_.getVertex(vertex_id);
    for (size_t i = 0; i < vertex.numFrames(); ++i) {
      const aslam::VisualFrame& const_vframe = vertex.getVisualFrame(i);
      if (const_vframe.isValid() && const_vframe.hasRawImage()) {
        aslam::VisualFrame::Ptr visual_frame =
            map_.getVertex(vertex_id).getVisualFrameShared(i);
        visual_frame->releaseRawImage();
        CHECK(!visual_frame->hasRawImage());
      }
    }
  }
}

size_t VIMapManipulation::removeBadLandmarks() {
  vi_map::LandmarkIdList bad_landmark_ids;
  queries_.getAllNotWellConstrainedLandmarkIds(&bad_landmark_ids);

  for (const vi_map::LandmarkId& bad_landmark_id : bad_landmark_ids) {
    map_.removeLandmark(bad_landmark_id);
  }
  return bad_landmark_ids.size();
}


}  // namespace vi_map_helpers
