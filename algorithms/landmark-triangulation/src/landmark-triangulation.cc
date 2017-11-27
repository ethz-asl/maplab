#include "landmark-triangulation/landmark-triangulation.h"

#include <functional>
#include <string>
#include <unordered_map>

#include <aslam/common/statistics/statistics.h>
#include <aslam/triangulation/triangulation.h>
#include <maplab-common/multi-threaded-progress-bar.h>
#include <maplab-common/parallel-process.h>
#include <vi-map/landmark-quality-metrics.h>
#include <vi-map/vi-map.h>

#include "landmark-triangulation/pose-interpolator.h"

namespace landmark_triangulation {
typedef AlignedUnorderedMap<aslam::FrameId, aslam::Transformation>
    FrameToPoseMap;

namespace {
void interpolateVisualFramePosesAllMissions(
    const vi_map::VIMap& map, FrameToPoseMap* interpolated_frame_poses) {
  CHECK_NOTNULL(interpolated_frame_poses)->clear();
  // Loop over all missions, vertices and frames and add the interpolated poses
  // to the map.
  size_t total_num_frames = 0u;
  vi_map::MissionIdList mission_ids;
  map.getAllMissionIds(&mission_ids);
  for (const vi_map::MissionId mission_id : mission_ids) {
    // Check if there is IMU data.
    std::unordered_map<pose_graph::VertexId, int64_t> vertex_to_time_map;
    PoseInterpolator imu_timestamp_collector;
    imu_timestamp_collector.getVertexToTimeStampMap(
        map, mission_id, &vertex_to_time_map);
    if (vertex_to_time_map.empty()) {
      VLOG(2) << "Couldn't find any IMU data to interpolate exact landmark "
                 "observer positions in "
              << "mission " << mission_id;
      continue;
    }

    pose_graph::VertexIdList vertex_ids;
    map.getAllVertexIdsInMissionAlongGraph(mission_id, &vertex_ids);

    // Compute upper bound for number of VisualFrames.
    const aslam::NCamera& ncamera =
        map.getSensorManager().getNCameraForMission(mission_id);
    const unsigned int upper_bound_num_frames =
        vertex_ids.size() * ncamera.numCameras();

    // Extract the timestamps for all VisualFrames.
    std::vector<aslam::FrameId> frame_ids(upper_bound_num_frames);
    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> pose_timestamps(
        upper_bound_num_frames);
    unsigned int frame_counter = 0u;
    for (const pose_graph::VertexId& vertex_id : vertex_ids) {
      const vi_map::Vertex& vertex = map.getVertex(vertex_id);
      for (unsigned int frame_idx = 0u; frame_idx < vertex.numFrames();
           ++frame_idx) {
        if (vertex.isFrameIndexValid(frame_idx)) {
          CHECK_LT(frame_counter, upper_bound_num_frames);
          const aslam::VisualFrame& visual_frame =
              vertex.getVisualFrame(frame_idx);
          const int64_t timestamp = visual_frame.getTimestampNanoseconds();
          // Only interpolate if the VisualFrame timestamp and the vertex
          // timestamp do not match.
          std::unordered_map<pose_graph::VertexId, int64_t>::const_iterator it =
              vertex_to_time_map.find(vertex_id);
          if (it != vertex_to_time_map.end() && it->second != timestamp) {
            pose_timestamps(0, frame_counter) = timestamp;
            frame_ids[frame_counter] = visual_frame.getId();
            ++frame_counter;
          }
        }
      }
    }

    // Shrink time stamp and frame id arrays if necessary.
    if (upper_bound_num_frames > frame_counter) {
      frame_ids.resize(frame_counter);
      pose_timestamps.conservativeResize(Eigen::NoChange, frame_counter);
    }

    // Reserve enough space in the frame_to_pose_map to add the poses of the
    // current mission.
    total_num_frames += frame_counter;
    interpolated_frame_poses->reserve(total_num_frames);

    // Interpolate poses for all the VisualFrames.
    if (frame_counter > 0u) {
      VLOG(1) << "Interpolating the exact visual frame poses for "
              << frame_counter << " frames of mission " << mission_id;
      PoseInterpolator pose_interpolator;
      aslam::TransformationVector poses_M_I;
      pose_interpolator.getPosesAtTime(
          map, mission_id, pose_timestamps, &poses_M_I);
      CHECK_EQ(poses_M_I.size(), frame_counter);
      for (size_t frame_num = 0u; frame_num < frame_counter; ++frame_num) {
        interpolated_frame_poses->emplace(
            frame_ids[frame_num], poses_M_I.at(frame_num));
      }
      CHECK_EQ(interpolated_frame_poses->size(), total_num_frames);
    } else {
      VLOG(10) << "No frames found for mission " << mission_id
               << " that need to be interpolated.";
    }
  }
  if (total_num_frames == 0u) {
    VLOG(2) << "No frame pose in any of the missions needs interpolation!";
  }
}

void retriangulateLandmarksOfVertex(
    const FrameToPoseMap& interpolated_frame_poses,
    pose_graph::VertexId storing_vertex_id, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  vi_map::Vertex& storing_vertex = map->getVertex(storing_vertex_id);
  vi_map::LandmarkStore& landmark_store = storing_vertex.getLandmarks();

  const aslam::Transformation& T_M_I_storing = storing_vertex.get_T_M_I();
  const aslam::Transformation& T_G_M_storing =
      const_cast<const vi_map::VIMap*>(map)
          ->getMissionBaseFrameForVertex(storing_vertex_id)
          .get_T_G_M();
  const aslam::Transformation T_G_I_storing = T_G_M_storing * T_M_I_storing;

  for (vi_map::Landmark& landmark : landmark_store) {
    // The following have one entry per measurement:
    Eigen::Matrix3Xd G_bearing_vectors;
    Eigen::Matrix3Xd p_G_C_vector;

    landmark.setQuality(vi_map::Landmark::Quality::kBad);

    const vi_map::KeypointIdentifierList& observations =
        landmark.getObservations();
    if (observations.size() < 2u) {
      statistics::StatsCollector stats(
          "Landmark triangulation failed too few observations.");
      stats.IncrementOne();
      continue;
    }

    G_bearing_vectors.resize(Eigen::NoChange, observations.size());
    p_G_C_vector.resize(Eigen::NoChange, observations.size());

    int num_measurements = 0;
    for (const vi_map::KeypointIdentifier& observation : observations) {
      const pose_graph::VertexId& observer_id = observation.frame_id.vertex_id;
      CHECK(map->hasVertex(observer_id))
          << "Observer " << observer_id << " of store landmark "
          << landmark.id() << " not in currently loaded map!";

      const vi_map::Vertex& observer =
          const_cast<const vi_map::VIMap*>(map)->getVertex(observer_id);
      const aslam::VisualFrame& visual_frame =
          observer.getVisualFrame(observation.frame_id.frame_index);
      const aslam::Transformation& T_G_M_observer =
          const_cast<const vi_map::VIMap*>(map)
              ->getMissionBaseFrameForVertex(observer_id)
              .get_T_G_M();

      // If there are precomputed/interpolated T_M_I, use those.
      aslam::Transformation T_G_I_observer;
      FrameToPoseMap::const_iterator it =
          interpolated_frame_poses.find(visual_frame.getId());
      if (it != interpolated_frame_poses.end()) {
        const aslam::Transformation& T_M_I_observer = it->second;
        T_G_I_observer = T_G_M_observer * T_M_I_observer;
      } else {
        const aslam::Transformation& T_M_I_observer = observer.get_T_M_I();
        T_G_I_observer = T_G_M_observer * T_M_I_observer;
      }

      Eigen::Vector2d measurement =
          visual_frame.getKeypointMeasurement(observation.keypoint_index);

      Eigen::Vector3d C_bearing_vector;
      bool projection_result =
          observer.getCamera(observation.frame_id.frame_index)
              ->backProject3(measurement, &C_bearing_vector);
      if (!projection_result) {
        statistics::StatsCollector stats(
            "Landmark triangulation failed proj failed.");
        stats.IncrementOne();
        continue;
      }

      const aslam::CameraId& cam_id =
          observer.getCamera(observation.frame_id.frame_index)->getId();
      aslam::Transformation T_G_C =
          (T_G_I_observer *
           observer.getNCameras()->get_T_C_B(cam_id).inverse());
      G_bearing_vectors.col(num_measurements) =
          T_G_C.getRotationMatrix() * C_bearing_vector;
      p_G_C_vector.col(num_measurements) = T_G_C.getPosition();
      ++num_measurements;
    }
    G_bearing_vectors.conservativeResize(Eigen::NoChange, num_measurements);
    p_G_C_vector.conservativeResize(Eigen::NoChange, num_measurements);

    if (num_measurements < 2) {
      statistics::StatsCollector stats("Landmark triangulation too few meas.");
      stats.IncrementOne();
      continue;
    }

    Eigen::Vector3d p_G_fi;
    aslam::TriangulationResult triangulation_result =
        aslam::linearTriangulateFromNViews(
            G_bearing_vectors, p_G_C_vector, &p_G_fi);

    if (triangulation_result.wasTriangulationSuccessful()) {
      landmark.set_p_B(T_G_I_storing.inverse() * p_G_fi);
      constexpr bool kReEvaluateQuality = true;
      if (vi_map::isLandmarkWellConstrained(
              *map, landmark, kReEvaluateQuality)) {
        statistics::StatsCollector stats_good("Landmark good");
        stats_good.IncrementOne();
        landmark.setQuality(vi_map::Landmark::Quality::kGood);
      } else {
        statistics::StatsCollector stats("Landmark bad after triangulation");
        stats.IncrementOne();
      }
    } else {
      statistics::StatsCollector stats("Landmark triangulation failed");
      stats.IncrementOne();
      if (triangulation_result.status() ==
          aslam::TriangulationResult::UNOBSERVABLE) {
        statistics::StatsCollector stats(
            "Landmark triangulation failed - unobservable");
        stats.IncrementOne();
      } else if (
          triangulation_result.status() ==
          aslam::TriangulationResult::UNINITIALIZED) {
        statistics::StatsCollector stats(
            "Landmark triangulation failed - uninitialized");
        stats.IncrementOne();
      }
    }
  }
}

bool retriangulateLandmarksOfMission(
    const vi_map::MissionId& mission_id,
    const FrameToPoseMap& interpolated_frame_poses, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  VLOG(1) << "Getting vertices of mission: " << mission_id;
  pose_graph::VertexIdList relevant_vertex_ids;
  map->getAllVertexIdsInMissionAlongGraph(mission_id, &relevant_vertex_ids);

  const size_t num_vertices = relevant_vertex_ids.size();
  VLOG(1) << "Retriangulating landmarks of " << num_vertices << " vertices.";

  common::MultiThreadedProgressBar progress_bar;
  std::function<void(const std::vector<size_t>&)> retriangulator =
      [&relevant_vertex_ids, map, &progress_bar,
       &interpolated_frame_poses](const std::vector<size_t>& batch) {
        progress_bar.setNumElements(batch.size());
        size_t num_processed = 0u;
        for (size_t item : batch) {
          CHECK_LT(item, relevant_vertex_ids.size());
          retriangulateLandmarksOfVertex(
              interpolated_frame_poses, relevant_vertex_ids[item], map);
          progress_bar.update(++num_processed);
        }
      };

  static constexpr bool kAlwaysParallelize = false;
  const size_t num_threads = common::getNumHardwareThreads();
  common::ParallelProcess(
      num_vertices, retriangulator, kAlwaysParallelize, num_threads);
  return true;
}
}  // namespace

bool retriangulateLandmarks(vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  vi_map::MissionIdList all_mission_ids;
  map->getAllMissionIds(&all_mission_ids);

  FrameToPoseMap interpolated_frame_poses;
  interpolateVisualFramePosesAllMissions(*map, &interpolated_frame_poses);

  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    retriangulateLandmarksOfMission(mission_id, interpolated_frame_poses, map);
  }
  return true;
}

bool retriangulateLandmarksOfMission(
    const vi_map::MissionId& mission_id, vi_map::VIMap* map) {
  const FrameToPoseMap empty_frame_to_pose_map;
  return retriangulateLandmarksOfMission(
      mission_id, empty_frame_to_pose_map, map);
}

void retriangulateLandmarksOfVertex(
    const pose_graph::VertexId& storing_vertex_id, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  FrameToPoseMap empty_frame_to_pose_map;
  retriangulateLandmarksOfVertex(
      empty_frame_to_pose_map, storing_vertex_id, map);
}

}  // namespace landmark_triangulation
