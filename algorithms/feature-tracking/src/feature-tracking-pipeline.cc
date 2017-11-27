#include "feature-tracking/feature-tracking-pipeline.h"

#include <aslam/cameras/camera.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/statistics/statistics.h>
#include <aslam/common/timer.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <aslam/geometric-vision/match-outlier-rejection-twopt.h>
#include <aslam/matcher/match.h>
#include <aslam/triangulation/triangulation.h>
#include <aslam/visualization/basic-visualization.h>
#include <aslam/visualization/feature-track-visualizer.h>
#include <glog/logging.h>
#include <maplab-common/progress-bar.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vi-map/check-map-consistency.h>
#include <vi-map/vi-map.h>
#include <visualization/common-rviz-visualization.h>

DEFINE_bool(
    feature_tracker_visualize_feature_tracks, false,
    "Flag indicating whether the full feature tracks are visualized.");

DEFINE_bool(
    feature_tracker_visualize_keypoints, false,
    "Flag indicating whether keypoints are visualized.");

DEFINE_bool(
    feature_tracker_visualize_keypoints_individual_frames, false,
    "Flag indicating whether keypoints are visualized with each frame "
    "on a separate ROS topic.");

DEFINE_bool(
    feature_tracker_visualize_keypoint_matches, false,
    "Flag indicating whether keypoint matches are visualized.");

DEFINE_bool(
    feature_tracker_publish_raw_images, false,
    "Flag indicating whether the raw images are published over ROS.");

DEFINE_bool(
    feature_tracker_check_map_for_consistency, false,
    "Flag indicating whether the map is checked for consistency after "
    "rerunning the feature tracking.");

namespace feature_tracking {

FeatureTrackingPipeline::FeatureTrackingPipeline()
    : feature_tracking_ros_base_topic_("tracking/"),
      visualize_keypoint_matches_(
          FLAGS_feature_tracker_visualize_keypoint_matches),
      processed_first_nframe_(false) {}

void FeatureTrackingPipeline::runTrackingAndTriangulationForAllMissions(
    vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  // First, remove all landmarks from the map.
  vi_map::LandmarkIdList landmark_ids;
  map->getAllLandmarkIds(&landmark_ids);

  common::ProgressBar cleaning_bar(landmark_ids.size());
  VLOG(1) << "Removing all " << landmark_ids.size()
          << " landmarks from the map.";
  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    map->removeLandmark(landmark_id);
    cleaning_bar.increment();
  }

  vi_map::MissionIdList mission_id_list;
  map->getAllMissionIds(&mission_id_list);

  VLOG(1) << "Running tracking and triangulation for each of the "
          << mission_id_list.size() << " missions.";
  for (const vi_map::MissionId& id : mission_id_list) {
    runTrackingAndTriangulationForMission(id, map);
  }
}

void FeatureTrackingPipeline::assignRawImagesToNFrame(
    const pose_graph::VertexId& vertex_id, vi_map::VIMap* map) const {
  CHECK_NOTNULL(map)->hasVertex(vertex_id);
  vi_map::Vertex& vertex = map->getVertex(vertex_id);
  const aslam::VisualNFrame::Ptr& nframe = vertex.getVisualNFrameShared();
  CHECK(nframe);
  const size_t num_frames = nframe->getNumFrames();
  // Set the raw images of nframe_kp1.
  for (size_t frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
    cv::Mat image;
    CHECK(map->getRawImage(vertex, frame_idx, &image))
        << "Vertex " << vertex_id << " does not have a raw image for frame "
        << frame_idx;

    if (FLAGS_feature_tracker_publish_raw_images) {
      const std::string topic = feature_tracking_ros_base_topic_ +
                                "camera_raw_" + std::to_string(frame_idx);
      visualization::RVizVisualizationSink::publish(topic, image);
    }
    nframe->getFrameShared(frame_idx)->setRawImage(image);
  }
}

void FeatureTrackingPipeline::extractAndTriangulateTerminatedFeatureTracks(
    const aslam::VisualNFrame::ConstPtr& nframe, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK(track_extractor_);

  aslam::FeatureTracksList terminated_tracks;
  track_extractor_->extractFromNFrameStream(nframe, &terminated_tracks);

  if (FLAGS_feature_tracker_visualize_feature_tracks) {
    cv::Mat image;
    feature_track_visualizer_.drawContinuousFeatureTracks(
        nframe, terminated_tracks, &image);

    const std::string topic =
        feature_tracking_ros_base_topic_ + "/feature_tracks";
    visualization::RVizVisualizationSink::publish(topic, image);
  }

  // Iterate over all terminated feature tracks, triangulate them and add them
  // to the map.
  for (const aslam::FeatureTracks& tracks : terminated_tracks) {
    for (const aslam::FeatureTrack& track : tracks) {
      VLOG(3) << "Triangulating track with length " << track.getTrackLength();
      // Transformations between all vertices and the world frame.
      Aligned<std::vector, aslam::Transformation> T_G_Is;

      const aslam::KeypointIdentifierList& keypoint_identifiers =
          track.getKeypointIdentifiers();

      // Get the normalized measurements for all observations on the track.
      Aligned<std::vector, Eigen::Vector2d> normalized_measurements;
      normalized_measurements.reserve(track.getTrackLength());

      const aslam::Camera::ConstPtr& track_camera =
          track.getFirstKeypointIdentifier().getCamera();
      CHECK(track_camera);

      // Get the T_G_Is for all the observation vertices.
      for (const aslam::KeypointIdentifier& keypoint_identifier :
           keypoint_identifiers) {
        NFrameIdToVertexIdMap::iterator vertex_iterator =
            nframe_id_to_vertex_id_map_.find(keypoint_identifier.getNFrameId());
        CHECK(vertex_iterator != nframe_id_to_vertex_id_map_.end());
        const pose_graph::VertexId& vertex_id = vertex_iterator->second;
        T_G_Is.emplace_back(map->getVertex_T_G_I(vertex_id));

        // Obtain the normalized keypoint measurements.
        const Eigen::Vector2d& keypoint_measurement =
            keypoint_identifier.getKeypointMeasurement();
        Eigen::Vector3d C_ray;
        track_camera->backProject3(keypoint_measurement, &C_ray);
        CHECK_GT(C_ray[2], 1e-8)
            << "Keypoint backprojection has zero z-component.";
        normalized_measurements.emplace_back(C_ray.head<2>() / C_ray[2]);
      }

      // Triangulate the landmark.
      Eigen::Vector3d G_landmark;
      const aslam::TriangulationResult triangulation_result =
          aslam::linearTriangulateFromNViews(
              normalized_measurements, T_G_Is,
              track.getFirstKeypointIdentifier().get_T_C_B().inverse(),
              &G_landmark);

      if (triangulation_result.wasTriangulationSuccessful()) {
        // Transform the landmark from the global frame to the vertex frame of
        // its first
        // observation.
        const aslam::KeypointIdentifier& first_observation_keypoint_identifier =
            track.getFirstKeypointIdentifier();
        NFrameIdToVertexIdMap::iterator first_observation_vertex_iterator =
            nframe_id_to_vertex_id_map_.find(
                first_observation_keypoint_identifier.getNFrameId());
        CHECK(
            first_observation_vertex_iterator !=
            nframe_id_to_vertex_id_map_.end());
        const pose_graph::VertexId& first_observation_vertex_id =
            first_observation_vertex_iterator->second;

        // Frames: Ib = Landmark base-frame.
        //         G = Global frame.
        const aslam::Transformation T_Ib_G =
            map->getVertex_T_G_I(first_observation_vertex_id).inverse();

        vi_map::Landmark landmark;
        landmark.set_p_B(T_Ib_G * G_landmark);

        // Create store landamrk ID and add the landmark to the map.
        vi_map::LandmarkId landmark_id;
        common::generateId(&landmark_id);
        landmark.setId(landmark_id);
        map->addNewLandmark(
            landmark, first_observation_vertex_id,
            first_observation_keypoint_identifier.getFrameIndex(),
            first_observation_keypoint_identifier.getKeypointIndex());

        // Add all observations to the map.
        for (const aslam::KeypointIdentifier& keypoint_identifier :
             keypoint_identifiers) {
          NFrameIdToVertexIdMap::iterator vertex_iterator =
              nframe_id_to_vertex_id_map_.find(
                  keypoint_identifier.getNFrameId());
          CHECK(vertex_iterator != nframe_id_to_vertex_id_map_.end());
          const pose_graph::VertexId& vertex_id = vertex_iterator->second;

          // Skip the base observation. This is already added above when we add
          // the new landmark.
          if (vertex_id == first_observation_vertex_id) {
            continue;
          }

          map->associateKeypointWithExistingLandmark(
              vertex_id, keypoint_identifier.getFrameIndex(),
              keypoint_identifier.getKeypointIndex(), landmark_id);
        }
      }
      successfully_triangulated_landmarks_accumulator_.Add(
          triangulation_result.wasTriangulationSuccessful() ? 1u : 0u);
    }
  }
}

void FeatureTrackingPipeline::runTrackingAndTriangulationForMission(
    vi_map::MissionId mission_id, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK(map->hasMission(mission_id)) << "The given mission " << mission_id
                                     << " is not present in the map.";
  VLOG(1) << "Running tracking and triangulation for mission with ID "
          << mission_id;
  nframe_id_to_vertex_id_map_.clear();

  pose_graph::VertexIdList all_vertices;
  map->getAllVertexIdsInMission(mission_id, &all_vertices);

  VLOG(1) << "Processing a total of " << all_vertices.size() << " vertices.";
  common::ProgressBar progress_bar(all_vertices.size());

  pose_graph::VertexId vertex_id_k =
      map->getMission(mission_id).getRootVertexId();
  vi_map::Vertex& root_vertex = map->getVertex(vertex_id_k);
  // Initialize pipeline.
  const size_t num_frames = root_vertex.numFrames();
  CHECK_GT(num_frames, 0u);

  const aslam::NCamera::ConstPtr ncamera =
      root_vertex.getVisualNFrame().getNCameraShared();
  CHECK(ncamera);
  CHECK_EQ(ncamera->numCameras(), num_frames);

  feature_track_visualizer_.setNumFrames(num_frames);
  track_extractor_.reset(new vio_common::FeatureTrackExtractor(ncamera));
  initialize(ncamera);

  // Process first nframe.
  assignRawImagesToNFrame(vertex_id_k, map);
  aslam::VisualNFrame::Ptr nframe_k = root_vertex.getVisualNFrameShared();
  nframe_k->clearKeypointChannelsOfAllFrames();
  nframe_id_to_vertex_id_map_.insert(
      std::make_pair(nframe_k->getId(), vertex_id_k));
  progress_bar.increment();

  pose_graph::VertexId vertex_id_kp1;
  while (map->getNextVertex(
      vertex_id_k, map->getGraphTraversalEdgeType(mission_id),
      &vertex_id_kp1)) {
    CHECK(vertex_id_k.isValid());
    CHECK(vertex_id_kp1.isValid());
    CHECK_NE(vertex_id_k, vertex_id_kp1);
    CHECK(nframe_k);

    assignRawImagesToNFrame(vertex_id_kp1, map);

    vi_map::Vertex& vertex_kp1 = map->getVertex(vertex_id_kp1);
    CHECK_EQ(vertex_kp1.numFrames(), num_frames);
    aslam::VisualNFrame::Ptr nframe_kp1 = vertex_kp1.getVisualNFrameShared();
    nframe_kp1->clearKeypointChannelsOfAllFrames();
    nframe_id_to_vertex_id_map_.insert(
        std::make_pair(nframe_kp1->getId(), vertex_id_kp1));

    const aslam::Transformation T_Ik_Ikp1 =
        map->getVertex_T_G_I(vertex_id_k).inverse() *
        map->getVertex_T_G_I(vertex_id_kp1);
    trackFeaturesNFrame(T_Ik_Ikp1, nframe_k.get(), nframe_kp1.get());

    if (!processed_first_nframe_) {
      map->getVertex(vertex_id_k).resetObservedLandmarkIdsToInvalid();
      extractAndTriangulateTerminatedFeatureTracks(nframe_k, map);
      processed_first_nframe_ = true;
    }
    map->getVertex(vertex_id_kp1).resetObservedLandmarkIdsToInvalid();
    extractAndTriangulateTerminatedFeatureTracks(nframe_kp1, map);

    if (FLAGS_feature_tracker_visualize_keypoints) {
      cv::Mat image;
      aslam_cv_visualization::visualizeKeypoints(nframe_kp1, &image);
      const std::string topic = feature_tracking_ros_base_topic_ + "/keypoints";
      visualization::RVizVisualizationSink::publish(topic, image);
    }

    if (FLAGS_feature_tracker_visualize_keypoints_individual_frames) {
      CHECK_EQ(num_frames, nframe_kp1->getNumFrames());
      for (size_t frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
        cv::Mat image;
        aslam_cv_visualization::drawKeypoints(
            nframe_kp1->getFrame(frame_idx), &image);
        const std::string topic = feature_tracking_ros_base_topic_ +
                                  "/keypoints_cam" + std::to_string(frame_idx);
        visualization::RVizVisualizationSink::publish(topic, image);
      }
    }

    vertex_id_k = vertex_id_kp1;
    nframe_k->releaseRawImagesOfAllFrames();
    nframe_k = nframe_kp1;
    progress_bar.increment();
  }

  VLOG(1) << "Successfully retracked features and triangulated new landmarks"
          << " for mission " << mission_id;

  VLOG(1) << "\t Total num successfully triangulated landmarks: "
          << successfully_triangulated_landmarks_accumulator_.sum();
  if (successfully_triangulated_landmarks_accumulator_.total_samples() > 0) {
    VLOG(1) << "\t In percent: "
            << static_cast<double>(
                   successfully_triangulated_landmarks_accumulator_.sum()) /
                   static_cast<double>(
                       successfully_triangulated_landmarks_accumulator_
                           .total_samples()) *
                   100.0
            << "%";
  }

  if (FLAGS_feature_tracker_check_map_for_consistency) {
    VLOG(1) << "Checking the modified map for consistency...";
    CHECK(vi_map::checkMapConsistency(*map));
    VLOG(1) << "The modified map is consistent";
  }
}
}  // namespace feature_tracking
