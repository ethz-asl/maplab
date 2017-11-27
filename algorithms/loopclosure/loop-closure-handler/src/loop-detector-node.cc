#include "loop-closure-handler/loop-detector-node.h"

#include <algorithm>
#include <mutex>
#include <sstream>  // NOLINT
#include <string>

#include <Eigen/Geometry>
#include <aslam/common/statistics/statistics.h>
#include <descriptor-projection/descriptor-projection.h>
#include <descriptor-projection/flags.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <localization-summary-map/localization-summary-map.h>
#include <loopclosure-common/types.h>
#include <maplab-common/accessors.h>
#include <maplab-common/geometry.h>
#include <maplab-common/multi-threaded-progress-bar.h>
#include <maplab-common/parallel-process.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/progress-bar.h>
#include <matching-based-loopclosure/detector-settings.h>
#include <matching-based-loopclosure/loop-detector-interface.h>
#include <matching-based-loopclosure/matching-based-engine.h>
#include <matching-based-loopclosure/scoring.h>
#include <vi-map/landmark-quality-metrics.h>

#include "loop-closure-handler/loop-closure-handler.h"
#include "loop-closure-handler/visualization/loop-closure-visualizer.h"

DEFINE_bool(
    lc_filter_underconstrained_landmarks, true,
    "If underconstrained landmarks should be filtered for the "
    "loop-closure.");
DEFINE_bool(lc_use_random_pnp_seed, true, "Use random seed for pnp RANSAC.");

namespace loop_detector_node {
LoopDetectorNode::LoopDetectorNode()
    : use_random_pnp_seed_(FLAGS_lc_use_random_pnp_seed) {
  matching_based_loopclosure::MatchingBasedEngineSettings
      matching_engine_settings;
  loop_detector_ =
      std::make_shared<matching_based_loopclosure::MatchingBasedLoopDetector>(
          matching_engine_settings);
}

const std::string LoopDetectorNode::serialization_filename_ =
    "loop_detector_node";

std::string LoopDetectorNode::printStatus() const {
  std::stringstream ss;
  ss << "Loop-detector status:" << std::endl;
  if (loop_detector_ != nullptr) {
    ss << "\tNum entries:" << loop_detector_->NumEntries() << std::endl;
    ss << "\tNum descriptors: " << loop_detector_->NumDescriptors()
       << std::endl;
  } else {
    ss << "\t NULL" << std::endl;
  }
  return ss.str();
}

bool LoopDetectorNode::convertFrameMatchesToConstraint(
    const loop_closure::FrameIdMatchesPair& query_frame_id_and_matches,
    vi_map::LoopClosureConstraint* constraint_ptr) const {
  CHECK_NOTNULL(constraint_ptr);

  const loop_closure::MatchVector& matches = query_frame_id_and_matches.second;
  if (matches.empty()) {
    return false;
  }

  vi_map::LoopClosureConstraint& constraint = *constraint_ptr;

  using vi_map::FrameKeyPointToStructureMatch;

  // Translate frame_ids to vertex id and frame index.
  constraint.structure_matches.clear();
  constraint.structure_matches.reserve(matches.size());
  constraint.query_vertex_id = query_frame_id_and_matches.first.vertex_id;
  for (const FrameKeyPointToStructureMatch& match : matches) {
    CHECK(match.isValid());
    vi_map::VertexKeyPointToStructureMatch structure_match;
    structure_match.landmark_result = match.landmark_result;
    structure_match.keypoint_index_query =
        match.keypoint_id_query.keypoint_index;
    structure_match.frame_identifier_result = match.keyframe_id_result;
    structure_match.frame_index_query =
        match.keypoint_id_query.frame_id.frame_index;
    constraint.structure_matches.push_back(structure_match);
  }
  return true;
}

void LoopDetectorNode::convertFrameToProjectedImage(
    const vi_map::VIMap& map, const vi_map::VisualFrameIdentifier& frame_id,
    const aslam::VisualFrame& frame,
    const vi_map::LandmarkIdList& observed_landmark_ids,
    const vi_map::MissionId& mission_id, const bool skip_invalid_landmark_ids,
    loop_closure::ProjectedImage* projected_image) const {
  CHECK_NOTNULL(projected_image);
  // We want to add all landmarks.
  vi_map::LandmarkIdSet landmarks_to_add(
      observed_landmark_ids.begin(), observed_landmark_ids.end());
  convertFrameToProjectedImageOnlyUsingProvidedLandmarkIds(
      map, frame_id, frame, observed_landmark_ids, mission_id,
      skip_invalid_landmark_ids, landmarks_to_add, projected_image);
}

void LoopDetectorNode::convertFrameToProjectedImageOnlyUsingProvidedLandmarkIds(
    const vi_map::VIMap& map, const vi_map::VisualFrameIdentifier& frame_id,
    const aslam::VisualFrame& frame,
    const vi_map::LandmarkIdList& observed_landmark_ids,
    const vi_map::MissionId& mission_id, const bool skip_invalid_landmark_ids,
    const vi_map::LandmarkIdSet& landmarks_to_add,
    loop_closure::ProjectedImage* projected_image) const {
  CHECK_NOTNULL(projected_image);
  projected_image->dataset_id = mission_id;
  projected_image->keyframe_id = frame_id;
  projected_image->timestamp_nanoseconds = frame.getTimestampNanoseconds();

  CHECK_EQ(
      static_cast<int>(observed_landmark_ids.size()),
      frame.getKeypointMeasurements().cols());
  CHECK_EQ(
      static_cast<int>(observed_landmark_ids.size()),
      frame.getDescriptors().cols());

  const Eigen::Matrix2Xd& original_measurements =
      frame.getKeypointMeasurements();
  const aslam::VisualFrame::DescriptorsT& original_descriptors =
      frame.getDescriptors();

  aslam::VisualFrame::DescriptorsT valid_descriptors(
      original_descriptors.rows(), original_descriptors.cols());
  Eigen::Matrix2Xd valid_measurements(2, original_measurements.cols());
  vi_map::LandmarkIdList valid_landmark_ids(original_measurements.cols());

  int num_valid_landmarks = 0;
  for (int i = 0; i < original_measurements.cols(); ++i) {
    const bool is_landmark_id_valid = observed_landmark_ids[i].isValid();
    const bool is_landmark_valid =
        !skip_invalid_landmark_ids || is_landmark_id_valid;

    const bool landmark_well_constrained =
        !skip_invalid_landmark_ids ||
        !FLAGS_lc_filter_underconstrained_landmarks ||
        (is_landmark_id_valid &&
         vi_map::isLandmarkWellConstrained(
             map, map.getLandmark(observed_landmark_ids[i])));

    if (skip_invalid_landmark_ids && is_landmark_id_valid) {
      CHECK(
          map.getLandmark(observed_landmark_ids[i]).getQuality() !=
          vi_map::Landmark::Quality::kUnknown)
          << "Please "
          << "retriangulate the landmarks before using the loop closure "
          << "engine.";
    }

    const bool is_landmark_in_set_to_add =
        landmarks_to_add.count(observed_landmark_ids[i]) > 0u;

    if (landmark_well_constrained && is_landmark_in_set_to_add &&
        is_landmark_valid) {
      valid_measurements.col(num_valid_landmarks) =
          original_measurements.col(i);
      valid_descriptors.col(num_valid_landmarks) = original_descriptors.col(i);
      valid_landmark_ids[num_valid_landmarks] = observed_landmark_ids[i];
      ++num_valid_landmarks;
    }
  }

  valid_measurements.conservativeResize(Eigen::NoChange, num_valid_landmarks);
  valid_descriptors.conservativeResize(Eigen::NoChange, num_valid_landmarks);
  valid_landmark_ids.resize(num_valid_landmarks);

  if (skip_invalid_landmark_ids) {
    statistics::StatsCollector stats_landmarks("LC num_landmarks insertion");
    stats_landmarks.AddSample(num_valid_landmarks);
  } else {
    statistics::StatsCollector stats_landmarks("LC num_landmarks query");
    stats_landmarks.AddSample(num_valid_landmarks);
  }

  projected_image->landmarks.swap(valid_landmark_ids);
  projected_image->measurements.swap(valid_measurements);
  loop_detector_->ProjectDescriptors(
      valid_descriptors, &projected_image->projected_descriptors);
}

void LoopDetectorNode::convertLocalizationFrameToProjectedImage(
    const aslam::VisualNFrame& nframe,
    const loop_closure::KeyframeId& keyframe_id,
    const bool skip_untracked_keypoints,
    const loop_closure::ProjectedImage::Ptr& projected_image,
    KeyframeToKeypointReindexMap* keyframe_to_keypoint_reindexing,
    vi_map::LandmarkIdList* observed_landmark_ids) const {
  CHECK(projected_image != nullptr);
  CHECK_NOTNULL(keyframe_to_keypoint_reindexing);
  CHECK_NOTNULL(observed_landmark_ids)->clear();

  const aslam::VisualFrame& frame = nframe.getFrame(keyframe_id.frame_index);
  if (skip_untracked_keypoints) {
    CHECK(frame.hasTrackIds()) << "Can only skip untracked keypoints if the "
                               << "track id channel is available.";
  }

  // Create some dummy ids for the localization frame that isn't part of the map
  // yet. This is required to use the same interfaces from the loop-closure
  // backend.
  projected_image->dataset_id = common::createRandomId<vi_map::MissionId>();
  projected_image->keyframe_id = keyframe_id;
  projected_image->timestamp_nanoseconds = frame.getTimestampNanoseconds();

  // Project the selected binary descriptors.
  const Eigen::Matrix2Xd& original_measurements =
      frame.getKeypointMeasurements();
  const aslam::VisualFrame::DescriptorsT& original_descriptors =
      frame.getDescriptors();
  CHECK_EQ(original_measurements.cols(), original_descriptors.cols());
  const Eigen::VectorXi* frame_trackids = nullptr;
  if (frame.hasTrackIds()) {
    frame_trackids = &frame.getTrackIds();
  }

  aslam::VisualFrame::DescriptorsT valid_descriptors(
      original_descriptors.rows(), original_descriptors.cols());
  Eigen::Matrix2Xd valid_measurements(2, original_measurements.cols());
  vi_map::LandmarkIdList valid_landmark_ids;
  valid_landmark_ids.reserve(original_measurements.cols());
  observed_landmark_ids->resize(original_measurements.cols());

  unsigned int num_valid_landmarks = 0;
  for (int i = 0; i < original_measurements.cols(); ++i) {
    if (skip_untracked_keypoints && (frame_trackids != nullptr) &&
        (*frame_trackids)(i) < 0) {
      continue;
    }

    valid_measurements.col(num_valid_landmarks) = original_measurements.col(i);
    valid_descriptors.col(num_valid_landmarks) = original_descriptors.col(i);
    const vi_map::LandmarkId random_landmark_id =
        common::createRandomId<vi_map::LandmarkId>();
    (*observed_landmark_ids)[i] = random_landmark_id;
    valid_landmark_ids.push_back(random_landmark_id);
    (*keyframe_to_keypoint_reindexing)[keyframe_id].emplace_back(i);
    ++num_valid_landmarks;
  }

  valid_measurements.conservativeResize(Eigen::NoChange, num_valid_landmarks);
  valid_descriptors.conservativeResize(Eigen::NoChange, num_valid_landmarks);
  valid_landmark_ids.shrink_to_fit();

  projected_image->landmarks.swap(valid_landmark_ids);
  projected_image->measurements.swap(valid_measurements);
  loop_detector_->ProjectDescriptors(
      valid_descriptors, &projected_image->projected_descriptors);
}

void LoopDetectorNode::addVertexToDatabase(
    const pose_graph::VertexId& vertex_id, const vi_map::VIMap& map) {
  CHECK(map.hasVertex(vertex_id));
  const vi_map::Vertex& vertex = map.getVertex(vertex_id);
  const unsigned int num_frames = vertex.numFrames();
  for (unsigned int frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
    if (vertex.isVisualFrameSet(frame_idx) &&
        vertex.isVisualFrameValid(frame_idx)) {
      std::shared_ptr<loop_closure::ProjectedImage> projected_image =
          std::make_shared<loop_closure::ProjectedImage>();
      constexpr bool kSkipInvalidLandmarkIds = true;
      vi_map::LandmarkIdList landmark_ids;
      vertex.getFrameObservedLandmarkIds(frame_idx, &landmark_ids);
      VLOG(200) << "Frame " << frame_idx << " of vertex " << vertex_id
                << " with "
                << vertex.getVisualFrame(frame_idx).getDescriptors().cols()
                << " descriptors";
      convertFrameToProjectedImage(
          map, vi_map::VisualFrameIdentifier(vertex.id(), frame_idx),
          vertex.getVisualFrame(frame_idx), landmark_ids, vertex.getMissionId(),
          kSkipInvalidLandmarkIds, projected_image.get());

      loop_detector_->Insert(projected_image);
    }
  }
}

bool LoopDetectorNode::hasMissionInDatabase(
    const vi_map::MissionId& mission_id) const {
  return missions_in_database_.count(mission_id) > 0u;
}

void LoopDetectorNode::addMissionToDatabase(
    const vi_map::MissionId& mission_id, const vi_map::VIMap& map) {
  CHECK(map.hasMission(mission_id));
  missions_in_database_.emplace(mission_id);

  VLOG(1) << "Getting vertices in mission " << mission_id;
  pose_graph::VertexIdList all_vertices;
  map.getAllVertexIdsInMissionAlongGraph(mission_id, &all_vertices);

  VLOG(1) << "Got vertices in mission " << mission_id;
  VLOG(1) << "Adding mission " << mission_id << " to database.";

  addVerticesToDatabase(all_vertices, map);
}

void LoopDetectorNode::addVerticesToDatabase(
    const pose_graph::VertexIdList& vertex_ids, const vi_map::VIMap& map) {
  common::ProgressBar progress_bar(vertex_ids.size());

  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    progress_bar.increment();
    addVertexToDatabase(vertex_id, map);
  }
}

void LoopDetectorNode::addLocalizationSummaryMapToDatabase(
    const summary_map::LocalizationSummaryMap& localization_summary_map) {
  CHECK(
      summary_maps_in_database_.emplace(localization_summary_map.id()).second);

  pose_graph::VertexIdList observer_ids;
  localization_summary_map.getAllObserverIds(&observer_ids);

  const Eigen::Matrix3Xf& G_observer_positions =
      localization_summary_map.GObserverPosition();
  if (observer_ids.empty()) {
    if (G_observer_positions.cols() > 0) {
      // Vertex ids were not stored in the summary map. Generating random ones.
      observer_ids.resize(G_observer_positions.cols());
      for (pose_graph::VertexId& vertex_id : observer_ids) {
        common::generateId(&vertex_id);
      }
    } else {
      LOG(FATAL) << "No observers in the summary map found. Is it initialized?";
    }
  }

  std::vector<std::vector<int>> observer_observations;
  observer_observations.resize(observer_ids.size());

  // The index of the observer for every observation in the summary map.
  const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>& observer_indices =
      localization_summary_map.observerIndices();

  // Accumulate the observation indices per observer.
  for (int i = 0; i < observer_indices.rows(); ++i) {
    const int observer_index = observer_indices(i, 0);
    CHECK_LT(observer_index, static_cast<int>(observer_observations.size()));
    observer_observations[observer_index].push_back(i);
  }
  // Generate a random mission_id for this map.
  vi_map::MissionId mission_id;
  common::generateId(&mission_id);

  vi_map::LandmarkIdList observed_landmark_ids;
  localization_summary_map.getAllLandmarkIds(&observed_landmark_ids);

  const Eigen::MatrixXf& projected_descriptors =
      localization_summary_map.projectedDescriptors();

  const int descriptor_dimensionality = projected_descriptors.rows();

  const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>&
      observation_to_landmark_index =
          localization_summary_map.observationToLandmarkIndex();

  for (size_t observer_idx = 0; observer_idx < observer_observations.size();
       ++observer_idx) {
    std::shared_ptr<loop_closure::ProjectedImage> projected_image_ptr =
        std::make_shared<loop_closure::ProjectedImage>();
    loop_closure::ProjectedImage& projected_image = *projected_image_ptr;

    // Timestamp not relevant since this image will not collide with any
    // other image given its unique mission-id.
    projected_image.timestamp_nanoseconds = 0;
    projected_image.dataset_id = mission_id;
    constexpr unsigned int kFrameIndex = 0;
    projected_image.keyframe_id =
        vi_map::VisualFrameIdentifier(observer_ids[observer_idx], kFrameIndex);

    const std::vector<int>& observations = observer_observations[observer_idx];
    projected_image.landmarks.resize(observations.size());
    // Measurements have no meaning, so we add a zero block.
    projected_image.measurements.setZero(2, observations.size());
    projected_image.projected_descriptors.resize(
        descriptor_dimensionality, observations.size());

    for (size_t i = 0; i < observations.size(); ++i) {
      const int observation_index = observations[i];
      CHECK_LT(observation_index, projected_descriptors.cols());
      projected_image.projected_descriptors.col(i) =
          projected_descriptors.col(observation_index);

      CHECK_LT(observation_index, observation_to_landmark_index.rows());
      const size_t landmark_index =
          observation_to_landmark_index(observation_index, 0);
      CHECK_LT(landmark_index, observed_landmark_ids.size());
      projected_image.landmarks[i] = observed_landmark_ids[landmark_index];
    }
    loop_detector_->Insert(projected_image_ptr);
  }
}

void LoopDetectorNode::addLandmarkSetToDatabase(
    const vi_map::LandmarkIdSet& landmark_id_set,
    const vi_map::VIMap& map) {
  typedef std::unordered_map<vi_map::VisualFrameIdentifier,
                             vi_map::LandmarkIdSet>
      VisualFrameToGlobalLandmarkIdsMap;
  VisualFrameToGlobalLandmarkIdsMap visual_frame_to_global_landmarks_map;

  for (const vi_map::LandmarkId& landmark_id : landmark_id_set) {
    const vi_map::Landmark& landmark = map.getLandmark(landmark_id);
    landmark.forEachObservation(
        [&](const vi_map::KeypointIdentifier& observer_backlink) {
          visual_frame_to_global_landmarks_map[observer_backlink.frame_id]
              .emplace(landmark_id);
        });
  }

  for (const VisualFrameToGlobalLandmarkIdsMap::value_type&
           frameid_and_landmarks : visual_frame_to_global_landmarks_map) {
    const vi_map::VisualFrameIdentifier& frame_identifier =
        frameid_and_landmarks.first;
    const vi_map::Vertex& vertex = map.getVertex(frame_identifier.vertex_id);

    vi_map::LandmarkIdList landmark_ids;
    vertex.getFrameObservedLandmarkIds(
        frame_identifier.frame_index, &landmark_ids);

    std::shared_ptr<loop_closure::ProjectedImage> projected_image =
        std::make_shared<loop_closure::ProjectedImage>();
    constexpr bool kSkipInvalidLandmarkIds = true;
    convertFrameToProjectedImageOnlyUsingProvidedLandmarkIds(
        map, frame_identifier,
        vertex.getVisualFrame(frame_identifier.frame_index), landmark_ids,
        vertex.getMissionId(), kSkipInvalidLandmarkIds,
        frameid_and_landmarks.second, projected_image.get());

    loop_detector_->Insert(projected_image);
  }
}

bool LoopDetectorNode::findNFrameInSummaryMapDatabase(
    const aslam::VisualNFrame& n_frame, const bool skip_untracked_keypoints,
    const summary_map::LocalizationSummaryMap& localization_summary_map,
    pose::Transformation* T_G_I, unsigned int* num_of_lc_matches,
    vi_map::VertexKeyPointToStructureMatchList* inlier_structure_matches)
    const {
  CHECK_NOTNULL(T_G_I);
  CHECK_NOTNULL(num_of_lc_matches);
  CHECK_NOTNULL(inlier_structure_matches);

  CHECK(!summary_maps_in_database_.empty())
      << "No summary maps were added "
      << "to the database. This method only operates on summary maps.";

  loop_closure::FrameToMatches frame_matches_list;

  std::vector<vi_map::LandmarkIdList> query_vertex_observed_landmark_ids;

  findNearestNeighborMatchesForNFrame(
      n_frame, skip_untracked_keypoints, &query_vertex_observed_landmark_ids,
      num_of_lc_matches, &frame_matches_list);

  timing::Timer timer_compute_relative("lc compute absolute transform");
  constexpr bool kMergeLandmarks = false;
  constexpr bool kAddLoopclosureEdges = false;
  loop_closure_handler::LoopClosureHandler handler(&localization_summary_map,
                                                   &landmark_id_old_to_new_);

  constexpr pose_graph::VertexId* kVertexIdClosestToStructureMatches = nullptr;
  const bool success = computeAbsoluteTransformFromFrameMatches(
      n_frame, query_vertex_observed_landmark_ids, frame_matches_list,
      kMergeLandmarks, kAddLoopclosureEdges, handler, T_G_I,
      inlier_structure_matches, kVertexIdClosestToStructureMatches);

  if (visualizer_ && success) {
    visualizer_->visualizeSummaryMapDatabase(localization_summary_map);
    visualizer_->visualizeKeyframeToStructureMatch(
        *inlier_structure_matches, T_G_I->getPosition(),
        localization_summary_map);
  }

  return success;
}

bool LoopDetectorNode::findNFrameInDatabase(
    const aslam::VisualNFrame& n_frame, const bool skip_untracked_keypoints,
    vi_map::VIMap* map, pose::Transformation* T_G_I,
    unsigned int* num_of_lc_matches,
    vi_map::VertexKeyPointToStructureMatchList* inlier_structure_matches,
    pose_graph::VertexId* vertex_id_closest_to_structure_matches) const {
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(T_G_I);
  CHECK_NOTNULL(num_of_lc_matches);
  CHECK_NOTNULL(inlier_structure_matches)->clear();
  // Note: vertex_id_closest_to_structure_matches is optional and may be NULL.

  loop_closure::FrameToMatches frame_matches_list;

  std::vector<vi_map::LandmarkIdList> query_vertex_observed_landmark_ids;

  findNearestNeighborMatchesForNFrame(
      n_frame, skip_untracked_keypoints, &query_vertex_observed_landmark_ids,
      num_of_lc_matches, &frame_matches_list);

  timing::Timer timer_compute_relative("lc compute absolute transform");
  constexpr bool kMergeLandmarks = false;
  constexpr bool kAddLoopclosureEdges = false;
  loop_closure_handler::LoopClosureHandler handler(map,
                                                   &landmark_id_old_to_new_);
  return computeAbsoluteTransformFromFrameMatches(
      n_frame, query_vertex_observed_landmark_ids, frame_matches_list,
      kMergeLandmarks, kAddLoopclosureEdges, handler, T_G_I,
      inlier_structure_matches, vertex_id_closest_to_structure_matches);
}

void LoopDetectorNode::findNearestNeighborMatchesForNFrame(
    const aslam::VisualNFrame& n_frame, const bool skip_untracked_keypoints,
    std::vector<vi_map::LandmarkIdList>* query_vertex_observed_landmark_ids,
    unsigned int* num_of_lc_matches,
    loop_closure::FrameToMatches* frame_matches_list) const {
  CHECK_NOTNULL(query_vertex_observed_landmark_ids)->clear();
  CHECK_NOTNULL(num_of_lc_matches);
  CHECK_NOTNULL(frame_matches_list);

  *num_of_lc_matches = 0u;

  timing::Timer timer_preprocess("Loop Closure: preprocess frames");
  const size_t num_frames = n_frame.getNumFrames();
  loop_closure::ProjectedImagePtrList projected_image_ptr_list;
  projected_image_ptr_list.reserve(num_frames);
  query_vertex_observed_landmark_ids->resize(num_frames);
  std::vector<loop_closure::KeyframeId> frame_ids;
  frame_ids.reserve(num_frames);
  KeyframeToKeypointReindexMap keyframe_to_keypoint_reindexing;
  keyframe_to_keypoint_reindexing.reserve(num_frames);

  const pose_graph::VertexId query_vertex_id(
      common::createRandomId<pose_graph::VertexId>());
  for (size_t frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
    if (n_frame.isFrameSet(frame_idx) && n_frame.isFrameValid(frame_idx)) {
      const aslam::VisualFrame::ConstPtr frame =
          n_frame.getFrameShared(frame_idx);

      CHECK(frame->hasKeypointMeasurements());
      if (frame->getNumKeypointMeasurements() == 0u) {
        // Skip frame if zero measurements found.
        continue;
      }
      frame_ids.emplace_back(query_vertex_id, frame_idx);

      projected_image_ptr_list.push_back(
          std::make_shared<loop_closure::ProjectedImage>());
      convertLocalizationFrameToProjectedImage(
          n_frame, frame_ids.back(), skip_untracked_keypoints,
          projected_image_ptr_list.back(), &keyframe_to_keypoint_reindexing,
          &(*query_vertex_observed_landmark_ids)[frame_idx]);
    }
  }
  timer_preprocess.Stop();
  constexpr bool kParallelFindIfPossible = true;
  loop_detector_->Find(
      projected_image_ptr_list, kParallelFindIfPossible, frame_matches_list);

  // Correct the indices in case untracked keypoints were removed.
  // For the pose recovery with RANSAC, the keypoint indices of the frame are
  // decisive, not those stored in the projected image. Therefore, the
  // keypoint indices of the matches (inferred from the projected image) have to
  // be mapped back to the keypoint indices of the frame.
  if (skip_untracked_keypoints) {
    for (loop_closure::FrameToMatches::value_type& frame_matches :
         *frame_matches_list) {
      for (loop_closure::Match& match : frame_matches.second) {
        KeyframeToKeypointReindexMap::const_iterator iter_keyframe_supsampling =
            keyframe_to_keypoint_reindexing.find(
                match.keypoint_id_query.frame_id);
        CHECK(
            iter_keyframe_supsampling != keyframe_to_keypoint_reindexing.end());
        match.keypoint_id_query.keypoint_index =
            iter_keyframe_supsampling
                ->second[match.keypoint_id_query.keypoint_index];
      }
    }
  }

  *num_of_lc_matches = loop_closure::getNumberOfMatches(*frame_matches_list);
}

bool LoopDetectorNode::findVertexInDatabase(
    const vi_map::Vertex& query_vertex, const bool merge_landmarks,
    const bool add_lc_edges, vi_map::VIMap* map, pose::Transformation* T_G_I,
    unsigned int* num_of_lc_matches,
    vi_map::LoopClosureConstraint* inlier_constraint) const {
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(T_G_I);
  CHECK_NOTNULL(num_of_lc_matches);
  CHECK_NOTNULL(inlier_constraint);

  if (loop_detector_->NumEntries() == 0u) {
    return false;
  }

  const size_t num_frames = query_vertex.numFrames();
  loop_closure::ProjectedImagePtrList projected_image_ptr_list;
  projected_image_ptr_list.reserve(num_frames);

  for (size_t frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
    if (query_vertex.isVisualFrameSet(frame_idx) &&
        query_vertex.isVisualFrameValid(frame_idx)) {
      vi_map::VisualFrameIdentifier query_frame_id(
          query_vertex.id(), frame_idx);

      std::vector<vi_map::LandmarkId> observed_landmark_ids;
      query_vertex.getFrameObservedLandmarkIds(
          frame_idx, &observed_landmark_ids);
      projected_image_ptr_list.push_back(
          std::make_shared<loop_closure::ProjectedImage>());
      constexpr bool kSkipInvalidLandmarkIds = false;
      convertFrameToProjectedImage(
          *map, query_frame_id, query_vertex.getVisualFrame(frame_idx),
          observed_landmark_ids, query_vertex.getMissionId(),
          kSkipInvalidLandmarkIds, projected_image_ptr_list.back().get());
    }
  }

  loop_closure::FrameToMatches frame_matches_list;
  constexpr bool kParallelFindIfPossible = true;
  loop_detector_->Find(
      projected_image_ptr_list, kParallelFindIfPossible, &frame_matches_list);
  *num_of_lc_matches = loop_closure::getNumberOfMatches(frame_matches_list);

  timing::Timer timer_compute_relative("lc compute absolute transform");
  pose_graph::VertexId vertex_id_closest_to_structure_matches;
  bool ransac_ok = computeAbsoluteTransformFromFrameMatches(
      frame_matches_list, merge_landmarks, add_lc_edges, map, T_G_I,
      inlier_constraint, &vertex_id_closest_to_structure_matches);
  timer_compute_relative.Stop();
  return ransac_ok;
}

bool LoopDetectorNode::computeAbsoluteTransformFromFrameMatches(
    const loop_closure::FrameToMatches& frame_to_matches,
    const bool merge_landmarks, const bool add_lc_edges, vi_map::VIMap* map,
    pose::Transformation* T_G_I,
    vi_map::LoopClosureConstraint* inlier_constraints,
    pose_graph::VertexId* vertex_id_closest_to_structure_matches) const {
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(T_G_I);
  CHECK_NOTNULL(inlier_constraints);
  CHECK_NOTNULL(vertex_id_closest_to_structure_matches);

  const size_t num_matches = loop_closure::getNumberOfMatches(frame_to_matches);
  if (num_matches == 0u) {
    return false;
  }

  vi_map::LoopClosureConstraint constraint;
  for (const loop_closure::FrameIdMatchesPair& frame_matches_pair :
       frame_to_matches) {
    vi_map::LoopClosureConstraint tmp_constraint;
    const bool conversion_success =
        convertFrameMatchesToConstraint(frame_matches_pair, &tmp_constraint);
    if (!conversion_success) {
      continue;
    }
    constraint.query_vertex_id = tmp_constraint.query_vertex_id;
    constraint.structure_matches.insert(
        constraint.structure_matches.end(),
        tmp_constraint.structure_matches.begin(),
        tmp_constraint.structure_matches.end());
  }
  int num_inliers = 0;
  double inlier_ratio = 0.0;

  // The estimated transformation of this vertex to the map.
  pose::Transformation& T_G_I_ransac = *T_G_I;
  loop_closure_handler::LoopClosureHandler::MergedLandmark3dPositionVector
      landmark_pairs_merged;
  std::mutex map_mutex;
  bool ransac_ok = handleLoopClosures(
      constraint, merge_landmarks, add_lc_edges, &num_inliers, &inlier_ratio,
      map, &T_G_I_ransac, inlier_constraints, &landmark_pairs_merged,
      vertex_id_closest_to_structure_matches, &map_mutex);

  statistics::StatsCollector stats_ransac_inliers(
      "LC AbsolutePoseRansacInliers");
  stats_ransac_inliers.AddSample(num_inliers);
  statistics::StatsCollector stats_ransac_inlier_ratio(
      "LC AbsolutePoseRansacInlierRatio");
  stats_ransac_inlier_ratio.AddSample(num_inliers);

  return ransac_ok;
}

bool LoopDetectorNode::computeAbsoluteTransformFromFrameMatches(
    const aslam::VisualNFrame& query_vertex_n_frame,
    const std::vector<vi_map::LandmarkIdList>&
        query_vertex_observed_landmark_ids,
    const loop_closure::FrameToMatches& frame_to_matches,
    const bool merge_landmarks, const bool add_lc_edges,
    const loop_closure_handler::LoopClosureHandler& handler,
    pose::Transformation* T_G_I,
    vi_map::VertexKeyPointToStructureMatchList* inlier_structure_matches,
    pose_graph::VertexId* vertex_id_closest_to_structure_matches) const {
  CHECK_NOTNULL(T_G_I);
  CHECK_NOTNULL(inlier_structure_matches);
  // Note: vertex_id_closest_to_structure_matches is optional and may be NULL.

  const size_t num_matches = loop_closure::getNumberOfMatches(frame_to_matches);
  if (num_matches == 0u) {
    return false;
  }
  pose_graph::VertexId invalid_vertex_id;
  vi_map::LoopClosureConstraint constraint;
  constraint.query_vertex_id = invalid_vertex_id;
  for (const loop_closure::FrameIdMatchesPair& frame_matches_pair :
       frame_to_matches) {
    vi_map::LoopClosureConstraint tmp_constraint;
    const bool conversion_success =
        convertFrameMatchesToConstraint(frame_matches_pair, &tmp_constraint);
    if (!conversion_success) {
      continue;
    }
    constraint.query_vertex_id = tmp_constraint.query_vertex_id;
    constraint.structure_matches.insert(
        constraint.structure_matches.end(),
        tmp_constraint.structure_matches.begin(),
        tmp_constraint.structure_matches.end());
  }
  int num_inliers = 0;
  double inlier_ratio = 0.0;

  // The estimated transformation of this vertex to the map.
  pose::Transformation& T_G_I_ransac = *T_G_I;
  loop_closure_handler::LoopClosureHandler::MergedLandmark3dPositionVector
      landmark_pairs_merged;
  std::mutex map_mutex;

  bool ransac_ok = handler.handleLoopClosure(
      query_vertex_n_frame, query_vertex_observed_landmark_ids,
      invalid_vertex_id, constraint.structure_matches, merge_landmarks,
      add_lc_edges, &num_inliers, &inlier_ratio, &T_G_I_ransac,
      inlier_structure_matches, &landmark_pairs_merged,
      vertex_id_closest_to_structure_matches, &map_mutex);

  statistics::StatsCollector stats_ransac_inliers(
      "LC AbsolutePoseRansacInliers");
  stats_ransac_inliers.AddSample(num_inliers);
  statistics::StatsCollector stats_ransac_inlier_ratio(
      "LC AbsolutePoseRansacInlierRatio");
  stats_ransac_inlier_ratio.AddSample(num_inliers);

  return ransac_ok;
}

void LoopDetectorNode::queryVertexInDatabase(
    const pose_graph::VertexId& query_vertex_id, const bool merge_landmarks,
    const bool add_lc_edges, vi_map::VIMap* map,
    vi_map::LoopClosureConstraint* raw_constraint,
    vi_map::LoopClosureConstraint* inlier_constraint,
    std::vector<double>* inlier_ratios,
    aslam::TransformationVector* T_G_M2_vector,
    loop_closure_handler::LoopClosureHandler::MergedLandmark3dPositionVector*
        landmark_pairs_merged,
    std::mutex* map_mutex) const {
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(raw_constraint);
  CHECK_NOTNULL(inlier_constraint);
  CHECK_NOTNULL(inlier_ratios);
  CHECK_NOTNULL(T_G_M2_vector);
  CHECK_NOTNULL(landmark_pairs_merged);
  CHECK_NOTNULL(map_mutex);
  CHECK(query_vertex_id.isValid());

  map_mutex->lock();
  const vi_map::Vertex& query_vertex = map->getVertex(query_vertex_id);
  const size_t num_frames = query_vertex.numFrames();
  loop_closure::ProjectedImagePtrList projected_image_ptr_list;
  projected_image_ptr_list.reserve(num_frames);

  for (size_t frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
    if (query_vertex.isVisualFrameSet(frame_idx) &&
        query_vertex.isVisualFrameValid(frame_idx)) {
      const aslam::VisualFrame& frame = query_vertex.getVisualFrame(frame_idx);
      CHECK(frame.hasKeypointMeasurements());
      if (frame.getNumKeypointMeasurements() == 0u) {
        // Skip frame if zero measurements found.
        continue;
      }

      std::vector<vi_map::LandmarkId> observed_landmark_ids;
      query_vertex.getFrameObservedLandmarkIds(frame_idx,
                                               &observed_landmark_ids);
      projected_image_ptr_list.push_back(
          std::make_shared<loop_closure::ProjectedImage>());
      const vi_map::VisualFrameIdentifier query_frame_id(
          query_vertex_id, frame_idx);
      constexpr bool kSkipInvalidLandmarkIds = false;
      convertFrameToProjectedImage(
          *map, query_frame_id, query_vertex.getVisualFrame(frame_idx),
          observed_landmark_ids, query_vertex.getMissionId(),
          kSkipInvalidLandmarkIds, projected_image_ptr_list.back().get());
    }
  }
  map_mutex->unlock();

  loop_closure::FrameToMatches frame_matches;
  // Do not parallelize if the current function is running in multiple
  // threads to avoid decrease in performance.
  constexpr bool kParallelFindIfPossible = false;
  loop_detector_->Find(
      projected_image_ptr_list, kParallelFindIfPossible, &frame_matches);

  if (!frame_matches.empty()) {
    for (const loop_closure::FrameIdMatchesPair& id_and_matches :
         frame_matches) {
      vi_map::LoopClosureConstraint tmp_constraint;
      const bool conversion_success =
          convertFrameMatchesToConstraint(id_and_matches, &tmp_constraint);
      if (!conversion_success) {
        continue;
      }
      raw_constraint->query_vertex_id = tmp_constraint.query_vertex_id;
      raw_constraint->structure_matches.insert(
          raw_constraint->structure_matches.end(),
          tmp_constraint.structure_matches.begin(),
          tmp_constraint.structure_matches.end());
    }

    int num_inliers = 0;
    double inlier_ratio = 0.0;

    // The estimated transformation of this vertex to the map.
    pose::Transformation T_G_I_ransac;
    constexpr pose_graph::VertexId* kVertexIdClosestToStructureMatches =
        nullptr;
    bool ransac_ok = handleLoopClosures(
        *raw_constraint, merge_landmarks, add_lc_edges, &num_inliers,
        &inlier_ratio, map, &T_G_I_ransac, inlier_constraint,
        landmark_pairs_merged, kVertexIdClosestToStructureMatches, map_mutex);

    if (ransac_ok && inlier_ratio != 0.0) {
      map_mutex->lock();
      const pose::Transformation& T_M_I = query_vertex.get_T_M_I();
      const pose::Transformation T_G_M2 = T_G_I_ransac * T_M_I.inverse();
      map_mutex->unlock();

      T_G_M2_vector->push_back(T_G_M2);
      inlier_ratios->push_back(inlier_ratio);
    }
  }
}

void LoopDetectorNode::detectLoopClosuresMissionToDatabase(
    const MissionId& mission_id, const bool merge_landmarks,
    const bool add_lc_edges, int* num_vertex_candidate_links,
    double* summary_landmark_match_inlier_ratio, vi_map::VIMap* map,
    pose::Transformation* T_G_M_estimate,
    vi_map::LoopClosureConstraintVector* inlier_constraints) const {
  CHECK(map->hasMission(mission_id));
  pose_graph::VertexIdList vertices;
  map->getAllVertexIdsInMission(mission_id, &vertices);
  detectLoopClosuresVerticesToDatabase(
      vertices, merge_landmarks, add_lc_edges, num_vertex_candidate_links,
      summary_landmark_match_inlier_ratio, map, T_G_M_estimate,
      inlier_constraints);
}

void LoopDetectorNode::detectLoopClosuresVerticesToDatabase(
    const pose_graph::VertexIdList& vertices, const bool merge_landmarks,
    const bool add_lc_edges, int* num_vertex_candidate_links,
    double* summary_landmark_match_inlier_ratio, vi_map::VIMap* map,
    pose::Transformation* T_G_M_estimate,
    vi_map::LoopClosureConstraintVector* inlier_constraints) const {
  CHECK(!vertices.empty());
  CHECK_NOTNULL(num_vertex_candidate_links);
  CHECK_NOTNULL(summary_landmark_match_inlier_ratio);
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(T_G_M_estimate)->setIdentity();
  CHECK_NOTNULL(inlier_constraints)->clear();

  *num_vertex_candidate_links = 0;
  *summary_landmark_match_inlier_ratio = 0.0;

  if (VLOG_IS_ON(1)) {
    std::ostringstream ss;
    for (const MissionId mission : missions_in_database_) {
      ss << mission << ", ";
    }
    VLOG(1) << "Searching for loop closures in missions " << ss.str();
  }

  std::vector<double> inlier_ratios;
  aslam::TransformationVector T_G_M_vector;

  std::mutex map_mutex;
  std::mutex output_mutex;

  loop_closure_handler::LoopClosureHandler::MergedLandmark3dPositionVector
      landmark_pairs_merged;
  vi_map::LoopClosureConstraintVector raw_constraints;

  // Then search for all in the database.
  common::MultiThreadedProgressBar progress_bar;

  std::function<void(const std::vector<size_t>&)> query_helper = [&](
      const std::vector<size_t>& range) {
    int num_processed = 0;
    progress_bar.setNumElements(range.size());
    for (const size_t job_index : range) {
      const pose_graph::VertexId& query_vertex_id = vertices[job_index];
      progress_bar.update(++num_processed);

      // Allocate local buffers to avoid locking.
      vi_map::LoopClosureConstraint raw_constraint_local;
      vi_map::LoopClosureConstraint inlier_constraint_local;
      using loop_closure_handler::LoopClosureHandler;
      LoopClosureHandler::MergedLandmark3dPositionVector
          landmark_pairs_merged_local;
      std::vector<double> inlier_ratios_local;
      aslam::TransformationVector T_G_M2_vector_local;

      // Perform the actual query.
      queryVertexInDatabase(
          query_vertex_id, merge_landmarks, add_lc_edges, map,
          &raw_constraint_local, &inlier_constraint_local, &inlier_ratios_local,
          &T_G_M2_vector_local, &landmark_pairs_merged_local, &map_mutex);

      // Lock the output buffers and transfer results.
      {
        std::unique_lock<std::mutex> lock_output(output_mutex);
        if (raw_constraint_local.query_vertex_id.isValid()) {
          raw_constraints.push_back(raw_constraint_local);
        }
        if (inlier_constraint_local.query_vertex_id.isValid()) {
          inlier_constraints->push_back(inlier_constraint_local);
        }

        landmark_pairs_merged.insert(
            landmark_pairs_merged.end(), landmark_pairs_merged_local.begin(),
            landmark_pairs_merged_local.end());
        inlier_ratios.insert(
            inlier_ratios.end(), inlier_ratios_local.begin(),
            inlier_ratios_local.end());
        T_G_M_vector.insert(
            T_G_M_vector.end(), T_G_M2_vector_local.begin(),
            T_G_M2_vector_local.end());
      }
    }
  };

  constexpr bool kAlwaysParallelize = true;
  const size_t num_threads = common::getNumHardwareThreads();

  timing::Timer timing_mission_lc("lc query mission");
  common::ParallelProcess(
      vertices.size(), query_helper, kAlwaysParallelize, num_threads);
  timing_mission_lc.Stop();

  VLOG(1) << "Searched " << vertices.size() << " frames.";

  // If the plotter object was assigned.
  if (visualizer_) {
    vi_map::MissionIdList missions(
        missions_in_database_.begin(), missions_in_database_.end());
    vi_map::MissionIdSet query_mission_set;
    map->getMissionIds(vertices, &query_mission_set);
    missions.insert(
        missions.end(), query_mission_set.begin(), query_mission_set.end());

    visualizer_->visualizeKeyframeToStructureMatches(
        *inlier_constraints, raw_constraints, landmark_id_old_to_new_, *map);
    visualizer_->visualizeMergedLandmarks(landmark_pairs_merged);
    visualizer_->visualizeFullMapDatabase(missions, *map);
  }

  if (inlier_ratios.empty()) {
    LOG(WARNING) << "No loop found!";
    *summary_landmark_match_inlier_ratio = 0;
  } else {
    // Compute the median inlier ratio:
    // nth_element is not used on purpose because this function will be used
    // only in offline scenarios. Additionally, we only sort once.
    std::sort(inlier_ratios.begin(), inlier_ratios.end());
    *summary_landmark_match_inlier_ratio =
        inlier_ratios[inlier_ratios.size() / 2];

    LOG(INFO) << "Successfully loopclosed " << inlier_ratios.size()
              << " vertices. Merged " << landmark_pairs_merged.size()
              << " landmark pairs.";

    VLOG(1) << "Median inlier ratio: " << *summary_landmark_match_inlier_ratio;

    if (VLOG_IS_ON(2)) {
      std::stringstream inlier_ss;
      inlier_ss << "Inlier ratios: ";
      for (double val : inlier_ratios) {
        inlier_ss << val << " ";
      }
      VLOG(2) << inlier_ss.str();
    }

    // RANSAC and LSQ estimate of the mission baseframe transformation.
    constexpr int kNumRansacIterations = 2000;
    constexpr double kPositionErrorThresholdMeters = 2;
    constexpr double kOrientationErrorThresholdRadians = 0.174;  // ~10 deg.
    constexpr double kInlierRatioThreshold = 0.2;
    const int kNumInliersThreshold =
        T_G_M_vector.size() * kInlierRatioThreshold;
    aslam::Transformation T_G_M_LS;
    int num_inliers = 0;
    std::random_device device;
    const int ransac_seed = device();

    common::transformationRansac(
        T_G_M_vector, kNumRansacIterations, kOrientationErrorThresholdRadians,
        kPositionErrorThresholdMeters, ransac_seed, &T_G_M_LS, &num_inliers);
    if (num_inliers < kNumInliersThreshold) {
      VLOG(1) << "Found loops rejected by RANSAC! (Inliers " << num_inliers
              << "/" << T_G_M_vector.size() << ")";
      *summary_landmark_match_inlier_ratio = 0;
      *num_vertex_candidate_links = inlier_ratios.size();
      return;
    }
    const Eigen::Quaterniond& q_G_M_LS =
        T_G_M_LS.getRotation().toImplementation();

    // The datasets should be gravity-aligned so only yaw-axis rotation is
    // necessary to prealign them.
    Eigen::Vector3d rpy_G_M_LS =
        common::RotationMatrixToRollPitchYaw(q_G_M_LS.toRotationMatrix());
    rpy_G_M_LS(0) = 0.0;
    rpy_G_M_LS(1) = 0.0;
    Eigen::Quaterniond q_G_M_LS_yaw_only(
        common::RollPitchYawToRotationMatrix(rpy_G_M_LS));

    T_G_M_LS.getRotation().toImplementation() = q_G_M_LS_yaw_only;
    *T_G_M_estimate = T_G_M_LS;
  }
  *num_vertex_candidate_links = inlier_ratios.size();
}

void LoopDetectorNode::detectLoopClosuresAndMergeLandmarks(
    const MissionId& mission, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  constexpr bool kMergeLandmarks = true;
  constexpr bool kAddLoopclosureEdges = false;
  int num_vertex_candidate_links;
  double summary_landmark_match_inlier_ratio;

  pose::Transformation T_G_M2;
  vi_map::LoopClosureConstraintVector inlier_constraints;
  detectLoopClosuresMissionToDatabase(
      mission, kMergeLandmarks, kAddLoopclosureEdges,
      &num_vertex_candidate_links, &summary_landmark_match_inlier_ratio, map,
      &T_G_M2, &inlier_constraints);

  VLOG(1) << "Handling loop closures done.";
}

bool LoopDetectorNode::handleLoopClosures(
    const vi_map::LoopClosureConstraint& constraint, const bool merge_landmarks,
    const bool add_lc_edges, int* num_inliers, double* inlier_ratio,
    vi_map::VIMap* map, pose::Transformation* T_G_I_ransac,
    vi_map::LoopClosureConstraint* inlier_constraints,
    loop_closure_handler::LoopClosureHandler::MergedLandmark3dPositionVector*
        landmark_pairs_merged,
    pose_graph::VertexId* vertex_id_closest_to_structure_matches,
    std::mutex* map_mutex) const {
  CHECK_NOTNULL(num_inliers);
  CHECK_NOTNULL(inlier_ratio);
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(T_G_I_ransac);
  CHECK_NOTNULL(inlier_constraints);
  CHECK_NOTNULL(landmark_pairs_merged);
  CHECK_NOTNULL(map_mutex);
  // Note: vertex_id_closest_to_structure_matches is optional and may beb NULL.
  loop_closure_handler::LoopClosureHandler handler(
      map, &landmark_id_old_to_new_);
  return handler.handleLoopClosure(
      constraint, merge_landmarks, add_lc_edges, num_inliers, inlier_ratio,
      T_G_I_ransac, inlier_constraints, landmark_pairs_merged,
      vertex_id_closest_to_structure_matches, map_mutex, use_random_pnp_seed_);
}

void LoopDetectorNode::instantiateVisualizer() {
  visualizer_.reset(new loop_closure_visualization::LoopClosureVisualizer());
}

void LoopDetectorNode::clear() {
  loop_detector_->Clear();
}

void LoopDetectorNode::serialize(
    proto::LoopDetectorNode* proto_loop_detector_node) const {
  CHECK_NOTNULL(proto_loop_detector_node);

  for (const vi_map::MissionId& mission : missions_in_database_) {
    mission.serialize(
        CHECK_NOTNULL(proto_loop_detector_node->add_mission_ids()));
  }

  loop_detector_->serialize(
      proto_loop_detector_node->mutable_matching_based_loop_detector());
}

void LoopDetectorNode::deserialize(
    const proto::LoopDetectorNode& proto_loop_detector_node) {
  const int num_missions = proto_loop_detector_node.mission_ids_size();
  VLOG(1) << "Parsing loop detector with " << num_missions << " missions.";
  for (int idx = 0; idx < num_missions; ++idx) {
    vi_map::MissionId mission_id;
    mission_id.deserialize(proto_loop_detector_node.mission_ids(idx));
    CHECK(mission_id.isValid());
    missions_in_database_.insert(mission_id);
  }

  CHECK(loop_detector_);
  loop_detector_->deserialize(
      proto_loop_detector_node.matching_based_loop_detector());
}

const std::string& LoopDetectorNode::getDefaultSerializationFilename() {
  return serialization_filename_;
}

}  // namespace loop_detector_node
