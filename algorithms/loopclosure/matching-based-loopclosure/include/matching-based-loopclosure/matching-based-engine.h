#ifndef MATCHING_BASED_LOOPCLOSURE_MATCHING_BASED_ENGINE_H_
#define MATCHING_BASED_LOOPCLOSURE_MATCHING_BASED_ENGINE_H_

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <aslam/common/reader-writer-lock.h>
#include <descriptor-projection/descriptor-projection.h>

#include "matching-based-loopclosure/detector-settings.h"
#include "matching-based-loopclosure/index-interface.h"
#include "matching-based-loopclosure/loop-detector-interface.h"
#include "matching-based-loopclosure/matching_based_loop_detector.pb.h"
#include "matching-based-loopclosure/scoring.h"

namespace matching_based_loopclosure {

class MatchingBasedLoopDetector : public loop_detector::LoopDetector {
 public:
  explicit MatchingBasedLoopDetector(
      const MatchingBasedEngineSettings& settings);

  virtual ~MatchingBasedLoopDetector() = default;

  // Find a set of provided images (consisting of projected descriptors), that
  // belong to the same vertex, in the database.
  void Find(
      const loop_closure::ProjectedImagePtrList& projected_image_ptr_list,
      const bool parallelize_if_possible,
      loop_closure::FrameToMatches* frame_matches) const override;

  // Add the provided image (consisting of projected descriptors) to the
  // descriptor index backend.
  void Insert(
      const loop_closure::ProjectedImage::Ptr& projected_image_ptr) override;

  // Transforms an image into a set of projected descriptors.
  void ProjectDescriptors(
      const loop_closure::DescriptorContainer& descriptors,
      Eigen::MatrixXf* projected_descriptors) const override;

  // Transforms an image into a set of projected descriptors.
  void ProjectDescriptors(
      const std::vector<aslam::common::FeatureDescriptorConstRef>& descriptors,
      Eigen::MatrixXf* projected_descriptors) const override;

  void Clear() override;

  void serialize(proto::MatchingBasedLoopDetector* matching_based_loop_detector)
      const override;
  void deserialize(
      const proto::MatchingBasedLoopDetector& matching_based_loop_detector)
      override;

 private:
  typedef std::unordered_map<loop_closure::KeyframeId,
                             loop_closure::ProjectedImage::Ptr>
      Database;
  typedef loop_closure::IdToNumDescriptors<loop_closure::KeyframeId>
      KeyframeIdToNumDescriptorsMap;
  typedef int DescriptorIndex;
  typedef std::unordered_map<DescriptorIndex, loop_closure::KeypointId>
      DescriptorIndexToKeypointIdMap;

  typedef loop_closure::IdToMatches<loop_closure::KeyframeId>
      KeyframeToMatchesMap;
  typedef loop_closure::IdToMatches<loop_closure::VertexId> VertexToMatchesMap;

  template <typename IdType>
  using IdToScoreMap = std::unordered_map<IdType, scoring::ScoreType>;

  void setKeyframeScoringFunction();
  void setDetectorEngine();

  size_t NumEntries() const override {
    return database_.size();
  }

  int NumDescriptors() const override {
    return index_interface_->GetNumDescriptorsInIndex();
  }

  // Find the largest connected subgraph of keyframes or vertices and landmarks
  // to be passed to RANSAC. This is just a plain BFS over landmarks and
  // keyframes or vertices. This function adds matches to the already existing
  // matches. There is an option to pass a mutex that is used to lock the
  // (output) frame matches.
  template <typename IdType>
  void doCovisibilityFiltering(
      const loop_closure::IdToMatches<IdType>& id_to_matches,
      const bool make_matches_unique,
      loop_closure::FrameToMatches* frame_matches,
      std::mutex* frame_matches_mutex = nullptr) const;
  template <typename IdType>
  void computeRelevantIdsForFiltering(
      const loop_closure::IdToMatches<IdType>& frame_to_matches,
      IdToScoreMap<IdType>* frame_to_score_map) const;
  // Skip match if not in the set of keyframes that see a lot of the matched
  // landmarks.
  template <typename IdType>
  bool skipMatch(
      const IdToScoreMap<IdType>& frame_to_score_map,
      const loop_closure::Match& match) const;
  template <typename IdType>
  typename loop_closure::IdToMatches<IdType>::const_iterator
  getIteratorForMatch(
      const loop_closure::IdToMatches<IdType>& frame_to_matches,
      const loop_closure::Match& match) const;

  // Returns true if the match has been successfully retrieved. Returns false,
  // if the match was too close in time to the query vertex.
  bool getMatchForDescriptorIndex(
      int nn_match_descriptor_index,
      const loop_closure::ProjectedImage& projected_image_query,
      int keypoint_index_query, loop_closure::Match* structure_match) const;
  int getNumNeighborsToSearch() const;

  const MatchingBasedEngineSettings settings_;
  Database database_;
  KeyframeIdToNumDescriptorsMap keyframe_id_to_num_descriptors_;
  DescriptorIndexToKeypointIdMap descriptor_index_to_keypoint_id_;
  int descriptor_index_;
  std::shared_ptr<loop_closure::IndexInterface> index_interface_;
  scoring::computeScoresFunction<loop_closure::KeyframeId>
      compute_keyframe_scores_;
  mutable aslam::ReaderWriterMutex read_write_mutex;
};
}  // namespace matching_based_loopclosure

#include "./matching-based-engine-inl.h"

#endif  // MATCHING_BASED_LOOPCLOSURE_MATCHING_BASED_ENGINE_H_
