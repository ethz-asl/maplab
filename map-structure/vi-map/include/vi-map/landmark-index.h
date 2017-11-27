#ifndef VI_MAP_LANDMARK_INDEX_H_
#define VI_MAP_LANDMARK_INDEX_H_

#include <algorithm>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest_prod.h>
#include <maplab-common/accessors.h>
#include <vi-map/unique-id.h>

class LoopClosureHandlerTest;

namespace map_optimization_legacy {
class ViwlsGraph;
class SixDofVIMapGenerator;
}  // namespace map_optimization_legacy

namespace vi_map {
class VIMap;

typedef std::unordered_map<LandmarkId, pose_graph::VertexId>
    LandmarkToVertexMap;

class LandmarkIndex {
  friend VIMap;
  friend ::LoopClosureHandlerTest;                     // Test.
  friend class MapConsistencyCheckTest;                // Test.
  friend class map_optimization_legacy::ViwlsGraph;    // Test.
  friend class map_optimization_legacy::SixDofVIMapGenerator;  // Test.

  LandmarkIndex() {}

  void shallowCopyFrom(const LandmarkIndex& other) {
    std::lock_guard<std::mutex> lock(access_mutex_);
    index_ = other.index_;
  }

  void swap(LandmarkIndex* other) {
    std::lock_guard<std::mutex> lock(access_mutex_);
    index_.swap(other->index_);
  }

  inline pose_graph::VertexId getStoringVertexId(
      const LandmarkId& landmark_id) const {
    CHECK(landmark_id.isValid()) << "The landark is is not valid.";
    std::lock_guard<std::mutex> lock(access_mutex_);
    LandmarkToVertexMap::const_iterator it = index_.find(landmark_id);
    CHECK(it != index_.end()) << "Landmark " << landmark_id << " is not "
                              << "present in the landmark index.";
    return it->second;
  }

  inline void addLandmarkAndVertexReference(
      const vi_map::LandmarkId& landmark_id,
      const pose_graph::VertexId& vertex_id) {
    CHECK(landmark_id.isValid());
    std::lock_guard<std::mutex> lock(access_mutex_);
    CHECK(!hasLandmarkInternal(landmark_id)) << "Landmark " << landmark_id
                                             << " is already in the index!";
    index_.emplace(landmark_id, vertex_id);
  }

  inline void getAllLandmarkIds(
      std::unordered_set<LandmarkId>* landmark_ids) const {
    CHECK_NOTNULL(landmark_ids)->clear();
    std::lock_guard<std::mutex> lock(access_mutex_);
    for (const LandmarkToVertexMap::value_type& item : index_) {
      landmark_ids->emplace(item.first);
    }
  }

  inline void getAllLandmarkIds(
      std::vector<LandmarkId>* landmark_ids) const {
    CHECK_NOTNULL(landmark_ids)->clear();
    landmark_ids->resize(index_.size());

    std::lock_guard<std::mutex> lock(access_mutex_);
    size_t idx = 0u;
    for (const LandmarkToVertexMap::value_type& item : index_) {
      (*landmark_ids)[idx] = item.first;
      ++idx;
    }
  }

  inline size_t numLandmarks() const {
    std::lock_guard<std::mutex> lock(access_mutex_);
    return index_.size();
  }

  inline bool hasLandmark(const LandmarkId& landmark_id) const {
    std::lock_guard<std::mutex> lock(access_mutex_);
    return index_.count(landmark_id) > 0u;
  }

  inline void updateVertexOfLandmark(
      const LandmarkId& landmark_id,
      const pose_graph::VertexId& vertex_id) {
    std::lock_guard<std::mutex> lock(access_mutex_);
    LandmarkToVertexMap::iterator it = index_.find(landmark_id);
    CHECK(it != index_.end());
    it->second = vertex_id;
  }

  inline void removeLandmark(
      const LandmarkId& landmark_id) {
    std::lock_guard<std::mutex> lock(access_mutex_);
    CHECK(hasLandmarkInternal(landmark_id)) << "Tried to remove a landmark "
        << "that does not exist!";
    index_.erase(landmark_id);
  }

  void setLandmarkToVertexMap(
      const LandmarkToVertexMap& landmark_to_vertex) {
    std::lock_guard<std::mutex> lock(access_mutex_);
    index_ = landmark_to_vertex;
  }

  inline void clear() {
    index_.clear();
  }

 private:
  inline bool hasLandmarkInternal(const LandmarkId& landmark_id) const {
    return index_.count(landmark_id) > 0u;
  }

  LandmarkToVertexMap index_;
  mutable std::mutex access_mutex_;
};

}  // namespace vi_map

#endif  // VI_MAP_LANDMARK_INDEX_H_
