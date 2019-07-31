#ifndef VI_MAP_SEMANTIC_LANDMARK_STORE_H_
#define VI_MAP_SEMANTIC_LANDMARK_STORE_H_

#include <unordered_map>
#include <vector>

#include <aslam/common/memory.h>

#include "vi-map/semantic-landmark.h"
#include "vi-map/unique-id.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

class SemanticLandmarkStore {
 public:
  typedef Aligned<std::vector, SemanticLandmark> SemanticLandmarkVector;

  double* get_p_B_Mutable(const SemanticLandmarkId& landmark_id);

  SemanticLandmark& getSemanticLandmark(const SemanticLandmarkId& landmark_id);
  const SemanticLandmark& getSemanticLandmark(const SemanticLandmarkId& landmark_id) const;

  void addSemanticLandmark(const SemanticLandmark& landmark);
  void removeSemanticLandmark(const SemanticLandmarkId& landmark_id);
  bool hasSemanticLandmark(const SemanticLandmarkId& landmark_id) const;

  unsigned int size() const;

  void serialize(vi_map::proto::SemanticLandmarkStore* proto) const;
  void deserialize(const vi_map::proto::SemanticLandmarkStore& proto);

  inline SemanticLandmarkVector::iterator begin() {
    return landmarks_.begin();
  }

  inline SemanticLandmarkVector::iterator end() {
    return landmarks_.end();
  }

  inline SemanticLandmarkVector::reverse_iterator rbegin() {
    return landmarks_.rbegin();
  }

  inline SemanticLandmarkVector::reverse_iterator rend() {
    return landmarks_.rend();
  }

  inline SemanticLandmarkVector::const_iterator begin() const {
    return landmarks_.cbegin();
  }

  inline SemanticLandmarkVector::const_iterator end() const {
    return landmarks_.cend();
  }

  inline SemanticLandmarkVector::const_reverse_iterator rbegin() const {
    return landmarks_.crbegin();
  }

  inline SemanticLandmarkVector::const_reverse_iterator rend() const {
    return landmarks_.crend();
  }

  inline SemanticLandmark& operator[](const size_t index) {
    CHECK_LT(index, landmarks_.size());
    return landmarks_[index];
  }
  inline const SemanticLandmark& operator[](const size_t index) const {
    CHECK_LT(index, landmarks_.size());
    return landmarks_[index];
  }

  inline bool operator==(const SemanticLandmarkStore& lhs) const {
    bool is_same = true;
    is_same &= landmark_id_map_ == lhs.landmark_id_map_;
    is_same &= landmarks_ == lhs.landmarks_;
    return is_same;
  }
  inline bool operator!=(const SemanticLandmarkStore& lhs) const {
    return !operator==(lhs);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  typedef std::unordered_map<SemanticLandmarkId, int> SemanticLandmarkIdToIdxMap;

  SemanticLandmarkIdToIdxMap landmark_id_map_;
  SemanticLandmarkVector landmarks_;
};

}  // namespace vi_map

#endif  // VI_MAP_SEMANTIC_LANDMARK_STORE_H_
