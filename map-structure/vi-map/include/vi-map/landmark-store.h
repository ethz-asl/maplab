#ifndef VI_MAP_LANDMARK_STORE_H_
#define VI_MAP_LANDMARK_STORE_H_

#include <unordered_map>
#include <vector>

#include <aslam/common/memory.h>

#include "vi-map/landmark.h"
#include "vi-map/unique-id.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

class LandmarkStore {
 public:
  typedef Aligned<std::vector, Landmark> LandmarkVector;

  double* get_p_B_Mutable(const LandmarkId& landmark_id);

  Landmark& getLandmark(const LandmarkId& landmark_id);
  const Landmark& getLandmark(const LandmarkId& landmark_id) const;

  void addLandmark(const Landmark& landmark);
  void removeLandmark(const LandmarkId& landmark_id);
  bool hasLandmark(const LandmarkId& landmark_id) const;

  unsigned int size() const;

  void serialize(vi_map::proto::LandmarkStore* proto) const;
  void deserialize(const vi_map::proto::LandmarkStore& proto);

  inline LandmarkVector::iterator begin() {
    return landmarks_.begin();
  }

  inline LandmarkVector::iterator end() {
    return landmarks_.end();
  }

  inline LandmarkVector::reverse_iterator rbegin() {
    return landmarks_.rbegin();
  }

  inline LandmarkVector::reverse_iterator rend() {
    return landmarks_.rend();
  }

  inline LandmarkVector::const_iterator begin() const {
    return landmarks_.cbegin();
  }

  inline LandmarkVector::const_iterator end() const {
    return landmarks_.cend();
  }

  inline LandmarkVector::const_reverse_iterator rbegin() const {
    return landmarks_.crbegin();
  }

  inline LandmarkVector::const_reverse_iterator rend() const {
    return landmarks_.crend();
  }

  inline Landmark& operator[](const size_t index) {
    CHECK_LT(index, landmarks_.size());
    return landmarks_[index];
  }
  inline const Landmark& operator[](const size_t index) const {
    CHECK_LT(index, landmarks_.size());
    return landmarks_[index];
  }

  inline bool operator==(const LandmarkStore& lhs) const {
    bool is_same = true;
    is_same &= landmark_id_map_ == lhs.landmark_id_map_;
    is_same &= landmarks_ == lhs.landmarks_;
    return is_same;
  }
  inline bool operator!=(const LandmarkStore& lhs) const {
    return !operator==(lhs);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  typedef std::unordered_map<LandmarkId, int> LandmarkIdToIdxMap;

  LandmarkIdToIdxMap landmark_id_map_;
  LandmarkVector landmarks_;
};

}  // namespace vi_map

#endif  // VI_MAP_LANDMARK_STORE_H_
