#ifndef VI_MAP_LANDMARK_H_
#define VI_MAP_LANDMARK_H_

#include <unordered_set>
#include <utility>
#include <vector>

#include <aslam/common/memory.h>
#include <maplab-common/macros.h>
#include <maplab-common/pose_types.h>
#include <posegraph/vertex.h>

#include "vi-map/unique-id.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

class Landmark {
 public:
  MAPLAB_POINTER_TYPEDEFS(Landmark);

  enum class Quality {
    kUnknown = 0,
    kBad = 1,
    kGood = 2,
    kLocalizationSummaryLandmark = 3
  };

  Landmark()
      : quality_(Quality::kUnknown), B_position_(Eigen::Vector3d::Zero()) {}

  Landmark(const Landmark& lhs) {
    *this = lhs;
  }

  inline Landmark& operator=(const Landmark& lhs) {
    id_ = lhs.id_;
    observations_ = lhs.observations_;
    quality_ = lhs.quality_;
    B_position_ = lhs.B_position_;
    appearances_ = lhs.appearances_;

    // Clone covariance if set.
    if (lhs.B_covariance_ != nullptr) {
      B_covariance_ = aligned_unique<Eigen::Matrix3d>(*lhs.B_covariance_);
    }
    return *this;
  }

  inline const LandmarkId& id() const {
    return id_;
  }

  inline void setId(const LandmarkId& id) {
    id_ = id;
  }

  inline const pose::Position3D& get_p_B() const {
    return B_position_;
  }

  inline void set_p_B(const pose::Position3D& p_B) {
    B_position_ = p_B;
  }

  inline bool get_p_B_Covariance(Eigen::Matrix3d* covariance) const {
    CHECK_NOTNULL(covariance);
    if (B_covariance_ == nullptr) {
      covariance->setZero();
      return false;
    }
    *covariance = *B_covariance_;
    return true;
  }

  inline void set_p_B_Covariance(const Eigen::Matrix3d& covariance) {
    if (B_covariance_ == nullptr) {
      B_covariance_ = aligned_unique<Eigen::Matrix3d>();
    }
    *B_covariance_ = covariance;
  }

  inline void unsetCovariance() {
    B_covariance_.reset();
  }

  inline void setQuality(Quality quality) {
    quality_ = quality;
  }
  inline Quality getQuality() const {
    return quality_;
  }

  void addObservation(
      const pose_graph::VertexId& vertex_id, unsigned int frame_idx,
      unsigned int keypoint_index);

  void addObservations(const KeypointIdentifierList& new_observations);
  void addObservation(const KeypointIdentifier& keypoint_id);

  bool hasObservation(
      const pose_graph::VertexId& vertex_id, size_t frame_idx,
      size_t keypoint_index) const;
  bool hasObservation(const KeypointIdentifier& keypoint_id) const;

  void removeAllObservationsAccordingToPredicate(
      const std::function<bool(const KeypointIdentifier&)>&  // NOLINT
      predicate);

  void removeAllObservationsOfVertex(const pose_graph::VertexId& vertex_id);

  void removeAllObservationsOfVertexAndFrame(
      const pose_graph::VertexId& vertex_id, unsigned int frame_idx);

  void removeObservation(const KeypointIdentifier& observation);

  inline void removeObservation(size_t index) {
    CHECK_LT(index, observations_.size());
    if (!appearances_.empty()) {
      CHECK_EQ(appearances_.size(), observations_.size())
          << "The appearances "
          << "of landmark with store id " << id_.hexString() << " are not in "
          << "sync with the observations as their respective number of "
             "elements "
          << "differs.";
      appearances_.erase(appearances_.begin() + index);
    }
    observations_.erase(observations_.begin() + index);
  }

  unsigned int numberOfObserverVertices() const;

  double* get_p_B_Mutable();
  double* get_p_B_CovarianceMutable();

  const KeypointIdentifierList& getObservations() const;

  unsigned int numberOfObservations() const;

  bool hasObservations() const;

  void clearObservations();

  void forEachObservation(
      const std::function<void(const KeypointIdentifier&)>& action) const;

  void forEachObservation(
      const std::function<void(const KeypointIdentifier&, const size_t)>&
          action) const;

  // Returns the appearance for a given observation index. CHECK-fails, if
  // no appearance have been allocated for the given observation index.
  int getAppearanceForObservationIndex(size_t observation_index) const;

  // Set the appearance for a given observation index. Allocates the appearance,
  // if it has not been allocated yet.
  void setAppearance(size_t observation_index, int appearance);

  // Returns a set of all distinct appearances existing for this landmark.
  void getAllDistinctAppearances(
      std::unordered_set<int>* distinct_appearances) const;

  // Returns all observations of this landmark with a given appearance.
  void getAllObservationsOfAppearance(
      int appearance, KeypointIdentifierList* observations) const;

  // Allocate invalid appearances (-1) for all observations.
  void allocateAppearances();

  // Allows checking, if appearances have been allocated or not.
  bool areAppearancesAllocated() const {
    return !appearances_.empty();
  }

  // Returns the appearances vector.
  const std::vector<int>& getAppearances() const;

  static constexpr int kInvalidAppearance = -1;
  static constexpr int kDefaultAppearance = 0;

  void serialize(vi_map::proto::Landmark* proto) const;
  void deserialize(const vi_map::proto::Landmark& proto);

  inline bool operator==(const Landmark& lhs) const {
    bool is_same = true;
    is_same &= quality_ == lhs.quality_;
    is_same &= B_position_ == lhs.B_position_;

    if (B_covariance_.get() != nullptr && lhs.B_covariance_.get() != nullptr) {
      is_same &= (*B_covariance_ == *lhs.B_covariance_);
    } else {
      is_same &=
          B_covariance_.get() == nullptr && lhs.B_covariance_.get() == nullptr;
    }
    return is_same;
  }
  inline bool operator!=(const Landmark& lhs) const {
    return !operator==(lhs);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  LandmarkId id_;
  KeypointIdentifierList observations_;
  Quality quality_;

  // Appearance vector, with one appearance per observation.
  std::vector<int> appearances_;

  // Position and covariance w.r.t. landmark baseframe. The covariance is
  // optional to reduce the memory usage.
  pose::Position3D B_position_;
  AlignedUniquePtr<Eigen::Matrix3d> B_covariance_;
};
}  // namespace vi_map

#endif  // VI_MAP_LANDMARK_H_
