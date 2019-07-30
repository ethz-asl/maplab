#ifndef VI_MAP_SEMANTIC_LANDMARK_H_
#define VI_MAP_SEMANTIC_LANDMARK_H_

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

// todo(jkuo): can refactor a landmark base class for landmark and emantic landmark
// common parts like pose, covariance, id ,and their related access functions
class SemanticLandmark {
 public:
  
  MAPLAB_POINTER_TYPEDEFS(SemanticLandmark);

  enum class Quality {
    kUnknown = 0,
    kBad = 1,
    kGood = 2,
    kLocalizationSummaryLandmark = 3
  };

  SemanticLandmark()
      : quality_(Quality::kUnknown),
        B_position_(Eigen::Vector3d::Zero()),
        class_id_(-1) {}

  SemanticLandmark(const SemanticLandmark& lhs) {
    *this = lhs;
  }

  inline SemanticLandmark& operator=(const SemanticLandmark& lhs) {
    if (this != &lhs) {
      id_ = lhs.id_;
      class_id_ = lhs.class_id_;
      observations_ = lhs.observations_;
      quality_ = lhs.quality_;
      B_position_ = lhs.B_position_;
      // check covariance because it does not necessarily exist
      // Clone covariance if set.
      if (lhs.B_covariance_ != nullptr) {
        B_covariance_ = aligned_unique<Eigen::Matrix3d>(*lhs.B_covariance_);
      }
    }
    return *this;
  }

  inline const SemanticLandmarkId& id() const {
    return id_;
  }

  inline void setId(const SemanticLandmarkId& id) {
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

  inline void setClassId(int class_id) {
    class_id_ = class_id;
  }
  inline int getClassId() const {
    return class_id_;
  }

  void addObservation(
      const pose_graph::VertexId& vertex_id, unsigned int frame_idx,
      unsigned int measurement_index);

  void addObservations(const SemanticObjectIdentifierList& new_observations);
  void addObservation(const SemanticObjectIdentifier& new_observation);

  bool hasObservation(
      const pose_graph::VertexId& vertex_id, size_t frame_idx,
      size_t measurement_index) const;
  bool hasObservation(const SemanticObjectIdentifier& identifier) const;

  void removeAllObservationsAccordingToPredicate(
      const std::function<bool(const SemanticObjectIdentifier&)>&  // NOLINT
      predicate);

  void removeAllObservationsOfVertex(const pose_graph::VertexId& vertex_id);

  void removeAllObservationsOfVertexAndFrame(
      const pose_graph::VertexId& vertex_id, unsigned int frame_idx);

  void removeObservation(const SemanticObjectIdentifier& observation);

  inline void removeObservation(size_t index) {
    CHECK_LT(index, observations_.size());
    observations_.erase(observations_.begin() + index);
  }

  unsigned int numberOfObserverVertices() const;

  double* get_p_B_Mutable();
  double* get_p_B_CovarianceMutable();

  const SemanticObjectIdentifierList& getObservations() const;

  unsigned int numberOfObservations() const;

  bool hasObservations() const;

  void clearObservations();

  void forEachObservation(
      const std::function<void(const SemanticObjectIdentifier&)>& action) const;

  void forEachObservation(
      const std::function<void(const SemanticObjectIdentifier&, const size_t)>&
          action) const;

  void serialize(vi_map::proto::SemanticLandmark* proto) const;
  void deserialize(const vi_map::proto::SemanticLandmark& proto);

  inline bool operator==(const SemanticLandmark& lhs) const {
    bool is_same = true;
    is_same &= quality_ == lhs.quality_;
    is_same &= B_position_ == lhs.B_position_;
    is_same &= class_id_ == lhs.class_id_;

    if (B_covariance_.get() != nullptr && lhs.B_covariance_.get() != nullptr) {
      is_same &= (*B_covariance_ == *lhs.B_covariance_);
    } else {
      is_same &=
          B_covariance_.get() == nullptr && lhs.B_covariance_.get() == nullptr;
    }
    return is_same;
  }
  inline bool operator!=(const SemanticLandmark& lhs) const {
    return !operator==(lhs);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  SemanticLandmarkId id_;
  SemanticObjectIdentifierList observations_;
  Quality quality_;
  int class_id_; // -1 is used for unknown

  // Position and covariance w.r.t. landmark baseframe. The covariance is
  // optional to reduce the memory usage.
  pose::Position3D B_position_;
  AlignedUniquePtr<Eigen::Matrix3d> B_covariance_;
};

}  // namespace vi_map

#endif  // VI_MAP_SEMANTIC_LANDMARK_H_