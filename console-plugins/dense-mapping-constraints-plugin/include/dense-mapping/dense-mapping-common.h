#ifndef DENSE_MAPPING_DENSE_MAPPING_COMMON_H_
#define DENSE_MAPPING_DENSE_MAPPING_COMMON_H_

#include <unordered_set>
#include <vector>

#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/unique-id.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vi-map/vi-map.h>

namespace dense_mapping {

typedef Eigen::Matrix<double, 3, Eigen::Dynamic> PositionMatrix;
typedef Eigen::Matrix<int64_t, 1, Eigen::Dynamic> TimestampNsVector;

static std::unordered_set<backend::ResourceType, backend::ResourceTypeHash>
    kSupportedResourceTypes{
        backend::ResourceType::kPointCloudXYZ,
        backend::ResourceType::kPointCloudXYZI,
        backend::ResourceType::kPointCloudXYZRGBN,
        backend::ResourceType::kPointCloudXYZL
    };

struct AlignmentCandidate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  vi_map::MissionId mission_id;
  aslam::SensorId sensor_id;
  int64_t timestamp_ns;
  backend::ResourceType resource_type;
  aslam::Transformation T_G_S_resource;

  pose_graph::VertexId closest_vertex_id;
  aslam::Transformation T_G_B_closest_vertex;

  bool isValid(const bool verbose = false) const;

  // Check if these two candidates originate from the same dense data. This does
  // not check the closest vertex transformation.
  bool isSameCandidate(
      const AlignmentCandidate& rhs, const bool verbose = false) const;
  bool operator==(const AlignmentCandidate& rhs) const;

  // Convenience function for logging.
  friend std::ostream& operator<<(
      std::ostream& out, const AlignmentCandidate& candidate);
};

struct AlignmentCandidatePair {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AlignmentCandidate candidate_A;
  AlignmentCandidate candidate_B;

  // Initial guess for the relative transformation between these two sensor
  // frames. This can be either obtained through their relative position in the
  // pose graph or an algorithm for an intial rough alignment.
  aslam::Transformation T_SB_SA_init;

  // Final relative transformation between these two sensor frames after
  // alignment.
  bool success;
  aslam::Transformation T_SB_SA_final;
  aslam::TransformationCovariance T_SB_SA_final_covariance;

  bool isValid(const bool verbose = false) const;

  // Check if this candidate pair corresponds to the same underlying data. This
  // does not check if their initial alignment or computed alignment is the
  // same.
  bool isSameCandidatePair(
      const AlignmentCandidatePair& rhs, const bool verbose = false) const;
  bool operator==(const AlignmentCandidatePair& rhs) const;

  // Convenience function for logging.
  friend std::ostream& operator<<(
      std::ostream& out, const AlignmentCandidatePair& pair);
};

}  // namespace dense_mapping

namespace std {

template <>
struct hash<dense_mapping::AlignmentCandidate> {
  std::size_t operator()(
      const dense_mapping::AlignmentCandidate& candidate) const {
    // Fits into two bytes for sure.
    const uint64_t resource_type_number =
        static_cast<uint64_t>(candidate.resource_type);

    // Fits into 4 bytes for sure.
    const uint64_t uint_timestamp_ns =
        static_cast<uint64_t>(candidate.timestamp_ns);

    // Fill the rest with hashes from the vertex and sensor id.
    return (0xFFFFFFFFFFF00000 &
            (std::hash<pose_graph::VertexId>()(candidate.closest_vertex_id) ^
             std::hash<aslam::SensorId>()(candidate.sensor_id))) |
           (0x00000000000FFF00 & (uint_timestamp_ns << 16)) |
           (0x00000000000000FF & resource_type_number);
  }
};

template <>
struct hash<dense_mapping::AlignmentCandidatePair> {
  std::size_t operator()(
      const dense_mapping::AlignmentCandidatePair& pair) const {
    return std::hash<dense_mapping::AlignmentCandidate>()(pair.candidate_A) ^
           std::hash<dense_mapping::AlignmentCandidate>()(pair.candidate_B);
  }
};
}  // namespace std

namespace dense_mapping {

typedef AlignedUnorderedSet<AlignmentCandidatePair> AlignmentCandidatePairs;
typedef Aligned<std::vector, AlignmentCandidate> AlignmentCandidateList;
typedef AlignedUnorderedMap<vi_map::MissionId, AlignmentCandidateList>
    MissionToAlignmentCandidatesMap;

}  // namespace dense_mapping

#endif  // DENSE_MAPPING_DENSE_MAPPING_COMMON_H_
