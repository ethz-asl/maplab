#include "dense-mapping/dense-mapping-common.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace dense_mapping {

bool isSupportedResourceType(const backend::ResourceType resource_type) {
  return kSupportedResourceTypes.count(resource_type) > 0u;
}

std::ostream& operator<<(
    std::ostream& out, const AlignmentCandidate& candidate) {
  out << " mission: " << candidate.mission_id
      << " sensor: " << candidate.sensor_id << "\n";
  out << " resource timestamp [ns]: " << candidate.timestamp_ns << "\n";
  const int resource_type_idx = static_cast<int>(candidate.resource_type);
  out << " resource type: " << backend::ResourceTypeNames[resource_type_idx]
      << "(" << resource_type_idx << ")\n";
  out << " T_G_S_resource:\n" << candidate.T_G_S_resource << "\n";
  out << " closest vertex: " << candidate.closest_vertex_id << "\n";
  out << " T_G_B_closest_vertex:\n" << candidate.T_G_B_closest_vertex << "\n";
  return out;
}

std::ostream& operator<<(
    std::ostream& out, const AlignmentCandidatePair& pair) {
  out << "\n================== Alignment Candidate Pair +================\n";
  out << " ### A ###\n" << pair.candidate_A;
  out << " ### B ###\n" << pair.candidate_B;
  out << "\n";
  out << " success: " << std::boolalpha << pair.success;
  out << " T_SB_SA_init:\n" << pair.T_SB_SA_init << "\n";
  out << " T_SB_SA:\n" << pair.T_SB_SA_final << "\n";
  out << " T_SB_SA_covariance:\n" << pair.T_SB_SA_final_covariance << "\n";
  out << "\n=============================================================";
  return out;
}

bool AlignmentCandidate::isValid(const bool verbose) const {
  bool is_valid = true;

  is_valid &= mission_id.isValid();
  is_valid &= sensor_id.isValid();
  is_valid &= closest_vertex_id.isValid();
  is_valid &= timestamp_ns >= 0;
  is_valid &= isSupportedResourceType(resource_type);

  if (verbose) {
    LOG(ERROR) << "Invalid AlignmentCandidate: " << *this;
  }

  return is_valid;
}

bool AlignmentCandidatePair::isValid(const bool verbose) const {
  bool is_valid = true;

  is_valid &= candidate_A.isValid();
  is_valid &= candidate_B.isValid();
  is_valid &= candidate_A.closest_vertex_id != candidate_B.closest_vertex_id;

  if (verbose) {
    LOG(ERROR) << "Invalid AlignmentCandidatePair: " << *this;
  }

  return is_valid;
}

bool AlignmentCandidate::isSameCandidate(
    const AlignmentCandidate& rhs, const bool verbose) const {
  bool is_same = true;

  is_same &= mission_id == rhs.mission_id;
  is_same &= sensor_id == rhs.sensor_id;
  is_same &= closest_vertex_id == rhs.closest_vertex_id;
  is_same &= timestamp_ns == rhs.timestamp_ns;
  is_same &= resource_type == rhs.resource_type;

  if (verbose) {
    LOG(ERROR) << "AlignmentCandidates are not the same!\n"
               << "LHS" << *this << "\nRHS" << rhs;
  }
  return is_same;
}

bool AlignmentCandidate::operator==(const AlignmentCandidate& rhs) const {
  return isSameCandidate(rhs);
}

bool AlignmentCandidatePair::isSameCandidatePair(
    const AlignmentCandidatePair& rhs, const bool verbose) const {
  bool is_same = false;

  // Check both directions.
  is_same |=
      (candidate_A.isSameCandidate(rhs.candidate_A) &&
       candidate_B.isSameCandidate(rhs.candidate_B));
  is_same |=
      (candidate_A.isSameCandidate(rhs.candidate_B) &&
       candidate_B.isSameCandidate(rhs.candidate_A));

  if (verbose) {
    LOG(ERROR) << "AlignmentCandidates are not the same!\n"
               << "LHS" << *this << "\nRHS" << rhs;
  }
  return is_same;
}

bool AlignmentCandidatePair::operator==(
    const AlignmentCandidatePair& rhs) const {
  return isSameCandidatePair(rhs);
}

}  // namespace dense_mapping
