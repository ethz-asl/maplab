#ifndef LOOPCLOSURE_COMMON_TYPES_H_
#define LOOPCLOSURE_COMMON_TYPES_H_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/feature-descriptor-ref.h>
#include <aslam/frames/visual-frame.h>
#include <posegraph/unique-id.h>
#include <vi-map/loop-constraint.h>

namespace loop_closure {
static const std::string kFeatureDescriptorBRISK = "brisk";
static const std::string kFeatureDescriptorFREAK = "freak";

// The Brisk implementation to which we refer is
// https://github.com/ethz-asl/ethzasl_brisk.
// It uses 384 bits instead of 512 (e.g. OpenCV implementation).
static constexpr int kBriskDescriptorLengthBits = 384;
static constexpr int kFreakDescriptorLengthBits = 512;

typedef aslam::VisualFrame::DescriptorsT DescriptorContainer;
typedef aslam::common::FeatureDescriptorRef DescriptorType;

typedef pose_graph::VertexId VertexId;
typedef vi_map::VisualFrameIdentifier KeyframeId;
typedef vi_map::KeypointIdentifier KeypointId;

typedef vi_map::FrameKeyPointToStructureMatch Match;
typedef std::vector<Match> MatchVector;

// Map from ID to associated matches.
template <typename IdType>
using IdToMatches = std::unordered_map<IdType, MatchVector>;

typedef IdToMatches<KeyframeId> FrameToMatches;
typedef FrameToMatches::value_type FrameIdMatchesPair;
typedef IdToMatches<VertexId> VertexToMatches;

// Map that contains the number of descriptors associated with each ID.
template <typename IdType>
using IdToNumDescriptors = std::unordered_map<IdType, size_t>;
}  // namespace loop_closure

#endif  // LOOPCLOSURE_COMMON_TYPES_H_
