#ifndef SIMULATION_VISUAL_NFRAME_SIMULATOR_CHANNELS_H_
#define SIMULATION_VISUAL_NFRAME_SIMULATOR_CHANNELS_H_

#include <string>

#include <Eigen/Core>
#include <aslam/common/channel-external-declaration.h>

namespace simulation {
namespace nframe_channels {
// Ground_truth_landmark_ids channel. The id corresponds to the index in the
// groundtruth landmark
// position matrix.
DECLARE_EXTERNAL_CHANNEL_WITH_SWAP(GroundTruthLandmarkIds, Eigen::VectorXi)

// Sequential_id channel. An id that is incremented for each successive frame.
DECLARE_EXTERNAL_CHANNEL(SequentialId, size_t)

}  // namespace nframe_channels
}  // namespace simulation

#endif  // SIMULATION_VISUAL_NFRAME_SIMULATOR_CHANNELS_H_
