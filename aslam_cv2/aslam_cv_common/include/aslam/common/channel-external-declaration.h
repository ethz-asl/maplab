#ifndef ASLAM_CV_COMMON_CHANNEL_EXTERNAL_DECLARATIONS_H_
#define ASLAM_CV_COMMON_CHANNEL_EXTERNAL_DECLARATIONS_H_

#include <string>

#include <aslam/frames/visual-frame.h>

#define DECLARE_EXTERNAL_CHANNEL_HELPERS_IMPL(ChannelName, ChannelType)              \
  inline const ChannelType& get##ChannelName(const aslam::VisualFrame& frame) {      \
    return frame.getChannelData<ChannelType>(k##ChannelName);                        \
  }                                                                                  \
  inline void set##ChannelName(const ChannelType& data, aslam::VisualFrame* frame) { \
    CHECK_NOTNULL(frame);                                                            \
    frame->setChannelData<ChannelType>(k##ChannelName, data);                        \
  }                                                                                  \
  inline void add##ChannelName(aslam::VisualFrame* frame) {                          \
    CHECK_NOTNULL(frame);                                                            \
    frame->addChannel<ChannelType>(k##ChannelName);                                  \
  }

#define DECLARE_EXTERNAL_CHANNEL_SWAP_IMPL(ChannelName, ChannelType)                 \
  inline void swap##ChannelName(ChannelType* data, aslam::VisualFrame* frame) {      \
    CHECK_NOTNULL(frame);                                                            \
    frame->swapChannelData<ChannelType>(k##ChannelName, data);                       \
  }

#define DECLARE_EXTERNAL_CHANNEL_IMPL(ChannelName, ChannelType)                      \
  const std::string k##ChannelName = #ChannelName;                                   \
  typedef ChannelType ChannelName;

// Declare an additional channel for an aslam:VisualFrame. The underlying type does not provide
// a swap method.
#define DECLARE_EXTERNAL_CHANNEL(ChannelName, ChannelType)                           \
  DECLARE_EXTERNAL_CHANNEL_IMPL(ChannelName, ChannelType)                            \
  DECLARE_EXTERNAL_CHANNEL_HELPERS_IMPL(ChannelName, ChannelType)

// Declare an additional channel for an aslam:VisualFrame. The underlying type provides a method
// to swap data.
#define DECLARE_EXTERNAL_CHANNEL_WITH_SWAP(ChannelName, ChannelType)                 \
  DECLARE_EXTERNAL_CHANNEL_IMPL(ChannelName, ChannelType)                            \
  DECLARE_EXTERNAL_CHANNEL_HELPERS_IMPL(ChannelName, ChannelType)                    \
  DECLARE_EXTERNAL_CHANNEL_SWAP_IMPL(ChannelName, ChannelType)

// Example:
//  The macros below will implement the following functions in the calling scope.
//   -> const Eigen::VectorXi& getGroundTruthLandmarkIds(const aslam::VisualFrame& frame)
//   -> void setGroundTruthLandmarkIds(const Eigen::VectorXi& data, aslam::VisualFrame* frame)
//   -> void addGroundTruthLandmarkIds(aslam::VisualFrame* frame)
//   -> void swapGroundTruthLandmarkIds(Eigen::VectorXi* data, aslam::VisualFrame* frame)
//
//  namespace simulation {
//  namespace channels {
//    DECLARE_EXTERNAL_CHANNEL_WITH_SWAP(GroundTruthLandmarkIds, Eigen::VectorXi)
//  }
//  }

#endif  // ASLAM_CV_COMMON_CHANNEL_EXTERNAL_DECLARATIONS_H_
