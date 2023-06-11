#ifndef ASLAM_MATCH_H_
#define ASLAM_MATCH_H_

#include <Eigen/Core>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <utility>
#include <vector>

namespace aslam {

/// \brief A struct to encapsulate a match between two lists.
///        The matches are indices into these lists.
struct FrameToFrameMatch : std::pair<int, int> {
  FrameToFrameMatch() = default;
  FrameToFrameMatch(int first_, int second_)
      : std::pair<int, int>(first_, second_) {}
  virtual ~FrameToFrameMatch() = default;
  int getKeypointIndexInFrameA() const {
    return first;
  }
  void setKeypointIndexInFrameA(int first_) {
    first = first_;
  }
  int getKeypointIndexInFrameB() const {
    return second;
  }
  void setKeypointIndexInFrameB(int second_) {
    second = second_;
  }
};

typedef Aligned<std::vector, FrameToFrameMatch> FrameToFrameMatches;
typedef Aligned<std::vector, FrameToFrameMatches> FrameToFrameMatchesList;

}  // namespace aslam

#endif  // ASLAM_MATCH_H_
