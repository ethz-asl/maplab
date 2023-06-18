#ifndef ASLAM_VISUALIZATION_BASIC_VISUALIZATION_H_
#define ASLAM_VISUALIZATION_BASIC_VISUALIZATION_H_

#include <Eigen/Dense>
#include <aslam/common/memory.h>
#include <aslam/frames/feature-track.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <aslam/matcher/match.h>
#include <opencv2/core/core.hpp>
#include <vector>

namespace aslam_cv_visualization {

// Takes a frame and draws keypoints on it.
// Does not draw the raw image, but takes it from frame.
void drawKeypoints(
    const aslam::VisualFrame& frame, cv::Mat* image, int descriptor_type);

// Takes two frames and a list of matches between them and draws the matches.
// Does not draw the raw image, but takes it from frame_kp1.
void drawKeypointMatches(
    const aslam::VisualFrame& frame_kp1, const aslam::VisualFrame& frame_k,
    const aslam::FrameToFrameMatches& matches_kp1_k, int descriptor_type,
    cv::Mat* image);

}  // namespace aslam_cv_visualization

#endif  // ASLAM_VISUALIZATION_BASIC_VISUALIZATION_H_
