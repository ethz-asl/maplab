#ifndef ASLAM_CV_COMMON_CHANNEL_DEFINITIONS_H_
#define ASLAM_CV_COMMON_CHANNEL_DEFINITIONS_H_

#include <Eigen/Dense>
#include <aslam/common/channel-declaration.h>

/// Coordinates of the raw keypoints. (keypoint detector output)
/// (cols are keypoints)
DECLARE_CHANNEL(VISUAL_KEYPOINT_MEASUREMENTS, Eigen::Matrix2Xd)

/// Keypoint coordinate uncertainties of the raw keypoints. (keypoint detector output)
/// (cols are uncertainties)
DECLARE_CHANNEL(VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES, Eigen::VectorXd)

/// Keypoint orientation from keypoint extractor. (keypoint detector output)
/// Computed orientation of the keypoint (-1 if not applicable);
/// it's in [0,360) degrees and measured relative to image coordinate system, ie in clockwise.
DECLARE_CHANNEL(VISUAL_KEYPOINT_ORIENTATIONS, Eigen::VectorXd)

/// Diameter of the meaningful keypoint neighborhood. (keypoint detector output)
DECLARE_CHANNEL(VISUAL_KEYPOINT_SCALES, Eigen::VectorXd)

/// The score by which the most strong keypoints have been selected. Can be used for the further
/// sorting or subsampling. (keypoint detector output)
DECLARE_CHANNEL(VISUAL_KEYPOINT_SCORES, Eigen::VectorXd)

// 3D position of the 2D keypoint as provided by a depth sensor and also a time
// offset for the point with respect to the time in the frame
DECLARE_CHANNEL(VISUAL_KEYPOINT_3D_POSITIONS, Eigen::Matrix3Xd)
DECLARE_CHANNEL(VISUAL_KEYPOINT_TIME_OFFSETS, Eigen::VectorXi)

/// The keypoint descriptors. (extractor output)
/// (cols are descriptors)
DECLARE_CHANNEL(DESCRIPTORS,
                std::vector<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>>)
DECLARE_CHANNEL(DESCRIPTOR_TYPES, Eigen::VectorXi)

/// Track IDs for tracked features. (-1 if not tracked); (feature tracker output)
DECLARE_CHANNEL(TRACK_IDS, Eigen::VectorXi)

/// The raw image.
DECLARE_CHANNEL(RAW_IMAGE, cv::Mat)
DECLARE_CHANNEL(CV_MAT, cv::Mat)

#endif  // ASLAM_CV_COMMON_CHANNEL_DEFINITIONS_H_
