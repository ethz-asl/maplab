#ifndef ASLAM_TEST_CONVERT_MAPS_LEGACY_H
#define ASLAM_TEST_CONVERT_MAPS_LEGACY_H

#include <opencv2/imgproc/imgproc.hpp>

namespace aslam {

// This function is a copy of the convertMaps function of OpenCV's 2.4.13.1
// branch to circumvent a SSE2 bug that was introduced with OpenCV 3.* that
// causes a loss of accuracy in the map conversion from CV16SC2 to CV32FC1.
void convertMapsLegacy(cv::InputArray _map1, cv::InputArray _map2,
                       cv::OutputArray _dstmap1, cv::OutputArray _dstmap2,
                       int dstm1type, bool nninterpolate = false);
} // namespace aslam

#endif // ASLAM_TEST_CONVERT_MAPS_LEGACY_H
