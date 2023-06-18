#ifndef ASLAM_COVARIANCE_HELPERS_H_
#define ASLAM_COVARIANCE_HELPERS_H_

#include <Eigen/Dense>
#include <aslam/common/pose-types.h>

namespace aslam {
namespace common {

void rotateCovariance(
    const aslam::Transformation& T_B_A,
    const aslam::TransformationCovariance& A_covariance,
    aslam::TransformationCovariance* B_covariance);

}  // namespace common
}  // namespace aslam
#endif  // ASLAM_COVARIANCE_HELPERS_H_
