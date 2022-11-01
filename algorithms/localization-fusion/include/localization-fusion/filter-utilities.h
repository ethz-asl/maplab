#ifndef LOCALIZATION_FUSION_FILTER_UTILITIES_H_
#define LOCALIZATION_FUSION_FILTER_UTILITIES_H_

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

std::ostream& operator<<(std::ostream& os, const Eigen::MatrixXd& mat);
std::ostream& operator<<(std::ostream& os, const Eigen::VectorXd& vec);
std::ostream& operator<<(std::ostream& os, const std::vector<size_t>& vec);
std::ostream& operator<<(std::ostream& os, const std::vector<int>& vec);

namespace maplab {

//! @brief Utility method keeping RPY angles in the range [-pi, pi]
//! @param[in] rotation - The rotation to bind
//! @return the bounded value
//!
double clampRotation(double rotation);

void RPYtoQuaternion(
    double r, double p, double y,
    Eigen::Quaterniond& q);  // NOLINT

void matrixToRPY(
    const Eigen::Matrix3d& m, double& yaw,  // NOLINT
    double& pitch,                          // NOLINT
    double& roll);                          // NOLINT

}  // namespace maplab

#endif  // LOCALIZATION_FUSION_FILTER_UTILITIES_H_
