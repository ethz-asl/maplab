#ifndef MAPLAB_COMMON_HISTOGRAMS_H_
#define MAPLAB_COMMON_HISTOGRAMS_H_

#include <vector>

#include <Eigen/Dense>

namespace common {

namespace histograms {

template <typename InputScalar>
Eigen::MatrixXd histogram2d(
    const Eigen::Matrix<InputScalar, 2, Eigen::Dynamic>& points,
    const size_t num_x_bins, const size_t num_y_bins);

// In the result, x = column, y = row.
Eigen::MatrixXd logHistogram2dWithIndexAsX(
    const Eigen::MatrixXi y_values, const size_t num_x_bins,
    const size_t num_y_bins);

// "matching_indices" are k x query_size. Valid indices must be non-negative.
// Negative indices are considered invalid and not counted toward the histogram.
Eigen::MatrixXd logHistogram2dForKBestIndexMatching(
    const Eigen::MatrixXi matching_indices, const size_t num_x_bins,
    const size_t num_y_bins);

template <typename InputScalar>
Eigen::MatrixXd downsample(
    const Eigen::Matrix<InputScalar, Eigen::Dynamic, Eigen::Dynamic>& input,
    const size_t num_x_bins, const size_t num_y_bins);

// y_values[i] contains the y-coefficients of all points whose x-coefficents
// are i. So, if y_values = {{0, 1}, {}, {2, 3}}, the data points are
// {(0, 0), (0, 1), (2, 2), (2, 3)}.
template <typename InputScalar>
Eigen::MatrixXd downsampleWithIndexAsX(
    const std::vector<std::vector<InputScalar>>& y_values,
    const size_t num_x_bins, const size_t num_y_bins);

}  // namespace histograms

}  // namespace common

#include "./histograms-inl.h"

#endif  // MAPLAB_COMMON_HISTOGRAMS_H_
