#ifndef MAPLAB_COMMON_HISTOGRAMS_INL_H_
#define MAPLAB_COMMON_HISTOGRAMS_INL_H_

#include <algorithm>
#include <vector>

#include <glog/logging.h>

namespace common {

namespace histograms {

template <typename InputScalar>
Eigen::MatrixXd histogram2d(
    const Eigen::Matrix<InputScalar, 2, Eigen::Dynamic>& points,
    const size_t num_x_bins, const size_t num_y_bins) {
  typedef Eigen::Matrix<InputScalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
  Eigen::Matrix<InputScalar, 2, Eigen::Dynamic> positive_points(
      2, points.cols());
  positive_points.template topRows<1>() =
      points.template topRows<1>() -
      MatrixX::Constant(
          1, points.cols(), points.template topRows<1>().minCoeff());
  positive_points.template bottomRows<1>() =
      points.template bottomRows<1>() -
      MatrixX::Constant(
          1, points.cols(), points.template bottomRows<1>().minCoeff());

  const InputScalar x_max = positive_points.template topRows<1>().maxCoeff();
  const InputScalar y_max = positive_points.template bottomRows<1>().maxCoeff();

  Eigen::MatrixXi bin_counts(num_y_bins, num_x_bins);
  bin_counts.setZero();
  for (int i = 0; i < positive_points.cols(); ++i) {
    const size_t row = std::min(
        static_cast<size_t>(floor(positive_points(1, i) * num_y_bins / y_max)),
        num_y_bins - 1);
    const size_t col = std::min(
        static_cast<size_t>(floor(positive_points(0, i) * num_x_bins / x_max)),
        num_x_bins - 1);
    ++bin_counts(row, col);
  }

  CHECK_GT(num_x_bins, 0u);
  CHECK_GT(num_y_bins, 0u);
  const double bin_density = (x_max / num_x_bins) * (y_max / num_y_bins);

  return bin_counts.cast<double>() / bin_density;
}

template <typename InputScalar>
Eigen::MatrixXd downsample(
    const Eigen::Matrix<InputScalar, Eigen::Dynamic, Eigen::Dynamic>& input,
    const size_t num_x_bins, const size_t num_y_bins) {
  CHECK_GT(input.rows(), 0);
  CHECK_GT(input.cols(), 0);
  CHECK_GT(num_x_bins, 0u);
  CHECK_GT(num_y_bins, 0u);
  Eigen::MatrixXd bin_sums(num_y_bins, num_x_bins);
  bin_sums.setZero();
  for (size_t x_index = 0; x_index < input.cols(); ++x_index) {
    const int x = x_index * num_x_bins / input.cols();
    for (int y_index = 0; y_index < input.rows(); ++y_index) {
      const int y = y_index * num_y_bins / input.rows();
      bin_sums(y, x) += input(y_index, x_index);
    }
  }

  CHECK_GE(input.cols(), num_x_bins);
  CHECK_GE(input.rows(), num_y_bins);
  const double bin_density =
      (input.cols() / num_x_bins) * (input.rows() / num_y_bins);

  return bin_sums / bin_density;
}

template <typename InputScalar>
Eigen::MatrixXd downsampleWithIndexAsX(
    const std::vector<std::vector<InputScalar>>& y_values,
    const size_t num_x_bins, const size_t num_y_bins) {
  static_assert(
      std::is_integral<InputScalar>::value, "Input scalar must be integral");
  const size_t x_ceiling = y_values.size();
  CHECK_GT(x_ceiling, 0);
  InputScalar y_max = static_cast<InputScalar>(0);
  for (const std::vector<InputScalar>& y_of_x : y_values) {
    InputScalar vec_y_max = *std::max_element(y_of_x.begin(), y_of_x.end());
    if (vec_y_max > y_max) {
      y_max = vec_y_max;
    }
  }
  const InputScalar y_ceiling = y_max + static_cast<InputScalar>(1);

  Eigen::MatrixXi bin_counts(num_y_bins, num_x_bins);
  bin_counts.setZero();
  for (size_t x_index = 0u; x_index < x_ceiling; ++x_index) {
    const int x = x_index * num_x_bins / x_ceiling;
    for (size_t y_index = 0u; y_index < y_values[x_index].size(); ++y_index) {
      const int y = y_values[x_index][y_index] * num_y_bins / y_ceiling;
      ++bin_counts(y, x);
    }
  }

  const double bin_density = (static_cast<double>(x_ceiling) / num_x_bins) *
                             (static_cast<double>(y_ceiling) / num_y_bins);

  return bin_counts.cast<double>() / bin_density;
}

}  // namespace histograms

}  // namespace common

#endif  // MAPLAB_COMMON_HISTOGRAMS_INL_H_
