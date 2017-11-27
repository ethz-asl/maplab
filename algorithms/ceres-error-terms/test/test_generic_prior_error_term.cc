#include <Eigen/Core>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <ceres-error-terms/generic-prior-error-term.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

class PriorErrorTerms : public ::testing::Test {
 public:
  static constexpr int VectorDim = 9;

  static constexpr int PriorBlockSize = 3;
  static constexpr int PriorBlockIndex = 4;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  virtual void SetUp() {
    prior_mean_ << 1, 2, 3, 4, 5, 6, 7, 8, -3;
    block_prior_mean_ = prior_mean_.segment<PriorBlockSize>(PriorBlockIndex);

    current_value_ = prior_mean_;
    covariance_matrix_.setIdentity();
    block_covariance_matrix_ =
        covariance_matrix_.block<PriorBlockSize, PriorBlockSize>(
            PriorBlockIndex, PriorBlockIndex);
  }

  void addResidual();
  void addMoreGenericResidual();
  void solve();

  ceres::Problem problem_;
  ceres::Solver::Summary summary_;
  Eigen::Matrix<double, VectorDim, 1> prior_mean_;
  Eigen::Matrix<double, VectorDim, VectorDim> covariance_matrix_;

  Eigen::Matrix<double, PriorBlockSize, 1> block_prior_mean_;
  Eigen::Matrix<double, PriorBlockSize, PriorBlockSize>
      block_covariance_matrix_;

  Eigen::Matrix<double, VectorDim, 1> current_value_;
};

void PriorErrorTerms::addResidual() {
  ceres::CostFunction* cost_function =
      new ceres_error_terms::GenericPriorErrorTerm<VectorDim, PriorBlockIndex,
                                                   PriorBlockSize>(
          block_prior_mean_, block_covariance_matrix_);
  problem_.AddResidualBlock(cost_function, NULL, current_value_.data());
}

void PriorErrorTerms::solve() {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.parameter_tolerance = 1e-20;
  options.gradient_tolerance = 1e-20;
  options.function_tolerance = 1e-20;
  options.max_num_iterations = 1e3;
  ceres::Solve(options, &problem_, &summary_);

  LOG(INFO) << summary_.BriefReport() << std::endl;
  LOG(INFO) << summary_.message << std::endl;
}

TEST_F(PriorErrorTerms, TestGenericPriorErrorTermZeroCost) {
  addResidual();
  solve();

  EXPECT_EQ(summary_.initial_cost, 0.0);
  EXPECT_EQ(summary_.final_cost, 0.0);
  EXPECT_EQ(summary_.iterations.size(), 1u);
}

TEST_F(PriorErrorTerms, TestGenericPriorErrorTermOptimization) {
  current_value_.segment<PriorBlockSize>(PriorBlockIndex).setRandom();
  current_value_.segment<PriorBlockSize>(PriorBlockIndex) *= 1000.0;

  addResidual();
  solve();

  EXPECT_NEAR_EIGEN(current_value_, prior_mean_, 1e-10);
  EXPECT_LT(summary_.final_cost, 1e-10);
}

MAPLAB_UNITTEST_ENTRYPOINT
