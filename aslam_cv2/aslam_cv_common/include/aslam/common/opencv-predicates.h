#ifndef GTEST_OPENCV_PREDICATE_H_
#define GTEST_OPENCV_PREDICATE_H_

#include <cmath>
#include <sstream>

// Deliberately not including OpenCV here, to avoid the dependency.
#include <gtest/gtest.h>
#include <glog/logging.h>

#define EXPECT_NEAR_OPENCV(image_A, image_B, precision) \
  EXPECT_TRUE(gtest_catkin::ImagesEqual(image_A, image_B, precision))

#define ASSERT_NEAR_OPENCV(image_A, image_B, precision) \
  ASSERT_TRUE(gtest_catkin::ImagesEqual(image_A, image_B, precision))

namespace gtest_catkin {

// Most types are integral. Set the default tolerance to zero.
template<typename T>
struct TestTolerance {
  static constexpr T kDiffTolerance = 0;
};

// Specialize the tolerance for floating point types.
template<>
struct TestTolerance<float> {
  static constexpr float kDiffTolerance = 1e-8;
};
template<>
struct TestTolerance<double> {
  static constexpr double kDiffTolerance = 1e-15;
};

template< typename SCALAR >
inline testing::AssertionResult ImagesEqualTyped(const cv::Mat& A,
                                                 const cv::Mat& B,
                                                 double tolerance) {
  typedef SCALAR Scalar;
  bool success = true;
  if (A.rows != B.rows || A.cols != B.cols || A.channels() != B.channels()) {
    return testing::AssertionFailure()
      << "Image size mismatch: "
      << A.rows << "x" << A.cols << "x" << A.channels() << " != "
      << B.rows << "x" << B.cols << "x" << B.channels();
  }

  if(tolerance < 0.0) {
    tolerance = TestTolerance<Scalar>::kDiffTolerance;
  }

  std::stringstream spy_difference;
  testing::AssertionResult failure_reason(false);
  for (int r = 0; r < A.rows; ++r) {
    for (int c = 0; c < A.cols; ++c) {
      for(int ch = 0; ch < A.channels(); ++ch) {
        Scalar Aij = (&A.at<Scalar>(r, c))[ch];
        Scalar Bij = (&B.at<Scalar>(r, c))[ch];
        if (!std::isfinite(Aij) ||
            !std::isfinite(Bij) ||
            !Eigen::internal::isApprox(Aij, Bij, tolerance)) {
          spy_difference << "x";
          if (A.rows == 1) {
            failure_reason <<
                "\nMismatch at position " << c << ": " << Aij << " != " << Bij;
          } else if (A.cols == 1) {
            failure_reason <<
                "\nMismatch at position " << c << ": " << Aij << " != " << Bij;
          } else {
            failure_reason << "\nMismatch at "
                << r << "," << c << ": " << Aij << " != " << Bij;
          }
          success = false;
        } else {
          spy_difference << " ";
        }
      }
    }
    spy_difference << "\n";

    failure_reason << "\n";
  }
  // If we have a matrix and it is not very small then print the sparsity
  // pattern of the difference between A and B.
  if (A.rows > 1 && A.cols > 1 && A.rows * A.cols > 12) {
    failure_reason << "Sparsity pattern of difference:\n"
        << spy_difference.str();
  }

  if(success) {
    return ::testing::AssertionSuccess();
  } else {
    return failure_reason;
  }
}

// Compare two opencv images by checking whether the absolute difference
// between any two elements exceeds a threshold. On failure, a helpful error
// message is returned. Use like this:
//   EXPECT_TRUE(MatricesEqual(first_matrix, second_matrix, tolerance));
inline testing::AssertionResult ImagesEqual(const cv::Mat& A,
                                            const cv::Mat& B,
                                            double tolerance) {
  if(A.type() != B.type()) {
    return testing::AssertionFailure() << "Image type mismatch"
        << A.type() << " != " << B.type();
  }

  // http://docs.opencv.org/modules/core/doc/basic_structures.html#mat-depth
  switch(A.depth()) {
    case cv::DataType<uint8_t>::depth:
    return ImagesEqualTyped<uint8_t>(A, B, tolerance);
    break;
    case cv::DataType<int8_t>::depth:
    return ImagesEqualTyped<int8_t>(A, B, tolerance);
    break;
    case cv::DataType<uint16_t>::depth:
    return ImagesEqualTyped<uint16_t>(A, B, tolerance);
    break;
    case cv::DataType<int16_t>::depth:
    return ImagesEqualTyped<int16_t>(A, B, tolerance);
    break;
    case cv::DataType<int>::depth:
    return ImagesEqualTyped<int>(A, B, tolerance);
    break;
    case cv::DataType<double>::depth:
    return ImagesEqualTyped<double>(A, B, tolerance);
    break;
    case cv::DataType<float>::depth:
    return ImagesEqualTyped<float>(A, B, tolerance);
    break;
    default:
      return ::testing::AssertionFailure() << "cv::Mat depth " << A.depth() << " is not supported for "
      << "comparison.";
      break;
  }
  return ::testing::AssertionFailure() << "Unreachable code.";
}

// Compare two opencv images by checking whether the absolute difference
// between any two elements exceeds a threshold. On failure, a helpful error
// message is returned. Use like this:
//   EXPECT_TRUE(ImagesEqual(first_matrix, second_matrix));
inline testing::AssertionResult ImagesEqual(const cv::Mat& A,
                                            const cv::Mat& B) {
  return ImagesEqual(A, B, -1.0);
}
}  // namespace gtest_catkin

#endif  // GTEST_EIGEN_PREDICATE_H_
