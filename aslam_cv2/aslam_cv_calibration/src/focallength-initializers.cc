#include <vector>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/common/memory.h>
#include <aslam/common/stl-helpers.h>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <glog/logging.h>
#include <opencv2/calib3d/calib3d.hpp>

#include "aslam/calibration/target-observation.h"
#include "aslam/calibration/helpers.h"

namespace aslam {
namespace calibration {

// Initializes the intrinsics vector based on one views of a calibration targets.
// On success it returns true. These functions are based on functions from Lionel Heng and
// the excellent camodocal: https://github.com/hengli/camodocal.
// This algorithm can be used with high distortion lenses.
//
// C. Hughes, P. Denny, M. Glavin, and E. Jones,
// Equidistant Fish-Eye Calibration and Rectification by Vanishing Point
// Extraction, PAMI 2010
// Find circles from rows of chessboard corners, and for each pair
// of circles, find vanishing points: v1 and v2.
// f = ||v1 - v2|| / PI;
bool initFocalLengthVanishingPoints(
    const std::vector<TargetObservation::Ptr>& observations, Eigen::VectorXd* intrinsics) {
  CHECK_NOTNULL(intrinsics);
  CHECK(!observations.empty()) << "Need at least one observation.";

  const double cu = (observations.at(0)->getImageWidth() - 1.0) / 2.0;
  const double cv = (observations.at(0)->getImageHeight() - 1.0) / 2.0;

  const size_t num_obs = observations.size();
  std::vector<double> f_guesses;

  for (size_t obs_idx = 0; obs_idx < num_obs; ++obs_idx) {
    TargetObservation::ConstPtr obs = observations[obs_idx];
    CHECK(obs);
    TargetBase::ConstPtr current_target = obs->getTarget();
    CHECK(current_target) << "The TargetObservation has no target object.";

    // We can only process complete image observations.
    if (!obs->allCornersObservered()) {
      continue;
    }

    // Try to fit circles to each row of observations.
    Aligned<std::vector, Eigen::Vector2d> center(current_target->rows());
    std::vector<double> radius(current_target->rows());

    for (size_t r = 0u; r < current_target->rows(); ++r) {
      std::vector<cv::Point2d> points_on_circle;
      for (size_t c = 0u; c < current_target->cols(); ++c) {
        const size_t corner_idx = r * current_target->cols() + c;
        Eigen::Vector2d obs_corner;
        bool success = obs->getObservedCornerById(corner_idx, &obs_corner);
        if (success == true){
          points_on_circle.emplace_back(
              obs_corner[0],
              obs_corner[1]);
        }
      }
      InitializerHelpers::fitCircle(points_on_circle, &center[r](0), &center[r](1), &radius[r]);
    }

    // Intersect all circles to find the focal length guesses.
    for (size_t j = 0u; j < current_target->rows(); ++j) {
      for (size_t k = j + 1u; k < current_target->rows(); ++k) {
        // Find the distance between pair of vanishing points which
        // correspond to intersection points of 2 circles.
        std::vector<cv::Point2d> intersection_points;
        CHECK_LT(j, center.size());
        CHECK_LT(k, center.size());
        CHECK_LT(j, radius.size());
        CHECK_LT(k, radius.size());
        InitializerHelpers::intersectCircles(center[j](0), center[j](1), radius[j],
                                             center[k](0), center[k](1), radius[k],
                                             &intersection_points);
        if (intersection_points.size() >= 2) {
          const double f_guess = cv::norm(intersection_points[0] - intersection_points[1]) / M_PI;
          f_guesses.emplace_back(f_guess);
        }
      }
    }
  }

  // Gets the median of the guesses.
  if (f_guesses.empty()) {
    return false;
  }
  const double f0 = aslam::common::median(f_guesses.begin(), f_guesses.end());

  // Sets the first intrinsics estimate.
  intrinsics->resize(aslam::PinholeCamera::parameterCount());
  (*intrinsics)(PinholeCamera::kFu) = f0;
  (*intrinsics)(PinholeCamera::kFv) = f0;
  (*intrinsics)(PinholeCamera::kCu) = cu;
  (*intrinsics)(PinholeCamera::kCv) = cv;

  return true;
}

// Initializes the intrinsics vector based on one view of a gridded calibration target.
// On success it returns true. These functions are based on functions from Lionel Heng
// and the excellent camodocal https://github.com/hengli/camodocal.
//
// Z. Zhang
// A Flexible New Technique for Camera Calibration,
// Extraction, PAMI 2000
// Intrinsics estimation with image of absolute conic;
bool initFocalLengthAbsoluteConic(
  const std::vector<TargetObservation::Ptr>& observations, Eigen::VectorXd* intrinsics) {
 CHECK_NOTNULL(intrinsics);
 CHECK(!observations.empty()) << "Need at least one observation.";

 const double cu = (observations.at(0)->getImageWidth() - 1.0) / 2.0;
 const double cv = (observations.at(0)->getImageHeight() - 1.0) / 2.0;

 const size_t num_obs = observations.size();
 cv::Mat A(num_obs * 2, 2, CV_64F);
 cv::Mat b(num_obs * 2, 1, CV_64F);

 for (size_t obs_idx = 0u; obs_idx < num_obs; ++obs_idx) {
   TargetObservation::ConstPtr obs = observations[obs_idx];
   CHECK(obs);
   TargetBase::ConstPtr current_target = obs->getTarget();
   CHECK(current_target) << "The TargetObservation has no target object.";

   // We can only process complete image observations.
   if (!obs->allCornersObservered()) {
     continue;
   }

   std::vector<cv::Point2f> image_corners(obs->numObservedCorners());
   std::vector<cv::Point2f> M(obs->numObservedCorners());

   for (size_t j = 0; j < image_corners.size(); ++j) {
     Eigen::Vector2d obs_corner;
     bool success = obs->getObservedCornerById(j, &obs_corner);
     if (success == true){
       image_corners[j] = cv::Point2f(obs_corner[0],
                                      obs_corner[1]);
       M[j] = cv::Point2f(current_target->point(j)[0],
                          current_target->point(j)[1]);
     }
   }

   cv::Mat H = cv::findHomography(M, image_corners);

   H.at<double>(0,0) -= H.at<double>(2,0) * cu;
   H.at<double>(0,1) -= H.at<double>(2,1) * cu;
   H.at<double>(0,2) -= H.at<double>(2,2) * cu;
   H.at<double>(1,0) -= H.at<double>(2,0) * cv;
   H.at<double>(1,1) -= H.at<double>(2,1) * cv;
   H.at<double>(1,2) -= H.at<double>(2,2) * cv;

   double h[3], v[3], d1[3], d2[3];
   double n[4] = {0,0,0,0};

   for (int j = 0; j < 3; ++j) {
     double t0 = H.at<double>(j,0);
     double t1 = H.at<double>(j,1);
     h[j] = t0; v[j] = t1;
     d1[j] = (t0 + t1) * 0.5;
     d2[j] = (t0 - t1) * 0.5;
     n[0] += t0 * t0; n[1] += t1 * t1;
     n[2] += d1[j] * d1[j]; n[3] += d2[j] * d2[j];
   }

   for (int j = 0; j < 4; ++j) {
     n[j] = 1.0 / sqrt(n[j]);
   }

   for (int j = 0; j < 3; ++j) {
     h[j] *= n[0]; v[j] *= n[1];
     d1[j] *= n[2]; d2[j] *= n[3];
   }

   A.at<double>(obs_idx * 2, 0) = h[0] * v[0];
   A.at<double>(obs_idx * 2, 1) = h[1] * v[1];
   A.at<double>(obs_idx * 2 + 1, 0) = d1[0] * d2[0];
   A.at<double>(obs_idx * 2 + 1, 1) = d1[1] * d2[1];
   b.at<double>(obs_idx * 2, 0) = -h[2] * v[2];
   b.at<double>(obs_idx * 2 + 1, 0) = -d1[2] * d2[2];
 }

 cv::Mat f(2, 1, CV_64F);
 cv::solve(A, b, f, cv::DECOMP_NORMAL | cv::DECOMP_LU);

 // Sets the first intrinsics estimate.
 intrinsics->resize(aslam::PinholeCamera::parameterCount());
 (*intrinsics)(PinholeCamera::kFu) = sqrt(fabs(1.0 / f.at<double>(0)));
 (*intrinsics)(PinholeCamera::kFv) = sqrt(fabs(1.0 / f.at<double>(1)));
 (*intrinsics)(PinholeCamera::kCu) = cu;
 (*intrinsics)(PinholeCamera::kCv) = cv;

 return true;
}

}  // namespace calibration
}  // namespace aslam
