#ifndef ASLAM_UNDISTORT_HELPERS_H_
#define ASLAM_UNDISTORT_HELPERS_H_

#include <algorithm>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// This file contains modified opencv routines which use aslam's distort and project functionality.
// Original functions can be found here:
//  getUndistortRectangles: https://github.com/Itseez/opencv/blob/5f590ebed084a5002c9013e11c519dcb139d47e9/modules/calib3d/src/calibration.cpp#L2094
//  getOptimalNewCameraMatrix: https://github.com/Itseez/opencv/blob/0ffc53bafea9052d1fac17e6fc6f0a07ddf9f789/modules/imgproc/src/undistort.cpp#L62
//  buildUndistortMap: https://github.com/Itseez/opencv/blob/5f590ebed084a5002c9013e11c519dcb139d47e9/modules/calib3d/src/calibration.cpp#L3389
namespace aslam {
namespace common {

/// \brief: calculates the inner(min)/outer(max) rectangle on the undistorted image
///  @param[in] input_camera Input camera geometry
///  @param[in] undistort_to_pinhole Undistort image to a pinhole projection
///                                  (remove distortion and projection effects)
///  @param[out] inner Inscribed image rectangle (all pixels valid)
///  @param[out] outer Circumscribed image rectangle (no pixels lost)
template<typename DerivedCameraType>
static void getUndistortRectangles(const DerivedCameraType& input_camera, bool undistort_to_pinhole,
                                   cv::Rect_<float>& inner, cv::Rect_<float>& outer) {
  const int N = 9;
  int x, y, k;
  cv::Ptr<CvMat> _pts(cvCreateMat(1, N * N, CV_32FC2));
  CvPoint2D32f* pts = (CvPoint2D32f*) (_pts->data.ptr);

  for (y = k = 0; y < N; y++) {
    for (x = 0; x < N; x++) {
      Eigen::Vector2d keypoint(x * input_camera.imageWidth()/ (N - 1),
                               y * input_camera.imageHeight() / (N - 1));

      // Transform keypoint from image to normalized image plane. (incl. projection effects)
      Eigen::Vector2d keypoint_normalized;

      Eigen::Matrix3d camera_matrix;
      switch(input_camera.getType()) {
        case Camera::Type::kPinhole: {
          const aslam::PinholeCamera *const cam =
              dynamic_cast<const aslam::PinholeCamera*>(&input_camera);
          CHECK(cam != nullptr);
          camera_matrix = cam->getCameraMatrix();
          break;
        }
        case Camera::Type::kUnifiedProjection: {
          const aslam::UnifiedProjectionCamera *const cam =
              dynamic_cast<const aslam::UnifiedProjectionCamera*>(&input_camera);
          CHECK(cam != nullptr);
          camera_matrix = cam->getCameraMatrix();
          break;
        }
        default: {
          LOG(FATAL) << "Unknown camera model: "
                     << static_cast<std::underlying_type<Camera::Type>::type>(
                          input_camera.getType());
        }
      }

      if (undistort_to_pinhole) {
        // Transform keypoint from image to normalized image plane. (incl. projection effects)
        Eigen::Vector3d point_3d;
        input_camera.backProject3(keypoint, &point_3d);
        point_3d /= point_3d[2];
        keypoint_normalized[0] = point_3d[0];
        keypoint_normalized[1] = point_3d[1];
      } else {
        // Transform keypoint from image to normalized image plane.

        keypoint_normalized[0] = 1.0 / camera_matrix(0,0) * (keypoint[0] - camera_matrix(0,2));
        keypoint_normalized[1] = 1.0 / camera_matrix(1,1) * (keypoint[1] - camera_matrix(1,2));

        input_camera.getDistortion().undistort(&keypoint_normalized);
      }

      pts[k++] = cvPoint2D32f((float) keypoint_normalized[0], (float) keypoint_normalized[1]);
    }
  }

  float iX0 = -FLT_MAX, iX1 = FLT_MAX, iY0 = -FLT_MAX, iY1 = FLT_MAX;
  float oX0 = FLT_MAX, oX1 = -FLT_MAX, oY0 = FLT_MAX, oY1 = -FLT_MAX;
  // find the inscribed rectangle.
  // the code will likely not work with extreme rotation matrices (R) (>45%)
  for (y = k = 0; y < N; y++)
  {
    for (x = 0; x < N; x++) {
      CvPoint2D32f p = pts[k++];
      oX0 = MIN(oX0, p.x);
      oX1 = MAX(oX1, p.x);
      oY0 = MIN(oY0, p.y);
      oY1 = MAX(oY1, p.y);

      if (x == 0)     iX0 = std::max(iX0, p.x);
      if (x == N - 1) iX1 = std::min(iX1, p.x);
      if (y == 0)     iY0 = std::max(iY0, p.y);
      if (y == N - 1) iY1 = std::min(iY1, p.y);
    }
  }
  inner = cv::Rect_<float>(iX0, iY0, iX1 - iX0, iY1 - iY0);
  outer = cv::Rect_<float>(oX0, oY0, oX1 - oX0, oY1 - oY0);
}

/// \brief Returns the new camera matrix based on the free scaling parameter.
/// INPUT:
/// @param[in] input_camera Aslam camera geometry (distortion and intrinsics used)
/// @param[in] scale Output image size scaling parameter wrt. to input image size.
/// @param[in] alpha Free scaling parameter between 0 (when all the pixels in the undistorted image
///                  will be valid) and 1 (when all the source image pixels will be retained in the
///                  undistorted image)
///  @param[in] undistort_to_pinhole Undistort image to a pinhole projection
///                                  (remove distortion and projection effects)
/// @return The output camera matrix.
template<typename DerivedCameraType>
Eigen::Matrix3d getOptimalNewCameraMatrix(const DerivedCameraType& input_camera,
                                          double alpha, double scale,
                                          bool undistort_to_pinhole) {

  CHECK_GE(alpha, 0.0); CHECK_LE(alpha, 1.0);
  CHECK_GT(scale, 0.0);

  cv::Size output_size(static_cast<int>(scale * input_camera.imageWidth()),
                       static_cast<int>(scale * input_camera.imageHeight()));

  // Get inscribed and circumscribed rectangles in normalized
  cv::Rect_<float> inner, outer;
  getUndistortRectangles(input_camera, undistort_to_pinhole, inner, outer);

  // Projection mapping inner rectangle to viewport
  double fx0 = (output_size.width - 1) / inner.width;
  double fy0 = (output_size.height - 1) / inner.height;
  double cx0 = -fx0 * inner.x;
  double cy0 = -fy0 * inner.y;

  // Projection mapping outer rectangle to viewport
  double fx1 = (output_size.width - 1) / outer.width;
  double fy1 = (output_size.height - 1) / outer.height;
  double cx1 = -fx1 * outer.x;
  double cy1 = -fy1 * outer.y;

  // Interpolate between the two extremal projections
  Eigen::Matrix3d output_camera_matrix = Eigen::Matrix3d::Zero();
  output_camera_matrix(0, 0) = fx0 * (1.0 - alpha) + fx1 * alpha;
  output_camera_matrix(1, 1) = fy0 * (1.0 - alpha) + fy1 * alpha;
  output_camera_matrix(0, 2) = cx0 * (1.0 - alpha) + cx1 * alpha;
  output_camera_matrix(1, 2) = cy0 * (1.0 - alpha) + cy1 * alpha;
  output_camera_matrix(2, 2) = 1.0;

  return output_camera_matrix;
}

/// \brief Calculates the undistortion maps for the given camera geometries.
/// @param[in] input_camera Input camera geometry
/// @param[in] output_camera_matrix Desired output camera matrix (see \ref getOptimalNewCameraMatrix)
/// @param[in] scale Output image size scaling parameter wrt. to input image size.
/// @param[in] map_type Type of the output maps. (cv::CV_32FC1, cv::CV_32FC2 or cv::CV_16SC2)
///                     Use cv::CV_16SC2 if you don't know what to choose. (fastest fixed-point)
/// @param[out] map_u Map that transforms u-coordinates from distorted to undistorted image plane.
/// @param[out] map_v Map that transforms v-coordinates from distorted to undistorted image plane.
template<typename InputDerivedCameraType, typename OutputDerivedCameraType>
void buildUndistortMap(const InputDerivedCameraType& input_camera,
                       const OutputDerivedCameraType& output_camera, int map_type,
                       cv::OutputArray map_u, cv::OutputArray map_v) {
  // Output image size
  cv::Size output_size(output_camera.imageWidth(), output_camera.imageHeight());

  // Allocate the outputs maps
  CHECK(map_type == CV_16SC2 || map_type == CV_32FC1 || map_type == CV_32FC2);
  map_u.create(output_size, map_type);
  cv::Mat map1 = map_u.getMat(), map2;
  if (map_type != CV_32FC2) {
    map_v.create(output_size, map_type == CV_16SC2 ? CV_16UC1 : CV_32FC1);
    map2 = map_v.getMat();
  } else
    map_v.release();

  // Build the maps.
  for (int i = 0; i < output_size.height; i++) {
    float* m1f = (float*) (map1.data + map1.step * i);
    float* m2f = (float*) (map2.data + map2.step * i);
    short* m1 = (short*) m1f;
    ushort* m2 = (ushort*) m2f;

    for (int j = 0; j < output_size.width; j++) {
      // Convert point on normalized image plane to keypoints. (projection and distortion)
      const Eigen::Vector2d keypoint(j, i);
      Eigen::Vector2d keypoint_dist;
      Eigen::Vector3d point_3d;
      output_camera.backProject3(keypoint, &point_3d);
      point_3d /= point_3d[2];
      input_camera.project3(point_3d, &keypoint_dist);

      const double& u = keypoint_dist[0];
      const double& v = keypoint_dist[1];

      // Store in output format
      if (map_type == CV_16SC2) {
        int iu = cv::saturate_cast<int>(u * cv::INTER_TAB_SIZE);
        int iv = cv::saturate_cast<int>(v * cv::INTER_TAB_SIZE);
        m1[j * 2] = (short) (iu >> cv::INTER_BITS);
        m1[j * 2 + 1] = (short) (iv >> cv::INTER_BITS);
        m2[j] = (ushort) ((iv & (cv::INTER_TAB_SIZE - 1)) * cv::INTER_TAB_SIZE
            + (iu & (cv::INTER_TAB_SIZE - 1)));
      } else if (map_type == CV_32FC1) {
        m1f[j] = (float) u;
        m2f[j] = (float) v;
      } else {
        m1f[j * 2] = (float) u;
        m1f[j * 2 + 1] = (float) v;
      }
    }
  }
}

} //namespace common
} //namespace aslam

#endif // ASLAM_UNDISTORT_HELPERS_H_
