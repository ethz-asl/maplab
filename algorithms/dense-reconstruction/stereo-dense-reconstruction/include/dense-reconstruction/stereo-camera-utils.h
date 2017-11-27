#ifndef DENSE_RECONSTRUCTION_STEREO_CAMERA_UTILS_H_
#define DENSE_RECONSTRUCTION_STEREO_CAMERA_UTILS_H_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// The implementations in this file were adapted from the
// image_undistort ROS library written by Zachary Taylor
// Github: https://github.com/ethz-asl/image_undistort/
// This file combines the following sources:
//  - image_undistort/include/image_undistort/camera_parameters.h
//  - image_undistort/src/camera_parameters.cpp
//  - image_undistort/include/image_undistort/undistorter.h
//  - image_undistort/src/undistorter.cpp

namespace dense_reconstruction {
namespace stereo {

enum class CameraSide { FIRST, SECOND };
enum class CameraIO { INPUT, OUTPUT };
enum class DistortionModel { RADTAN, EQUIDISTANT, FOV };
enum class DistortionProcessing { UNDISTORT, PRESERVE };

// Holds basic properties of a camera.
class CameraParametersBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraParametersBase(
      const cv::Size& resolution_in,
      const Eigen::Matrix<double, 4, 4>& T_C_G_in,
      const Eigen::Matrix<double, 3, 3>& K_in);

  const cv::Size& resolution() const;

  const Eigen::Matrix<double, 4, 4>& T_C_G() const;
  const Eigen::Ref<const Eigen::Matrix<double, 3, 3>> R() const;
  const Eigen::Ref<const Eigen::Matrix<double, 3, 1>> p() const;

  const Eigen::Matrix<double, 3, 4>& P() const;  // get projection matrix
  const Eigen::Matrix<double, 3, 3>& K() const;  // get camera matrix

  bool operator==(const CameraParametersBase& B) const;
  bool operator!=(const CameraParametersBase& B) const;

 private:
  cv::Size resolution_;
  Eigen::Matrix<double, 4, 4> T_C_G_;
  Eigen::Matrix<double, 3, 4> P_;
  Eigen::Matrix<double, 3, 3> K_;
};

// Basic camera properties + distortion parameters.
class InputCameraParameters : public CameraParametersBase {
 public:
  InputCameraParameters(
      const cv::Size& resolution_in,
      const Eigen::Matrix<double, 4, 4>& T_C_G_in,
      const Eigen::Matrix<double, 3, 3>& K_in, const std::vector<double>& D_in,
      const DistortionModel& distortion_model);

  const std::vector<double>& D() const;
  const DistortionModel& distortionModel() const;

  bool operator==(const InputCameraParameters& B) const;
  bool operator!=(const InputCameraParameters& B) const;

 private:
  static DistortionModel stringToDistortion(
      const std::string& distortion_model);

  std::vector<double> D_;
  DistortionModel distortion_model_;
};

// Basic camera properties + anything special to output cameras (currently
// nothing).
class OutputCameraParameters : public CameraParametersBase {
 public:
  using CameraParametersBase::CameraParametersBase;
};

// Holds the camera parameters of the input camera and virtual output camera.
class CameraParametersPair {
 public:
  CameraParametersPair(
      const DistortionProcessing distortion_processing =
          DistortionProcessing::UNDISTORT);

  bool setInputCameraParameters(
      const cv::Size& resolution, const Eigen::Matrix<double, 4, 4>& T,
      const Eigen::Matrix<double, 3, 3>& K, const std::vector<double>& D,
      const DistortionModel& distortion_model);

  bool setOutputCameraParameters(
      const cv::Size& resolution, const Eigen::Matrix<double, 4, 4>& T,
      const Eigen::Matrix<double, 3, 3>& K);

  bool setOutputFromInput();

  bool setOptimalOutputCameraParameters(const double scale);

  const DistortionProcessing& distortionProcessing() const;

  const std::shared_ptr<InputCameraParameters>& getInputPtr() const;
  const std::shared_ptr<OutputCameraParameters>& getOutputPtr() const;

  bool valid() const;
  bool valid(const CameraIO& io) const;

  bool operator==(const CameraParametersPair& B) const;
  bool operator!=(const CameraParametersPair& B) const;

 private:
  std::shared_ptr<InputCameraParameters> input_ptr_;
  std::shared_ptr<OutputCameraParameters> output_ptr_;

  DistortionProcessing distortion_processing_;

  static constexpr double kFocalLengthEstimationAttempts = 100.0;
};

// Holds the camera parameters of the first and second camera and uses them to
// generate virtual output cameras with properties that will produce correctly
// rectified images.
class StereoCameraParameters {
 public:
  explicit StereoCameraParameters(const double scale = 1.0);

  bool setInputCameraParameters(
      const cv::Size& resolution, const Eigen::Matrix<double, 4, 4>& T,
      const Eigen::Matrix<double, 3, 3>& K, const std::vector<double>& D,
      const DistortionModel& distortion_model, const CameraSide& side);

  const CameraParametersPair& getFirst() const;
  const CameraParametersPair& getSecond() const;

  bool valid() const;
  bool valid(const CameraSide& side, const CameraIO& io) const;

 private:
  bool generateRectificationParameters();

  double scale_;
  CameraParametersPair first_;
  CameraParametersPair second_;
};

class Undistorter {
 public:
  explicit Undistorter(
      const CameraParametersPair& input_camera_parameters_pair);

  void undistortImage(const cv::Mat& image, cv::Mat* undistored_image);

  // Get camera parameters used to build undistorter.
  const CameraParametersPair& getCameraParametersPair();

  // Generates a new output camera with fx = fy = (scale * (input_fx +
  // input_fy)/2, center point in the center of the image, R = I, and a
  // resolution that is as large as possible while having no empty pixels.
  static void setOptimalOutputCameraParameters(
      const double scale, CameraParametersPair* camera_parameters_pair);

  static void distortPixel(
      const Eigen::Matrix<double, 3, 3>& K_in,
      const Eigen::Matrix<double, 3, 3>& R_in,
      const Eigen::Matrix<double, 3, 4>& P_out,
      const DistortionModel& distortion_model, const std::vector<double>& D,
      const Eigen::Vector2d& pixel_location,
      Eigen::Vector2d* distorted_pixel_location);

 private:
  const CameraParametersPair used_camera_parameters_pair_;

  cv::Mat map_x_;
  cv::Mat map_y_;

  double empty_pixels_;
};

}  // namespace stereo
}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_STEREO_CAMERA_UTILS_H_
