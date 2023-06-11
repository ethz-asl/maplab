#ifndef ASLAM_PIPELINE_MAPPED_UNDISTORTER_H_
#define ASLAM_PIPELINE_MAPPED_UNDISTORTER_H_

#include <opencv2/core/core.hpp>

#include <aslam/common/types.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/pipeline/undistorter.h>

namespace aslam {

/// \brief Factory method to create a mapped undistorter for this camera geometry.
///        NOTE: The undistorter stores a copy of this camera and changes to the original geometry
///              are not connected with the undistorter!
/// @param[in] camera     The camera object a mapped undistorter should be
///                       created for. Currently the method supports pinhole and unified projection
///                       cameras. Any other type of camera will result in the hard failure.
/// @param[in] alpha Free scaling parameter between 0 (when all the pixels in the undistorted image
///                  will be valid) and 1 (when all the source image pixels will be retained in the
///                  undistorted image)
/// @param[in] scale Output image size scaling parameter wrt. to input image size.
/// @param[in] interpolation_type Check \ref InterpolationMethod to see the available types.
/// @return Pointer to the created mapped undistorter.
template <typename CameraType>
std::unique_ptr<MappedUndistorter> createMappedUndistorter(
    const CameraType& camera, float alpha, float scale,
    aslam::InterpolationMethod interpolation_type);

/// \brief Factory method to create a mapped undistorter for this camera geometry to undistorts
///        the image to a pinhole view.
///        NOTE: The undistorter stores a copy of the input camera and changes to the original
///              geometry are not connected with the undistorter!
/// @param[in] unified_proj_camera_ptr Shared pointer to the unified projection camera object.
/// @param[in] alpha Free scaling parameter between 0 (when all the pixels in the undistorted image
///                  will be valid) and 1 (when all the source image pixels will be retained in the
///                  undistorted image)
/// @param[in] scale Output image size scaling parameter wrt. to input image size.
/// @param[in] interpolation_type Check \ref MappedUndistorter to see the available types.
/// @return Pointer to the created mapped undistorter.
std::unique_ptr<MappedUndistorter> createMappedUndistorterToPinhole(
    const aslam::UnifiedProjectionCamera& unified_proj_camera,
    float alpha, float scale, aslam::InterpolationMethod interpolation_type);

/// \class MappedUndistorter
/// \brief A class that encapsulates image undistortion for building frames from images.
///
/// This class utilizes the OpenCV remap() function:
/// http://docs.opencv.org/modules/imgproc/doc/geometric_transformations.html?highlight=remap#remap
class MappedUndistorter : public Undistorter {
public:
  ASLAM_POINTER_TYPEDEFS(MappedUndistorter);
  ASLAM_DISALLOW_EVIL_CONSTRUCTORS(MappedUndistorter);

protected:
  MappedUndistorter();

public:
  /// \brief Create a mapped undistorter using externally provided maps.
  ///
  /// Map matrices (map_u and map_v) must be the size of the output camera geometry.
  /// This will be checked by the constructor.
  ///
  /// \param[in] input_camera  The camera intrinsics for the original image.
  /// \param[in] output_camera The camera intrinsics after undistortion.
  /// \param[in] map_u         The map from input to output u coordinates.
  /// \param[in] map_v         The map from input to output v coordinates.
  /// \param[in] interpolation Interpolation method used for undistortion.
  ///                          (\ref InterpolationMethod)
  MappedUndistorter(aslam::Camera::Ptr input_camera, aslam::Camera::Ptr output_camera,
                    const cv::Mat& map_u, const cv::Mat& map_v, InterpolationMethod interpolation);

  virtual ~MappedUndistorter() = default;

  /// \brief Produce an undistorted image from an input image.
  virtual void processImage(const cv::Mat& input_image, cv::Mat* output_image) const;

  /// Get the undistorter map for the u-coordinate.
  const cv::Mat& getUndistortMapU() const { return map_u_; };

  /// Get the undistorter map for the u-coordinate.
  const cv::Mat& getUndistortMapV() const { return map_v_; };

private:
  /// \brief LUT for u coordinates.
  const cv::Mat map_u_;
  /// \brief LUT for v coordinates.
  const cv::Mat map_v_;
  /// \brief Interpolation strategy
  InterpolationMethod interpolation_method_;
};

}  // namespace aslam

#include "aslam/pipeline/undistorter-mapped-inl.h"

#endif // ASLAM_PIPELINE_MAPPED_UNDISTORTER_H_
