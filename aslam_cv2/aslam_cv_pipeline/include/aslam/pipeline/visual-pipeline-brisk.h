#ifndef ASLAM_BRISK_PIPELINE_H_
#define ASLAM_BRISK_PIPELINE_H_

#include <aslam/pipeline/visual-pipeline.h>
#include <aslam/pipeline/visual-pipeline-null.h>

namespace cv {
class Feature2D;
}  // namespace cv

namespace aslam {

class Undistorter;

/// \class BriskVisualPipeline
/// \brief A visual pipeline to extract Brisk features.
class BriskVisualPipeline : public VisualPipeline {
public:
  ASLAM_POINTER_TYPEDEFS(BriskVisualPipeline);
  ASLAM_DISALLOW_EVIL_CONSTRUCTORS(BriskVisualPipeline);

protected:
  /// Constructor for serialization.
  BriskVisualPipeline();

public:
  /// \brief Initialize the brisk pipeline with a camera.
  ///
  /// \param[in] camera             The intrinsic calibration of this camera.
  /// \param[in] copy_images        Should we deep copy the images passed in?
  /// \param[in] octaves            Number of octaves for BRISK scale computation.
  /// \param[in] uniformity_radius  Gets multiplied on the keypoint scale and determines
  ///                               the distance to the next neighboring keypoint.
  /// \param[in] absolute_threshold The brisk absolute threshold.
  ///                               Low makes more keypoints.
  /// \param[in] max_number_of_keypoints The maximum number of keypoints to return.
  /// \param[in] rotation_invariant Should Brisk estimate the keypoint orientation?
  /// \param[in] scale_invariant    Should Brisk estimate the keypoint scale?
  BriskVisualPipeline(const Camera::ConstPtr& camera, bool copy_images, size_t octaves,
                      double uniformity_radius, double absolute_threshold,
                      size_t max_number_of_keypoints, bool rotation_invariant,
                      bool scale_invariant);

  /// \brief Initialize the brisk pipeline with a preprocessing pipeline.
  ///
  /// \param[in] preprocessing      An undistorter to do preprocessing such as
  ///                               contrast enhancement or undistortion.
  /// \param[in] copy_images        Should we deep copy the images passed in?
  /// \param[in] octaves            Number of octaves for BRISK scale computation.
  /// \param[in] uniformity_radius  Gets multiplied on the keypoint scale and determines
  ///                               the distance to the next neighboring keypoint.
  /// \param[in] absolute_threshold The Brisk absolute threshold.
  ///                               Low makes more keypoints.
  /// \param[in] max_number_of_keypoints The maximum number of keypoints to return.
  /// \param[in] rotation_invariant Should brisk estimate the keypoint orientation?
  /// \param[in] scale_invariant    Should Brisk estimate the keypoint scale?
  BriskVisualPipeline(std::unique_ptr<Undistorter>& preprocessing, bool copy_images, size_t octaves,
                      double uniformity_radius, double absolute_threshold,
                      size_t max_number_of_keypoints, bool rotation_invariant,
                      bool scale_invariant);

  virtual ~BriskVisualPipeline();

  /// \brief Initialize the brisk pipeline.
  ///
  /// \param[in] octaves            Number of octaves for BRISK scale computation.
  /// \param[in] uniformity_radius  Gets multiplied on the keypoint scale and determines
  ///                               the distance to the next neighboring keypoint.
  /// \param[in] absolute_threshold The brisk absolute threshold.
  ///                               Low makes more keypoints.
  /// \param[in] max_number_of_keypoints The maximum number of keypoints to return.
  /// \param[in] rotation_invariant Should Brisk estimate the keypoint orientation?
  /// \param[in] scale_invariant    Should Brisk estimate the keypoint scale?
  void initializeBrisk(size_t octaves, double uniformity_radius,
                       double absolute_threshold, size_t max_number_of_keypoints,
                       bool rotation_invariant, bool scale_invariant);

protected:
  /// \brief Process the frame and fill the results into the frame variable
  ///
  /// The top level function will already fill in the timestamps and the output camera.
  /// \param[in]     image The image data.
  /// \param[in/out] frame The visual frame. This will be constructed before calling.
  virtual void processFrameImpl(const cv::Mat& image,
                                VisualFrame* frame) const;
private:
  std::shared_ptr<cv::Feature2D> detector_;
  std::shared_ptr<cv::Feature2D> extractor_;

  size_t octaves_;
  double uniformity_radius_;
  double absolute_threshold_;
  size_t max_number_of_keypoints_;
  bool rotation_invariant_;
  bool scale_invariant_;
};

}  // namespace aslam

#endif // ASLAM_BRISK_PIPELINE_H_
