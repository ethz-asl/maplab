#ifndef ASLAM_KEYPOINT_IDENTIFIER_H_
#define ASLAM_KEYPOINT_IDENTIFIER_H_

#include <memory>
#include <vector>

#include <Eigen/Core>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/macros.h>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/unique-id.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>

namespace aslam {

/// \class KeypointIdentifier
/// \brief A reference to a single keypoint observation.
class KeypointIdentifier {
 public:
  ASLAM_POINTER_TYPEDEFS(KeypointIdentifier);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  KeypointIdentifier() = delete;

  // Factory function.
  static inline KeypointIdentifier create(const std::shared_ptr<const aslam::VisualNFrame>& nframe,
                                          size_t frame_index, size_t keypoint_index) {
    return KeypointIdentifier(nframe, frame_index, keypoint_index);
  }

 protected:
  KeypointIdentifier(const std::shared_ptr<const aslam::VisualNFrame>& nframe,
                     size_t frame_index, size_t keypoint_index)
  : frame_index_(frame_index), keypoint_index_(keypoint_index), nframe_(nframe) {
    CHECK(nframe);
    CHECK_LT(frame_index, nframe->getNumFrames());
    CHECK(nframe->isFrameSet(frame_index));
    CHECK_LT(keypoint_index, nframe->getFrame(frame_index).getNumKeypointMeasurements());
  }

 public:
  inline size_t getKeypointIndex() const { return keypoint_index_; }
  inline size_t getFrameIndex() const { return frame_index_; }
  inline const aslam::NFramesId& getNFrameId() const { return nframe_->getId(); }
  inline const aslam::FrameId& getFrameId() const {
    return nframe_->getFrame(frame_index_).getId();
  }
  inline const aslam::VisualFrame& getFrame() const { return nframe_->getFrame(frame_index_); }
  inline const aslam::VisualNFrame& getNFrame() const { return *nframe_; }

  const Eigen::Block<const Eigen::Matrix2Xd, 2, 1> getKeypointMeasurement() const {
    return nframe_->getFrame(frame_index_).getKeypointMeasurement(keypoint_index_);
  }

  const uint8_t* getDescriptor() const {
    return nframe_->getFrame(frame_index_).getDescriptor(keypoint_index_);
  }

  std::shared_ptr<const aslam::Camera> getCamera() const {
    return nframe_->getNCamera().getCameraShared(frame_index_);
  }

  const aslam::NCamera& getNCamera() const {
    return nframe_->getNCamera();
  }

  const aslam::Transformation& get_T_C_B() const {
    return nframe_->getNCamera().get_T_C_B(frame_index_);
  }

 private:
  /// Frame index in in the VisualFrame. Corresponds to the NCamera index.
  size_t frame_index_;
  /// Keypoint index in in the VisualFrame.
  size_t keypoint_index_;
  /// Pointer to the VisualNFrame that contains the keypoint.
  std::shared_ptr<const aslam::VisualNFrame> nframe_;
};

typedef Aligned<std::vector, KeypointIdentifier> KeypointIdentifierList;

}  // namespace aslam
#endif  // ASLAM_KEYPOINT_IDENTIFIER_H_
