#ifndef ASLAM_VISUAL_MULTI_FRAME_H
#define ASLAM_VISUAL_MULTI_FRAME_H

#include <memory>
#include <vector>

#include <aslam/common/macros.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/unique-id.h>
#include <Eigen/Dense>

namespace aslam {
class Camera;
class NCamera;
class VisualFrame;

/// \class VisualMultiFrame
/// \brief A class representing images and keypoints and 
///        calibration data from a multi-camera system.
class VisualNFrame {
 public:
  ASLAM_POINTER_TYPEDEFS(VisualNFrame);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::vector<VisualNFrame::Ptr> PtrVector;
  typedef std::vector<VisualNFrame::ConstPtr> ConstPtrVector;

 protected:
  /// \brief Creates an empty visual VisualNFrame.
  VisualNFrame() = default;

 public:
  /// \brief Creates a visual n-frame from an id and number of frames.
  ///
  /// This constructor should only be used in a specific situation when there
  /// is no information about the camera system when constructing the object.
  /// This may happen e.g. when deserializing data.
  ///
  ///        The individual frames are initialized to NULL.
  ///        The camera system is initialized to NULL.
  ///
  /// \param[in] id          The unique id for this object.
  /// \param[in] num_frames  The number of frames to be constructed.
  VisualNFrame(const aslam::NFramesId& id, unsigned int num_frames);

  /// \brief Creates a visual n-frame from an id and camera system.
  ///
  ///        The individual frames are initialized to NULL.
  ///
  /// \param[in] id       The unique id for this object.
  /// \param[in] ncameras The camera system associated with this object.
  VisualNFrame(const aslam::NFramesId& id, std::shared_ptr<NCamera> ncameras);

  /// \brief Creates a visual n-frame from a camera system.
  ///
  ///        The id is set randomly and the individual frames are initialized to NULL.
  ///
  /// \param[in] ncameras The camera system associated with this object.
  VisualNFrame(std::shared_ptr<NCamera> ncameras);

  virtual ~VisualNFrame() {}

  /// Copy constructor for clone operation. (NCamera is not cloned!)
  VisualNFrame(const VisualNFrame& other);
  VisualNFrame& operator=(const VisualNFrame& other);

  /// \brief Get the multiframe id.
  inline const aslam::NFramesId& getId() const { return id_; }

  /// \brief Set the multiframe id.
  inline void setId(const aslam::NFramesId& n_frames_id) { id_ = n_frames_id; }

  /// \brief Get the camera rig.
  const NCamera& getNCamera() const;
  
  /// \brief Get the camera rig.
  std::shared_ptr<NCamera> getNCameraShared();

  /// \brief Get the camera rig.
  std::shared_ptr<const NCamera> getNCameraShared() const;
  
  /// \brief Set the camera rig.
  ///
  /// This method fills in in multi-camera system information. It should be
  /// used if we had no such knowledge at time of construction of this object.
  /// This method will also assign cameras to the already existing visual
  /// frames.
  void setNCameras(std::shared_ptr<NCamera> ncameras);

  /// \brief Is a frame set for this index?
  bool isFrameSet(size_t frame_index) const;

  /// \brief Are all the frames set?
  bool areAllFramesSet() const;

  /// \brief Is the frame at this index valid.
  bool isFrameValid(size_t frame_index) const;

  /// \brief Get one frame.
  const VisualFrame& getFrame(size_t frame_index) const;

  /// \brief Get one frame, mutable.
  std::shared_ptr<VisualFrame> getFrameShared(size_t frame_index);

  /// \brief Get one frame.
  std::shared_ptr<const VisualFrame> getFrameShared(size_t frame_index) const;

  /// \brief Set the frame at the index.
  ///
  /// The method will fail hard if the frame does not have the same camera
  /// as specified in the camera system. It is expected that this method will
  /// mostly be used by the pipeline code when building a VisualNFrame for the
  /// first time.
  void setFrame(size_t frame_index, std::shared_ptr<VisualFrame> frame);

  /// \brief Replace the frame at the index with a nullptr.
  void unSetFrame(size_t frame_index);

  /// \brief The number of frames.
  size_t getNumFrames() const;

  /// \brief The number of frames.
  size_t getNumCameras() const;

  /// \brief Get the pose of body frame with respect to the camera i.
  const Transformation& get_T_C_B(size_t camera_index) const;

  /// \brief Get the geometry object for camera i.
  const Camera& getCamera(size_t camera_index) const;

  /// \brief Get the id for the camera at index i.
  const CameraId& getCameraId(size_t camera_index) const;

  /// \brief Does this rig have a camera with this id?
  bool hasCameraWithId(const CameraId& id) const;

  /// \brief Get the index of the camera with the id.
  /// @returns -1 if the rig doesn't have a camera with this id
  size_t getCameraIndex(const CameraId& id) const;

  /// \brief Get the min. timestamp over all frames in nanoseconds.
  int64_t getMinTimestampNanoseconds() const;
  /// \brief Get the max. timestamp over all frames in nanoseconds.
  int64_t getMaxTimestampNanoseconds() const;

  /// \brief Binary equality.
  bool operator==(const VisualNFrame& other) const;

  /// \brief Binary equality excluding the camera system.
  bool compareWithoutCameraSystem(const VisualNFrame& other) const;

  /// \brief Creates an empty visual nframe with the given ncamera system.
  /// @param[in] ncamera                NCamera.
  /// @param[in] timestamp_nanoseconds  Timestamp of the individual frames in the nframe. [ns]
  /// @return Pointer to the created VisualNFrame.
  static VisualNFrame::Ptr createEmptyTestVisualNFrame(
      const std::shared_ptr<NCamera>& ncamera, int64_t timestamp_nanoseconds);

  /// \brief Checks if all frames have raw image each.
  /// @return False if any one frame does not have raw image, true otherwise.
  bool hasRawImagesInAllFrames() const;

  /// \brief Iterates over all frames and releases (removes) the raw image.
  void releaseRawImagesOfAllFrames();

  /// \brief Iterates over all frames and resets all keypoint channels
  ///        (keypoints, keypoint_uncertainties, track_ids, keypoint_scales, ...)
  ///        to vectors of length zero.
  void clearKeypointChannelsOfAllFrames();

 private:
  /// \brief The unique frame id.
  NFramesId id_;

  /// \brief The camera rig.
  std::shared_ptr<NCamera> camera_rig_;

  /// \brief The list of individual image frames.
  std::vector<std::shared_ptr<VisualFrame>> frames_;
};

} // namespace aslam

#endif /* ASLAM_VISUAL_MULTI_FRAME_H */
