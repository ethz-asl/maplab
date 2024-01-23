#ifndef ASLAM_FRAMES_VISUAL_FRAME_H_
#define ASLAM_FRAMES_VISUAL_FRAME_H_

#include <memory>
#include <unordered_map>
#include <cstdint>

#include <aslam/cameras/camera.h>
#include <aslam/common/channel.h>
#include <aslam/common/channel-declaration.h>
#include <aslam/common/macros.h>
#include <aslam/common/unique-id.h>
#include <Eigen/Dense>

namespace aslam {
class Camera;

/// \class VisualFrame
/// \brief An image and keypoints from a single camera.
///
/// This class stores data from an image and keypoints taken from a single
/// camera. It stores a pointer to the camera's intrinsic calibration,
/// an id that uniquely identifies this frame, and a measurement timestamp.
///
/// The class also stores a ChannelGroup object that can be used to hold
/// keypoint data, the raw image, and other associated information.
///
/// The camera geometry stored in the frame and accessible by getCameraGeometry()
/// may refer to a transformed geometry that includes downsampling and undistortion.
/// However, we recommend to always store the raw image with the frame so that
/// no information is lost. In order to be able to plot on these raw images, we must
/// also store the original camera geometry object (accessible with
/// getRawCameraGeometry()). To plot transformed keypoints on the raw image, one
/// may use toRawImageCoordinates() or toRawImageCoordinatesVectorized() to recover
/// the raw image coordinates of any keypoint.
class VisualFrame  {
 public:
  /// \brief The descriptor matrix stores descriptors in columns, i.e. the descriptor matrix
  ///        has num_bytes_per_descriptor rows and num_descriptors columns.
  typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> DescriptorsT;
  typedef Eigen::VectorXd KeypointScoresT;

  ASLAM_POINTER_TYPEDEFS(VisualFrame);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //////////////////////////////////////////////////////////////
  /// \name Constructors/destructors and operators
  /// @{
  VisualFrame();
  virtual ~VisualFrame() {};

  /// Copy constructor for clone operation. (Cameras are not cloned!)
  VisualFrame(const VisualFrame& other);
  VisualFrame& operator=(const VisualFrame& other);

  virtual bool operator==(const VisualFrame& other) const;
  /// @}

  virtual bool compareWithoutCameraGeometry(const VisualFrame& other) const;

  template<typename CHANNEL_DATA_TYPE>
  void addChannel(const std::string& channel) {
    aslam::channels::addChannel<CHANNEL_DATA_TYPE>(channel, &channels_);
  }

  /// Are there keypoint measurements stored in this frame?
  bool hasKeypointMeasurements() const;

  /// Are there keypoint measurement uncertainties stored in this frame?
  bool hasKeypointMeasurementUncertainties() const;

  /// Are there keypoint orientations stored in this frame?
  bool hasKeypointOrientations() const;

  /// Are there keypoint scores stored in this frame?
  bool hasKeypointScores() const;

  /// Are there keypoint scales stored in this frame?
  bool hasKeypointScales() const;

  /// Are there any 3D measurements stored in this frame?
  bool hasKeypoint3DPositions() const;

  /// Are there any time offsets stored in this frame?
  bool hasKeypointTimeOffsets() const;

  /// Are there descriptors stored in this frame?
  bool hasDescriptors() const;

  /// Are there track ids stored in this frame?
  bool hasTrackIds() const;

  /// Is there a raw image stored in this frame?
  bool hasRawImage() const;

  /// Is a certain channel stored in this frame?
  bool hasChannel(const std::string& channel) const {
    return aslam::channels::hasChannel(channel, channels_);
  }

  /// Clears the following channels: KeypointMeasurements, KeypointMeasurementUncertainties,
  /// KeypointOrientations, KeypointScores, KeypointScales, Descriptors, TrackIds
  void clearKeypointChannels();

  /// The keypoint measurements stored in a frame.
  const Eigen::Matrix2Xd& getKeypointMeasurements() const;

  /// Get the number of keypoint measurements stored in this frame.
  inline size_t getNumKeypointMeasurements() const {
    return hasKeypointMeasurements() ? getKeypointMeasurements().cols() : 0u;
  }

  size_t getNumDescriptors() const;

  /// The keypoint measurement uncertainties stored in a frame.
  const Eigen::VectorXd& getKeypointMeasurementUncertainties() const;

  /// The keypoint orientations stored in a frame.
  const Eigen::VectorXd& getKeypointOrientations() const;

  /// The keypoint scores stored in a frame.
  const Eigen::VectorXd& getKeypointScores() const;

  /// The keypoint scales stored in a frame.
  const Eigen::VectorXd& getKeypointScales() const;

  /// The keypoint 3D measurements stored in a frame.
  const Eigen::Matrix3Xd& getKeypoint3DPositions() const;

  /// The keypoint time offsets stored in a frame.
  const Eigen::VectorXi& getKeypointTimeOffsets() const;

  /// The descriptors stored in a frame.
  const DescriptorsT& getDescriptors(size_t index = 0) const;

  /// The track ids stored in this frame.
  const Eigen::VectorXi& getTrackIds() const;

  /// The raw image stored in a frame.
  const cv::Mat& getRawImage() const;

  /// Release the raw image. Only if the cv::Mat reference count is 1 the memory will be freed.
  void releaseRawImage();

  template<typename CHANNEL_DATA_TYPE>
  const CHANNEL_DATA_TYPE& getChannelData(const std::string& channel) const {
    return aslam::channels::getChannelData<CHANNEL_DATA_TYPE>(channel, channels_);
  }

  /// A pointer to the keypoint measurements, can be used to swap in new data.
  Eigen::Matrix2Xd* getKeypointMeasurementsMutable();

  /// A pointer to the keypoint measurement uncertainties, can be used to swap in new data.
  Eigen::VectorXd* getKeypointMeasurementUncertaintiesMutable();

  /// A pointer to the keypoint orientations, can be used to swap in new data.
  Eigen::VectorXd* getKeypointOrientationsMutable();

  /// A pointer to the keypoint scores, can be used to swap in new data.
  Eigen::VectorXd* getKeypointScoresMutable();

  /// A pointer to the keypoint scales, can be used to swap in new data.
  Eigen::VectorXd* getKeypointScalesMutable();

  /// A pointer to the keypoint 3D positions, can be used to swap in new data.
  Eigen::Matrix3Xd* getKeypoint3DPositionsMutable();

  /// A pointer to the keypoint time offsets, can be used to swap in new data.
  Eigen::VectorXi* getKeypointTimeOffsetsMutable();

  /// A pointer to the descriptors, can be used to swap in new data.
  DescriptorsT* getDescriptorsMutable(size_t index = 0);

  /// A pointer to the track ids, can be used to swap in new data.
  Eigen::VectorXi* getTrackIdsMutable();

  /// A pointer to the raw image, can be used to swap in new data.
  cv::Mat* getRawImageMutable();

  template<typename CHANNEL_DATA_TYPE>
  CHANNEL_DATA_TYPE* getChannelDataMutable(const std::string& channel) const {
    CHANNEL_DATA_TYPE& data =
        aslam::channels::getChannelData<CHANNEL_DATA_TYPE>(channel,
                                                           channels_);
    return &data;
  }

  /// Return block expression of the keypoint measurement pointed to by index.
  const Eigen::Block<const Eigen::Matrix2Xd, 2, 1> getKeypointMeasurement(size_t index) const;

  /// Return the keypoint measurement uncertainty at index.
  double getKeypointMeasurementUncertainty(size_t index) const;

  /// Return the keypoint orientation at index.
  double getKeypointOrientation(size_t index) const;

  /// Return the keypoint score at index.
  double getKeypointScore(size_t index) const;

  /// Return the keypoint scale at index.
  double getKeypointScale(size_t index) const;

  /// Return block expression of the keypoint 3D position pointed to by index.
  const Eigen::Block<const Eigen::Matrix3Xd, 3, 1> getKeypoint3DPosition(size_t index) const;

  /// Return the keypoint timeoffset at index.
  int getKeypointTimeOffset(size_t index) const;

  /// Return pointer location of the descriptor pointed to by index.
  const unsigned char* getDescriptor(size_t index) const;

  /// Return the track id at index. (-1: not tracked)
  int getTrackId(size_t index) const;

  /// Replace (copy) the internal keypoint measurements by the passed ones.
  void setKeypointMeasurements(const Eigen::Matrix2Xd& keypoints);

  /// Replace (copy) the internal keypoint measurement uncertainties
  ///        by the passed ones.
  void setKeypointMeasurementUncertainties(const Eigen::VectorXd& uncertainties);

  /// Replace (copy) the internal keypoint orientations by the passed ones.
  void setKeypointOrientations(const Eigen::VectorXd& orientations);

  /// Replace (copy) the internal keypoint scores by the passed ones.
  void setKeypointScores(const Eigen::VectorXd& scores);

  /// Replace (copy) the internal keypoint orientations by the passed ones.
  void setKeypointScales(const Eigen::VectorXd& scales);

  /// Replace (copy) the internal keypoint 3D positions by the passed ones.
  void setKeypoint3DPositions(const Eigen::Matrix3Xd& positions);

  /// Replace (copy) the internal track ids by the passed ones.
  void setKeypointTimeOffsets(const Eigen::VectorXi& offsets);

  /// Replace (copy) the internal descriptors by the passed ones.
  template <typename Derived>
  void setDescriptors(const Derived& descriptors, size_t index = 0,
                      int descriptor_type = 0);

  /// Replace (copy) the internal track ids by the passed ones.
  void setTrackIds(const Eigen::VectorXi& track_ids);

  /// Replace (copy) the internal raw image by the passed ones.
  ///        This is a shallow copy by default. Please clone the image if it
  ///        should be owned by the VisualFrame.
  void setRawImage(const cv::Mat& image);

  template<typename CHANNEL_DATA_TYPE>
  void setChannelData(const std::string& channel,
                      const CHANNEL_DATA_TYPE& data_new) {
    if (!aslam::channels::hasChannel(channel, channels_)) {
      aslam::channels::addChannel<CHANNEL_DATA_TYPE>(channel, &channels_);
    }
    CHANNEL_DATA_TYPE& data =
        aslam::channels::getChannelData<CHANNEL_DATA_TYPE>(channel, channels_);
    data = data_new;
  }

  /// Replace (swap) the internal keypoint measurements by the passed ones.
  /// This method creates the channel if it doesn't exist
  void swapKeypointMeasurements(Eigen::Matrix2Xd* keypoints);

  /// Replace (swap) the internal keypoint measurement uncertainties
  /// by the passed ones.
  void swapKeypointMeasurementUncertainties(Eigen::VectorXd* uncertainties);

  /// Replace (swap) the internal keypoint orientations by the passed ones.
  void swapKeypointOrientations(Eigen::VectorXd* orientations);

  /// Replace (swap) the internal keypoint scores by the passed ones.
  void swapKeypointScores(Eigen::VectorXd* scores);

  /// Replace (swap) the internal keypoint orientations by the passed ones.
  void swapKeypointScales(Eigen::VectorXd* scales);

  /// Replace (swap) the internal keypoint 3D positions by the passed ones.
  /// This method creates the channel if it doesn't exist
  void swapKeypoint3DPositions(Eigen::Matrix3Xd* positions);

  /// Replace (swap) the internal keypoint timeoffsets by the passed ones.
  void swapKeypointTimeOffsets(Eigen::VectorXi* offsets);

  /// Replace (swap) the internal descriptors by the passed ones.
  /// Returns the type of the swapped out descriptor
  int swapDescriptors(DescriptorsT* descriptors, size_t index = 0,
                       int descriptor_type = 0);

  /// Replace (swap) the internal track ids by the passed ones.
  void swapTrackIds(Eigen::VectorXi* track_ids);

  /// Swap channel data with the data passed in. This will only work
  /// if the channel data type has a swap() method.
  template<typename CHANNEL_DATA_TYPE>
  void swapChannelData(const std::string& channel,
                       CHANNEL_DATA_TYPE* data_new) {
    CHECK_NOTNULL(data_new);
    if (!aslam::channels::hasChannel(channel, channels_)) {
      aslam::channels::addChannel<CHANNEL_DATA_TYPE>(channel, &channels_);
    }
    CHANNEL_DATA_TYPE& data =
        aslam::channels::getChannelData<CHANNEL_DATA_TYPE>(channel, channels_);
    data.swap(*data_new);
  }

  /// The camera geometry.
  const Camera::ConstPtr getCameraGeometry() const;

  /// Set the camera geometry.
  void setCameraGeometry(const Camera::ConstPtr& camera);

  /// The camera geometry.
  const Camera::ConstPtr getRawCameraGeometry() const;

  /// Set the camera geometry.
  void setRawCameraGeometry(const Camera::ConstPtr& camera);

  /// \brief Return keypoint measurement pointed to by index in raw image coordinates.
  /// \param[in] keypoint_idx              Index of the keypoint.
  /// \param[out] keypoint_raw_coordinates The keypoint in raw image coordinates.
  /// \return Was the projection successful?
  aslam::ProjectionResult getKeypointInRawImageCoordinates(size_t keypoint_idx,
      Eigen::Vector2d* keypoint_raw_coordinates) const;

  /// \brief Convert keypoint coordinates to raw image coordinates.
  ///
  /// \param[in] keypoint               The keypoint coordinates with respect to
  ///                                   the camera calibration available from
  ///                                   getCameraGeometry().
  /// \param[out] out_image_coordinates The image coordinates with respect to the
  ///                                   camera calibration available from
  ///                                   getRawCameraGeometry().
  aslam::ProjectionResult toRawImageCoordinates(const Eigen::Vector2d& keypoint,
                                                Eigen::Vector2d* out_image_coordinates) const;

  /// \brief Convert keypoint coordinates to raw image coordinates.
  ///
  /// \param[in] keypoints              The keypoint coordinates with respect to
  ///                                   the camera calibration available from
  ///                                   getCameraGeometry().
  /// \param[out] out_image_coordinates The image coordinates with respect to the
  ///                                   camera calibration available from
  ///                                   getRawCameraGeometry().
  /// \param[out] results               One result for each keypoint.
  void toRawImageCoordinatesVectorized(const Eigen::Matrix2Xd& keypoints,
                                       Eigen::Matrix2Xd* out_image_coordinates,
                                       std::vector<aslam::ProjectionResult>* results) const;

  /// Return a list of normalized bearing vectors for the specified keypoint indices.
  Eigen::Matrix3Xd getNormalizedBearingVectors(
      const std::vector<size_t>& keypoint_indices, int descriptor_type,
      std::vector<unsigned char>* backprojection_success) const;

  /// Get the frame id.
  inline const aslam::FrameId& getId() const { return id_; }

  /// Set the frame id.
  inline void setId(const aslam::FrameId& id) { id_ = id; }

  /// Get the timestamp.
  inline int64_t getTimestampNanoseconds() const { return timestamp_nanoseconds_; }

  /// Set the timestamp.
  inline void setTimestampNanoseconds(int64_t timestamp_nanoseconds) {
    timestamp_nanoseconds_ = timestamp_nanoseconds;
  }

  /// Set the size of the descriptor in bytes.
  size_t getDescriptorSizeBytes(size_t index = 0) const;

  /// Set the validity flag to true.
  void validate() { is_valid_ = true; }
  /// Set the validity flag to false.
  void invalidate(){ is_valid_ = false; }
  /// Check the validity flag.
  bool isValid() const { return is_valid_; }
  /// Set the validity flag.
  void setValid(bool is_valid) { is_valid_ = is_valid; }

  /// Print out a human-readable version of this frame
  void print(std::ostream& out, const std::string& label) const;

  /// Lock frame for processing, a matching call to unlock() must exist
  void lock() { m_frame_lock_.lock(); }
  /// Unlock a locked frame, must be called after lock
  void unlock() { m_frame_lock_.unlock(); }

  /// \brief Creates an empty frame. The following channels are added without any data attached:
  ///        {KeypointMeasurements, KeypointMeasurementUncertainties, Descriptors}
  /// @param[in]  camera                  Camera which will be assigned to the frame.
  /// @param[in]  timestamp_nanoseconds   Timestamp of the frame. [ns]
  /// @return Pointer to the created frame.
  static VisualFrame::Ptr createEmptyTestVisualFrame(const aslam::Camera::ConstPtr& camera,
                                                     int64_t timestamp_nanoseconds);

  void discardUntrackedObservations(std::vector<size_t>* discarded_indices);

  /* Additional functions for dealing with multiple different types of
     feature in the same channel, including different descriptor sizes */

  // Will add a new block of features that is separate from the previous one.
  //
  // The feature types are then defined only when adding descriptors, so for
  // measurements each new feature type a minimum requirement is to have  
  // keypoint and descriptors added. Other properties are optional.
  // 
  // The default value is used to pad the array for the previously added
  // feature type, if it didn't have that value defined.
  void extendKeypointMeasurements(const Eigen::Matrix2Xd& keypoints_new);
  void extendKeypointMeasurementUncertainties(
      const Eigen::VectorXd& uncertainties_new, double default_value = 0.0);
  void extendKeypointOrientations(
      const Eigen::VectorXd& orientations_new, double default_value = 0.0);
  void extendKeypointScores(
      const Eigen::VectorXd& scores_new, double default_value = 0.0);
  void extendKeypointScales(
      const Eigen::VectorXd& scales_new, double default_value = 0.0);
  void extendKeypoint3DPositions(
      const Eigen::Matrix3Xd& positions_new, double default_value = 0.0);
  void extendKeypointTimeOffsets(
      const Eigen::VectorXi& offsets_new, int default_value = -1);
  template <typename Derived>
  void extendDescriptors(const Derived& descriptors_new, int descriptor_type = 0);
  void extendTrackIds(const Eigen::VectorXi& track_ids_new, int default_value = -1);

  // Will serialize the multi-descriptor blocks into a single string
  void serializeDescriptorsToString(std::string* descriptors_string) const;
  void deserializeDescriptorsFromString(const std::string& descriptors_string);

  /* Descriptors are stored in a vector of Eigen "blocks". Each block has an
     index as well as a corresponding feature type for the entire block. */

  // Check if the frame has a certain feature type
  bool hasDescriptorType(int descriptor_type) const;
  // This function is for overwriting the internal feature type vector, which
  // is automatically managed. Currently used only when deserializing data into
  // and empty frame. Use with care!
  void setDescriptorTypes(const Eigen::VectorXi& descriptor_types);
  // Get the feature type of each descriptor block that is stored.
  const Eigen::VectorXi& getDescriptorTypes() const;
  // Get the feature type for a keypoint at a specific keypoint index
  int getDescriptorType(size_t index) const;
  // Get the type of a descriptor stored at the specified block index
  int getDescriptorBlockType(size_t block) const;
  // Get the size and keypoint index at which a descriptor block starts
  void getDescriptorBlockTypeStartAndSize(
      int descriptor_type, size_t *start, size_t *size) const;
  // Get the block index which contains the descriptor of a certain type
  size_t getDescriptorTypeBlock(int descriptor_type) const;
  // Get the size in bytes of a descriptor for a certain feature type
  size_t getDescriptorTypeSizeBytes(int descriptor_type) const;
  
  // Number of features that have a certain feature type. Will return 0
  // if the frame does not have features of that type or a feature channel.
  size_t getNumKeypointMeasurementsOfType(int descriptor_type) const;

  // Get the Eigen block only for a certain feature type.
  // WARNING! When using these functions pay attention to the return type
  // to avoid unnecessary copies:
  //   - BAD (will create copy):
  //   const Eigen::Matrix2Xd& keypoints = getKeypointMeasurementsOfType(0);
  //   - GOOD (will point to memory block in the aslam frame):
  //   const Eigen::Block<const Eigen::Matrix2Xd> = 
  //       getKeypointMeasurementsOfType(0);
  const Eigen::Block<const Eigen::Matrix2Xd> getKeypointMeasurementsOfType(
      int descriptor_type) const;
  const Eigen::VectorBlock<const Eigen::VectorXd> getKeypointMeasurementUncertaintiesOfType(
      int descriptor_type) const;
  const Eigen::VectorBlock<const Eigen::VectorXd> getKeypointOrientationsOfType(
      int descriptor_type) const;
  const Eigen::VectorBlock<const Eigen::VectorXd> getKeypointScoresOfType(
      int descriptor_type) const;
  const Eigen::VectorBlock<const Eigen::VectorXd> getKeypointScalesOfType(
      int descriptor_type) const;
  const Eigen::Block<const Eigen::Matrix3Xd> getKeypoint3DPositionsOfType(
      int descriptor_type) const;
  const Eigen::VectorBlock<const Eigen::VectorXi> getKeypointTimeOffsetsOfType(
      int descriptor_type) const;
  const DescriptorsT& getDescriptorsOfType(int descriptor_type) const;
  const Eigen::VectorBlock<const Eigen::VectorXi> getTrackIdsOfType(
      int descriptor_type) const;

  // Get mutable Eigen block for a certain feature type
  // WARNING! Same warning on return types as in the previous comment, even
  // more so here to ensure the editable memory block is not a temp copy.
  Eigen::VectorBlock<Eigen::VectorXi> getTrackIdsOfTypeMutable(
      int descriptor_type);

  // Get individual keypoint indices for a certain feature type.
  // WARNING: repeated calls are inefficient. In that case try to use the above
  // functions that get const refs to an entire block.
  const Eigen::Block<const Eigen::Matrix2Xd, 2, 1> getKeypointMeasurementOfType(
      size_t index, int descriptor_type) const;
  double getKeypointMeasurementUncertaintyOfType(size_t index, int descriptor_type) const;
  double getKeypointOrientationOfType(size_t index, int descriptor_type) const;
  double getKeypointScaleOfType(size_t index, int descriptor_type) const;
  double getKeypointScoreOfType(size_t index, int descriptor_type) const;
  const Eigen::Block<const Eigen::Matrix3Xd, 3, 1> getKeypoint3DPositionOfType(
      size_t index, int descriptor_type) const;
  int getKeypointTimeOffsetOfType(size_t index, int descriptor_type) const;
  int getTrackIdOfType(size_t index, int descriptor_type) const;

 private:
  /// Timestamp in nanoseconds.
  int64_t timestamp_nanoseconds_;

  aslam::FrameId id_;
  aslam::channels::ChannelGroup channels_;
  Camera::ConstPtr camera_geometry_;
  Camera::ConstPtr raw_camera_geometry_;

  // Provides capability to lock frame for processing in parallelized system
  std::mutex m_frame_lock_;

  /// Validity flag: can be used by an external algorithm to flag frames that should
  /// be excluded/included when processing a list of frames. Does not have any internal
  /// effect on the frame.
  bool is_valid_;
};

inline std::ostream& operator<<(std::ostream& out, const VisualFrame& rhs) {
  rhs.print(out, "");
  return out;
}
}  // namespace aslam
#endif  // ASLAM_FRAMES_VISUAL_FRAME_H_
