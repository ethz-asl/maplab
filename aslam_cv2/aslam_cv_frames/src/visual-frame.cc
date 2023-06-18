#include "aslam/frames/visual-frame.h"

#include <memory>
#include <aslam/common/channel-definitions.h>
#include <aslam/common/stl-helpers.h>
#include <aslam/common/time.h>

namespace aslam {
VisualFrame::VisualFrame()
    : timestamp_nanoseconds_(time::getInvalidTime()),
      is_valid_(true) {}

VisualFrame::VisualFrame(const VisualFrame& other) {
  *this = other;
}

VisualFrame& VisualFrame::operator=(const VisualFrame& other) {
  timestamp_nanoseconds_ = other.timestamp_nanoseconds_;
  id_ = other.id_;
  camera_geometry_ = other.camera_geometry_;
  raw_camera_geometry_ = other.raw_camera_geometry_;

  channels_ = channels::cloneChannelGroup(other.channels_);
  is_valid_ = other.is_valid_;
  return *this;
}

bool VisualFrame::operator==(const VisualFrame& other) const {
  bool same = true;
  same &= timestamp_nanoseconds_ == other.timestamp_nanoseconds_;
  same &= channels::isChannelGroupEqual(channels_, other.channels_);
  same &= static_cast<bool>(camera_geometry_) ==
      static_cast<bool>(other.camera_geometry_);
  if (static_cast<bool>(camera_geometry_) &&
      static_cast<bool>(other.camera_geometry_)) {
    same &= (*camera_geometry_) == (*other.camera_geometry_);
  }
  same &= is_valid_ == other.is_valid_;
  return same;
}

bool VisualFrame::compareWithoutCameraGeometry(const VisualFrame& other) const {
  bool same = true;
  same &= timestamp_nanoseconds_ == other.timestamp_nanoseconds_;
  same &= channels::isChannelGroupEqual(channels_, other.channels_);
  same &= is_valid_ == other.is_valid_;
  return same;
}

bool VisualFrame::hasKeypointMeasurements() const {
  return aslam::channels::has_VISUAL_KEYPOINT_MEASUREMENTS_Channel(channels_);
}
bool VisualFrame::hasKeypointMeasurementUncertainties() const{
  return aslam::channels::has_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Channel(channels_);
}
bool VisualFrame::hasKeypointOrientations() const{
  return aslam::channels::has_VISUAL_KEYPOINT_ORIENTATIONS_Channel(channels_);
}
bool VisualFrame::hasKeypointScores() const {
  return aslam::channels::has_VISUAL_KEYPOINT_SCORES_Channel(channels_);
}
bool VisualFrame::hasKeypointScales() const{
  return aslam::channels::has_VISUAL_KEYPOINT_SCALES_Channel(channels_);
}
bool VisualFrame::hasKeypoint3DPositions() const{
  return aslam::channels::has_VISUAL_KEYPOINT_3D_POSITIONS_Channel(channels_);
}
bool VisualFrame::hasKeypointTimeOffsets() const{
  return aslam::channels::has_VISUAL_KEYPOINT_TIME_OFFSETS_Channel(channels_);
}
bool VisualFrame::hasDescriptors() const{
  return aslam::channels::has_DESCRIPTORS_Channel(channels_);
}
bool VisualFrame::hasTrackIds() const {
  return aslam::channels::has_TRACK_IDS_Channel(channels_);
}
bool VisualFrame::hasRawImage() const {
  return aslam::channels::has_RAW_IMAGE_Channel(channels_);
}

const Eigen::Matrix2Xd& VisualFrame::getKeypointMeasurements() const {
  return aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENTS_Data(channels_);
}
const Eigen::VectorXd& VisualFrame::getKeypointMeasurementUncertainties() const {
  return aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Data(channels_);
}
const Eigen::VectorXd& VisualFrame::getKeypointOrientations() const {
  return aslam::channels::get_VISUAL_KEYPOINT_ORIENTATIONS_Data(channels_);
}
const Eigen::VectorXd& VisualFrame::getKeypointScores() const {
  return aslam::channels::get_VISUAL_KEYPOINT_SCORES_Data(channels_);
}
const Eigen::VectorXd& VisualFrame::getKeypointScales() const {
  return aslam::channels::get_VISUAL_KEYPOINT_SCALES_Data(channels_);
}
const Eigen::Matrix3Xd& VisualFrame::getKeypoint3DPositions() const {
  return aslam::channels::get_VISUAL_KEYPOINT_3D_POSITIONS_Data(channels_);
}
const Eigen::VectorXi& VisualFrame::getKeypointTimeOffsets() const {
  return aslam::channels::get_VISUAL_KEYPOINT_TIME_OFFSETS_Data(channels_);
}
const VisualFrame::DescriptorsT& VisualFrame::getDescriptors(size_t index) const {
  const std::vector<VisualFrame::DescriptorsT>& data =
      aslam::channels::get_DESCRIPTORS_Data(channels_);
  CHECK_LT(index, data.size());
  return data[index];
}
const Eigen::VectorXi& VisualFrame::getTrackIds() const {
  return aslam::channels::get_TRACK_IDS_Data(channels_);
}
const cv::Mat& VisualFrame::getRawImage() const {
  return aslam::channels::get_RAW_IMAGE_Data(channels_);
}

void VisualFrame::releaseRawImage() {
  aslam::channels::remove_RAW_IMAGE_Channel(&channels_);
}

Eigen::Matrix2Xd* VisualFrame::getKeypointMeasurementsMutable() {
  Eigen::Matrix2Xd& keypoints =
      aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENTS_Data(channels_);
  return &keypoints;
}
Eigen::VectorXd* VisualFrame::getKeypointMeasurementUncertaintiesMutable() {
  Eigen::VectorXd& uncertainties =
      aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Data(channels_);
  return &uncertainties;
}
Eigen::VectorXd* VisualFrame::getKeypointScalesMutable() {
  Eigen::VectorXd& scales =
      aslam::channels::get_VISUAL_KEYPOINT_SCALES_Data(channels_);
  return &scales;
}
Eigen::VectorXd* VisualFrame::getKeypointOrientationsMutable() {
  Eigen::VectorXd& orientations =
      aslam::channels::get_VISUAL_KEYPOINT_ORIENTATIONS_Data(channels_);
  return &orientations;
}
Eigen::VectorXd* VisualFrame::getKeypointScoresMutable() {
  Eigen::VectorXd& scores =
      aslam::channels::get_VISUAL_KEYPOINT_SCORES_Data(channels_);
  return &scores;
}
Eigen::Matrix3Xd* VisualFrame::getKeypoint3DPositionsMutable() {
  Eigen::Matrix3Xd& positions =
      aslam::channels::get_VISUAL_KEYPOINT_3D_POSITIONS_Data(channels_);
  return &positions;
}
Eigen::VectorXi* VisualFrame::getKeypointTimeOffsetsMutable() {
  Eigen::VectorXi& time_offsets =
      aslam::channels::get_VISUAL_KEYPOINT_TIME_OFFSETS_Data(channels_);
  return &time_offsets;
}
VisualFrame::DescriptorsT* VisualFrame::getDescriptorsMutable(size_t index) {
  std::vector<VisualFrame::DescriptorsT>& data =
      aslam::channels::get_DESCRIPTORS_Data(channels_);
  CHECK_LT(index, data.size());
  return &data[index];
}
Eigen::VectorXi* VisualFrame::getTrackIdsMutable() {
  Eigen::VectorXi& track_ids =
      aslam::channels::get_TRACK_IDS_Data(channels_);
  return &track_ids;
}
cv::Mat* VisualFrame::getRawImageMutable() {
  cv::Mat& image =
      aslam::channels::get_RAW_IMAGE_Data(channels_);
  return &image;
}

const Eigen::Block<const Eigen::Matrix2Xd, 2, 1>
VisualFrame::getKeypointMeasurement(size_t index) const {
  const Eigen::Matrix2Xd& keypoints =
      aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENTS_Data(channels_);
  CHECK_LT(static_cast<int>(index), keypoints.cols());
  return keypoints.block<2, 1>(0, index);
}
double VisualFrame::getKeypointMeasurementUncertainty(size_t index) const {
  const Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Data(channels_);
  CHECK_LT(static_cast<int>(index), data.rows());
  return data.coeff(index, 0);
}
double VisualFrame::getKeypointOrientation(size_t index) const {
  const Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_ORIENTATIONS_Data(channels_);
  CHECK_LT(static_cast<int>(index), data.rows());
  return data.coeff(index, 0);
}
double VisualFrame::getKeypointScore(size_t index) const {
  const Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_SCORES_Data(channels_);
  CHECK_LT(static_cast<int>(index), data.rows());
  return data.coeff(index, 0);
}
double VisualFrame::getKeypointScale(size_t index) const {
  const Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_SCALES_Data(channels_);
  CHECK_LT(static_cast<int>(index), data.rows());
  return data.coeff(index, 0);
}
const Eigen::Block<const Eigen::Matrix3Xd, 3, 1>
VisualFrame::getKeypoint3DPosition(size_t index) const {
  const Eigen::Matrix3Xd& data =
      aslam::channels::get_VISUAL_KEYPOINT_3D_POSITIONS_Data(channels_);
  CHECK_LT(static_cast<int>(index), data.cols());
  return data.block<3, 1>(0, index);
}
int VisualFrame::getKeypointTimeOffset(size_t index) const {
  const Eigen::VectorXi& data =
      aslam::channels::get_VISUAL_KEYPOINT_TIME_OFFSETS_Data(channels_);
  CHECK_LT(static_cast<int>(index), data.rows());
  return data.coeff(index, 0);
}

size_t getDescriptorBlockForIndex(
    const std::vector<VisualFrame::DescriptorsT>& descriptors, size_t *index) {
  for (size_t block = 0u; block < descriptors.size(); ++block) {
    const size_t num_descriptors = static_cast<int>(descriptors[block].cols());
    if (*index < num_descriptors) {
      return block;
    }
    *index -= num_descriptors;
  }

  LOG(FATAL) << "Descriptor index out of range";
}

const unsigned char* VisualFrame::getDescriptor(size_t index) const {
  const std::vector<VisualFrame::DescriptorsT>& descriptors =
      aslam::channels::get_DESCRIPTORS_Data(channels_);
  size_t block = getDescriptorBlockForIndex(descriptors, &index);
  return &descriptors[block].coeffRef(0, index);
}

int VisualFrame::getTrackId(size_t index) const {
  const Eigen::VectorXi& track_ids =
      aslam::channels::get_TRACK_IDS_Data(channels_);
  CHECK_LT(static_cast<int>(index), track_ids.rows());
  return track_ids.coeff(index, 0);
}

void VisualFrame::setKeypointMeasurements(
    const Eigen::Matrix2Xd& keypoints_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_MEASUREMENTS_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_MEASUREMENTS_Channel(&channels_);
  }
  Eigen::Matrix2Xd& keypoints =
      aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENTS_Data(channels_);
  keypoints = keypoints_new;
}
void VisualFrame::setKeypointMeasurementUncertainties(
    const Eigen::VectorXd& uncertainties_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Channel(&channels_);
  }
  Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Data(channels_);
  data = uncertainties_new;
}
void VisualFrame::setKeypointOrientations(
    const Eigen::VectorXd& orientations_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_ORIENTATIONS_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_ORIENTATIONS_Channel(&channels_);
  }
  Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_ORIENTATIONS_Data(channels_);
  data = orientations_new;
}
void VisualFrame::setKeypointScores(
    const Eigen::VectorXd& scores_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_SCORES_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_SCORES_Channel(&channels_);
  }
  Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_SCORES_Data(channels_);
  data = scores_new;
}
void VisualFrame::setKeypointScales(
    const Eigen::VectorXd& scales_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_SCALES_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_SCALES_Channel(&channels_);
  }
  Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_SCALES_Data(channels_);
  data = scales_new;
}
void VisualFrame::setKeypoint3DPositions(
    const Eigen::Matrix3Xd& positions_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_3D_POSITIONS_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_3D_POSITIONS_Channel(&channels_);
  }
  Eigen::Matrix3Xd& positions =
      aslam::channels::get_VISUAL_KEYPOINT_3D_POSITIONS_Data(channels_);
  positions = positions_new;
}
void VisualFrame::setKeypointTimeOffsets(const Eigen::VectorXi& offsets_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_TIME_OFFSETS_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_TIME_OFFSETS_Channel(&channels_);
  }
  Eigen::VectorXi& offsets =
      aslam::channels::get_VISUAL_KEYPOINT_TIME_OFFSETS_Data(channels_);
  offsets = offsets_new;
}

template <typename Derived>
void VisualFrame::setDescriptors(
    const Derived& descriptors_new, size_t index, int descriptor_type) {
  if (!aslam::channels::has_DESCRIPTORS_Channel(channels_)) {
    aslam::channels::add_DESCRIPTORS_Channel(&channels_);
    aslam::channels::add_DESCRIPTOR_TYPES_Channel(&channels_);
  }
  std::vector<VisualFrame::DescriptorsT>& descriptors =
      aslam::channels::get_DESCRIPTORS_Data(channels_);
  Eigen::VectorXi& descriptor_types =
      aslam::channels::get_DESCRIPTOR_TYPES_Data(channels_);
  CHECK_EQ(descriptors.size(), static_cast<size_t>(descriptor_types.size()));

  if (descriptors.size() == 0u) {
    descriptors.emplace_back(descriptors_new);
    descriptor_types.resize(1);
    descriptor_types(0) = descriptor_type;
  } else {
    CHECK_LT(index, descriptors.size());
    descriptors[index] = descriptors_new;
    descriptor_types(index) = descriptor_type;
  }
}
template void VisualFrame::setDescriptors(
    const DescriptorsT& descriptors_new, size_t index, int descriptor_type);
template void VisualFrame::setDescriptors(
    const Eigen::Map<const DescriptorsT>& descriptors_new, size_t index,
    int descriptor_type);

void VisualFrame::setTrackIds(const Eigen::VectorXi& track_ids_new) {
  if (!aslam::channels::has_TRACK_IDS_Channel(channels_)) {
    aslam::channels::add_TRACK_IDS_Channel(&channels_);
  }
  Eigen::VectorXi& data =
      aslam::channels::get_TRACK_IDS_Data(channels_);
  data = track_ids_new;
}

void VisualFrame::setRawImage(const cv::Mat& image_new) {
  if (!aslam::channels::has_RAW_IMAGE_Channel(channels_)) {
    aslam::channels::add_RAW_IMAGE_Channel(&channels_);
  }
  cv::Mat& image =
      aslam::channels::get_RAW_IMAGE_Data(channels_);
  image = image_new;
}

void VisualFrame::swapKeypointMeasurements(Eigen::Matrix2Xd* keypoints_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_MEASUREMENTS_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_MEASUREMENTS_Channel(&channels_);
  }
  Eigen::Matrix2Xd& keypoints =
      aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENTS_Data(channels_);
  keypoints.swap(*keypoints_new);
}
void VisualFrame::swapKeypointMeasurementUncertainties(Eigen::VectorXd* uncertainties_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Channel(&channels_);
  }
  Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Data(channels_);
  data.swap(*uncertainties_new);
}
void VisualFrame::swapKeypointOrientations(Eigen::VectorXd* orientations_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_ORIENTATIONS_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_ORIENTATIONS_Channel(&channels_);
  }
  Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_ORIENTATIONS_Data(channels_);
  data.swap(*orientations_new);
}
void VisualFrame::swapKeypointScores(Eigen::VectorXd* scores_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_SCORES_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_SCORES_Channel(&channels_);
  }
  Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_SCORES_Data(channels_);
  data.swap(*scores_new);
}
void VisualFrame::swapKeypointScales(Eigen::VectorXd* scales_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_SCALES_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_SCALES_Channel(&channels_);
  }
  Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_SCALES_Data(channels_);
  data.swap(*scales_new);
}
void VisualFrame::swapKeypoint3DPositions(Eigen::Matrix3Xd* positions_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_3D_POSITIONS_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_3D_POSITIONS_Channel(&channels_);
  }
  Eigen::Matrix3Xd& positions =
      aslam::channels::get_VISUAL_KEYPOINT_3D_POSITIONS_Data(channels_);
  positions.swap(*positions_new);
}
void VisualFrame::swapKeypointTimeOffsets(Eigen::VectorXi* offsets_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_TIME_OFFSETS_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_TIME_OFFSETS_Channel(&channels_);
  }
  Eigen::VectorXi& offsets =
      aslam::channels::get_VISUAL_KEYPOINT_TIME_OFFSETS_Data(channels_);
  offsets.swap(*offsets_new);
}
int VisualFrame::swapDescriptors(
    DescriptorsT* descriptors_new, size_t index, int descriptor_type) {
  CHECK_NOTNULL(descriptors_new);
  if (!aslam::channels::has_DESCRIPTORS_Channel(channels_)) {
    aslam::channels::add_DESCRIPTORS_Channel(&channels_);
    aslam::channels::add_DESCRIPTOR_TYPES_Channel(&channels_);
  }
  std::vector<VisualFrame::DescriptorsT>& descriptors =
      aslam::channels::get_DESCRIPTORS_Data(channels_);
  Eigen::VectorXi& descriptor_types =
      aslam::channels::get_DESCRIPTOR_TYPES_Data(channels_);
  CHECK_EQ(descriptors.size(), static_cast<size_t>(descriptor_types.size()));

  if (descriptors.size() == 0u) {
    descriptors.emplace_back();
    descriptor_types.resize(1);
  }
  CHECK(index < descriptors.size());
  descriptors[index].swap(*descriptors_new);
  std::swap(descriptor_type, descriptor_types(index));
  return descriptor_type;
}

void VisualFrame::swapTrackIds(Eigen::VectorXi* track_ids_new) {
  CHECK_NOTNULL(track_ids_new);
  if (!aslam::channels::has_TRACK_IDS_Channel(channels_)) {
    aslam::channels::add_TRACK_IDS_Channel(&channels_);
  }
  Eigen::VectorXi& track_ids = aslam::channels::get_TRACK_IDS_Data(channels_);
  track_ids.swap(*track_ids_new);
}

void VisualFrame::clearKeypointChannels() {
  Eigen::Matrix2Xd zero_keypoints;
  setKeypointMeasurements(zero_keypoints);

  Eigen::VectorXi zero_vector_int = Eigen::VectorXi::Zero(zero_keypoints.cols());
  setTrackIds(zero_vector_int);

  Eigen::VectorXd zero_vector_double = Eigen::VectorXd::Zero(zero_keypoints.cols());
  setKeypointMeasurementUncertainties(zero_vector_double);
  setKeypointOrientations(zero_vector_double);
  setKeypointScores(zero_vector_double);
  setKeypointScales(zero_vector_double);
  setDescriptors(aslam::VisualFrame::DescriptorsT());
}

const Camera::ConstPtr VisualFrame::getCameraGeometry() const {
  return camera_geometry_;
}

void VisualFrame::setCameraGeometry(const Camera::ConstPtr& camera) {
  camera_geometry_ = camera;
}

const Camera::ConstPtr VisualFrame::getRawCameraGeometry() const {
  return raw_camera_geometry_;
}

void VisualFrame::setRawCameraGeometry(const Camera::ConstPtr& camera) {
  raw_camera_geometry_ = camera;
}

void VisualFrame::print(std::ostream& out, const std::string& label) const {
  if(label.size() > 0) {
    out << label << std::endl;
  }
  out << "VisualFrame(" << this->id_ << ")" << std::endl;
  out << "  timestamp:          " << this->timestamp_nanoseconds_ << std::endl;
  if(camera_geometry_) {
    camera_geometry_->printParameters(out, "  VisualFrame::camera");
  } else {
    out << "  VisualFrame::camera is NULL" << std::endl;
  }
  channels_.printParameters(out);
}

aslam::ProjectionResult VisualFrame::getKeypointInRawImageCoordinates(
    size_t keypoint_idx, Eigen::Vector2d* keypoint_raw_coordinates) const {
  CHECK_NOTNULL(keypoint_raw_coordinates);
  const Eigen::Vector2d& keypoint = getKeypointMeasurement(keypoint_idx);
  return toRawImageCoordinates(keypoint, keypoint_raw_coordinates);
}

aslam::ProjectionResult VisualFrame::toRawImageCoordinates(
    const Eigen::Vector2d& keypoint, Eigen::Vector2d* out_image_coordinates) const {
  CHECK_NOTNULL(out_image_coordinates);
  Eigen::Vector3d bearing;
  // Creating a bearing vector from the transformed camera, then projecting this
  // bearing should recover the raw image coordinates.
  bool success = camera_geometry_->backProject3(keypoint, &bearing);
  if(success) {
    return raw_camera_geometry_->project3(bearing, out_image_coordinates );
  } else {
    return ProjectionResult::PROJECTION_INVALID;
  }
}

void VisualFrame::toRawImageCoordinatesVectorized(
    const Eigen::Matrix2Xd& keypoints, Eigen::Matrix2Xd* out_image_coordinates,
    std::vector<aslam::ProjectionResult>* results) const {
  CHECK_NOTNULL(out_image_coordinates);
  CHECK_NOTNULL(results);
  Eigen::Matrix3Xd bearings;
  std::vector<unsigned char> success;
  camera_geometry_->backProject3Vectorized(keypoints, &bearings, &success);
  raw_camera_geometry_->project3Vectorized(bearings, out_image_coordinates, results);
  for(size_t i = 0; i < success.size(); ++i) {
    if(!success[i]){
      (*results)[i] = ProjectionResult::PROJECTION_INVALID;
    }
  }
}

size_t VisualFrame::getDescriptorSizeBytes(size_t index) const {
  return getDescriptors(index).rows() * sizeof(DescriptorsT::Scalar);
}

Eigen::Matrix3Xd VisualFrame::getNormalizedBearingVectors(
    const std::vector<size_t>& keypoint_indices, int descriptor_type,
    std::vector<unsigned char>* backprojection_success) const {
  CHECK_NOTNULL(backprojection_success);
  if (keypoint_indices.empty()) {
    backprojection_success->clear();
    return Eigen::Matrix3Xd(3, 0);
  }

  const aslam::Camera& camera = *CHECK_NOTNULL(getCameraGeometry().get());
  const Eigen::Block<const Eigen::Matrix2Xd> keypoints = 
      getKeypointMeasurementsOfType(descriptor_type);
  const size_t num_keypoints = getNumKeypointMeasurementsOfType(descriptor_type);

  Eigen::Matrix2Xd keypoints_reduced;
  keypoints_reduced.resize(Eigen::NoChange, keypoint_indices.size());

  size_t list_idx = 0;
  for (const size_t keypoint_idx : keypoint_indices) {
    CHECK_LE(keypoint_idx, num_keypoints);
    keypoints_reduced.col(list_idx++) = keypoints.col(keypoint_idx);
  }

  Eigen::Matrix3Xd points_3d;
  camera.backProject3Vectorized(keypoints_reduced, &points_3d, backprojection_success);
  return points_3d.colwise().normalized();
}

VisualFrame::Ptr VisualFrame::createEmptyTestVisualFrame(const aslam::Camera::ConstPtr& camera,
                                                         int64_t timestamp_nanoseconds) {
  aslam::VisualFrame::Ptr frame(new aslam::VisualFrame);
  frame->setCameraGeometry(camera);
  frame->setTimestampNanoseconds(timestamp_nanoseconds);
  Eigen::Matrix2Xd keypoint_measurements = Eigen::Matrix2Xd::Zero(2, 0);
  frame->swapKeypointMeasurements(&keypoint_measurements);
  Eigen::VectorXd keypoint_uncertainties = Eigen::VectorXd::Zero(0);
  frame->swapKeypointMeasurementUncertainties(&keypoint_uncertainties);
  aslam::VisualFrame::DescriptorsT descriptors = aslam::VisualFrame::DescriptorsT::Zero(48, 0);
  frame->swapDescriptors(&descriptors);
  aslam::FrameId id;
  generateId(&id);
  frame->setId(id);
  return frame;
}

void VisualFrame::discardUntrackedObservations(
    std::vector<size_t>* discarded_indices) {
  CHECK_NOTNULL(discarded_indices)->clear();
  CHECK(hasTrackIds());
  const Eigen::VectorXi& track_ids = getTrackIds();
  const int original_count = track_ids.rows();
  discarded_indices->reserve(original_count);
  for (int i = 0; i < original_count; ++i) {
    if (track_ids(i) < 0) {
      discarded_indices->emplace_back(i);
    }
  }
  if (discarded_indices->empty()) {
    return;
  }

  if (hasKeypointMeasurements()) {
    common::stl_helpers::eraseIndicesFromContainer(
        *discarded_indices, original_count, getKeypointMeasurementsMutable());
  }
  if (hasKeypointMeasurementUncertainties()) {
    common::stl_helpers::eraseIndicesFromContainer(
        *discarded_indices, original_count,
        getKeypointMeasurementUncertaintiesMutable());
  }
  if (hasKeypointOrientations()) {
    common::stl_helpers::eraseIndicesFromContainer(
        *discarded_indices, original_count, getKeypointOrientationsMutable());
  }
  if (hasKeypointScores()) {
    common::stl_helpers::eraseIndicesFromContainer(
        *discarded_indices, original_count, getKeypointScoresMutable());
  }
  if (hasKeypointScales()) {
    common::stl_helpers::eraseIndicesFromContainer(
        *discarded_indices, original_count, getKeypointScalesMutable());
  }
  if (hasDescriptors()) {
    common::stl_helpers::OneDimensionAdapter<unsigned char,
    common::stl_helpers::kColumns> adapter(getDescriptorsMutable());
    common::stl_helpers::eraseIndicesFromContainer(
        *discarded_indices, original_count, &adapter);
  }
  common::stl_helpers::eraseIndicesFromContainer(
      *discarded_indices, original_count, getTrackIdsMutable());
}

void VisualFrame::extendKeypointMeasurements(
    const Eigen::Matrix2Xd& keypoints_new) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_MEASUREMENTS_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_MEASUREMENTS_Channel(&channels_);
  }
  Eigen::Matrix2Xd& keypoints =
      aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENTS_Data(channels_);

  size_t num_keypoints = keypoints.cols();
  keypoints.conservativeResize(Eigen::NoChange, num_keypoints + keypoints_new.cols());
  keypoints.block(0, num_keypoints, 2, keypoints_new.cols()) = keypoints_new;
}

template <typename Derived>
void extendDataVector(
    Eigen::Matrix<Derived, Eigen::Dynamic, 1>* data,
    const Eigen::Matrix<Derived, Eigen::Dynamic, 1>& data_new,
    size_t num_keypoints, Derived default_value) {
  const size_t num_data = static_cast<size_t>(data->size() + data_new.size());
  CHECK_GE(num_keypoints, num_data);
  const size_t num_padding = num_keypoints - num_data;
  const size_t original_size = data->size();
  data->conservativeResize(num_keypoints);

  // Check if we have to pad with the default value in case the previous set
  // of keypoints are missing this property
  if (num_padding > 0) {
    data->segment(original_size, num_padding).setConstant(default_value);
  }

  data->segment(original_size + num_padding, data_new.size()) = data_new;
}

void VisualFrame::extendKeypointMeasurementUncertainties(
    const Eigen::VectorXd& uncertainties_new, double default_value) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Channel(&channels_);
  }
  Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Data(channels_);
  extendDataVector<double>(
      &data, uncertainties_new, getNumKeypointMeasurements(), default_value);
}

void VisualFrame::extendKeypointOrientations(
    const Eigen::VectorXd& orientations_new, double default_value) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_ORIENTATIONS_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_ORIENTATIONS_Channel(&channels_);
  }
  Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_ORIENTATIONS_Data(channels_);
  extendDataVector<double>(
      &data, orientations_new, getNumKeypointMeasurements(), default_value);
}

void VisualFrame::extendKeypointScores(
    const Eigen::VectorXd& scores_new, double default_value) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_SCORES_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_SCORES_Channel(&channels_);
  }
  Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_SCORES_Data(channels_);
  extendDataVector<double>(
      &data, scores_new, getNumKeypointMeasurements(), default_value);
}

void VisualFrame::extendKeypointScales(
    const Eigen::VectorXd& scales_new, double default_value) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_SCALES_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_SCALES_Channel(&channels_);
  }
  Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_SCALES_Data(channels_);
  extendDataVector<double>(
      &data, scales_new, getNumKeypointMeasurements(), default_value);
}

void VisualFrame::extendKeypoint3DPositions(
    const Eigen::Matrix3Xd& positions_new, double default_value) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_3D_POSITIONS_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_3D_POSITIONS_Channel(&channels_);
  }
  Eigen::Matrix3Xd& positions =
      aslam::channels::get_VISUAL_KEYPOINT_3D_POSITIONS_Data(channels_);

  // TODO(smauq): Template this nicely
  size_t num_keypoints = getNumKeypointMeasurements();
  const size_t num_positions =
    static_cast<size_t>(positions.cols() + positions_new.cols());
  CHECK_GE(num_keypoints, num_positions);
  const size_t num_padding = num_keypoints - num_positions;
  const size_t original_size = positions.cols();
  positions.conservativeResize(Eigen::NoChange, num_keypoints);

  // Check if we have to pad with the default value in case the previous set
  // of keypoints are missing this property
  if (num_padding > 0) {
    positions.block(0, original_size, 3, num_padding).setConstant(default_value);
  }

  positions.block(0, original_size + num_padding, 3, positions_new.cols()) = positions_new;
}

void VisualFrame::extendKeypointTimeOffsets(
    const Eigen::VectorXi& offsets_new, int default_value) {
  if (!aslam::channels::has_VISUAL_KEYPOINT_TIME_OFFSETS_Channel(channels_)) {
    aslam::channels::add_VISUAL_KEYPOINT_TIME_OFFSETS_Channel(&channels_);
  }
  Eigen::VectorXi& data =
      aslam::channels::get_VISUAL_KEYPOINT_TIME_OFFSETS_Data(channels_);
  extendDataVector<int>(
      &data, offsets_new, getNumKeypointMeasurements(), default_value);
}

template <typename Derived>
void VisualFrame::extendDescriptors(
    const Derived& descriptors_new, int descriptor_type) {
  if (!aslam::channels::has_DESCRIPTORS_Channel(channels_)) {
    aslam::channels::add_DESCRIPTORS_Channel(&channels_);
    aslam::channels::add_DESCRIPTOR_TYPES_Channel(&channels_);
  }
  std::vector<VisualFrame::DescriptorsT>& descriptors =
      aslam::channels::get_DESCRIPTORS_Data(channels_);
  Eigen::VectorXi& descriptor_types =
      aslam::channels::get_DESCRIPTOR_TYPES_Data(channels_);
  CHECK_EQ(descriptors.size(), static_cast<size_t>(descriptor_types.size()));

  descriptors.emplace_back(descriptors_new);
  const size_t num_descriptor_types = descriptor_types.size();
  descriptor_types.conservativeResize(num_descriptor_types + 1);
  descriptor_types(num_descriptor_types) = descriptor_type;
}
template void VisualFrame::extendDescriptors(
    const DescriptorsT& descriptors_new, int descriptor_type);
template void VisualFrame::extendDescriptors(
    const Eigen::Map<const DescriptorsT>& descriptors_new, int descriptor_type);

void VisualFrame::extendTrackIds(
    const Eigen::VectorXi& track_ids_new, int default_value) {
  if (!aslam::channels::has_TRACK_IDS_Channel(channels_)) {
    aslam::channels::add_TRACK_IDS_Channel(&channels_);
  }
  Eigen::VectorXi& data =
      aslam::channels::get_TRACK_IDS_Data(channels_);
  extendDataVector<int>(
      &data, track_ids_new, getNumKeypointMeasurements(), default_value);
}

size_t VisualFrame::getNumDescriptors() const {
  if (!hasDescriptors()) {
    return 0;
  }

  size_t num_descriptors = 0;
  const std::vector<VisualFrame::DescriptorsT>& descriptors =
      aslam::channels::get_DESCRIPTORS_Data(channels_);
  for (size_t i = 0; i < descriptors.size(); ++i) {
    num_descriptors += static_cast<size_t>(descriptors[i].cols());
  }

  return num_descriptors;
}

void VisualFrame::serializeDescriptorsToString(std::string* descriptors_string) const {
  CHECK(aslam::channels::has_DESCRIPTORS_Channel(channels_));
  aslam::channels::serialize_DESCRIPTORS_Channel(channels_, descriptors_string);
}

void VisualFrame::deserializeDescriptorsFromString(const std::string& descriptors_string) {
  if (!aslam::channels::has_DESCRIPTORS_Channel(channels_)) {
    aslam::channels::add_DESCRIPTORS_Channel(&channels_);
  }
  aslam::channels::deserialize_DESCRIPTORS_Channel(channels_, descriptors_string);
}

bool VisualFrame::hasDescriptorType(int descriptor_type) const {
  if (!hasDescriptors()) {
    return false;
  }

  const Eigen::VectorXi& descriptor_types =
      aslam::channels::get_DESCRIPTOR_TYPES_Data(channels_);
  for (int block = 0; block < descriptor_types.size(); ++block) {
    if (descriptor_types.coeff(block) == descriptor_type) {
      return true;
    }
  }
  return false;
}

void VisualFrame::setDescriptorTypes(const Eigen::VectorXi& descriptor_types) {
  const std::vector<VisualFrame::DescriptorsT>& descriptors =
      aslam::channels::get_DESCRIPTORS_Data(channels_);
  CHECK_EQ(descriptors.size(), static_cast<size_t>(descriptor_types.size()));

  if (!aslam::channels::has_DESCRIPTOR_TYPES_Channel(channels_)) {
    aslam::channels::add_DESCRIPTOR_TYPES_Channel(&channels_);
  }
  Eigen::VectorXi& data =
      aslam::channels::get_DESCRIPTOR_TYPES_Data(channels_);
  data = descriptor_types;
}

const Eigen::VectorXi& VisualFrame::getDescriptorTypes() const {
  return aslam::channels::get_DESCRIPTOR_TYPES_Data(channels_);
}

int VisualFrame::getDescriptorType(size_t index) const {
  std::vector<VisualFrame::DescriptorsT>& descriptors =
      aslam::channels::get_DESCRIPTORS_Data(channels_);
  size_t block = getDescriptorBlockForIndex(descriptors, &index);
  return getDescriptorBlockType(block);
}

int VisualFrame::getDescriptorBlockType(size_t block) const {
  Eigen::VectorXi& descriptor_types =
      aslam::channels::get_DESCRIPTOR_TYPES_Data(channels_);
  CHECK_LT(block, static_cast<size_t>(descriptor_types.size()));
  return descriptor_types.coeff(block);
}

size_t VisualFrame::getDescriptorTypeBlock(int descriptor_type) const {
  const Eigen::VectorXi& descriptor_types =
      aslam::channels::get_DESCRIPTOR_TYPES_Data(channels_);
  for (int block = 0; block < descriptor_types.size(); ++block) {
    if (descriptor_types.coeff(block) == descriptor_type) {
      return block;
    }
  }
  LOG(FATAL)
      << "Descriptor type " << descriptor_type << " does not exist in frame "
      << " id " << id_ << " at timestamp " << timestamp_nanoseconds_ << " ns.";
}

size_t VisualFrame::getNumKeypointMeasurementsOfType(int descriptor_type) const {
  if (!hasKeypointMeasurements()) {
    return 0u;
  }

  const std::vector<VisualFrame::DescriptorsT>& descriptors =
      aslam::channels::get_DESCRIPTORS_Data(channels_);
  if (!hasDescriptorType(descriptor_type)) {
    return 0u;
  }
  const size_t block = getDescriptorTypeBlock(descriptor_type);
  CHECK_LT(block, descriptors.size());
  return descriptors[block].cols();
}

void VisualFrame::getDescriptorBlockTypeStartAndSize(
  int descriptor_type, size_t *start, size_t *size) const {
  CHECK_NOTNULL(start);
  CHECK_NOTNULL(size);

  const std::vector<VisualFrame::DescriptorsT>& descriptors =
      aslam::channels::get_DESCRIPTORS_Data(channels_);
  const size_t block = getDescriptorTypeBlock(descriptor_type);

  *start = 0;
  for (size_t i = 0u; i < block; i++) {
    *start += descriptors[i].cols();
  }
  *size = descriptors[block].cols();
}

const Eigen::Block<const Eigen::Matrix2Xd> VisualFrame::getKeypointMeasurementsOfType(
    int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  const Eigen::Matrix2Xd& keypoints =
      aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENTS_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(keypoints.cols()));
  return keypoints.block(0, start, 2, size);
}
const Eigen::VectorBlock<const Eigen::VectorXd>
VisualFrame::getKeypointMeasurementUncertaintiesOfType(int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  const Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(data.size()));
  return data.segment(start, size);
}
const Eigen::VectorBlock<const Eigen::VectorXd>
VisualFrame::getKeypointOrientationsOfType(int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  const Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_ORIENTATIONS_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(data.size()));
  return data.segment(start, size);
}
const Eigen::VectorBlock<const Eigen::VectorXd> VisualFrame::getKeypointScoresOfType(
    int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  const Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_SCORES_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(data.size()));
  return data.segment(start, size);
}
const Eigen::VectorBlock<const Eigen::VectorXd> VisualFrame::getKeypointScalesOfType(
    int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  const Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_SCALES_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(data.size()));
  return data.segment(start, size);
}
const Eigen::Block<const Eigen::Matrix3Xd> VisualFrame::getKeypoint3DPositionsOfType(
    int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  const Eigen::Matrix3Xd& positions =
      aslam::channels::get_VISUAL_KEYPOINT_3D_POSITIONS_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(positions.cols()));
  return positions.block(0, start, 3, size);
}
const Eigen::VectorBlock<const Eigen::VectorXi> VisualFrame::getKeypointTimeOffsetsOfType(
    int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  const Eigen::VectorXi& data =
      aslam::channels::get_VISUAL_KEYPOINT_TIME_OFFSETS_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(data.size()));
  return data.segment(start, size);
}
const VisualFrame::DescriptorsT& VisualFrame::getDescriptorsOfType(int descriptor_type) const {
  const size_t block = getDescriptorTypeBlock(descriptor_type);
  return getDescriptors(block);
}
const Eigen::VectorBlock<const Eigen::VectorXi> VisualFrame::getTrackIdsOfType(
    int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  const Eigen::VectorXi& data =
      aslam::channels::get_TRACK_IDS_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(data.size()));
  return data.segment(start, size);
}

Eigen::VectorBlock<Eigen::VectorXi> VisualFrame::getTrackIdsOfTypeMutable(
    int descriptor_type) {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  Eigen::VectorXi& data =
      aslam::channels::get_TRACK_IDS_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(data.size()));
  return data.segment(start, size);
}

const Eigen::Block<const Eigen::Matrix2Xd, 2, 1> VisualFrame::getKeypointMeasurementOfType(
    size_t index, int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  CHECK_LT(index, size);
  const Eigen::Matrix2Xd& keypoints =
      aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENTS_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(keypoints.cols()));
  return keypoints.block<2, 1>(0, start + index);
}
double VisualFrame::getKeypointMeasurementUncertaintyOfType(
    size_t index, int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  CHECK_LT(index, size);
  const Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_MEASUREMENT_UNCERTAINTIES_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(data.size()));
  return data.coeff(start + index);
}
double VisualFrame::getKeypointOrientationOfType(size_t index, int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  CHECK_LT(index, size);
  const Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_ORIENTATIONS_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(data.size()));
  return data.coeff(start + index);
}
double VisualFrame::getKeypointScoreOfType(size_t index, int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  CHECK_LT(index, size);
  const Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_SCORES_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(data.size()));
  return data.coeff(start + index);
}
double VisualFrame::getKeypointScaleOfType(size_t index, int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  CHECK_LT(index, size);
  const Eigen::VectorXd& data =
      aslam::channels::get_VISUAL_KEYPOINT_SCALES_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(data.size()));
  return data.coeff(start + index);
}
const Eigen::Block<const Eigen::Matrix3Xd, 3, 1> VisualFrame::getKeypoint3DPositionOfType(
    size_t index, int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  CHECK_LT(index, size);
  const Eigen::Matrix3Xd& positions =
      aslam::channels::get_VISUAL_KEYPOINT_3D_POSITIONS_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(positions.cols()));
  return positions.block<3, 1>(0, start + index);
}
int VisualFrame::getKeypointTimeOffsetOfType(size_t index, int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  CHECK_LT(index, size);
  const Eigen::VectorXi& data =
      aslam::channels::get_VISUAL_KEYPOINT_TIME_OFFSETS_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(data.size()));
  return data.coeff(start + index);
}
int VisualFrame::getTrackIdOfType(size_t index, int descriptor_type) const {
  size_t start, size;
  getDescriptorBlockTypeStartAndSize(descriptor_type, &start, &size);
  CHECK_LT(index, size);
  const Eigen::VectorXi& track_ids =
      aslam::channels::get_TRACK_IDS_Data(channels_);
  CHECK_LE(start + size, static_cast<size_t>(track_ids.size()));
  return track_ids.coeff(start + index);
}

size_t VisualFrame::getDescriptorTypeSizeBytes(int descriptor_type) const {
  const size_t block = getDescriptorTypeBlock(descriptor_type);
  return getDescriptors(block).rows() * sizeof(DescriptorsT::Scalar);
}

}  // namespace aslam
