#ifndef VI_MAP_VERTEX_INL_H_
#define VI_MAP_VERTEX_INL_H_

#include <string>
#include <vector>

namespace vi_map {

inline const vi_map::MissionId& Vertex::getMissionId() const {
  return mission_id_;
}

inline void Vertex::setMissionId(const vi_map::MissionId& mission_id) {
  mission_id_ = mission_id;
}

inline aslam::VisualNFrame& Vertex::getVisualNFrame() {
  CHECK(n_frame_ != nullptr);
  return *n_frame_;
}
inline const aslam::VisualNFrame& Vertex::getVisualNFrame() const {
  CHECK(n_frame_ != nullptr);
  return *n_frame_;
}
inline aslam::VisualNFrame::Ptr& Vertex::getVisualNFrameShared() {
  return n_frame_;
}
inline aslam::VisualNFrame::ConstPtr Vertex::getVisualNFrameShared() const {
  return n_frame_;
}

inline aslam::VisualFrame& Vertex::getVisualFrame(unsigned int frame_idx) {
  CHECK(n_frame_ != nullptr);
  CHECK(n_frame_->getFrameShared(frame_idx) != nullptr);
  return *(n_frame_->getFrameShared(frame_idx));
}

inline bool Vertex::isVisualFrameSet(unsigned int frame_idx) const {
  CHECK(n_frame_ != nullptr);
  return n_frame_->isFrameSet(frame_idx);
}

inline bool Vertex::isVisualFrameValid(unsigned int frame_idx) const {
  CHECK(n_frame_ != nullptr);
  return n_frame_->isFrameValid(frame_idx);
}

inline aslam::VisualFrame::Ptr Vertex::getVisualFrameShared(
    unsigned int frame_idx) {
  return n_frame_->getFrameShared(frame_idx);
}

inline aslam::VisualFrame::Ptr Vertex::getVisualFrameShared(
    aslam::FrameId frame_id) {
  for (unsigned int i = 0; i < n_frame_->getNumCameras(); ++i) {
    aslam::VisualFrame::Ptr frame = n_frame_->getFrameShared(i);
    if (frame != nullptr && frame->getId() == frame_id) {
      return n_frame_->getFrameShared(i);
    }
  }
  CHECK(false) << "Frame " << frame_id << " not found in vertex " << id();
  return nullptr;
}

inline unsigned int Vertex::getVisualFrameIndex(aslam::FrameId frame_id) const {
  for (unsigned int i = 0; i < n_frame_->getNumCameras(); ++i) {
    aslam::VisualFrame::ConstPtr frame = n_frame_->getFrameShared(i);
    if (frame != nullptr && frame->getId() == frame_id) {
      return i;
    }
  }
  CHECK(false) << "Frame " << frame_id << " not found in vertex " << id();
  return -1;
}

inline const aslam::VisualFrame& Vertex::getVisualFrame(
    unsigned int frame_idx) const {
  CHECK(n_frame_ != nullptr);
  CHECK(n_frame_->isFrameSet(frame_idx));
  return n_frame_->getFrame(frame_idx);
}

inline const aslam::VisualFrame::ConstPtr Vertex::getVisualFrameShared(
    unsigned int frame_idx) const {
  return n_frame_->getFrameShared(frame_idx);
}

inline aslam::NCamera::ConstPtr Vertex::getNCameras() const {
  CHECK(n_frame_ != nullptr);
  return n_frame_->getNCameraShared();
}

inline aslam::Camera::ConstPtr Vertex::getCamera(unsigned int frame_idx) const {
  CHECK(n_frame_ != nullptr);
  CHECK(n_frame_->getNCameraShared() != nullptr);
  return n_frame_->getNCameraShared()->getCameraShared(frame_idx);
}

inline aslam::Camera::Ptr Vertex::getCamera(unsigned int frame_idx) {
  CHECK(n_frame_ != nullptr);
  CHECK(n_frame_->getNCameraShared() != nullptr);
  return n_frame_->getNCameraShared()->getCameraShared(frame_idx);
}

inline void Vertex::setCamera(
    unsigned int frame_idx, const aslam::Camera::Ptr& camera) {
  CHECK(n_frame_ != nullptr);
  CHECK(n_frame_->getNCameraShared() != nullptr);
  n_frame_->getNCameraShared()->setCamera(frame_idx, camera);
}

inline void Vertex::setNCameras(const aslam::NCamera::Ptr& n_cameras) {
  CHECK(n_frame_ != nullptr);
  n_frame_->setNCameras(n_cameras);
}

inline size_t Vertex::numFrames() const {
  CHECK(n_frame_ != nullptr);
  return n_frame_->getNumFrames();
}

inline LandmarkStore& Vertex::getLandmarks() {
  return landmarks_;
}

inline const LandmarkStore& Vertex::getLandmarks() const {
  return landmarks_;
}

inline void Vertex::setLandmarks(const LandmarkStore& landmark_store) {
  landmarks_ = landmark_store;
}

inline void Vertex::forEachUnassociatedKeypoint(
    const unsigned int frame_idx,
    const std::function<void(const int keypoint_index)>& action) const {
  const aslam::VisualFrame& frame = getVisualFrame(frame_idx);
  const int num_keypoint_measurements = frame.getNumKeypointMeasurements();
  for (int i = 0; i < num_keypoint_measurements; ++i) {
    if (!getObservedLandmarkId(frame_idx, i).isValid()) {
      action(i);
    }
  }
}

inline void Vertex::getUnassociatedKeypoints(
    const unsigned int frame_idx, std::vector<int>* result) const {
  CHECK_NOTNULL(result)->clear();
  const aslam::VisualFrame& frame = getVisualFrame(frame_idx);
  const int num_keypoint_measurements = frame.getNumKeypointMeasurements();
  result->reserve(num_keypoint_measurements);
  for (int i = 0; i < num_keypoint_measurements; ++i) {
    if (!getObservedLandmarkId(frame_idx, i).isValid()) {
      result->push_back(i);
    }
  }
}

int64_t Vertex::getMinTimestampNanoseconds() const {
  CHECK(n_frame_);
  return n_frame_->getMinTimestampNanoseconds();
}

inline bool Vertex::operator==(const Vertex& lhs) const {
  return isSameApartFromOutgoingEdges(lhs) &&
         outgoing_edges_ == lhs.outgoing_edges_;
}

inline bool Vertex::operator!=(const Vertex& lhs) const {
  return !operator==(lhs);
}

inline bool Vertex::isSameApartFromOutgoingEdges(const Vertex& lhs) const {
  bool is_same = true;
  is_same &= T_M_I_ == lhs.T_M_I_;
  is_same &= v_M_ == lhs.v_M_;
  is_same &= accel_bias_ == lhs.accel_bias_;
  is_same &= gyro_bias_ == lhs.gyro_bias_;

  is_same &= incoming_edges_ == lhs.incoming_edges_;

  is_same &= id_ == lhs.id_;
  is_same &= mission_id_ == lhs.mission_id_;
  is_same &= static_cast<bool>(n_frame_) == static_cast<bool>(lhs.n_frame_);
  if (n_frame_ && lhs.n_frame_) {
    // Since we store the camera system in the mission, we exclude it from this
    // test.
    is_same &= n_frame_->compareWithoutCameraSystem(*lhs.n_frame_);
  }
  is_same &= landmarks_ == lhs.landmarks_;
  return is_same;
}

}  // namespace vi_map

#endif  // VI_MAP_VERTEX_INL_H_
