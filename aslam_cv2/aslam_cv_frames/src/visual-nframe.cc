#include <limits>
#include <memory>
#include <vector>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/predicates.h>
#include <aslam/common/time.h>
#include <aslam/common/unique-id.h>
#include <aslam/frames/visual-frame.h>

#include "aslam/frames/visual-nframe.h"

namespace aslam {

VisualNFrame::VisualNFrame(const aslam::NFramesId& id, unsigned int num_frames)
    : id_(id) {
  frames_.resize(num_frames);
}

VisualNFrame::VisualNFrame(const aslam::NFramesId& id, std::shared_ptr<NCamera> ncamera)
    : id_(id),
      camera_rig_(ncamera) {
  CHECK_NOTNULL(camera_rig_.get());
  frames_.resize(ncamera->getNumCameras());
}

VisualNFrame::VisualNFrame(std::shared_ptr<NCamera> ncamera)
    : camera_rig_(ncamera) {
  CHECK_NOTNULL(camera_rig_.get());
  generateId(&id_);
  frames_.resize(ncamera->getNumCameras());
}

VisualNFrame::VisualNFrame(const VisualNFrame& other) {
  *this = other;
}

VisualNFrame& VisualNFrame::operator=(const VisualNFrame& other) {
  id_ = other.id_;
  camera_rig_ = other.camera_rig_;
  frames_.clear();
  frames_.reserve(other.frames_.size());
  for (const VisualFrame::Ptr& other_frame : other.frames_) {
    if (other_frame) {
      frames_.emplace_back(new VisualFrame(*other_frame.get()));
    } else {
      frames_.emplace_back(nullptr);
    }
  }
  return *this;
}

const NCamera& VisualNFrame::getNCamera() const {
  CHECK_NOTNULL(camera_rig_.get());
  return *camera_rig_;
}

NCamera::Ptr VisualNFrame::getNCameraShared() {
  return camera_rig_;
}

NCamera::ConstPtr VisualNFrame::getNCameraShared() const {
  return camera_rig_;
}

void VisualNFrame::setNCameras(NCamera::Ptr ncamera) {
  CHECK(ncamera != nullptr);
  camera_rig_ = ncamera;
  CHECK_EQ(frames_.size(), camera_rig_->numCameras()) << "Number of cameras "
      << "in camera system does not match the current number of frames.";

  // Also assign the camera to the existing non-NULL frames.
  for (unsigned int i = 0; i < frames_.size(); ++i) {
    if (frames_[i] != nullptr) {
      frames_[i]->setCameraGeometry(camera_rig_->getCameraShared(i));
      CHECK_EQ(frames_[i]->getCameraGeometry().get(),
               camera_rig_->getCameraShared(i).get());
    }
  }
}

const VisualFrame& VisualNFrame::getFrame(size_t frame_index) const {
  CHECK(isFrameSet(frame_index));
  return *frames_[frame_index];
}

VisualFrame::Ptr VisualNFrame::getFrameShared(size_t frame_index) {
  CHECK_LT(frame_index, frames_.size());
  return frames_[frame_index];
}

VisualFrame::ConstPtr VisualNFrame::getFrameShared(size_t frame_index) const {
  CHECK_LT(frame_index, frames_.size());
  return frames_[frame_index];
}

size_t VisualNFrame::getNumFrames() const {
  return frames_.size();
}

size_t VisualNFrame::getNumCameras() const {
  CHECK_NOTNULL(camera_rig_.get());
  return camera_rig_->getNumCameras();
}

const Transformation& VisualNFrame::get_T_C_B(size_t camera_index) const {
  CHECK_NOTNULL(camera_rig_.get());
  return camera_rig_->get_T_C_B(camera_index);
}

const Camera& VisualNFrame::getCamera(size_t camera_index) const {
  CHECK_NOTNULL(camera_rig_.get());
  return camera_rig_->getCamera(camera_index);
}

const CameraId& VisualNFrame::getCameraId(size_t camera_index) const {
  CHECK_NOTNULL(camera_rig_.get());
  return camera_rig_->getCameraId(camera_index);
}

bool VisualNFrame::hasCameraWithId(const CameraId& id) const {
  CHECK_NOTNULL(camera_rig_.get());
  return camera_rig_->hasCameraWithId(id);
}

size_t VisualNFrame::getCameraIndex(const CameraId& id) const {
  CHECK_NOTNULL(camera_rig_.get());
  return camera_rig_->getCameraIndex(id);
}

void VisualNFrame::setFrame(size_t frame_index, VisualFrame::Ptr frame) {
  CHECK_LT(frame_index, frames_.size());
  CHECK(frame != nullptr);
  if (camera_rig_ != nullptr) {
    CHECK_EQ(&camera_rig_->getCamera(frame_index), frame->getCameraGeometry().get());
  }
  frames_[frame_index] = frame;
}

void VisualNFrame::unSetFrame(size_t frame_index) {
  CHECK_LT(frame_index, frames_.size());
  frames_[frame_index] = nullptr;
}

bool VisualNFrame::isFrameSet(size_t frame_index) const {
  CHECK_LT(frame_index, frames_.size());
  return static_cast<bool>(frames_[frame_index]);
}

bool VisualNFrame::areAllFramesSet() const {
  for (size_t frame_idx = 0u; frame_idx < frames_.size(); ++frame_idx) {
    if (!static_cast<bool>(frames_[frame_idx])) {
      return false;
    }
  }
  return true;
}

bool VisualNFrame::isFrameValid(size_t frame_index) const {
  CHECK_LT(frame_index, frames_.size());
  CHECK(isFrameSet(frame_index));
  return frames_[frame_index]->isValid();
}

int64_t VisualNFrame::getMinTimestampNanoseconds() const {
  int64_t min_timestamp_nanoseconds = std::numeric_limits<int64_t>::max();
  for (size_t camera_idx = 0; camera_idx < getNumCameras(); ++camera_idx) {
    if (isFrameSet(camera_idx)) {
      const int64_t timestamp_frame_nanoseconds = getFrame(camera_idx).getTimestampNanoseconds();
      if (timestamp_frame_nanoseconds < min_timestamp_nanoseconds)
        min_timestamp_nanoseconds = timestamp_frame_nanoseconds;
    }
  }
  CHECK(aslam::time::isValidTime(min_timestamp_nanoseconds)) << "Time " << min_timestamp_nanoseconds
      << "[ns] is invalid.";
  return min_timestamp_nanoseconds;
}

int64_t VisualNFrame::getMaxTimestampNanoseconds() const {
  int64_t max_timestamp_nanoseconds = aslam::time::getInvalidTime();
  for (size_t camera_idx = 0; camera_idx < getNumCameras(); ++camera_idx) {
    if (isFrameSet(camera_idx)) {
      const int64_t timestamp_frame_nanoseconds = getFrame(camera_idx).getTimestampNanoseconds();
      if (timestamp_frame_nanoseconds > max_timestamp_nanoseconds)
        max_timestamp_nanoseconds = timestamp_frame_nanoseconds;
    }
  }
  CHECK(aslam::time::isValidTime(max_timestamp_nanoseconds));
  return max_timestamp_nanoseconds;
}

bool VisualNFrame::operator==(const VisualNFrame& other) const {
  bool same = true;
  same &= id_ == other.id_;
  same &= aslam::checkSharedEqual(camera_rig_, other.camera_rig_);
  same &= frames_.size() == other.frames_.size();
  if(same) {
    for(size_t i = 0; i < frames_.size(); ++i) {
      same &= *CHECK_NOTNULL(frames_[i].get()) == *CHECK_NOTNULL(other.frames_[i].get());
    }
  }
  return same;
}

bool VisualNFrame::compareWithoutCameraSystem(const VisualNFrame& other) const {
  bool same = true;
  same &= id_ == other.id_;
  same &= frames_.size() == other.frames_.size();
  if(same) {
    for(size_t i = 0; i < frames_.size(); ++i) {
      same &= CHECK_NOTNULL(frames_[i].get())->compareWithoutCameraGeometry(
          *CHECK_NOTNULL(other.frames_[i].get()));
    }
  }
  return same;
}

VisualNFrame::Ptr VisualNFrame::createEmptyTestVisualNFrame(const NCamera::Ptr& ncamera,
                                                            int64_t timestamp_nanoseconds) {
  CHECK(ncamera);
  const size_t kNumFrames = ncamera->getNumCameras();
  aslam::NFramesId id;
  generateId(&id);
  aslam::VisualNFrame::Ptr nframe = aligned_shared<aslam::VisualNFrame>(id, ncamera);
  for (size_t frame_idx = 0; frame_idx < kNumFrames; ++frame_idx) {
    aslam::VisualFrame::Ptr frame =
        VisualFrame::createEmptyTestVisualFrame(ncamera->getCameraShared(frame_idx),
                                                timestamp_nanoseconds);
    nframe->setFrame(frame_idx, frame);
  }
 return nframe;
}

bool VisualNFrame::hasRawImagesInAllFrames() const {
  for (const VisualFrame::ConstPtr& frame : frames_) {
    CHECK(frame);
    if (!frame->hasRawImage()) {
      return false;
    }
  }
  return true;
}

void VisualNFrame::releaseRawImagesOfAllFrames() {
  for (const VisualFrame::Ptr& frame : frames_) {
    CHECK(frame);
    frame->releaseRawImage();
  }
}

void VisualNFrame::clearKeypointChannelsOfAllFrames() {
  for (const VisualFrame::Ptr& frame : frames_) {
    CHECK(frame);
    frame->clearKeypointChannels();
  }
}

} // namespace aslam
