#ifndef ASLAM_SERIALIZATION_VISUAL_FRAME_SERIALIZATION_H_
#define ASLAM_SERIALIZATION_VISUAL_FRAME_SERIALIZATION_H_

#include <aslam/cameras/camera.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>

#include "aslam-serialization/visual-frame.pb.h"

namespace aslam {
namespace serialization {

void serializeVisualFrame(
    const aslam::VisualFrame& frame, aslam::proto::VisualFrame* proto);
void deserializeVisualFrame(
    const aslam::proto::VisualFrame& proto, aslam::VisualFrame::Ptr* frame);
void deserializeVisualFrame(
    const aslam::proto::VisualFrame& proto,
    const aslam::Camera::ConstPtr& camera, aslam::VisualFrame::Ptr* frame);

void serializeVisualNFrame(
    const aslam::VisualNFrame& n_frame, aslam::proto::VisualNFrame* proto);
void deserializeVisualNFrame(
    const aslam::proto::VisualNFrame& proto, aslam::VisualNFrame::Ptr* n_frame);
void deserializeVisualNFrame(
    const aslam::proto::VisualNFrame& proto,
    const aslam::NCamera::Ptr& n_camera, aslam::VisualNFrame::Ptr* n_frame);

namespace internal {

void serializeDescriptors(
    const aslam::VisualFrame::DescriptorsT& descriptors,
    aslam::proto::VisualFrame* proto);
void deserializeDescriptors(
    const aslam::proto::VisualFrame& proto,
    aslam::VisualFrame::DescriptorsT* descriptors);

}  // namespace internal

}  // namespace serialization
}  // namespace aslam

#endif  // ASLAM_SERIALIZATION_VISUAL_FRAME_SERIALIZATION_H_
