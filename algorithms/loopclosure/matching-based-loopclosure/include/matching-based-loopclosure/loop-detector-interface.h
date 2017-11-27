#ifndef MATCHING_BASED_LOOPCLOSURE_LOOP_DETECTOR_INTERFACE_H_
#define MATCHING_BASED_LOOPCLOSURE_LOOP_DETECTOR_INTERFACE_H_

#include <memory>
#include <vector>

#include <descriptor-projection/descriptor-projection.h>
#include <loopclosure-common/types.h>

#include "matching-based-loopclosure/helpers.h"
#include "matching-based-loopclosure/matching_based_loop_detector.pb.h"

namespace loop_detector {

// Interface for the matching-based loop detector.
class LoopDetector {
 public:
  virtual ~LoopDetector() = default;

  // Find a set of provided images (consisting of projected descriptors), that
  // belong to the same vertex, in the database.
  virtual void Find(
      const loop_closure::ProjectedImagePtrList& projected_image_ptr_list,
      const bool parallelize_if_possible,
      loop_closure::FrameToMatches* frame_matches) const = 0;

  // Add the provided image (consisting of projected descriptors) to the
  // descriptor index backend.
  virtual void Insert(
      const std::shared_ptr<loop_closure::ProjectedImage>&
          projected_image_ptr) = 0;

  // Transforms an image into a set of projected descriptors.
  virtual void ProjectDescriptors(
      const std::vector<aslam::common::FeatureDescriptorConstRef>& descriptors,
      Eigen::MatrixXf* projected_descriptors) const = 0;

  // Transforms an image into a set of projected descriptors.
  virtual void ProjectDescriptors(
      const loop_closure::DescriptorContainer& descriptors,
      Eigen::MatrixXf* projected_descriptors) const = 0;

  virtual void Clear() = 0;
  virtual size_t NumEntries() const = 0;
  virtual int NumDescriptors() const = 0;

  virtual void serialize(
      matching_based_loopclosure::proto::MatchingBasedLoopDetector*
          matching_based_loop_detector) const = 0;
  virtual void deserialize(
      const matching_based_loopclosure::proto::MatchingBasedLoopDetector&
          matching_based_loop_detector) = 0;
};

}  // namespace loop_detector

#endif  // MATCHING_BASED_LOOPCLOSURE_LOOP_DETECTOR_INTERFACE_H_
