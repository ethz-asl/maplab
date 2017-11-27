#ifndef SIMULATION_VISUAL_NFRAME_SIMULATOR_H_
#define SIMULATION_VISUAL_NFRAME_SIMULATOR_H_

#include <memory>
#include <mutex>
#include <vector>

#include <aslam/cameras/ncamera.h>
#include <aslam/frames/visual-nframe.h>
#include <maplab-common/macros.h>
#include <vi-map/unique-id.h>

#include "simulation/generic-path-generator.h"

using test_trajectory_gen::VisualInertialPathGenerator;

namespace simulation {

class VisualNFrameSimulator {
 public:
  MAPLAB_POINTER_TYPEDEFS(VisualNFrameSimulator);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VisualNFrameSimulator() = default;
  virtual ~VisualNFrameSimulator() {}

  /// \brief Takes a path generator with a generated path and landmarks and
  /// returns a
  ///        list of Visual NFrames filled with projected landmarks.
  /// @param[in]  timestamps_seconds      Timestamps in seconds.
  /// @param[in]  T_G_Bs                  Transformations taking points from
  /// body frame to
  ///                                     global frame.
  /// @param[in]  G_landmarks             Landmarks in global frame.
  /// @param[in]  camera_rig              A pointer to a valid camera rig. This
  /// rig
  ///                                     will be referenced in all the Visual
  ///                                     N-Frames.
  /// @param[in]  keypoint_sigma_px       The noise sigma on the keypoints in
  /// [px].
  ///                                     This will be used to set the
  ///                                     uncertainty matrix
  ///                                     in the VisualFrame too. So set this
  ///                                     value even if you do
  ///                                     not add any noise.
  /// @param[in]  add_noise_to_keypoints  Say yes or no to adding noise to the
  /// keypoints.
  ///                                     If yes, noise will be sampled acc. to
  ///                                     the
  ///                                     keypoint variance specified above.
  /// @param[in]  num_bits_to_flip        Number of bits to flip wrt. the ground
  /// truth descriptor
  ///                                     when generating keypoints for the
  ///                                     frames.
  /// @param[out] nframe_list             List of Visual NFrames filled with
  /// projected keypoints.
  /// @param[out] poses_without_keypoints Optional. If != nullptr, this returns
  /// a list of
  ///                                     poses where zero landmarks are seen
  ///                                     from.
  void simulateVisualNFrames(
      const Eigen::VectorXd& timestamps_seconds,
      const aslam::TransformationVector& T_G_Bs,
      const Eigen::Matrix3Xd& G_landmarks,
      const aslam::NCamera::Ptr& camera_rig, double keypoint_sigma_px,
      bool add_noise_to_keypoints, size_t num_bits_to_flip,
      aslam::VisualNFrame::PtrVector* nframe_list,
      aslam::TransformationVector* poses_without_keypoints);

  /// \brief Returns a ref to const to the Eigen matrix containing the ground
  /// truth descriptors.
  const aslam::VisualFrame::DescriptorsT& getGroundTruthDescriptors() const;

  /// \brief Returns a ref to const to the vector containing the ground truth
  /// landmark ids.
  const vi_map::LandmarkIdList& getGroundTruthLandmarkIds() const;

  /// \brief Returns a ref to const to the vector containing the ground truth
  /// landmark scores.
  const Eigen::VectorXd& getGroundTruthLandmarkScores() const;

  /// The descriptor size.
  static constexpr uint32_t kDescriptorSizeBytes = 48;

  const std::vector<size_t>& getGroundTruthLandmarkObservationCount() {
    return ground_truth_landmark_observation_count_;
  }

 private:
  /// \brief Takes as list of landmarks in the global frame,
  ///        a transformation from the global into the camera rig frame (B),
  ///        a pointer to a VisualFrame, projects all landmarks into the camera
  ///        frame and adds keypoint measurement incl. uncertainty for all
  ///        visible landmarks.
  ///
  ///
  /// @param[in]  G_landmarks                     List of landmarks expressed in
  /// the global frame.
  /// @param[in]  T_B_G                           Transformation taking points
  /// in the global frame
  ///                                             and expressing them in the
  ///                                             camera rig frame (B).
  /// @param[in]  keypoint_sigma_px               The The noise sigma on the
  /// keypoints in [px].
  ///                                             This will be used to set the
  ///                                             uncertainty matrix
  ///                                             in the VisualFrame. It is
  ///                                             independent of
  ///                                             any noise sampled on the
  ///                                             keypoints.
  /// @param[in]  num_bits_to_flip                Number of bits to flip wrt.
  /// the ground truth
  ///                                             descriptor when generating
  ///                                             keypoints for the
  ///                                             frames.
  /// @param[in]  keypoint_std_dev                Optional. 2x2 diagonal
  /// uncertainty matrix for
  ///                                             noise sampling. Contains
  ///                                             std_dev of the keypoint
  ///                                             uncertainty [px] on the
  ///                                             diagonal.
  ///                                             If null, no noise will be
  ///                                             added.
  ///                                             Note: this is used for
  ///                                             sampling! It is independent
  ///                                             of the keypoint variance
  ///                                             parameter.
  /// @param[out] nframe                          Pointer to the frame to be
  /// filled.
  /// @param[out] poses_with_no_visible_landmarks Optional. If != nullptr, this
  /// returns a list of
  ///                                             poses where zero landmarks are
  ///                                             seen from.
  void projectLandmarksAndFillFrame(
      const Eigen::Matrix3Xd& G_landmarks, const aslam::Transformation& T_B_G,
      double keypoint_sigma_px, size_t num_bits_to_flip,
      const Eigen::Matrix2d* keypoint_std_dev, aslam::VisualNFrame* nframe,
      aslam::TransformationVector* poses_with_no_visible_landmarks);

  /// Generates ground truth data (i.e. landmark, descriptors, etc.)
  void generateGroundTruth(size_t num_landmarks);

  /// Checks whether the given keypoint lies inside the image box minus some
  /// border.
  inline bool withinImageBoxWithBorder(
      Eigen::Block<Eigen::Matrix2Xd, 2, 1> keypoint,
      const aslam::Camera& camera) {
    static constexpr double kMinDistanceToBorderPx = 10.0;
    CHECK_LT(2 * kMinDistanceToBorderPx, camera.imageWidth());
    CHECK_LT(2 * kMinDistanceToBorderPx, camera.imageHeight());
    return (keypoint(0) >= kMinDistanceToBorderPx) &&
           (keypoint(1) >= kMinDistanceToBorderPx) &&
           (keypoint(0) < static_cast<double>(camera.imageWidth()) -
                              kMinDistanceToBorderPx) &&
           (keypoint(1) <
            static_cast<double>(camera.imageHeight()) - kMinDistanceToBorderPx);
  }

  /// The ground truth data.
  aslam::VisualFrame::DescriptorsT ground_truth_landmark_descriptors_;
  vi_map::LandmarkIdList ground_truth_landmark_ids_;
  Eigen::VectorXd ground_truth_landmark_scores_;
  std::vector<size_t> ground_truth_landmark_observation_count_;
  std::mutex m_parallel_projection_;
};

}  // namespace simulation
#endif  // SIMULATION_VISUAL_NFRAME_SIMULATOR_H_
