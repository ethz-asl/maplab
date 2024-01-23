#ifndef TRIANGULATION_H_
#define TRIANGULATION_H_
#include <vector>

#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/frames/feature-track.h>
#include <Eigen/Dense>

namespace aslam {

/// \brief This struct is returned by the triangulator and holds the result state
///        of the triangulation operation.
struct TriangulationResult {
  /// Possible triangulation state.
  enum class Status {
    /// The triangulation was successful.
    kSuccessful,
    /// There were too few landmark observations.
    kTooFewMeasurments,
    /// The landmark is not fully observable (rank deficiency).
    kUnobservable,
    /// Default value after construction.
    kUninitialized
  };
  // Make the enum values accessible from the outside without the additional
  // indirection.
  static Status SUCCESSFUL;
  static Status TOO_FEW_MEASUREMENTS;
  static Status UNOBSERVABLE;
  static Status UNINITIALIZED;

  constexpr TriangulationResult() : status_(Status::kUninitialized) {};
  constexpr TriangulationResult(Status status) : status_(status) {};

  /// \brief The triangulation result can be typecasted to bool and is true if
  ///        the triangulation was successful.
  explicit operator bool() const { return wasTriangulationSuccessful(); };

  /// \brief Convenience function to print the state using streams.
  friend std::ostream& operator<< (std::ostream& out, const TriangulationResult& state)  {
    std::string enum_str;
    switch (state.status_){
      case Status::kSuccessful:         enum_str = "SUCCESSFUL"; break;
      case Status::kTooFewMeasurments:  enum_str = "TOO_FEW_MEASUREMENTS"; break;
      case Status::kUnobservable:       enum_str = "UNOBSERVABLE"; break;
      default:
        case Status::kUninitialized:    enum_str = "UNINITIALIZED"; break;
    }
    out << "ProjectionResult: " << enum_str << std::endl;
    return out;
  }

  /// \brief Check whether the triangulation was successful.
  bool wasTriangulationSuccessful() const {
    return (status_ == Status::kSuccessful); };

  /// \brief Returns the exact state of the triangulation operation.
  Status status() const { return status_; };

 private:
  /// Stores the triangulation state.
  Status status_;
};

/// brief Triangulate a 3d point from a set of n keypoint measurements on the
///       normalized camera plane.
/// @param measurements_normalized Keypoint measurements on normalized camera
///       plane.
/// @param T_G_B Pose of the body frame of reference w.r.t. the global frame,
///       expressed in the global frame.
/// @param T_B_C Pose of the camera w.r.t. the body frame expressed in the body
///       frame of reference.
/// @param G_point Triangulated point in global frame.
/// @return Was the triangulation successful?
TriangulationResult linearTriangulateFromNViews(
    const Aligned<std::vector, Eigen::Vector2d>& measurements_normalized,
    const aslam::TransformationVector& T_G_B,
    const aslam::Transformation& T_B_C, Eigen::Vector3d* G_point);

/// brief Triangulate a 3d point from a set of n keypoint measurements on the
///       normalized camera plane.
/// @param measurements_normalized Keypoint measurements on normalized camera
///       plane.
/// @param T_G_B Pose of the body frame of reference w.r.t. the global frame,
///       expressed in the global frame.
/// @param T_B_C Pose of the camera w.r.t. the body frame expressed in the body
///       frame of reference.
/// @param G_point Triangulated point in global frame.
/// @return Was the triangulation successful?
TriangulationResult iterativeGaussNewtonTriangulateFromNViews(
    const Aligned<std::vector, Eigen::Vector2d>& measurements_normalized,
    const Aligned<std::vector, aslam::Transformation>& T_G_B,
    const aslam::Transformation& T_B_C, Eigen::Vector3d* G_point);

/// brief Triangulate a 3d point from a set of n keypoint measurements as
///       bearing vectors.
/// @param t_G_bv Back-projected bearing vectors from visual frames to
///               observations, expressed in the global frame.
/// @param p_G_C Global positions of visual frames (cameras).
/// @param p_G_P Triangulated point in global frame.
/// @return Was the triangulation successful?
TriangulationResult linearTriangulateFromNViews(
    const Eigen::Matrix3Xd& t_G_bv, const Eigen::Matrix3Xd& p_G_C, Eigen::Vector3d* p_G_P);

/// brief Triangulate a 3d point from a set of n keypoint measurements in
///       m cameras.
/// @param measurements_normalized Keypoint measurements on normalized image
///        plane. Should be n long.
/// @param measurement_camera_indices Which camera index each measurement
///        corresponds to. Should be n long, and should be 0 <= index < m.
/// @param T_W_B Pose of the body frame of reference w.r.t. the global frame,
///        expressed in the global frame. Should be n long.
/// @param T_B_C Pose of the cameras w.r.t. the body frame expressed in the body
///        frame of reference. Should be m long.
/// @param G_point Triangulated point in global frame.
/// @return Was the triangulation successful?
TriangulationResult linearTriangulateFromNViewsMultiCam(
    const Aligned<std::vector, Eigen::Vector2d>& measurements_normalized,
    const std::vector<size_t>& measurement_camera_indices,
    const Aligned<std::vector, aslam::Transformation>& T_G_B,
    const Aligned<std::vector, aslam::Transformation>& T_B_C,
    Eigen::Vector3d* G_point);

/// brief Triangulates a feature track together with a list of body poses.
///       Track length and size of T_W_Bs is expected to be equal.
///
/// Frames: W: Arbitrary frame which the resulting landmark will be expressed in.
///         B: Body frame (of the nframe).
///
/// @param[in]   track      The feature track to be triangulated.
/// @param[in]   T_W_Bs     The list of body poses the landmark was seen from.
/// @param[out]  W_Landmark The triangulated landmark, expressed in frame W.
TriangulationResult triangulateFeatureTrack(
    const aslam::FeatureTrack& track,
    const aslam::TransformationVector& T_W_Bs,
    Eigen::Vector3d* W_landmark);

/// brief Triangulates a feature track together with a list of body poses.
///       Track length and size of T_W_Bs is expected to be equal.
///       Uses the faster linearTriangulateFromNViews(Matrix3d, Matrix3d, Vector3d*).
///
/// Frames: W: Arbitrary frame which the resulting landmark will be expressed in.
///         B: Body frame (of the nframe).
///
/// @param[in]   track      The feature track to be triangulated.
/// @param[in]   T_W_Bs     The list of body poses the landmark was seen from.
/// @param[out]  W_Landmark The triangulated landmark, expressed in frame W.
TriangulationResult fastTriangulateFeatureTrack(
    const aslam::FeatureTrack& track,
    const aslam::TransformationVector& T_W_Bs,
    Eigen::Vector3d* W_landmark);
}  // namespace aslam
#endif  // TRIANGULATION_H_
