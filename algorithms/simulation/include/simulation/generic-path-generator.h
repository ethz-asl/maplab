#ifndef SIMULATION_GENERIC_PATH_GENERATOR_H_
#define SIMULATION_GENERIC_PATH_GENERATOR_H_

#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <aslam/common/memory.h>
#include <gflags/gflags.h>

#include "simulation/visual-inertial-path-generator.h"

namespace test_trajectory_gen {

class GenericPathGenerator : public VisualInertialPathGenerator {
 public:
  explicit GenericPathGenerator(const PathAndLandmarkSettings& settings)
      : VisualInertialPathGenerator(settings), settings_(settings) {}

  /// \brief Wrapper function to generate the path. The path properties can be
  /// set by gflags if called via generic-path-generator-demo.
  virtual void generatePath();

  /// \brief Wrapper function to generate landmarks. The keypoint locations can
  /// be controlled by gflags if called via generic-path-generator-demo.
  virtual void generateLandmarks();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  /// \brief Generates constraints for a circular path. Called inside
  /// function generatePath. The path properties (e.g. circle radius)
  /// can be controlled by gflags if called via generic-path-generator-demo.
  void generatePathConstraintsCircle(
      const double circle_radius_meter, const size_t num_of_path_constraints,
      const Eigen::Vector4d start_offset);

  /// \brief Generates constraints for an elliptical path. Called inside
  /// function generatePath. The path properties (e.g. alpha and kappa of
  /// the ellipse equation) can be controlled by gflags if called via generic-
  /// path-generator-demo.
  void generatePathConstraintsEllipse(
      const double kappa, const double lambda, const double num_of_rounds,
      const size_t num_of_path_constraints, const Eigen::Vector4d start_offset);

  /// \brief Generates constraints for a simpe rotation-only path. Called
  /// inside function generatePath.
  void generatePathConstraintsRotationOnly();

  /// \brief Generate constraints for a simple translation-only path. Called
  /// inside function generatePath.
  void generatePathConstraintsTranslationOnly();

  /// \brief Generates constraints for an user-defined path. Called
  /// inside function generatePath. The waypoints are read in from file in
  /// the format [x,y,z,yaw].
  void generatePathConstraintsFromFile(const std::string& filename);

  /// \brief Generates constraints for an user-defined path. Called
  /// inside function generatePath. The waypoints are passed to ctor.
  void generatePathConstraintsFromCtor(
      const Aligned<std::vector, Eigen::Vector4d>& pose_waypoints);

  /// \brief Inserts the calculated path constrains.
  /// @param[in] max_position Maximal position change.
  /// @param[in] max_yaw Maximal yaw change.
  void insertPathConstraints(
      mav_planning_utils::path_planning::Vertex1D* max_position,
      mav_planning_utils::path_planning::Vertex1D* max_yaw);

  /// \brief Generates landmarks around the circular trajectory. The
  /// properties of the keypoint location (e.g. distance to path) can
  /// be controlled by gflags  if called via generic-path-generator-demo.
  void generateLandmarksCircle(
      const double circle_radius_meter,
      const double distance_to_keypoints_meter,
      const double landmark_variance_meter,
      const double vertical_to_radial_variance_factor,
      const size_t num_of_landmarks, const size_t landmark_seed,
      const Eigen::Vector3d start_offset_meter);

  /// \brief Generates landmarks around the elliptical trajectory. The
  /// properties of the keypoint location (e.g. distance to path) can
  /// be controlled by gflags if called via generic-path-generator-demo.
  void generateLandmarksEllipse(
      const double distance_to_keypoints_meter, const double kappa,
      const double lambda, const double landmark_variance_meter,
      const double num_of_rounds,
      const double vertical_to_radial_variance_factor,
      const size_t num_of_landmarks, const size_t landmark_seed,
      const Eigen::Vector3d start_offset_meter);

  /// \brief Generates landmarks in a volume. The width, length, height
  /// as well as x-, y-, z-offsets from the origin can be set by gflags.
  /// Make sure that the volume is placed where it can be observed by
  /// the camera rig.
  void generateLandmarksVolume(
      Eigen::Vector3d landmark_offset_meter,
      Eigen::Vector3d landmark_volume_meter,
      const double landmark_variance_meter, const size_t num_of_landmarks,
      const size_t landmark_seed);

  /// Struct storing path and landmark settings.
  PathAndLandmarkSettings settings_;

  /// Index for the velocity derivative. Do not change.
  static constexpr int kDerivativeToOptimizeVelocity = 1;

  /// Index for the acceleration derivative. Do not change.
  static constexpr int kDerivativeToOptimizeAcceleration = 2;

  /// Index for the position constraint. Do not change.
  static constexpr int kConstraintPosition = 0;

  /// Index for the velocity constraint. Do not change.
  static constexpr int kConstraintVelocity = 1;

  // Currently completely ignored in path_planning.h.
  static constexpr double kTimeToNext = 1.0;

  /// Set constraints up to derivatives to zero. This is useful for
  /// beginning and end vertices.
  static constexpr int kDerivatives = 4;

  /// Constraint for linear velocity.
  static constexpr double kConstraintValuePosition = 3.0;

  /// Constraint for angular velocity.
  static constexpr double kConstraintValueYaw = 3.0;

  /// Defines up to which derivative the path should be continuous.
  static constexpr int kContinuity = 4;

  /// Vector containing the pose waypoints.
  Aligned<std::vector, Eigen::Vector4d> pose_waypoints_;

  /// Four-dimensional vertex containing the path constraints.
  Aligned<std::vector, mav_planning_utils::path_planning::Vertex4D> sv_;
};

}  // namespace test_trajectory_gen
#endif  // SIMULATION_GENERIC_PATH_GENERATOR_H_
