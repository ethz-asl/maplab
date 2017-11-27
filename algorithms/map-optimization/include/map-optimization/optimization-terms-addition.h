#ifndef MAP_OPTIMIZATION_OPTIMIZATION_TERMS_ADDITION_H_
#define MAP_OPTIMIZATION_OPTIMIZATION_TERMS_ADDITION_H_

#include "map-optimization/optimization-problem.h"

#include <memory>

namespace map_optimization {

void addVisualTerms(
    const bool fix_landmark_positions, const bool fix_intrinsics,
    const bool fix_extrinsics_rotation, const bool fix_extrinsics_translation,
    const size_t min_landmarks_per_frame, OptimizationProblem* problem);

void addVisualTermsForVertices(
    const bool fix_landmark_positions, const bool fix_intrinsics,
    const bool fix_extrinsics_rotation, const bool fix_extrinsics_translation,
    const size_t min_landmarks_per_frame,
    const std::shared_ptr<ceres::LocalParameterization>& pose_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        baseframe_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        camera_parameterization,
    const pose_graph::VertexIdList& vertices, OptimizationProblem* problem);

bool addVisualTermForKeypoint(
    const int keypoint_idx, const int frame_idx,
    const bool fix_landmark_positions, const bool fix_intrinsics,
    const bool fix_extrinsics_rotation, const bool fix_extrinsics_translation,
    const std::shared_ptr<ceres::LocalParameterization>& pose_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        baseframe_parameterization,
    const std::shared_ptr<ceres::LocalParameterization>&
        camera_parameterization,
    vi_map::Vertex* vertex_ptr, OptimizationProblem* problem);

void addInertialTerms(
    const bool fix_gyro_bias, const bool fix_accel_bias,
    const bool fix_velocity, const double gravity_magnitude,
    OptimizationProblem* problem);

int addInertialTermsForEdges(
    const bool fix_gyro_bias, const bool fix_accel_bias,
    const bool fix_velocity, const double gravity_magnitude,
    const vi_map::ImuSigmas& imu_sigmas,
    const std::shared_ptr<ceres::LocalParameterization>& pose_parameterization,
    const pose_graph::EdgeIdList& edges, OptimizationProblem* problem);

}  // namespace map_optimization

#endif  // MAP_OPTIMIZATION_OPTIMIZATION_TERMS_ADDITION_H_
