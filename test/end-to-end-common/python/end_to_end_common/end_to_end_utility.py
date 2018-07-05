#!/usr/bin/env python

from __future__ import print_function

import bisect

import numpy as np

from hand_eye_calibration.quaternion import Quaternion
import hand_eye_calibration.time_alignment as time_alignment

from end_to_end_common.dataset_loader import csv_row_data_to_transformation
from end_to_end_common.umeyama import umeyama


def get_cumulative_trajectory_length_for_each_point(trajectory):
    """Returns a list of trajectory lengths up to each point in the trajectory.
    """
    assert trajectory.shape[1] == 3
    num_points = trajectory.shape[0]
    trajectory_length = np.zeros(num_points)
    for row_idx in range(1, num_points):
        delta = np.linalg.norm(
            trajectory[row_idx, :] - trajectory[row_idx - 1, :])
        trajectory_length[row_idx] = trajectory_length[row_idx - 1] + delta
    return trajectory_length


def align_datasets(estimator_data_unaligned_G_I,
                   ground_truth_data_unaligned_W_M,
                   estimate_scale,
                   align_data_to_use_meters=-1,
                   time_offset=0.0):
    """Sample and align the estimator trajectory to the ground truth trajectory.

    Input:
      - estimator_data_unaligned_G_I: Unaligned estimator trajectory expressed
            from global frame G to IMU frame I.
      - ground_truth_data_unaligned_W_M: Unaligned ground truth data expressed
            from ground truth world frame W (e.g. Vicon frame) to marker frame M
            (e.g. Vicon marker). M and I should be at the same position at a
            given time.
      - estimate_scale: Estimate a scale factor in addition to the transform
            and apply it to the aligned estimator trajectory.
      - align_data_to_use_meters: Only use data points up to a total data length
            as given by this parameter. If a negative value is given, the
            entire trajectory is used for the alignment.
      - time_offset: Indicates the time offset (in seconds) between the data
            from the bag file and the ground truth data. In the alignment, the
            estimator data will be shifted by this offset.

    Returns:
        Sampled, aligned and transformed estimator trajectory, sampled ground
        truth.

    Output data frames:
        Estimator:      W --> I

        Ground truth:   W --> M
        (M and I should be at the same position.)
    """
    [estimator_data_G_I,
     ground_truth_data_W_M] = time_alignment.compute_aligned_poses(
         estimator_data_unaligned_G_I, ground_truth_data_unaligned_W_M,
         time_offset)

    # Create a matrix of the right size to store aligned trajectory.
    estimator_data_aligned_W_I = estimator_data_G_I

    # Align trajectories (umeyama).
    if align_data_to_use_meters < 0.0:
        umeyama_estimator_data_G_I = estimator_data_G_I[:, 1:4]
        umeyama_ground_truth_W_M = ground_truth_data_W_M[:, 1:4]
    else:
        trajectory_length = get_cumulative_trajectory_length_for_each_point(
            ground_truth_data_W_M[:, 1:4])
        align_num_data_points = bisect.bisect_left(trajectory_length,
                                                   align_data_to_use_meters)
        umeyama_estimator_data_G_I = estimator_data_G_I[:align_num_data_points,
                                                        1:4]
        umeyama_ground_truth_W_M = \
            ground_truth_data_W_M[:align_num_data_points, 1:4]

    T_W_G, scale = umeyama(umeyama_estimator_data_G_I.T,
                           umeyama_ground_truth_W_M.T, estimate_scale)
    assert T_W_G.shape[0] == 4
    assert T_W_G.shape[1] == 4
    if abs(1.0 - scale) > 1e-5:
        print("WARNING: Estimator scale is not 1, but ", scale, ".", end='')

    num_data_points = ground_truth_data_W_M.shape[0]
    for index in range(num_data_points):
        T_estimator_G_I_scaled = csv_row_data_to_transformation(
            scale * estimator_data_G_I[index, 1:4],
            estimator_data_G_I[index, 4:8] / np.linalg.norm(
                estimator_data_G_I[index, 4:8]))
        T_estimator_W_I_scaled = np.dot(T_W_G, T_estimator_G_I_scaled)
        estimator_data_aligned_W_I[index, 1:4] = T_estimator_W_I_scaled[0:3, 3]
        q_estimator_W_I = Quaternion.from_rotation_matrix(
            T_estimator_W_I_scaled[0:3, 0:3])
        estimator_data_aligned_W_I[index, 4:8] = q_estimator_W_I.q

    return estimator_data_aligned_W_I, ground_truth_data_W_M
