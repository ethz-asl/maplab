#!/usr/bin/env python

import bisect
import math
import os

import numpy as np

from end_to_end_common.dataset_loader import csv_row_data_to_transformation
from end_to_end_common.umeyama import umeyama
from hand_eye_calibration.quaternion import Quaternion
import hand_eye_calibration.time_alignment as time_alignment


def get_cumulative_trajectory_length_for_each_point(trajectory):
  """Returns a list of trajectory lengths up to each point in the trajectory.
  """
  assert trajectory.shape[1] == 3
  num_points = trajectory.shape[0]
  trajectory_length = np.zeros(num_points)
  for row_idx in range(1, num_points):
    delta = np.linalg.norm(trajectory[row_idx, :] - trajectory[row_idx - 1, :])
    trajectory_length[row_idx] = trajectory_length[row_idx - 1] + delta
  return trajectory_length


def align_datasets(estimator_data_unaligned_G_I,
                   ground_truth_data_unaligned_W_M,
                   align_data_to_use_meters=-1,
                   time_offset=0.0):
  """Sample and align the ground truth trajectory to the estimator trajectory.

  Input:
    - estimator_data_unaligned_G_I: Unaligned estimator trajectory expressed
          from global frame G to IMU frame I.
    - ground_truth_data_unaligned_W_M: Unaligned ground truth data expressed
          from ground truth world frame W (e.g. Vicon frame) to marker frame M
          (e.g. Vicon marker). M and I should be at the same position at a given
          time.
    - align_data_to_use_meters: Only use data points up to a total data length
          as given by this parameter. If a negative value is given, the
          entire trajectory is used for the alignment.
    - time_offset: Indicates the time offset (in seconds) between the data from
          the bag file and the ground truth data. In the alignment, the ground
          truth data will be shifted by this offset.

  Returns:
      Aligned estimator trajectory, aligned and sampled ground truth trajectory.

  Output data frames:
      Estimator:      G --> I

      Ground truth:   G --> M
      (M and I should be at the same position.)
  """
  [ground_truth_data_W_M, estimator_data_G_I] = \
      time_alignment.compute_aligned_poses(
          ground_truth_data_unaligned_W_M, estimator_data_unaligned_G_I,
          time_offset)

  # Create a matrix of the right size to store aligned trajectory.
  ground_truth_data_aligned_G_M = ground_truth_data_W_M

  # Align trajectories (umeyama).
  if align_data_to_use_meters < 0.0:
    umeyama_ground_truth_W_M = ground_truth_data_W_M[:, 1:4]
    umeyama_estimator_data_G_I = estimator_data_G_I[:, 1:4]
  else:
    trajectory_length = get_cumulative_trajectory_length_for_each_point(
        ground_truth_data_W_M[:, 1:4])
    align_num_data_points = bisect.bisect_left(trajectory_length,
                                               align_data_to_use_meters)
    umeyama_ground_truth_W_M = \
        ground_truth_data_W_M[:align_num_data_points, 1:4]
    umeyama_estimator_data_G_I = estimator_data_G_I[:align_num_data_points, 1:4]
  R_G_W, t_G_W, scale = umeyama(umeyama_ground_truth_W_M.T,
                                umeyama_estimator_data_G_I.T)
  T_G_W = np.identity(4)
  T_G_W[0:3, 0:3] = R_G_W
  T_G_W[0:3, 3] = t_G_W
  if abs(1 - scale) < 1e-8:
    print "WARNING: Scale is not 1, but", scale, "\b."

  num_data_points = estimator_data_G_I.shape[0]
  for index in range(num_data_points):
    # Create transformations to compare.
    T_ground_truth_G_M = np.dot(
        T_G_W,
        csv_row_data_to_transformation(
            ground_truth_data_W_M[index, 1:4],
            ground_truth_data_W_M[index, 4:8] / np.linalg.norm(
                ground_truth_data_W_M[index, 4:8])))
    ground_truth_data_aligned_G_M[index, 1:4] = T_ground_truth_G_M[0:3, 3]
    q_ground_truth_G_M = Quaternion.from_rotation_matrix(
        T_ground_truth_G_M[0:3, 0:3])
    ground_truth_data_aligned_G_M[index, 4:8] = q_ground_truth_G_M.q

  return estimator_data_G_I, ground_truth_data_aligned_G_M
