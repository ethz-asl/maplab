#!/usr/bin/env python

import sys

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import end_to_end_common.plotting_common as plotting_common


def plot_position_error(
        labels, ground_truth_data_list, estimator_data_list,
        localization_state_list,
        plotting_settings=plotting_common.PlottingSettings()):
  """
  Creates a plot of the absolute position error over the trajectory length.
  """
  assert len(labels) == len(ground_truth_data_list)
  assert len(ground_truth_data_list) == len(estimator_data_list)
  assert len(localization_state_list) == len(estimator_data_list)

  num_elements = len(labels)
  fig, ax = plt.subplots()

  trajectory_lengths = []
  min_starting_pos = 1e100
  max_end_pos = 0
  for idx in range(num_elements):
    size = ground_truth_data_list[idx].shape[0]
    trajectory_length = np.zeros(size)
    trajectory_length[0] = np.linalg.norm(ground_truth_data_list[idx][0, 1:4])
    for row_idx in range(size - 1):
      delta = np.linalg.norm(
          ground_truth_data_list[idx][row_idx + 1, 1:4] - \
                  ground_truth_data_list[idx][row_idx, 1:4])
      trajectory_length[row_idx + 1] = trajectory_length[row_idx] + delta
    trajectory_lengths.append(trajectory_length)

    if trajectory_length[0] < min_starting_pos:
      min_starting_pos = trajectory_length[0]

  for idx in range(num_elements):
    trajectory_lengths[idx][:] -= min_starting_pos
    if trajectory_lengths[idx][-1] > max_end_pos:
      max_end_pos = trajectory_lengths[idx][-1]

  max_error = 0
  for idx in range(num_elements):
    print "Plotting position error for ", labels[idx]
    pos_errors = np.linalg.norm(estimator_data_list[idx][:, 1:4] -
                                ground_truth_data_list[idx][:, 1:4], axis=1)
    if np.max(pos_errors) > max_error:
      max_error = np.max(pos_errors)

    # Plot by trajectroy length.
    plt.plot(
      trajectory_lengths[idx],
      pos_errors,
      label=labels[idx])

    # Plot localization states.
    if localization_state_list[idx].shape[0] > 0:
      for i, row in enumerate(localization_state_list[idx]):
        index_same_timestamp = \
                np.where(estimator_data_list[idx][:, 0] == row[0])
        if len(index_same_timestamp) > 0 and \
              index_same_timestamp[0].shape[0] > 0:
          row[0] = trajectory_lengths[idx][index_same_timestamp]
          row[1] = pos_errors[index_same_timestamp]
        else:
          row[1] = -1

      localization_state_list[idx] = \
              localization_state_list[idx] \
                  [localization_state_list[idx][:, 1] >= 0, :]
      plt.scatter(localization_state_list[idx][:, 0],
                  localization_state_list[idx][:, 1],
                  label="Localizations",
                  color="black",
                  zorder=100)

  plt.xlabel("Trajectory length [m]")
  plt.ylabel("Position error [m]")
  plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
             ncol=4, mode="expand", borderaxespad=0., frameon=False)
  ax.grid(True)
  ax.spines['right'].set_visible(False)
  ax.spines['top'].set_visible(False)
  ax.set_xlim((0, max_end_pos))
  ax.set_ylim((0, max_error))

  plotting_common.show_and_save_plot(
      plotting_settings, "error_over_trajectory")


def plot_top_down_aligned_data(
    estimator_data_G_I, ground_truth_data_G_M, title="",
    plotting_settings=plotting_common.PlottingSettings()):
  """
  Creates a top-down plot of the given estimated and ground truth trajectory.

  Inputs: aligned estimator and ground truth trajectory matrices (assumed to be
  in the output format of end_to_end_common.end_to_end_utility.align_datasets).
  """
  fig = plt.figure()
  plt.plot(
      ground_truth_data_G_M[:, 1], ground_truth_data_G_M[:, 2],
      label="Ground truth")
  plt.plot(
      estimator_data_G_I[:, 1], estimator_data_G_I[:, 2], label="Estimator")
  plt.xlabel("x position [m]")
  plt.ylabel("y position [m]")
  plt.legend()
  plt.title(title)

  plotting_common.show_and_save_plot(plotting_settings, "top_down_xy")


def plot_trajectory3d_aligned_data(
    estimator_data_G_I, ground_truth_data_G_M,
    title="", plotting_settings=plotting_common.PlottingSettings()):
  """
  Creates a 3d plot of the given estimated and ground truth trajectory.
  Inputs: aligned estimator and ground truth trajectory matrices (assumed to be
  in the output format of end_to_end_common.end_to_end_utility.align_datasets).
  """
  fig = plt.figure()
  ax = fig.gca(projection='3d')
  ax.plot(
      ground_truth_data_G_M[:, 1], ground_truth_data_G_M[:, 2],
      ground_truth_data_G_M[:, 3], label="Ground truth")
  ax.plot(
      estimator_data_G_I[:, 1], estimator_data_G_I[:, 2],
      estimator_data_G_I[:, 3], label="Estimator")

  ax.set_xlabel("x position [m]")
  ax.set_ylabel("y position [m]")
  ax.set_zlabel("z position [m]")
  plt.legend()
  plt.title(title)

  plotting_common.show_and_save_plot(plotting_settings, "trajectory_3d")


def plot_xyz_vs_groundtruth(
    estimator_data_G_I, ground_truth_data_G_M, title="",
    plotting_settings=plotting_common.PlottingSettings()):
  """
  Creates one plot each to compare the estimated x, y and z positions with the
  ground truth values.
  """
  fig = plt.figure()
  plt.suptitle(title)

  labels = ["x", "y", "z"]
  for i in range(3):
    plt.subplot(3, 1, i + 1)
    plt.plot(
        ground_truth_data_G_M[:, 0], ground_truth_data_G_M[:, 1 + i],
        label="Ground truth")
    plt.plot(
        estimator_data_G_I[:, 0], estimator_data_G_I[:, 1 + i],
        label="Estimator")
    plt.xlabel("Time [s]")
    plt.ylabel(labels[i] + " position [m]")
    plt.legend()

    plotting_common.show_and_save_plot(plotting_settings, "xyz_vs_groundtruth")


def plot_xyz_errors(
    estimator_data_G_I, ground_truth_data_G_M, title="",
    plotting_settings=plotting_common.PlottingSettings()):
  """
  Creates one plot each showing the individual relative x, y and errors.
  """
  fig = plt.figure()
  plt.suptitle(title)

  labels = ["x", "y", "z"]
  for i in range(3):
    plt.subplot(3, 1, i + 1)
    plt.plot(
        estimator_data_G_I[:, 0],
        estimator_data_G_I[:, 1 + i] - ground_truth_data_G_M[:, 1 + i])
    plt.xlabel("Time [s]")
    plt.ylabel(labels[i] + " error [m]")
    plt.grid()

  plotting_common.show_and_save_plot(plotting_settings, "xyz_errors")
