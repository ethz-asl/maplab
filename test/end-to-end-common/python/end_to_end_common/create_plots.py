#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # pylint: disable=unused-import

from end_to_end_common.end_to_end_utility \
    import get_cumulative_trajectory_length_for_each_point
import end_to_end_common.plotting_common as plotting_common


def plot_position_error(test_data_list_or_dict,
                        plotting_settings=plotting_common.PlottingSettings(),
                        title=None,
                        horizontal_errors_only=False):
    """ Creates a plot of the absolute position error over the trajectory
    length.

    If title is None and a title should be set, a title will be chosen
    automatically: if there only one dataset is given, the name of that dataset
    is chosen, otherwise a generic title will be set. If the flag
    horizontal_errors_only is set, only the horizontal errors will be plotted.
    """
    if isinstance(test_data_list_or_dict, dict):
        test_data_list = []
        for _, value in test_data_list_or_dict.iteritems():
            test_data_list.append(value)
    else:
        test_data_list = test_data_list_or_dict

    num_elements = len(test_data_list)
    fig, ax = plt.subplots()

    max_end_pos = 0
    trajectory_lengths = [
        get_cumulative_trajectory_length_for_each_point(
            data.ground_truth_G_M[:, 1:4]) for data in test_data_list
    ]

    for idx in range(num_elements):
        if trajectory_lengths[idx][-1] > max_end_pos:
            max_end_pos = trajectory_lengths[idx][-1]

    max_error = 0
    for idx, data in enumerate(test_data_list):
        print "Plotting position error for ", data.label
        max_index = 4
        if horizontal_errors_only:
            max_index = 3
        pos_errors = np.linalg.norm(
            data.estimator_G_I[:, 1:max_index] -
            data.ground_truth_G_M[:, 1:max_index],
            axis=1)
        if np.max(pos_errors) > max_error:
            max_error = np.max(pos_errors)

        # Plot by trajectroy length.
        plt.plot(trajectory_lengths[idx], pos_errors, label=data.label)

        # Plot localization states.
        if data.localization_state_list.shape[0] > 0:
            for row in data.localization_state_list:
                index_same_timestamp = np.where(
                    data.estimator_G_I[:, 0] == row[0])
                if (index_same_timestamp
                        and index_same_timestamp[0].shape[0] > 0):
                    row[0] = trajectory_lengths[idx][index_same_timestamp]
                    row[1] = pos_errors[index_same_timestamp]
                else:
                    row[1] = -1

            data.localization_state_list = data.localization_state_list[
                data.localization_state_list[:, 1] >= 0, :]
            plt.scatter(
                data.localization_state_list[:, 0],
                data.localization_state_list[:, 1],
                label="Localizations",
                color="black",
                s=1,
                zorder=100)

    plt.xlabel("Trajectory length [m]")
    plt.ylabel("Position error [m]")
    if len(test_data_list) > 1:
        # Don't show legend if there's only one data.
        plotting_common.plot_legend_on_top(ncol=4)
    if plotting_settings.show_title:
        if title is None:
            if len(test_data_list) == 1:
                title = test_data_list[0].label
            else:
                title = 'Error over trajectory'
            plt.title(title)
    ax.grid(plotting_settings.show_grid)
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.set_xlim((0, max_end_pos))
    ax.set_ylim((0, max_error))

    plotting_common.show_and_save_plot(plotting_settings,
                                       "error_over_trajectory")
    plt.close(fig)


def plot_top_down_aligned_data(
        test_data, plotting_settings=plotting_common.PlottingSettings()):
    """
    Creates a top-down plot of the given estimated and ground truth trajectory.

    Inputs: aligned estimator and ground truth trajectory matrices (assumed to
    bewin the output format of
    end_to_end_common.end_to_end_utility.align_datasets).
    """
    fig = plt.figure()
    plt.plot(
        test_data.ground_truth_G_M[:, 1],
        test_data.ground_truth_G_M[:, 2],
        label="Ground truth")
    plt.plot(
        test_data.estimator_G_I[:, 1],
        test_data.estimator_G_I[:, 2],
        label="Estimator")
    plt.xlabel("x position [m]")
    plt.ylabel("y position [m]")
    plt.legend()
    if plotting_settings.show_title:
        plt.title(test_data.label)
    if plotting_settings.show_grid:
        plt.grid()

    plotting_common.show_and_save_plot(plotting_settings, "top_down_xy")
    plt.close(fig)


def plot_trajectory_3d_aligned_data(
        test_data, plotting_settings=plotting_common.PlottingSettings()):
    """
    Creates a 3d plot of the given estimated and ground truth trajectory.
    Inputs: aligned estimator and ground truth trajectory matrices (assumed to
    be in the output format of
    end_to_end_common.end_to_end_utility.align_datasets).
    """
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(
        test_data.ground_truth_G_M[:, 1],
        test_data.ground_truth_G_M[:, 2],
        test_data.ground_truth_G_M[:, 3],
        label="Ground truth")
    ax.plot(
        test_data.estimator_G_I[:, 1],
        test_data.estimator_G_I[:, 2],
        test_data.estimator_G_I[:, 3],
        label="Estimator")

    ax.set_xlabel("x position [m]")
    ax.set_ylabel("y position [m]")
    ax.set_zlabel("z position [m]")

    if plotting_settings.trajectory_3d_axis_equal:
        plotting_common.axis_equal_3d(ax)

    plt.legend()
    if plotting_settings.show_title:
        plt.title(test_data.label)

    plotting_common.show_and_save_plot(plotting_settings, "trajectory_3d")
    plt.close(fig)


def plot_xyz_vs_groundtruth(
        test_data, plotting_settings=plotting_common.PlottingSettings()):
    """
    Creates one plot each to compare the estimated x, y and z positions with the
    ground truth values.
    """
    fig = plt.figure()
    if plotting_settings.show_title:
        plt.suptitle(test_data.label)

    labels = ["x", "y", "z"]
    for i in range(3):
        plt.subplot(3, 1, i + 1)
        plt.plot(
            test_data.ground_truth_G_M[:, 0] -
            test_data.ground_truth_G_M[0, 0],
            test_data.ground_truth_G_M[:, 1 + i],
            label="Ground truth")
        plt.plot(
            test_data.estimator_G_I[:, 0] - test_data.estimator_G_I[0, 0],
            test_data.estimator_G_I[:, 1 + i],
            label="Estimator")
        plt.xlabel("Time [s]")
        plt.ylabel(labels[i] + " position [m]")
        if plotting_settings.show_grid:
            plt.grid()
        if i == 0:
            plotting_common.plot_legend_on_top(ncol=2)

    plotting_common.show_and_save_plot(plotting_settings, "xyz_vs_groundtruth")
    plt.close(fig)


def plot_xyz_errors(test_data,
                    plotting_settings=plotting_common.PlottingSettings()):
    """
    Creates one plot each showing the individual relative x, y and errors.
    """
    fig = plt.figure()
    if plotting_settings.show_title:
        plt.suptitle(test_data.label)

    labels = ["x", "y", "z"]
    for i in range(3):
        plt.subplot(3, 1, i + 1)
        plt.plot(
            test_data.estimator_G_I[:, 0] - test_data.estimator_G_I[0, 0],
            np.abs(test_data.estimator_G_I[:, 1 + i] -
                   test_data.ground_truth_G_M[:, 1 + i]))
        plt.xlabel("Time [s]")
        plt.ylabel(labels[i] + " error [m]")
        if plotting_settings.show_grid:
            plt.grid()

    plotting_common.show_and_save_plot(plotting_settings, "xyz_errors")
    plt.close(fig)
