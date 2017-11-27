#!/usr/bin/env python

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from Tkinter import *  # For the exception.

def plot_position_error(
        labels, ground_truth_data_list, estimator_data_list,
        localization_state_list):
    assert len(labels) == len(ground_truth_data_list)
    assert len(ground_truth_data_list) == len(estimator_data_list)
    assert len(localization_state_list) == len(estimator_data_list)

    num_elements = len(labels)
    try:
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

        plt.savefig("position_errors.pdf")
        plt.savefig("position_errors.png")
    except TclError:
        print "Plotting not possible.", \
              "No graphical environment may be present."
