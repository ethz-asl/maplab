#!/usr/bin/env python
# pylint: disable=bad-continuation

import os

# Force matplotlib to not use any Xwindows backend. Needs to be imported before
# any other matplotlib import.
import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
import yaml

from maplab_evaluation.report_common import format_date

PLOT_DIR = 'plots'


def _prepare_plot_folder():
    if not os.path.isdir(PLOT_DIR):
        os.makedirs(PLOT_DIR)


def overview_plot(labels, list_of_evaluation_summaries):
    _prepare_plot_folder()
    indices = np.arange(len(list_of_evaluation_summaries))
    width = 0.35
    _, ax = plt.subplots()

    num_of_successful_jobs = []
    num_of_failed_jobs = []
    for evaluation_summary in list_of_evaluation_summaries:
        successful_jobs = 0
        failed_jobs = 0
        for _, results in evaluation_summary.iteritems():
            job_succeeded = True
            for _, exit_code in results.iteritems():
                if exit_code != 0:
                    job_succeeded = False

            if job_succeeded:
                successful_jobs += 1
            else:
                failed_jobs += 1

        num_of_successful_jobs.append(successful_jobs)
        num_of_failed_jobs.append(failed_jobs)

    totals = [
        a + b for a, b in zip(num_of_successful_jobs, num_of_failed_jobs)
    ]
    total_plots = ax.bar(indices, totals, width, color='b')
    failed_plots = ax.bar(indices + width, totals, width, color='r')
    success_plots = ax.bar(
        indices + width, num_of_successful_jobs, width, color='g')

    ax.set_xticks(indices + width / 2)
    ax.set_xticklabels(labels, rotation=90)

    if num_of_successful_jobs and num_of_failed_jobs:
        max_number = max(max(num_of_successful_jobs), max(num_of_failed_jobs))
        yticks = range(0, max_number + 1)
        ax.set_yticks(yticks)
        ax.set_yticklabels(yticks)

        ax.legend((total_plots[0], success_plots[0], failed_plots[0]),
                  ('Total jobs', 'Successful jobs', 'Failed jobs'))

        plt.subplots_adjust(bottom=0.2)
    save_path = os.path.join(PLOT_DIR, 'summary.pdf')
    plt.savefig(save_path)
    plt.close()
    return save_path


def error_history_plot(evaluation_results, job_name, file_name):
    _prepare_plot_folder()
    labels = [format_date(obj.timestamp) for obj in evaluation_results]

    indices = np.arange(len(evaluation_results))
    indices_position_filtered = []
    position_errors_list = [[], []]
    indices_position_thresholds = []
    position_thresholds_list = []
    indices_orientation_filtered = []
    orientation_errors_list = [[], []]
    indices_orientation_thresholds = []
    orientation_thresholds_list = []
    for idx, evaluation_result in enumerate(evaluation_results):
        if job_name in evaluation_result.jobs:
            relative_job_path = evaluation_result.jobs[job_name]
            full_file_path = os.path.join(evaluation_result.path,
                                          relative_job_path, file_name)
            if os.path.isfile(full_file_path):
                errors_file = yaml.load(open(full_file_path))
                if 'position_errors' in errors_file:
                    indices_position_filtered.append(idx)
                    position_errors_list[0].append(
                        errors_file['position_errors']['rmse'])
                    position_errors_list[1].append(
                        errors_file['position_errors']['mean'])
                    if ('thresholds' in errors_file['position_errors']
                            and 'rmse' in errors_file['position_errors']
                        ['thresholds']):
                        indices_position_thresholds.append(idx)
                        position_thresholds_list.append(
                            errors_file['position_errors']['thresholds'][
                                'rmse'])
                if 'orientation_errors' in errors_file:
                    indices_orientation_filtered.append(idx)
                    orientation_errors_list[0].append(
                        errors_file['orientation_errors']['rmse'])
                    orientation_errors_list[1].append(
                        errors_file['orientation_errors']['mean'])
                    if ('thresholds' in errors_file['orientation_errors']
                            and 'rmse' in errors_file['orientation_errors']
                        ['thresholds']):
                        indices_orientation_thresholds.append(idx)
                        orientation_thresholds_list.append(
                            errors_file['orientation_errors']['thresholds'][
                                'rmse'])

    output_file_names = []
    for ylabel, file_name_extension, indices_filtered, values, \
            indices_thresholds, thresholds_values in zip(
        ['Position errors [m]', 'Orientation error [rad]'],
        ['_position', '_orientation'],
        [indices_position_filtered, indices_orientation_filtered],
        [position_errors_list, orientation_errors_list],
        [indices_position_thresholds, indices_orientation_thresholds],
        [position_thresholds_list, orientation_thresholds_list]):
        _, ax = plt.subplots()

        if len(indices_thresholds) > 1:
            ax.plot(
                indices_thresholds,
                thresholds_values,
                '--',
                label='RMSE Threshold')
        elif len(indices_thresholds) == 1:
            ax.plot(
                indices_thresholds,
                thresholds_values,
                'o',
                label='RMSE Threshold')

        ax.plot(indices_filtered, values[0], 'o-', label='RMSE')
        ax.plot(indices_filtered, values[1], 'o-', label='Mean')

        ax.legend()
        ax.set_xticks(indices)
        ax.set_xticklabels(labels, rotation=90)
        ax.grid()
        ax.set_ylabel(ylabel)

        output_file_name = os.path.join(
            PLOT_DIR,
            job_name.replace('/', '_') + '__' + file_name.replace(
                '.yaml', file_name_extension + '.pdf').replace('/', '_'))
        plt.subplots_adjust(bottom=0.2)
        plt.savefig(output_file_name)
        output_file_names.append(output_file_name)
        plt.close()
    return output_file_names
