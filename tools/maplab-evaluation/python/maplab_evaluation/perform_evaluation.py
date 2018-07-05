#!/usr/bin/env python

from __future__ import print_function

import argparse
import glob
import os
import time

import yaml

from evaluation_tools.run_experiment import Experiment

import maplab_evaluation.report_generator as report_generator


def get_filtered_files_in_folder_ordered(folder, file_filter):
    """Returns an ordered list of all files in a folder that match the filter.
    """
    return sorted(glob.glob(os.path.join(folder, file_filter)))


def run_evaluation(experiments_to_process, results_folder,
                   enable_progress_bars):
    print('Run experiments:', experiments_to_process)

    evaluation_results = {}
    for experiment_file in experiments_to_process:
        try:
            experiment = Experiment(
                experiment_file,
                results_folder,
                automatic_dataset_download=True,
                enable_progress_bars=enable_progress_bars)
            experiment.runAndEvaluate()
            evaluation_results.update(experiment.evaluation_results)
        except Exception as e:  # pylint: disable=broad-except
            # Experiment failed somewhere within evaluation_tools (usually
            # caused by an invalid configuration).
            print("Experiment", experiment_file, "failed:", e)
            evaluation_results['experiment ' + experiment_file] = {
                'evaluation_tools': -1
            }

    return evaluation_results


def print_evaluation_overview(evaluation_results):
    print('=' * 80)
    print('OVERVIEW OF ALL RUN EXPERIMENTS')
    print('=' * 80)
    for job_name, job_results in evaluation_results.iteritems():
        print('\n', job_name, sep='')
        print('-' * 80)
        for evaluation_name, evaluation_result in job_results.iteritems():
            if evaluation_result == 0:
                print('    SUCCESS:', evaluation_name, 'succeeded.')
            else:
                print(
                    '    ERROR:   ',
                    evaluation_name,
                    ' failed with return value ',
                    evaluation_result,
                    '.',
                    sep='')
        print('=' * 80)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('experiments_folder')
    parser.add_argument('--experiment_file_filter', default='*.yaml')
    parser.add_argument('--results_output_folder', default='./results')
    parser.add_argument(
        '--disable_progress_bars', action='store_true', default=False)
    args = parser.parse_args()

    results_folder = os.path.join(args.results_output_folder,
                                  time.strftime("%Y%m%d_%H%M%S",
                                                time.localtime()))
    experiments_to_process = get_filtered_files_in_folder_ordered(
        args.experiments_folder, args.experiment_file_filter)
    evaluation_results = run_evaluation(
        experiments_to_process,
        results_folder,
        enable_progress_bars=not args.disable_progress_bars)
    report_generator.ReportGenerator(
        os.path.join(results_folder), evaluation_results).generate_report()
    print_evaluation_overview(evaluation_results)
    with open(os.path.join(results_folder, 'evaluation_results.yaml'),
              "w") as out_file_stream:
        yaml.safe_dump(evaluation_results, out_file_stream)


if __name__ == "__main__":
    main()
