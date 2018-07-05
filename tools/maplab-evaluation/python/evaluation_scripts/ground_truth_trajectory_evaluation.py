#!/usr/bin/env python
# pylint: disable=undefined-variable
"""Compares the trajectory from the estimator and console to a ground truth.

Additional parameters that this script takes (specified in the
additional_parameters of the evaluation entry in the experiment yaml):
  - csv_base_folder: base folder name with the all CSV trajectories.
  - mission_info_file: location of the mission info yaml exported from maplab.
        This is only required if CSV data was exported from a map with more
        than one mission. For each dataset that is tested, an entry with the
        same name as the dataset name containing information about only one
        mission is required. This is best guaranteed if all maps are loaded
        individually first (without merging) using the suggested map key and
        then exporting the mission info file.

Additional dataset parameters needed by this:
  - ground_truth_csv: path to the ground truth trajectory.
  - ground_truth_csv_format: format of the ground truth CSV file. Optional,
        default is euroc format.
  - max_allowed_errors: list that contains all checks that should be performed.
        It contains the following entries:
        - label: folder name with the CSV data. The CSV files will be searched
              inside
              <job_dir>/<csv_base_folder>/<label>/<mission_id>/vertices.csv
              If this file doesn't exist, it is assumed that the trajectory
              was exported from the estimator as opposed to the maplab console.
              Read below for more details.
        - max_position_rmse_m: test will fail if the calculated position RMSE
              exceeds this value. Optional.
        - max_orientation_rmse_m: test will fail if the calculated orientation
              RMSE exceeds this value. Optional.
        - better_than: list containing references to other error comparisons so
              that it can be checked if certain commands improve the accuracy.
              This flag only adds an additional condition for the position
              RMSE, the orientation RMSE threshold remains unaffected. This
              entry is optional. The list entry can contain the following
              values:
              - label: name of the entry with which to compare (value from
                    max_allowed_errors -> label).
              - factor: optional number to scale the threshold, the additional
                    condition by this block will be:
                        <calculated_error> <= \
                            <factor> * <error_from_other_label>
                    If factor is not specified, it will take a default value of
                    1.

Using a CSV trajectory exported from the estimator (e.g., rovioli):
  If a folder under <job_dir>/<csv_base_folder>/<label> cannot be found,
  the alternative search path <dataset_log_dir>/trajectory_rovioli.csv is tried
  to find the CSV file.

Outputs of this script:
  A folder end_to_end_results will be created with all the output from this
  script. The folder contains a subfolder for each dataset with the following
  contents:
  - error_over_trajectory.pdf: plot showing the position error over trajectory
        for all processed test cases of this dataset.
  - <label>_errors.yaml: textual information containing the calculated
        mean, rmse, max, min position and orientation errors for the comparison
        named <label> and the corresponding thresholds.
  - Subfolders <counter>_<label> containing further plots for the comparison
        <label>. <counter> is a rising integer that causes the created folders
        to be sorted the same way as they are listed in the experiments yaml.
"""

from __future__ import print_function

import os
import yaml

import matplotlib as mpl
mpl.use('Agg')

import numpy as np

from end_to_end_common.create_plots import *  # pylint: disable=wildcard-import, unused-wildcard-import
from end_to_end_common.dataset_loader import load_dataset
from end_to_end_common.end_to_end_test import EndToEndTest
from end_to_end_common.plotting_common import PlottingSettings
from end_to_end_common.test_structs import \
        TestErrorStruct, ThresholdExceededException
from evaluation_tools.evaluation_arg_parse import \
        EvaluationArgParse as ArgParser


class GroundTruthTrajectoryEvaluation(object):
    def __init__(self, args):
        self.job_dir = args.job_dir
        self.datasets = args.dataset_paths
        self.dataset_log_dirs = args.dataset_log_dirs
        assert len(self.datasets) == len(self.dataset_log_dirs)
        self.csv_base_folder = args.csv_base_folder
        self.additional_dataset_parameters = args.additional_dataset_parameters
        assert len(self.datasets) == len(self.additional_dataset_parameters)
        self.result_folder = os.path.join(self.job_dir, 'end_to_end_results')
        self.mission_info_file = args.mission_info_file
        self.plot_settings = PlottingSettings(
            show_plots=False,
            save_path=self.result_folder,
            save_format='pdf',
            show_grid=True)
        self.mission_info = None

    def calculate_and_check_errors(self):
        """Calcualates all errors for all datasets.

        Inputs:
        - job_dir: base path for input and output data.
        - csv_base_folder: name of the folder within job_dir that contains all
            CSV trajectories exported from the console.
        - additional_dataset_parameters: additional dataset parameters, as given
            by the experiment yaml. This scripts looks for the ground_truth_csv
            entry and the max_allowed_errors block.
        """
        all_checks_passed = True
        for dataset_index, additional_parameters in enumerate(
                self.additional_dataset_parameters):
            end_to_end_test = EndToEndTest()
            ground_truth_trajectory = load_dataset(
                additional_parameters['ground_truth_csv'],
                input_format=(
                    additional_parameters['ground_truth_csv_format']
                    if 'ground_truth_csv_format' in additional_parameters else
                    'euroc_ground_truth'))

            calculated_errors_map = {}
            dataset = self.datasets[dataset_index]
            dataset_name = os.path.basename(dataset).replace('.bag', '')
            save_folder = os.path.join(self.result_folder, dataset_name)
            for error_comparison_index, e2e_info in enumerate(
                    additional_parameters['max_allowed_errors']):
                label = e2e_info['label']

                max_errors_list = self._get_max_errors_list(
                    e2e_info, calculated_errors_map)
                estimated_trajectory, localizations = self._load_trajectory(
                    dataset, self.dataset_log_dirs[dataset_index], label)

                calculated_errors_map[
                    label] = end_to_end_test.calculate_errors_of_datasets(
                        label,
                        estimated_trajectory,
                        ground_truth_trajectory,
                        estimate_scale=False,
                        max_errors_list=max_errors_list,
                        localization_state_list=localizations)

                save_folder_for_label = os.path.join(
                    save_folder,
                    str(error_comparison_index) + '_' + label)
                self._create_plots_for_single_check(
                    save_folder_for_label, end_to_end_test.test_results[label])
                end_to_end_test.export_results_to_file_for_label(
                    os.path.join(save_folder_for_label, 'errors.yaml'), label)

            end_to_end_test.print_errors()

            # Create position error (one for all values of dataset).
            self.plot_settings.save_path = save_folder
            plot_position_error(end_to_end_test.test_results,
                                self.plot_settings)

            # This will assert if one of the errors is above the threshold and
            # notify the evaluation framework that the test has failed.
            try:
                end_to_end_test.check_errors()
            except ThresholdExceededException as e:
                all_checks_passed = False
                print(e)

        assert all_checks_passed

    def _get_max_errors_list(self, e2e_info, calculated_errors_map):
        max_errors = TestErrorStruct()
        if "max_position_rmse_m" in e2e_info:
            max_errors.position_rmse_m = e2e_info["max_position_rmse_m"]
        if "max_orientation_rmse_rad" in e2e_info:
            max_errors.orientation_rmse_rad = e2e_info[
                "max_orientation_rmse_rad"]
        max_errors_list = [max_errors]
        if 'better_than' in e2e_info:
            for better_than_info in e2e_info['better_than']:
                assert 'label' in better_than_info
                max_errors_better_than = calculated_errors_map[
                    better_than_info['label']]
                # better_than only affects position errors because small
                # variances are easily possible for orientation errors.
                max_errors_better_than.orientation_mean_rad = -1
                max_errors_better_than.orientation_rmse_rad = -1
                if 'factor' in better_than_info:
                    factor = better_than_info['factor']
                    max_errors_better_than.position_mean_m *= factor
                    max_errors_better_than.position_rmse_m *= factor
                max_errors_list.append(max_errors_better_than)

        return max_errors_list

    def _load_trajectory(self, dataset, dataset_log_dir, label):
        """Loads the trajectory from a CSV.

        If the folder <job_dir>/<csv_base_folder>/<label> exists, then the
        trajectory from
        <job_dir>/<csv_base_folder>/<label>/<mission_id>/vertices.csv is loaded.
        Only one mission per csv folder is supported at the moment.
        If the folder <job_dir>/<csv_base_folder>/<label> does not exist, then
        the trajectory from <dataset_log_dir>/trajectory_rovioli.csv is loaded.
        """
        csv_folder = os.path.join(self.job_dir, self.csv_base_folder, label)
        dataset_name = os.path.basename(dataset).replace('.bag', '')
        if not os.path.isdir(csv_folder):
            # Special case -- output is from rovioli and not maplab (so
            # different location). This is a bit hacky.
            path_to_csv_file = os.path.join(dataset_log_dir,
                                            'trajectory_rovioli.csv')
            assert os.path.isfile(path_to_csv_file)
            print(
                'Reference to CSV folder "',
                csv_folder,
                '" which doesn\'t exist. Loading file from "',
                path_to_csv_file,
                '" instead.',
                sep='')
            estimated_trajectory = load_dataset(
                path_to_csv_file, input_format='rovioli')

            # Get localizations. This should get an empty list when there were
            # no localizations.
            localizations = np.genfromtxt(
                path_to_csv_file, delimiter=",", skip_header=1)
            localizations = localizations[:, [0, 15]]
            localizations = localizations[localizations[:, 1] == 1, :]
        else:
            if self.mission_info_file:
                assert os.path.exists(self.mission_info_file), (
                    'Mission info file under "' + self.mission_info_file +
                    '" not found.')
                # CSV is from a multi-mission map, need to read mission info
                # file.
                if not hasattr(self, 'mission_info'):
                    # Load mission info.
                    self.mission_info = yaml.load(open(self.mission_info_file))
                if not dataset_name in self.mission_info:
                    raise ValueError('The mission info file has no entry for '
                                     'the mission from map "' + dataset_name +
                                     '" (full path: "' + dataset + '").')
                map_mission_info = self.mission_info[dataset_name]
                assert len(map_mission_info) == 1, (
                    'To be able to get the proper mission id, the entry for '
                    'the map "' + dataset_name +
                    '" should only contain one entry. Please export the '
                    'mission info after loading ALL maps into the console, but '
                    'before any merging is done.')
                mission_id = map_mission_info[0]['id']
            else:
                # No mission info file: assume there is only one mission and
                # ground truth file per experiment/map.
                all_files_in_csv_folder = os.listdir(csv_folder)
                assert len(all_files_in_csv_folder) == 1, (
                    'The folder %s contains more than one element. This '
                    'indicates a multi-mission map. For multi-mission maps, '
                    'please provide a mission info file.') % (csv_folder)
                mission_id = all_files_in_csv_folder[0]
            path_to_csv_file = os.path.join(csv_folder, mission_id,
                                            'vertices.csv')
            print('Using CSV file from "', path_to_csv_file, '".', sep='')
            estimated_trajectory = load_dataset(path_to_csv_file)
            localizations = np.zeros((0, 2))

        return estimated_trajectory, localizations

    def _create_plots_for_single_check(self, save_folder, test_result):
        """Creates plots for a single CSV comparison.

        Input:
        - save_folder: name of the folder to save the results in.
        - test_result: the actual test result from the end-to-end comparison.
        """
        self.plot_settings.save_path = save_folder
        if not os.path.isdir(self.plot_settings.save_path):
            os.makedirs(self.plot_settings.save_path)
        plot_top_down_aligned_data(test_result, self.plot_settings)
        plot_trajectory_3d_aligned_data(test_result, self.plot_settings)
        plot_xyz_vs_groundtruth(test_result, self.plot_settings)
        plot_xyz_errors(test_result, self.plot_settings)


if __name__ == "__main__":
    arg_parser = ArgParser()
    arg_parser.parser.add_argument('--csv_base_folder', default='')
    arg_parser.parser.add_argument('--mission_info_file', default='')
    parsed_args = arg_parser.parser.parse_args()

    ground_truth_evaluation = GroundTruthTrajectoryEvaluation(parsed_args)
    ground_truth_evaluation.calculate_and_check_errors()
