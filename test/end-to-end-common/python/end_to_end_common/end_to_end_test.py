#!/usr/bin/env python

from __future__ import print_function

import argparse
import copy
import os
import sys

import numpy as np

import end_to_end_common.compute_rmse as rmse
import end_to_end_common.create_plots as plots
from end_to_end_common.dataset_loader import load_dataset
from end_to_end_common.end_to_end_utility import align_datasets
from end_to_end_common.plotting_common import PlottingSettings
from end_to_end_common.test_structs import TestDataStruct, TestErrorStruct


def rad_to_deg(rad):
    return rad * 180 / np.pi


class EndToEndTest(object):
    def __init__(self):
        self.test_results = {}

    def calculate_errors_of_datasets(self,
                                     label,
                                     estimated_poses_csv_G_I,
                                     ground_truth_csv_W_M,
                                     estimate_scale=False,
                                     align_data_to_use_meters=-1,
                                     time_offset=0.0,
                                     marker_calibration_file=None,
                                     horizontal_errors_only=False,
                                     max_errors_list=None,
                                     localization_state_list=np.zeros((0, 2))):
        """Calculates the errors between the given datasets and appends the
        result to an internal list.

        Input:
          - label: name of the dataset that is compared. Used for later output
                and plotting.
          - estimated_poses_csv_G_I: path to the CSV containing the estimated
                poses from ROVIOLI or another estimator.
          - ground_truth_csv_W_M: path to the CSV containing the ground truth
                values for this dataset.
          - estimate_scale: Estimate a scale factor in addition to the transform
                and apply it to the aligned estimator trajectory.
          - align_data_to_use_meters: (optional) only use data up to a total
                trajectory length as given by this parameter. If a negative
                value is provided, the entire trajectory is used for the
                alignment.
          - time_offset: (optional) Time offset in seconds by which the
                estimator time stamps will be shifted in the alignment.
          - marker_calibration_file: Contains the calibration between the marker
                frame M and the eye frame I. If no file is given, the frames M
                and I are assumed to be identical.
          - horizontal_errors_only: Only the horizontal errors (x-y plane) are
                calculated.
          - max_errors_list: list containing TestErrorStruct which this dataset
                should test against.
          - localization_state_list: (optional) Nx2-numpy-array where the first
                column contains timestamps and the second column localization
                results (1 = successful localization, 0 = no localization). The
                timestamps should match with the values in the estimator poses
                CSV file.
        Returns:
          A TestErrorStruct struct containing the errors of the given dataset.
        """
        # TODO(eggerk): explain CSV formats somewhere.
        assert label not in self.test_results, \
            "A dataset with the label " + label + " was already run."
        if max_errors_list is None:
            max_errors_list = []

        estimator_data_W_I, ground_truth_data_W_M = align_datasets(
            estimated_poses_csv_G_I,
            ground_truth_csv_W_M,
            estimate_scale,
            align_data_to_use_meters=align_data_to_use_meters,
            time_offset=time_offset,
            marker_calibration_file=marker_calibration_file)

        position_errors_m = rmse.compute_position_mean_and_rmse(
            estimator_data_W_I[:, 1:4],
            ground_truth_data_W_M[:, 1:4],
            position_error_type=("total" if not horizontal_errors_only else
                                 "horizontal"))
        orientation_errors_rad = rmse.compute_orientation_mean_and_rmse(
            estimator_data_W_I[:, 4:8], ground_truth_data_W_M[:, 4:8])

        axis_position_errors = []
        axis_names = ["x", "y", "z"]
        for axis_name in axis_names:
            current_axis_errors = rmse.compute_position_mean_and_rmse(
                estimator_data_W_I[:, 1:4],
                ground_truth_data_W_M[:, 1:4],
                position_error_type=axis_name)
            axis_position_errors.append(current_axis_errors)

        errors = TestErrorStruct()
        errors.set_position_errors(position_errors_m)
        errors.set_orientation_errors(orientation_errors_rad)
        errors.axis_position_errors = axis_position_errors
        result = TestDataStruct(
            estimator_data_W_I,
            ground_truth_data_W_M,
            label=label,
            calculated_errors=errors,
            max_errors_list=max_errors_list,
            localization_state_list=localization_state_list)

        self.test_results.update({label: result})
        return copy.copy(errors)

    def print_errors(self):
        """Prints all test errors (position and orientation mean and RMSE)."""
        for label, result in self.test_results.iteritems():
            print("\nErrors of dataset ", label, ":", sep="")
            print("    position [m]:\tmean =",
                  result.calculated_errors.position_mean_m, "\tRMSE =",
                  result.calculated_errors.position_rmse_m, "\tmin =",
                  result.calculated_errors.position_min_error_m, "\tmax =",
                  result.calculated_errors.position_max_error_m)
            if result.max_errors.position_mean_m >= 0 or \
                result.max_errors.position_rmse_m >= 0:
                print(
                    "        (Thresholds:\tmean = ",
                    result.max_errors.position_mean_m,
                    "\tRMSE = ",
                    result.max_errors.position_rmse_m,
                    ")",
                    sep="")

            print(
                "    orientation [deg]:\tmean =",
                rad_to_deg(
                    result.calculated_errors.orientation_mean_rad), "\tRMSE =",
                rad_to_deg(
                    result.calculated_errors.orientation_rmse_rad), "\tmin =",
                rad_to_deg(result.calculated_errors.orientation_min_error_rad),
                "\tmax =",
                rad_to_deg(result.calculated_errors.orientation_max_error_rad))
            if result.max_errors.orientation_mean_rad >= 0 or \
                result.max_errors.orientation_rmse_rad >= 0:
                print(
                    "        (Thresholds:\tmean = ",
                    rad_to_deg(result.max_errors.orientation_mean_rad),
                    "\tRMSE = ",
                    rad_to_deg(result.max_errors.orientation_rmse_rad),
                    ")",
                    sep="")

            all_axes = ['x', 'y', 'z']
            for axis_idx, axis_name in enumerate(all_axes):
                print(
                    axis_name,
                    ' errors:\n    ',
                    result.calculated_errors.axis_position_errors[axis_idx],
                    sep='')

            if result.cpu_mean > 0 and result.cpu_stddev >= 0:
                print("    CPU load [%]:\tmean:", result.cpu_mean,
                      "\tstd dev:", result.cpu_stddev)

            print("=" * 80)

    def check_errors(self):
        """Compares each of the stored test results if its values are below the
        given maximum test errors.
        """
        for label, result in self.test_results.iteritems():
            print("Checking results from ", label, ".", sep="")
            result.check_errors()

        print("Done.")

    def get_output_file_name(self, export_path, label):
        if os.path.isfile(label):
            file_name = os.path.basename(label)
        else:
            file_name = label.replace("/", "_")
        return os.path.join(export_path, file_name + "_errors.yaml")

    def export_results_to_file(self, export_path):
        assert export_path
        for label, _ in self.test_results.iteritems():
            path = self.get_output_file_name(export_path, label)
            self.export_results_to_file_for_label(path, label)

    def export_results_to_file_for_label(self, export_file_name, label):
        if not os.path.isdir(os.path.dirname(export_file_name)):
            os.makedirs(os.path.dirname(export_file_name))
        errors = self.test_results[label].calculated_errors
        error_thresholds = self.test_results[label].max_errors
        out_file = open(export_file_name, "w+")
        out_file.write("metadata:\n")
        out_file.write("  label: %s\n" % label)
        out_file.write("position_errors:  # [m]\n")
        out_file.write("  mean: %f\n" % errors.position_mean_m)
        out_file.write("  rmse: %f\n" % errors.position_rmse_m)
        out_file.write("  min: %f\n" % errors.position_min_error_m)
        out_file.write("  max: %f\n" % errors.position_max_error_m)
        if error_thresholds.position_mean_m > 0 or \
            error_thresholds.position_rmse_m > 0:
            out_file.write("  thresholds:\n")
            if error_thresholds.position_mean_m > 0:
                out_file.write(
                    "    mean: %f\n" % error_thresholds.position_mean_m)
            if error_thresholds.position_rmse_m > 0:
                out_file.write(
                    "    rmse: %f\n" % error_thresholds.position_rmse_m)

        all_axes = ['x', 'y', 'z']
        for axis_idx, axis_name in enumerate(all_axes):
            axis_errors = errors.axis_position_errors[axis_idx]
            out_file.write('  %s:\n' % axis_name)
            out_file.write('    mean: %f\n' % axis_errors.mean)
            out_file.write('    rmse: %f\n' % axis_errors.rmse)
            out_file.write('    min: %f\n' % axis_errors.min_value)
            out_file.write('    max: %f\n' % axis_errors.max_value)

        out_file.write("orientation_errors:  # [rad]\n")
        out_file.write("  mean: %f\n" % errors.orientation_mean_rad)
        out_file.write("  rmse: %f\n" % errors.orientation_rmse_rad)
        out_file.write("  min: %f\n" % errors.orientation_min_error_rad)
        out_file.write("  max: %f\n" % errors.orientation_max_error_rad)
        if error_thresholds.orientation_mean_rad > 0 or \
            error_thresholds.orientation_rmse_rad > 0:
            out_file.write("  thresholds:\n")
            if error_thresholds.orientation_mean_rad > 0:
                out_file.write(
                    "    mean: %f\n" % error_thresholds.orientation_mean_rad)
            if error_thresholds.orientation_rmse_rad > 0:
                out_file.write(
                    "    rmse: %f\n" % error_thresholds.orientation_rmse_rad)
        out_file.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="End to end test")
    parser.add_argument("--ground-truth-csv", required=True)
    parser.add_argument("--estimator-csv", required=True)
    parser.add_argument("--ground-truth-csv-format", default="end_to_end")
    parser.add_argument("--estimator-csv-format", default="maplab_console")
    parser.add_argument(
        "--marker-calibration-file",
        help="Output of the hand_eye_calibration algorithm which contains the "
        "transform from marker frame to camera/IMU frame (hand to eye).")
    parser.add_argument(
        "--horizontal-errors-only",
        action="store_true",
        help="If set, only the horizontal errors are computed and visualized.")
    parser.add_argument(
        "--estimate-scale",
        action="store_true",
        help=
        "If set, the scale between the datasets is estimated and corrected. "
        "This is turned off by default.")
    parser.add_argument(
        "--time-offset",
        default=0.0,
        type=float,
        help="Time offset in seconds by which the estimator time stamps will "
        "be shifted in the alignment.")
    parser.add_argument(
        "--dataset-label",
        help="Dataset label. If not provided, the name of the estimator csv "
        "file will be used.")
    parser.add_argument(
        "--no-visualization",
        action="store_true",
        help="If set, the plot visualization is suppressed.")
    parser.add_argument(
        "--save-path",
        default="",
        help="If provided, will save plots and the error statistics in this "
        "directory.")
    parser.add_argument(
        "--plot-save-format",
        default="pdf",
        help=
        "Format in which the plots should be saved in. Check with matplotlib "
        "for a list of supported formats.")

    parsed_args = parser.parse_args()

    assert os.path.isfile(parsed_args.estimator_csv)
    assert os.path.isfile(parsed_args.ground_truth_csv)

    estimator_trajectory = load_dataset(
        parsed_args.estimator_csv,
        input_format=parsed_args.estimator_csv_format)
    ground_truth_trajectory = load_dataset(
        parsed_args.ground_truth_csv,
        input_format=parsed_args.ground_truth_csv_format)

    if parsed_args.dataset_label:
        dataset_label = parsed_args.dataset_label
    else:
        dataset_label = os.path.basename(parsed_args.estimator_csv).replace(
            '.csv', '')

    end_to_end_test = EndToEndTest()
    end_to_end_test.calculate_errors_of_datasets(
        dataset_label,
        estimator_trajectory,
        ground_truth_trajectory,
        parsed_args.estimate_scale,
        align_data_to_use_meters=-1,
        time_offset=parsed_args.time_offset,
        marker_calibration_file=parsed_args.marker_calibration_file,
        horizontal_errors_only=parsed_args.horizontal_errors_only)
    end_to_end_test.print_errors()

    exit_code = 0
    try:
        end_to_end_test.check_errors()
    except:  # pylint: disable=bare-except
        print("RMSE of dataset is worse than allowed.")
        exit_code += 1

    plotting_settings = PlottingSettings(
        show_plots=(not parsed_args.no_visualization),
        save_path=parsed_args.save_path,
        save_format=parsed_args.plot_save_format,
        show_grid=True)
    plotting_settings.trajectory_3d_axis_equal = True

    data = end_to_end_test.test_results[dataset_label]

    plots.plot_position_error(
        [data],
        plotting_settings,
        horizontal_errors_only=parsed_args.horizontal_errors_only)
    plots.plot_trajectory_3d_aligned_data(data, plotting_settings)
    plots.plot_xyz_vs_groundtruth(data, plotting_settings)
    plots.plot_xyz_errors(data, plotting_settings)

    if parsed_args.save_path:
        end_to_end_test.export_results_to_file(parsed_args.save_path)

    sys.exit(exit_code)
