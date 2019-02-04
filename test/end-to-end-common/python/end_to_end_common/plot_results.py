#!/usr/bin/env python

import argparse

import numpy as np

from end_to_end_common.end_to_end_utility import align_datasets
from end_to_end_common.plotting_common import PlottingSettings
from end_to_end_common.create_plots import *

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Plot evaluation results.")
  parser.add_argument(
      "--ground_truth_csv",
      dest="ground_truth_csv",
      required=True)
  parser.add_argument(
      "--estimator_trajectory_csv",
      dest="estimator_trajectory_csv",
      required=True)
  parser.add_argument(
      "--estimator_csv_format",
      dest="estimator_csv_format",
      required=False,
      default="maplab_console")
  parser.add_argument(
      "--plot_title",
      dest="plot_title",
      required=False,
      default="",
      help="If unspecified, the estimator file path is used as a title.")

  parser.add_argument(
      "--create_rmse_plot",
      dest="create_rmse_plot",
      required=False,
      default=False,
      action="store_true",
      help="Create a plot of the RMSE over the total traversed trajectory.")
  parser.add_argument(
      "--create_top_down",
      dest="create_top_down",
      required=False,
      default=False,
      action="store_true",
      help="Creates a top-down (x-y) plot comparing the estimated trajectory with the ground truth.")
  parser.add_argument(
      "--create_trajectory_3d",
      dest="create_trajectory_3d",
      required=False,
      default=False,
      action="store_true",
      help="Creates a 3d plot comparing the estimated trajectory to the ground truth.")
  parser.add_argument(
      "--create_xyz_vs_ground_truth",
      dest="create_xyz_vs_ground_truth",
      required=False,
      default=False,
      action="store_true",
      help="Creates a plot comparing the individual x, y and z estimates against the ground truth.")
  parser.add_argument(
      "--create_xyz_errors",
      dest="create_xyz_errors",
      required=False,
      default=False,
      action="store_true",
      help="Creates a plot showing the relative x, y and z errors.")

  parser.add_argument(
      "--do_not_show_plots",
      dest="do_not_show_plots",
      required=False,
      default=False,
      action="store_true",
      help="If set, the plots won't be shown. (Default is to show the plots.)")
  parser.add_argument(
      "--save_path",
      dest="save_path",
      required=False,
      default="",
      help="If specified, all plots will be saved in the given path.")
  parser.add_argument(
      "--save_format",
      dest="save_format",
      required=False,
      default="pdf",
      help="Specifies in which format matplotlib should save the plots.")

  parsed_args = parser.parse_args()

  plotting_settings = PlottingSettings(
      not parsed_args.do_not_show_plots, parsed_args.save_path,
      parsed_args.save_format)

  estimator_data_G_I, ground_truth_data_G_M = align_datasets(
      parsed_args.estimator_trajectory_csv, parsed_args.ground_truth_csv,
      estimator_input_format=parsed_args.estimator_csv_format)

  # Zero timestamps.
  initial_timestamp = min(
      np.min(estimator_data_G_I[:, 0]), np.min(ground_truth_data_G_M[:, 0]))
  estimator_data_G_I[:, 0] -= initial_timestamp
  ground_truth_data_G_M[:, 0] -= initial_timestamp

  plot_title = parsed_args.plot_title if len(parsed_args.plot_title) > 0 \
      else parsed_args.estimator_trajectory_csv
  print plot_title
  print len(parsed_args.plot_title)

  if parsed_args.create_rmse_plot:
    plot_position_error(
        [plot_title], [ground_truth_data_G_M],
        [estimator_data_G_I], localization_state_list=[np.zeros((0, 2))],
        plotting_settings=plotting_settings)

  if parsed_args.create_top_down:
    plot_top_down_aligned_data(
        estimator_data_G_I, ground_truth_data_G_M,
        title=plot_title, plotting_settings=plotting_settings)

  if parsed_args.create_trajectory_3d:
    plot_trajectory3d_aligned_data(
        estimator_data_G_I, ground_truth_data_G_M,
        title=plot_title, plotting_settings=plotting_settings)

  if parsed_args.create_xyz_vs_ground_truth:
    plot_xyz_vs_groundtruth(
        estimator_data_G_I, ground_truth_data_G_M,
        title=plot_title, plotting_settings=plotting_settings)

  if parsed_args.create_xyz_errors:
    plot_xyz_errors(
        estimator_data_G_I, ground_truth_data_G_M,
        title=plot_title, plotting_settings=plotting_settings)
