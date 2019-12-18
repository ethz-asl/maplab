#!/usr/bin/env python

import os
import numpy as np
# Force matplotlib to not use any Xwindows backend. Needs to be imported before
# any other matplotlib import (time_alignment and end_to_end_test import
# matplotlib).
import matplotlib
matplotlib.use('Agg')

from end_to_end_common.dataset_loader import load_dataset
from end_to_end_common.create_plots import plot_position_error
from end_to_end_common.roscore_handling import Roscore
import end_to_end_common.download_helpers
import end_to_end_common.end_to_end_test
import end_to_end_common.bash_utils

# If set to false, the MaplabNode isn't run in a given case.
# A previous CSV file with
# the estimated trajectory has to exist in that case.
# VIO = the MaplabNode without localization (i.e., the odometry)
# VIL = the MaplabNode with localization
RUN_ESTIMATOR_VIO = True
RUN_ESTIMATOR_VIL = True

VIO_MAX_POSITION_RMSE_M = 0.08
VIL_MAX_POSITION_RMSE_M = 0.05
VIO_MAX_ORIENTATION_RMSE_RAD = 0.07
VIL_MAX_ORIENTATION_RMSE_RAD = VIO_MAX_ORIENTATION_RMSE_RAD


def test_maplab_node_end_to_end():

    sensor_config_file = "end_to_end_test/euroc-mono.yaml"
    rosbag_local_path = "dataset.bag"
    download_url = \
            "http://robotics.ethz.ch/~asl-datasets/maplab/test_data/end_to_end_tests/V1_01_short_rovioli.bag" # pylint: disable=line-too-long
    end_to_end_common.download_helpers.download_dataset(
        download_url, rosbag_local_path)
    ground_truth_data_path = "end_to_end_test/V1_01_easy_ground_truth.csv"

    end_to_end_test = end_to_end_common.end_to_end_test.EndToEndTest()

    vio_max_allowed_errors = end_to_end_common.end_to_end_test.TestErrorStruct(
        VIO_MAX_POSITION_RMSE_M, VIO_MAX_POSITION_RMSE_M,
        VIO_MAX_ORIENTATION_RMSE_RAD, VIO_MAX_ORIENTATION_RMSE_RAD)
    vil_max_allowed_errors = end_to_end_common.end_to_end_test.TestErrorStruct(
        VIL_MAX_POSITION_RMSE_M, VIL_MAX_POSITION_RMSE_M,
        VIL_MAX_ORIENTATION_RMSE_RAD, VIL_MAX_ORIENTATION_RMSE_RAD)

    roscore = Roscore()
    roscore.run()

    # Run estimator VIO only mode.
    estimator_vio_csv_path = "maplab_node_estimated_poses_vio.csv"
    vio_output_map_path = "vio_output_map"

    if RUN_ESTIMATOR_VIO:
        end_to_end_common.bash_utils.run(
            "rosrun maplab_node maplab_node "
            "__ns:=maplab_node "
            "_alsologtostderr:=true "
            "_sensor_calibration_file:=\"%s\" "
            "_datasource_type:=rosbag "
            "_datasource_rosbag:=\"%s\" "
            "_enable_visual_localization:=false "
            "_export_estimated_poses_to_csv:=\"%s\" "
            "_map_save_on_shutdown:=true "
            "_map_overwrite_enabled:=true "
            "_map_output_folder:=\"%s\" " %
            (sensor_config_file, rosbag_local_path, estimator_vio_csv_path,
             vio_output_map_path))

    # Compare estimator csv - ground truth data.
    estimator_data_vio_unaligned_G_I = load_dataset(
        estimator_vio_csv_path, input_format="rovioli")
    assert estimator_data_vio_unaligned_G_I.shape[0] > 0, """Estimator failed to run properly.
        Make sure that the sensor.yaml file matches the rosbag contents."""

    ground_truth_data_unaligned_W_M = load_dataset(
        ground_truth_data_path, input_format="euroc_ground_truth")

    vio_errors = end_to_end_test.calculate_errors_of_datasets(
        "maplab_node_VIO",
        estimator_data_vio_unaligned_G_I,
        ground_truth_data_unaligned_W_M,
        estimate_scale=False,
        max_errors_list=[vio_max_allowed_errors])

    # Check map consistency and test basic maplab console commands on the map
    with open("maplab_commands.in", "w") as stdin:
        stdin.write("load --map_folder=\"%s\"\n"
                    "check_map_consistency\n"
                    "itl\n"
                    "rtl\n"
                    "kfh\n"
                    "check_map_consistency\n"
                    "optvi --ba_num_iterations=3\n"
                    "check_map_consistency\n"
                    "elq\n"
                    "lc\n"
                    "check_map_consistency\n"
                    "optvi --ba_num_iterations=3\n"
                    "check_map_consistency\n"
                    "save --map_folder \"%s_optvi\"\n"
                    "exit\n" % (vio_output_map_path, vio_output_map_path))

    with open("maplab_commands.in", "r") as stdin:
        end_to_end_common.bash_utils.run(
            "rosrun maplab_console maplab_console "
            "__ns:=maplab_console "
            "_alsologtostderr:=true ",
            stdin=stdin)

    # Run estimator VIL mode.
    estimator_vil_csv_path = "maplab_node_estimated_poses_vil.csv"
    localization_reference_map = "V1_01_maplab_node_summary_map"
    if not os.path.isdir(localization_reference_map):
        os.mkdir(localization_reference_map)
    localization_reference_map_download_path = \
            "http://robotics.ethz.ch/~asl-datasets/maplab/test_data/end_to_end_tests/ml_node_localization_summary_map" # pylint: disable=line-too-long
    end_to_end_common.download_helpers.download_dataset(
        localization_reference_map_download_path,
        os.path.join(localization_reference_map, "localization_summary_map"))

    if RUN_ESTIMATOR_VIL:
        end_to_end_common.bash_utils.run(
            "rosrun maplab_node maplab_node "
            "__ns:=maplab_node "
            "_alsologtostderr:=true "
            "_sensor_calibration_file:=\"%s\" "
            "_datasource_type:=rosbag "
            "_datasource_rosbag:=\"%s\" "
            "_enable_visual_localization:=true "
            "_visual_localization_map_folder:=\"%s\" "
            "_export_estimated_poses_to_csv:=\"%s\" "
            "_map_save_on_shutdown:=true "
            "_map_overwrite_enabled:=true "
            "_map_output_folder:=\"\" " %
            (sensor_config_file, rosbag_local_path, localization_reference_map,
             estimator_vil_csv_path))

    # We don't need the ros stuff anymore -> kill it
    roscore.terminate()

    assert os.path.isfile(
        estimator_vil_csv_path
    ), "File " + estimator_vil_csv_path + " doesn't exist."

    # Ensure that VIL position errors are smaller than the VIO position errors.
    # For the orientation, we don't want to enforce such a requirement as the
    # differences are usually smaller.
    vio_position_errors = end_to_end_common.end_to_end_test.TestErrorStruct(
        vio_errors.position_mean_m, vio_errors.position_rmse_m,
        VIO_MAX_ORIENTATION_RMSE_RAD, VIO_MAX_ORIENTATION_RMSE_RAD)

    # Get localizations for VIL case.
    vil_localizations = np.genfromtxt(
        estimator_vil_csv_path, delimiter=",", skip_header=1)
    assert vil_localizations.shape[0] > 0, """Estimator failed to run properly in localization mode.
        Make sure that the sensor.yaml file matches the rosbag contents."""

    vil_localizations = vil_localizations[:, [0, 15]]
    vil_localizations = vil_localizations[vil_localizations[:, 1] == 1, :]

    estimator_data_vil_unaligned_G_I = load_dataset(
        estimator_vil_csv_path, input_format="rovioli")
    vil_errors = end_to_end_test.calculate_errors_of_datasets(
        "maplab_node_VIL",
        estimator_data_vil_unaligned_G_I,
        ground_truth_data_unaligned_W_M,
        estimate_scale=False,
        max_errors_list=[vil_max_allowed_errors, vio_position_errors],
        localization_state_list=vil_localizations)

    # Get VIWLS errors.
    viwls_csv_path = "end_to_end_test/V1_01_easy_short_viwls_maplab_node_vertices.csv"  # pylint: disable=line-too-long
    vil_position_errors = end_to_end_common.end_to_end_test.TestErrorStruct(
        vil_errors.position_mean_m, vil_errors.position_rmse_m,
        VIL_MAX_ORIENTATION_RMSE_RAD, VIL_MAX_ORIENTATION_RMSE_RAD)
    estimator_data_viwls_unaligned_G_I = load_dataset(viwls_csv_path)
    end_to_end_test.calculate_errors_of_datasets(
        "maplab_node_VIWLS",
        estimator_data_viwls_unaligned_G_I,
        ground_truth_data_unaligned_W_M,
        estimate_scale=False,
        max_errors_list=[
            vio_max_allowed_errors, vio_position_errors, vil_position_errors
        ])

    end_to_end_test.print_errors()
    end_to_end_test.check_errors()

    plot_position_error(end_to_end_test.test_results)


if __name__ == '__main__':
    test_maplab_node_end_to_end()
