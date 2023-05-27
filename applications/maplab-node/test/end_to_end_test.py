#!/usr/bin/env python
from __future__ import print_function

import urllib

from evo.core import sync, metrics, trajectory
from evo.tools import file_interface

import maplab_common.bash_utils
from maplab_common.roscore_handling import Roscore

MAX_POSITION_RMSE_RAW_M = 0.105
MAX_POSITION_RMSE_OPT_M = 0.050
MAX_POSITION_RMSE_VIL_M = 0.065


def test_maplab_node_end_to_end():
    sensor_config_file = "end_to_end_test/euroc-mono.yaml"
    ground_truth_data_path = "end_to_end_test/V1_01_ground_truth.csv"

    rosbag_local_path = "dataset.bag"
    download_url = \
        "http://robotics.ethz.ch/~asl-datasets/maplab/test_data/V1_01_easy.bag"
    urllib.urlretrieve(download_url, rosbag_local_path)

    roscore = Roscore()
    roscore.run()

    # Run map builder node
    estimator_csv_path = "maplab_node_estimated_poses.csv"
    output_map_path = "output_map"

    maplab_common.bash_utils.run("rosrun maplab_node maplab_node "
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
                                 (sensor_config_file, rosbag_local_path,
                                  estimator_csv_path, output_map_path))

    traj_ref = file_interface.read_tum_trajectory_file(ground_truth_data_path)
    traj_est = file_interface.read_tum_trajectory_file(estimator_csv_path)

    traj_ref_sync, traj_est_sync = sync.associate_trajectories(
        traj_ref, traj_est, 0.01)
    traj_est_aligned = trajectory.align_trajectory(
        traj_est_sync,
        traj_ref_sync,
        correct_scale=False,
        correct_only_scale=False)

    # Calculate the metrics
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data((traj_ref_sync, traj_est_aligned))
    ape_stats = ape_metric.get_all_statistics()
    print("RMSE RAW", ape_stats['rmse'])
    assert ape_stats['rmse'] < MAX_POSITION_RMSE_RAW_M

    # Check map consistency and test basic maplab console commands on the map
    optimized_map_path = output_map_path + '_optvi'
    with open("maplab_commands.in", "w") as stdin:
        stdin.write("load --map_folder=\"%s\"\n"
                    "check_map_consistency\n"
                    "itl\n"
                    "rtl\n"
                    "kfh\n"
                    "check_map_consistency\n"
                    "optvi --ba_num_iterations=20\n"
                    "check_map_consistency\n"
                    "elq\n"
                    "lc\n"
                    "check_map_consistency\n"
                    "optvi --ba_num_iterations=20\n"
                    "check_map_consistency\n"
                    "save --map_folder \"%s\" --overwrite\n"
                    "export_trajectory_to_csv --csv_export_format=rpg\n"
                    "exit\n" % (output_map_path, optimized_map_path))

    with open("maplab_commands.in", "r") as stdin:
        maplab_common.bash_utils.run(
            "rosrun maplab_console maplab_console "
            "__ns:=maplab_console "
            "_alsologtostderr:=true ",
            stdin=stdin)

    traj_est = file_interface.read_tum_trajectory_file(
        optimized_map_path + '/stamped_traj_estimate.txt')

    traj_ref_sync, traj_est_sync = sync.associate_trajectories(
        traj_ref, traj_est, 0.01)
    traj_est_aligned = trajectory.align_trajectory(
        traj_est_sync,
        traj_ref_sync,
        correct_scale=False,
        correct_only_scale=False)

    # Calculate the metrics
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data((traj_ref_sync, traj_est_aligned))
    ape_stats = ape_metric.get_all_statistics()
    print("RMSE OPT", ape_stats['rmse'])
    assert ape_stats['rmse'] < MAX_POSITION_RMSE_OPT_M

    # Run rovioli in localization mode with optimized map
    estimator_vil_csv_path = "rovioli_estimated_poses_vil.csv"
    maplab_common.bash_utils.run("rosrun rovioli rovioli "
                                 "_alsologtostderr:=true "
                                 "_sensor_calibration_file:=\"%s\" "
                                 "_datasource_type:=rosbag "
                                 "_datasource_rosbag:=\"%s\" "
                                 "_export_estimated_poses_to_csv:=\"%s\" "
                                 "_rovioli_run_map_builder:=true "
                                 "_vio_localization_map_folder:=\"%s\" "
                                 "_rovioli_zero_initial_timestamps:=false "
                                 "_rovio_enable_frame_visualization:=false " %
                                 (sensor_config_file, rosbag_local_path,
                                  estimator_vil_csv_path, optimized_map_path))

    traj_ref = file_interface.read_tum_trajectory_file(ground_truth_data_path)
    traj_est = file_interface.read_tum_trajectory_file(estimator_vil_csv_path)

    traj_ref_sync, traj_est_sync = sync.associate_trajectories(
        traj_ref, traj_est, 0.01)
    traj_est_aligned = trajectory.align_trajectory(
        traj_est_sync,
        traj_ref_sync,
        correct_scale=False,
        correct_only_scale=False)

    # Calculate the metrics
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data((traj_ref_sync, traj_est_aligned))
    ape_stats = ape_metric.get_all_statistics()
    print("RMSE VIL", ape_stats['rmse'])
    assert ape_stats['rmse'] < MAX_POSITION_RMSE_VIL_M

    # We don't need the ros stuff anymore -> kill it
    roscore.terminate()


if __name__ == '__main__':
    test_maplab_node_end_to_end()
