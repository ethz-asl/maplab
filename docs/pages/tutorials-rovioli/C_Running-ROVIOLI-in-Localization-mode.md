## Running ROVIOLI in Localization mode

In this tutorial, we will run ROVIOLI in Localization mode.
This can greatly improve the estimator accuracy and reduce the drift of the estimated trajectory.

### Requirements
As in the VIO case, you need to get the three calibration files (camera, IMU maplab, and IMU rovio) for your sensor setup.
Furthermore, you also need:
- A VI map to localize against. This map should be optimized and loop-closed to obtain better results in ROVIOLI.
- A dataset to run. Alternatively, a live source can be used.

### Preparing a localization map
There are two ways to get a localization map:
- Manually optimize the VI map from [running ROVIOLI in VIO mode](Running-ROVIOLI-in-VIO-mode) using the maplab console. See the [tutorial about optimizing a map](Preparing-a-single-session-map).
- Alternatively, you can run the rovioli script from the [previous tutorial](Running-ROVIOLI-in-VIO-mode) with the `--optimize_map_to_localization_map` flag set to true:
```bash
source ~/maplab_ws/devel/setup.bash
roscore&
rosrun rovioli tutorial_euroc save_folder_loc_map MH_01_easy.bag --optimize_map_to_localization_map
```

This will save an optimized localization map under `save_folder_loc_map_localization`:
```bash
$ tree save_folder_loc_map_localization
save_folder_loc_map_localization
└── localization_summary_map
```

You can either use a VI map (full map) or a localization summary map (summarized map with only the relevant information for localization) to localize in ROVIOLI.

### Running ROVIOLI with localization
For this tutorial, we use the *Machine Hall 02* dataset from the [Euroc dataset website](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).
As for localization map, we use an optimized VI map of the *Machine Hall 01* dataset that we assume to be saved under `save_folder_loc_map_localization`. (Alternatively, you can use the unoptimized map from the [[Sample Datasets]] page. Note, however, that this will yield worse performance than when using an optimized map.)

Use the [following script](https://github.com/ethz-asl/maplab/blob/master/applications/rovioli/scripts/tutorials/tutorial_euroc_localization):
```bash
#!/usr/bin/env bash
LOCALIZATION_MAP_INPUT=$1
LOCALIZATION_MAP_OUTPUT=$2
ROSBAG=$3
NCAMERA_CALIBRATION="$ROVIO_CONFIG_DIR/ncamera-euroc.yaml"
IMU_PARAMETERS_MAPLAB="$ROVIO_CONFIG_DIR/imu-adis16488.yaml"
IMU_PARAMETERS_ROVIO="$ROVIO_CONFIG_DIR/imu-sigmas-rovio.yaml"
REST=$@

rosrun rovioli rovioli \
  --alsologtostderr=1 \
  --v=2 \
  --ncamera_calibration=$NCAMERA_CALIBRATION  \
  --imu_parameters_maplab=$IMU_PARAMETERS_MAPLAB \
  --imu_parameters_rovio=$IMU_PARAMETERS_ROVIO \
  --publish_debug_markers  \
  --datasource_type="rosbag" \
  --optimize_map_to_localization_map=false \
  --vio_localization_map_folder=$LOCALIZATION_MAP_INPUT \
  --save_map_folder=$LOCALIZATION_MAP_OUTPUT \
  --map_builder_save_image_as_resources=false \
  --datasource_rosbag=$ROSBAG $REST
```

Now, run the script:
```bash
# Make sure that your maplab workspace is sourced!
source ~/maplab_ws/devel/setup.bash
roscore&
rosrun rovioli tutorial_euroc_localization save_folder_loc_map_localization save_map_with_localization MH_02_easy.bag
```

The map under `save_folder_loc_map_localization` is used as reference map for localization.
It can either be a *localization summary map* (generated with `--optimize_map_to_localization_map` in ROVIOLI) or a full VI map (generated from ROVIOLI and optimized with the maplab console).
If the argument is an empty string, ROVIOLI will fall back to VIO mode, i.e., ROVIOLI will not perform any localization.

After ROVIOLI is finished, the VI map will be stored under `save_folder_with_localization`.
If you do not want the resulting VI map to be saved, you can keep the argument emtpy:
```bash
bash run_rovioli_localization "" save_map MH_02_easy.bag
```

### Visualization during ROVIOLI with localization
In addition to the standard ROVIOLI topics ([as explained before](Running-ROVIOLI-in-VIO-mode)), localization mode adds the following visualization topics to RViz:
- `/loopclosure_database`: Point cloud of all landmarks in the loop closure database.
- `/loop_closures`: Lines connecting the current active vertex with all landmarks that have been used for the localization.
- `/loopclosure_inliers`: Point cloud with all landmarks that have been used for the last successful localization.
- `/debug_T_G_I_raw_localization`: Red points indicating the vertex positions where a successful localization occurred.
