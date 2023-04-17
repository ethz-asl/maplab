## Running ROVIOLI in VIO mode: calibartion files, rostopics, bag/topic mode, visualization

In this tutorial, we use ROVIOLI to build a map in the visual-inertial odometry (VIO) mode, i.e., without using any localization.
Two modes exists for running ROVIOLI in VIO mode:
- [Building from a rosbag](#building-a-map-from-a-rosbag)
- [Building from a rostopic](#building-a-map-from-a-rostopic)

### Requirements
To run a ROVIOLI, the following calibration files are required:
- Camera calibration, [example file for the Euroc datasets](https://github.com/ethz-asl/maplab/blob/master/applications/rovioli/share/ncamera-euroc.yaml)
- IMU parameters maplab, [example file for the Euroc datasets](https://github.com/ethz-asl/maplab/blob/master/applications/rovioli/share/imu-adis16488.yaml)
- IMU parameters Rovio, [example file for the Euroc datasets](https://github.com/ethz-asl/maplab/blob/master/applications/rovioli/share/imu-sigmas-rovio.yaml)
- Rovio calibration file ``rovio_default_config.info``, [example file](https://github.com/ethz-asl/maplab/blob/master/applications/rovioli/share/rovio_default_config.info)

See [[Sensor Calibration Format]] for more information on these files. The calibration files should be located in a directory pointed by ``$ROVIO_CONFIG_DIR`` environment variable, the default is ``maplab_ws/src/maplab/applications/rovioli/share``.

In addition, a bag file containing the dataset (see [[Sample Datasets]]) or a live source needs to be available.

### Building a map from a rosbag
For this tutorial, we build a map from one of the Euroc datasets.
Go to the [Euroc dataset website](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) and download the bag file for *Machine Hall 01*.

Use the [following script](https://github.com/ethz-asl/maplab/blob/master/applications/rovioli/scripts/tutorials/tutorial_euroc):
```bash
#!/usr/bin/env bash
LOCALIZATION_MAP_OUTPUT=$1
ROSBAG=$2
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
  --datasource_type="rosbag" \
  --save_map_folder="$LOCALIZATION_MAP_OUTPUT" \
  --optimize_map_to_localization_map=false \
  --map_builder_save_image_as_resources=false \
  --datasource_rosbag=$ROSBAG $REST
```
Note: 
```bash
# Use this flag to remove (or change) the suffix (default: image_raw) from the camera topic.
--vio_camera_topic_suffix=""
```


Now, start a roscore and run the dataset:
```bash
# Make sure that your maplab workspace is sourced!
source ~/maplab_ws/devel/setup.bash
roscore&
rosrun rovioli tutorial_euroc save_folder MH_01_easy.bag
```
where `save_folder` is the name of the folder where the generated VI map will be saved.

After ROVIOLI is finished and the map is saved, `save_folder` should look like this:
```bash
$ tree save_folder
save_folder
├── metadata
├── resource_info
├── resources
└── vi_map
    ├── edges
    ├── landmark_index
    ├── missions
    ├── other_fields
    ├── vertices0
    ├── vertices1
    ├── ...
```

### Building a map from a rostopic
Use the [following script](https://github.com/ethz-asl/maplab/blob/master/applications/rovioli/scripts/tutorials/tutorial_euroc_live):
```bash
#!/usr/bin/env bash
LOCALIZATION_MAP_OUTPUT=$1
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
  --datasource_type="rostopic" \
  --save_map_folder="$LOCALIZATION_MAP_OUTPUT" \
  --map_builder_save_image_as_resources=false \
  --optimize_map_to_localization_map=false $REST
```
Note: 
```bash
# Use this flag to remove (or change) the suffix (default: image_raw) from the camera topic.
--vio_camera_topic_suffix=""
```

Now, using a Euroc dataset as a live source, call:
```bash
# Make sure that your maplab workspace is sourced!
source ~/maplab_ws/devel/setup.bash
roscore&
rosrun rovioli tutorial_euroc_live save_folder
```
Then, in a separate terminal, start your data source:
```bash
rosbag play MH_01_easy.bag  # or start your sensor node.
```
### Output

By default the pose estimates are published on the following rostopics:
- `/maplab_rovio/T_G_I`
- `/maplab_rovio/T_G_M` Identity in the case of no localization.
- `/maplab_rovio/T_M_I`
- `/maplab_rovio/bias_acc`
- `/maplab_rovio/bias_gyro`
- `/maplab_rovio/velocity_I`

### Visualization

Use [this](https://github.com/ethz-asl/maplab/blob/pre_release_public/july-2018/applications/rovioli/share/rviz-rovioli.rviz) RViz config to visualize the most important ROVIOLI topics.

Here is a list of the most important flags to enable/disable visualization of certain components of maplab/ROVIOLI:

#### Visualization of ROVIO features/state

<img src="https://github.com/ethz-asl/maplab/wiki/images/rovioli_viz_rovio.png" width="800">

This OpenCV window visualizes the state and the features computed by ROVIO. These features are **NOT** used to build the VIMap or localization map later, but exist only within ROVIO to perform local state estimation.

Enable/disable using the following flag:
```bash
--rovio_enable_frame_visualization
```
#### Visualization of Maplab feature tracking module

<img src="https://github.com/ethz-asl/maplab/wiki/images/rovioli_viz_maplab_tracking.png" width="800">

The maplab feature tracking module computes and matches Brisk or FREAK features, which are used by the map builder to construct the pose graph/VIMap. The results of the keypoint detection and feature matching can be published as ROS image topics.

Enable/disable using the following flag:
```bash
--feature_tracker_visualize_keypoint_detections
--feature_tracker_visualize_feature_tracks
--feature_tracker_visualize_keypoint_matches
--feature_tracker_visualize_keypoints
--feature_tracker_visualize_keypoints_individual_frames
```
Topics:
```bash
# For each frame N a separate topic is used:
/tracking/keypoints_raw_cam{N}
/tracking/keypoint_matches_camera_{N}
/tracking/keypoint_outlier_matches_camera_{N}
```

#### Visualization of T_M_I and T_G_I

<img src="https://github.com/ethz-asl/maplab/wiki/images/rovioli_viz_debug_markers.png" width="800">

These markers visualize the pose estimates for each frame (odometry = T_M_I, localized = T_G_I).

Enable/disable using the following flag:
```bash
--publish_debug_markers
```
Topics:
```bash
/debug_T_G_I
/debug_T_M_I
```

#### Visualization of VIMap and Localization

<img src="https://github.com/ethz-asl/maplab/wiki/images/rovioli_viz_pose_graph.png" width="400">
<img src="https://github.com/ethz-asl/maplab/wiki/images/rovioli_viz_localization_results.png" width="400">

These markers visualize the VIMap that is being built by ROVIOLI as well as the landmark correspondences found by the localization module.

Enable/disable using the following flag:
```bash
--rovioli_visualize_map
```
Topics:
```bash
# Pose graph
/vi_map_baseframe
/vi_map_landmarks
/vi_map_vertices
/vi_map_edges/viwls

# Localizer results
/landmark_pairs
/loop_closures
/loopclosure_database
/loopclosure_inliers
```
