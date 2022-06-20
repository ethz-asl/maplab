# How to use the semantify plugin
## Preparation
* Install nvidia/cuda drivers if you want to use the gpu: https://www.tensorflow.org/install/source#gpu
  | Software | Version |
  | --- | ---|
  | Ubuntu | 18.04 |
  | nvidia-driver | 510 |
  | cuda | 10.0 |
  | cudnn | 7.4 |

* Install all requirements for Mask R-CNN, NetVLAD, and DeepSort (optionally in a virtual environment)
  ```
  cd <maplab_source>
  pip install -U -r console-plugins/semantify-plugin/requirements.txt
  ```

* Download the required mdoel files:
  ```
  mkdir -p $HOME/models
  wget 
  unzip $HOME/models.zip
  ```

## Attach RGB images resources to map
Use the following flag when building the map
```bash
map_builder_save_color_image_as_resources=true
```
or the following launch file
```bash
roslaunch maplab_launch lidarstick-maplab-node-w-rovioli.launch bag:=\your_bag
```

## Create semantic observations
In order to add semantic observations (bounding boxes and masks) to the map, use
```bash
roslaunch mask_rcnn_ros maplab.launch
```

Make sure the network is loaded before running this command:
```bash
rosrun maplab_console maplab_console
```

In `maplab console`:
Make sure a VI map with RGB images attached is available:
```bash
load --map_folder /path_to_map_resource/
res_stats
semantify
res_stats
save --map_folder path/to/save/the/map --overwrite
```

## Add descriptors to observations
To associate semantic observations with descriptors, one can use netvlad_tf
```bash
cp $HOME/models/netvlad/* $HOME/maplab_ws/src/maplab/dependencies/3rdparty/netvlad_tf_open/checkpoints
rosrun netvlad_tf netvlad_node.py
```

In `maplab_console`, the following command is used to call netvlad:
```bash
generate_and_save_semantic_object_measurements
```

For multi-object association, we use Deep Sort
```bash
rosrun deep_sort_ros deep_sort_node.py
```
In `maplab_console`, the following command is used to call Deep Sort:
```
generate_and_save_semantic_object_track_ids_from_deepsort --semantify_tracking_confidence_threshold 0.85
```

### Tuning parameters:
* Requires 3 consecutive hits to have a confirmed track
* Filters detections by confidence

## Vsualization:
```
visualize_semantic_object_channels_in_visual_frame --semantify_visualization_frequency 20 --semantify_tracking_confidence_threshold 0.85
evaluate_semantic_landmark_with_track_id --semantify_visualization_frequency 0.1 --semantify_semantic_landmark_track_id 1 --map_mission c277fc00e69fcc150d00000000000000  --generate_descriptor_filter_by_mask=false
```
load the rviz from `console-plugins/semantify-basic-plugin/maplab.rviz`

For saving maps:
```
save --map_folder path/to/save/the/map --overwrite --copy_resources_to_map_folder
```
## Triangulate semantic landmarks, need to finish all semantify commands
```
itsl --vi_map_semantic_landmark_quality_min_observation_angle_deg 20
rtsl
```
### Visualization of semantic landmarks
To update the class id and observation of semantic landmarks, currectly not saved in protobuf:
```
update_semantic_landmarks_class_ids
```
To set up the mapping and visualization for semantic landmarks:
```
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
```
## Descriptor clustering
To cluster descriptors and get medoids for each landmarks:
```
generate_descriptor_clusters
```

To show map status and find mission id:
```
ms
```
Some debugging for descriptors:
```
display_descriptor_clusters_scores --map_mission
compare_descriptor_clusters_scores --descriptor_comparison_mission_id_0 c277fc00e69fcc150d00000000000000 --descriptor_comparison_mission_id_1 c277fc00e69fcc150d00000000000000 --descriptor_comparison_semantic_landmark_track_ids_0 2,163 --descriptor_comparison_semantic_landmark_track_ids_1 2,163
```
## Map Anchoring
### Traditional map anchoring
```
load --map_folder /path/to/map 
load_merge_map --map_folder /path/to/map quadric_2_2_with_semantic_landmarks/
ms
```

for visualization:
```
spatially_distribute_missions --spatially_distribute_missions_dimension 2 --spatially_distribute_missions_meters 5
v --vis_color_by_mission
```
set baseframe:
```
sbk --map_mission 85965582c59fcc150d00000000000000
```
check status:
```
ms
```
now mission 0 `T_G_M` is set to the reference and becomes known

Starts anchoring:
```
aam
```
### Semantic landmark map anchoring
```
load --map_folder /path/to/map 
load_merge_map --map_folder /path/to/second/map
ms
sbk --map_mission 85965582c59fcc150d00000000000000
ms
spatially_distribute_missions --spatially_distribute_missions_dimension 2 --spatially_distribute_missions_meters 5
v --vis_color_by_mission
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
calculate_semantic_landmark_covariances
anchor_mission_with_semantic_landmarks --semantic_landmark_max_match_candidate_distance 8 --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.3
```
## Loop Closure
### Semantic landmark loop closure
```
load --map_folder /path/to/map
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map --semantify_semantic_landmark_class_filter "1,73,61,57"
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --semantify_loop_closure_mission_id_ref c3c402890f1ec6150d00000000000000 --semantify_loop_closure_mission_id_source c3c402890f1ec6150d00000000000000 --semantify_semantic_landmark_lc_extend_visible_verticies_num 100 --semantify_semantic_landmark_lc_extend_visible_verticies=true --semantify_semantic_landmark_lc_merge_matched_landmarks=false --semantify_semantic_landmark_lc_add_edge_between_topological_center_vertices=true
optvi --ba_visualize_every_n_iterations 1
rtsl
v
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
```
