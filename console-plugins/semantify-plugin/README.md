# How to use the semantify plugin
## Preparation
Install all requirements for Mask R-CNN, NetVLAD, and DeepSort
```
cd <maplab_source>
pip install -U -r dependencies/3rdparty/mask_rcnn_ros_private/requirements.txt
pip install -U -r dependencies/3rdparty/deep_sort/requirements.txt
pip install -U -r dependencies/3rdparty/netvlad/requirements.txt
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
rosrun mask_rcnn_ros mask_rcnn_node.py
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
rosrun netvlad_tf netvlad_node.py
```

In `maplab_console`, the following command is used to call netvlad:
```bash
generate_and_save_semantic_object_measurements
```

For multi-object association, we use Deeo Sort
```bash
rosrun deep_sort_ros deep_sort_node.py
generate_and_save_semantic_object_track_ids_from_deepsort --tracking_confidence_threshold 0.85
```

tuning parameters:
requires 3 consecutive hits to have a confirmed track
filters detections by confidence

For visualization:
visualize_semantic_object_channels_in_visual_frame --semantify_visualization_frequency 0.2 --tracking_confidence_threshold 0.85
evaluate_semantic_landmark_with_track_id --semantify_visualization_frequency 0.1 --semantic_landmark_track_id 1
evaluate_semantic_landmark_with_track_id --semantify_visualization_frequency 0.1 --semantic_landmark_track_id 501 --generate_descritpor_filter_by_mask=false
evaluate_semantic_landmark_with_track_id --map_mission c277fc00e69fcc150d00000000000000 --semantic_landmark_track_id 89
evaluate_semantic_landmark_with_track_id --semantic_landmark_track_id 89

load the rviz from maplab_private/experimental/console-plugins/semantify-basic-plugin/maplab.rviz

For saving maps:
save --map_folder path/to/save/the/map --overwrite

### Triangulate semantic landmarks, need to finish all semantify commands
itsl --vi_map_semantic_landmark_quality_min_observation_angle_deg 20
rtsl for retriangulation of semantic landmarks

### visualization for semantic landmarks
To update the class id and observation of semantic landmars, currectly not saved in protobuf:
update_semantic_landmarks_class_ids

To set up the mapping and visualization for semantic landmarks:
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map

### descriptor clustering
To cluster descriptors and get medoids for each landmarks:
generate_descriptor_clusters

To show map status and find mission id:
ms

some debugging for descriptors:
display_descriptor_clusters_scores --map_mission
compare_descriptor_clusters_scores --descriptor_comparison_mission_id_0 c277fc00e69fcc150d00000000000000 --descriptor_comparison_mission_id_1 c277fc00e69fcc150d00000000000000 --descriptor_comparison_semantic_landmark_track_ids_0 2,163 --descriptor_comparison_semantic_landmark_track_ids_1 2,163

## Map Anchoring
### traditional map anchoring
load --map_folder /media/jkuo/A8DD-8CBF/old_sensor_setup/data/quadric-2-1/quadric_2_1_with_semantic_landmarks/
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/old_sensor_setup/data/quadric-2-2/quadric_2_2_with_semantic_landmarks/
ms

for visualization:
spatially_distribute_missions --spatially_distribute_missions_dimension 2 --spatially_distribute_missions_meters 5
v --vis_color_by_mission

set baseframe:
sbk --map_mission 85965582c59fcc150d00000000000000

check status:
ms
now mission 0 T_G_M is set to the reference and becomes known

Starts anchoring:
aam

### semantic landmark map anchoring
load --map_folder /media/jkuo/A8DD-8CBF/old_sensor_setup/data/quadric-2-1/quadric_2_1_with_semantic_landmarks/
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/old_sensor_setup/data/quadric-2-2/quadric_2_2_with_semantic_landmarks/
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

## Loop Closure
### semantic landmark loop closure
load --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_both_sides_with_loop/with_semantic_landmarks20/
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map --semantic_landmark_class_filter "1,73,61,57"
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref c3c402890f1ec6150d00000000000000 --loop_closure_mission_id_source c3c402890f1ec6150d00000000000000 --semantic_landmark_lc_extend_visible_verticies_num 100 --semantic_landmark_lc_extend_visible_verticies=true --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_add_edge_between_topological_center_vertices=true
optvi --ba_visualize_every_n_iterations 1
rtsl
v
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map

#### Create groundtruth from all maps:
load --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_one_side_with_loop/with_optvi_nolc_semantic_landmarks/
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_round_and_round_he_goes/with_optvi_nolc_semantic_landmarks
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_both_sides_with_loop/with_semantic_landmarks20/
lc
optvi

#### viewpoint change within map:
load --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_both_sides_with_loop/with_semantic_landmarks20/
v
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref c3c402890f1ec6150d00000000000000 --loop_closure_mission_id_source c3c402890f1ec6150d00000000000000 --semantic_landmark_matching_filter_by_match_candidate_distance=true --semantic_landmark_lc_extend_visible_verticies_num 100 --semantic_landmark_lc_extend_visible_verticies=true --visualize_accepted_loop_closure_edge=true --semantic_landmark_anchoring_ransac_min_inlier_num=4 --semantic_landmark_lc_max_covisible_object_candidate_distance_difference 0.6 --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.6 --semantic_landmark_lc_add_edge_between_topological_center_vertices=false --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_remove_old_edges=false
visualize_semantic_loop_closure_edge_covariances

load --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_round_and_round_he_goes/with_optvi_nolc_semantic_landmarks
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref c9b4d48ec021c6150d00000000000000 --loop_closure_mission_id_source c9b4d48ec021c6150d00000000000000 --semantic_landmark_matching_filter_by_match_candidate_distance=true --semantic_landmark_lc_extend_visible_verticies_num 40 --semantic_landmark_lc_extend_visible_verticies=true --semantic_landmark_anchoring_ransac_min_inlier_num=5 --visualize_accepted_loop_closure_edge=true --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.8 --semantic_landmark_lc_add_edge_between_topological_center_vertices=false --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_remove_old_edges=false
visualize_semantic_loop_closure_edge_covariances

load --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_one_side_with_loop/with_optvi_nolc_semantic_landmarks/
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref 7c72ec38581ec6150d00000000000000 --loop_closure_mission_id_source 7c72ec38581ec6150d00000000000000 --semantic_landmark_matching_filter_by_match_candidate_distance=true --semantic_landmark_lc_extend_visible_verticies_num 60 --visualize_accepted_loop_closure_edge=true --semantic_landmark_anchoring_ransac_min_inlier_num=5 --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.5 --semantic_landmark_lc_add_edge_between_topological_center_vertices=false --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_remove_old_edges=false
visualize_semantic_loop_closure_edge_covariances

#### viewpoint change across maps:
load --map_folder /media/jkuo/A8DD-8CBF/old_sensor_setup/data/quadric-2-1/quadric_2_1_with_semantic_landmarks/
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/old_sensor_setup/data/quadric-2-2/quadric_2_2_with_semantic_landmarks/
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref 85965582c59fcc150d00000000000000 --loop_closure_mission_id_source c277fc00e69fcc150d00000000000000 --semantic_landmark_lc_extend_visible_verticies_num 100 --visualize_accepted_loop_closure_edge=false --semantic_landmark_anchoring_ransac_min_inlier_num=8 --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.6 --semantic_landmark_lc_max_covisible_object_candidate_distance_difference 0.2 --semantic_landmark_lc_add_edge_between_topological_center_vertices=false --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_remove_old_edges=false
evaluate_semantic_landmark_with_track_id --map_mission 85965582c59fcc150d00000000000000 --semantic_landmark_track_id 2
evaluate_semantic_landmark_with_track_id --map_mission c277fc00e69fcc150d00000000000000 --semantic_landmark_track_id 1


load --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_one_side_with_loop/with_optvi_nolc_semantic_landmarks/
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_both_sides_with_loop/with_semantic_landmarks20/
sbk --map_mission 7c72ec38581ec6150d00000000000000
anchor_mission_with_semantic_landmarks --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.15
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref 7c72ec38581ec6150d00000000000000 --loop_closure_mission_id_source c3c402890f1ec6150d00000000000000 --semantic_landmark_lc_extend_visible_verticies_num 60 --semantic_landmark_max_match_candidate_distance 8 --visualize_accepted_loop_closure_edge=false --semantic_landmark_anchoring_ransac_min_inlier_num=6 --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.6 --semantic_landmark_lc_max_covisible_object_candidate_distance_difference 0.2 --semantic_landmark_lc_add_edge_between_topological_center_vertices=false --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_remove_old_edges=false
evaluate_semantic_landmark_with_track_id --map_mission c3c402890f1ec6150d00000000000000 --semantic_landmark_track_id 415
evaluate_semantic_landmark_with_track_id --map_mission 7c72ec38581ec6150d00000000000000 --semantic_landmark_track_id 409

load --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_round_and_round_he_goes/visual_lc_with_semantic_lm
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_both_sides_with_loop/with_semantic_landmarks20/
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref c9b4d48ec021c6150d00000000000000 --loop_closure_mission_id_source c3c402890f1ec6150d00000000000000 --semantic_landmark_lc_extend_visible_verticies_num 80 --semantic_landmark_max_match_candidate_distance 5 --visualize_accepted_loop_closure_edge=false --semantic_landmark_anchoring_ransac_min_inlier_num=4 --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.6 --semantic_landmark_lc_max_covisible_object_candidate_distance_difference 0.3 --semantic_landmark_lc_add_edge_between_topological_center_vertices=false --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_remove_old_edges=false
evaluate_semantic_landmark_with_track_id --map_mission c3c402890f1ec6150d00000000000000 --semantic_landmark_track_id 853
evaluate_semantic_landmark_with_track_id --map_mission c9b4d48ec021c6150d00000000000000 --semantic_landmark_track_id 571

#### light intensity change:
for this map, we ignore chairs because we had to reduce the minimum view angle required to get more landmarks on the table.
As a consequence, more bad chair landmarks appear made the inlier ratio kind of useless. So we ignore chairs for this map.

load --map_folder /media/jkuo/A8DD-8CBF/dataset2/asl_koze_table_one_side_with_loop_light_change_medium/with_semantic_landmarks5_point7confidence/
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map --semantic_landmark_class_filter "1,73,61,57"
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref 093e162baf13df150d00000000000000 --loop_closure_mission_id_source 093e162baf13df150d00000000000000 --semantic_landmark_matching_filter_by_match_candidate_distance=true --semantic_landmark_lc_extend_visible_verticies_num 100 --semantic_landmark_lc_extend_visible_verticies=true --visualize_accepted_loop_closure_edge=true --semantic_landmark_anchoring_ransac_min_inlier_num=6 --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.6 --semantic_landmark_lc_max_covisible_object_candidate_distance_difference 0.6 --semantic_landmark_lc_add_edge_between_topological_center_vertices=false --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_remove_old_edges=false

## Evaluation
### export posegraph in rpg format for evaluation
ettc_rpg --pose_export_file ~/semantic_lc_top_OneSide.txt
ettc_rpg --map_mission 093e162baf13df150d00000000000000 --pose_export_file ~/gt_light_change_medium_OneSide.txt
ettc_rpg --map_mission 2e0aa9515913df150d00000000000000 --pose_export_file ~/gt_light_change_max_OneSide.txt
ettc_rpg --map_mission 80d309e60614df150d00000000000000 --pose_export_file ~/gt_light_change_medium_BothSide.txt

### trajectory evaluation
from rpg trajectory evaluation toolbox https://github.com/uzh-rpg/rpg_trajectory_evaluation :

rosrun rpg_trajectory_evaluation analyze_trajectories.py asl_map_lc_comparison.yaml --output_dir=/home/jkuo/maplab_ws/src/rpg_trajectory_evaluation/evaluation/asl_map_lc_comparison --results_dir=/home/jkuo/maplab_ws/src/rpg_trajectory_evaluation/results/asl_map --platform laptop --odometry_error_per_dataset --plot_trajectories --rmse_table --rmse_boxplot

rosrun rpg_trajectory_evaluation analyze_trajectory_single.py .

## Dataset storage
Ask for access
Username: j225-students
PW: ???
server: data.asl.ethz.ch
only sftp access

Location:
/J225-students/jkuo-semantic-mapping

From old_sensor_setup:
The 5C-1 and 5C-2 is  quadric 2-1 and 2-2 map
with_optvi_no_lc folder

From dataset1:
the OneSide map is /datasets/asl_koze_table_one_side_with_loop/with_optvi_nolc_semantic_landmarks
the BothSide map is datasets/asl_koze_table_both_sides_with_loop/with_semantic_landmarks20
the RR map is /datasets/asl_koze_table_round_and_round_he_goes/with_optvi_nolc_semantic_landmarks
the OSIC map is /dataset2/asl_koze_table_one_side_with_loop_light_change_medium/with_semantic_landmarks5_point7confidence

reference:https://www.digitalocean.com/community/tutorials/how-to-use-sftp-to-securely-transfer-files-with-a-remote-server
