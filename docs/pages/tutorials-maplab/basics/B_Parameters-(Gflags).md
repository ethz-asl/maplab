## Parameters (Gflags)

Maplab uses gflags ([gflags_catkin](https://github.com/ethz-asl/gflags_catkin)) to pass parameters to executables and commands inside the maplab console. If you have never used gflags before, it might help you to take a look at the [gflags documentation](https://gflags.github.io/gflags/).

### Special gflags and flag files

Have a look at [this section](https://gflags.github.io/gflags/#special) of the gflags documentation.

### Basic maplab flags

Here is a short list of the basic maplab flags that are needed to load and store a map, specify which mission a command should operate on and manage the map resources.
```
    -map_folder (The folder on which a console command operates.) type: string
      default: ""
    -map_key (The map key for a console command.) type: string default: ""
    -map_mission (The mission ID a command should operate on.) type: string
      default: ""
    -map_mission_list (List of comma separated mission IDs a command should
      operated on.) type: string default: ""
    -overwrite (If set to true, existing files on the disks will get
      overwritten.) type: bool default: false
    -target_map_key (Target map key for copy and move commands.) type: string
      default: ""
    -copy_resources_to_external_folder (Set this to true to migrate the
      resources to an extneral folder (by creating a copy of the resources)
      before saving the map. This flag will be reset after every use.)
      type: string default: ""
    -copy_resources_to_map_folder (Set this to true to migrate the resources to
      the map folder (by creating a copy of the resources) before saving the
      map. This flag will be reset after every use.) type: bool default: false
    -maps_folder (Folder which contains one or more maps on the filesystem.)
      type: string default: "."
    -move_resources_to_external_folder (Set this to true to migrate the
      resources to an extneral folder (by moving the resources) before saving
      the map. This flag will be reset after every use.) type: string
      default: ""
    -move_resources_to_map_folder (Set this to true to migrate the resources to
      the map folder (by moving the resources) before saving the map. This flag
      will be reset after every use.) type: bool default: false
    -resource_folder (Specifies the resource folder for certain commands.)
      type: string default: ""
```

### List all parameters

In order to get a full list of all parameters that are available for a specific maplab executable, call the executable with the `--help` flag. Example:
```bash
rosrun rovioli rovioli --help
```
A complete list can also be obtained from inside the maplab console by typing:
```bash
<any_valid_command> --help 
```

This list will be quite extensive, and it helps to narrow down your search with grep:
```bash
rosrun rovioli rovioli --help | grep loop
```
Unfortunately grep will not work inside the console.

### Find parameters for a specific algorithm

Maplab uses prefixes for most of the gflags to associate them with a specific algorithm or map component.
Examples:
```
-ba_*               # bundle adjustment
-lc_*               # loop closure
-vis_*              # visualization
-vi_map_landmark_*  # landmarks and landmark quality metric
-dense_*            # dense reconstruction
-lba_*              # legacy bundle adjustment
```

In order to get a list of all parameters that are related to the bundle adjustment, use the auto completion function in the console or grep the complete parameter list:
```bash
# Type and TAB:
optvi --ba

# Result:
--ba_altitude_meters                                        --ba_min_landmark_per_frame
--ba_fix_accel_bias                                         --ba_num_iterations
--ba_fix_gyro_bias                                          --ba_outlier_rejection_reject_every_n_iters
--ba_fix_landmark_positions                                 --ba_outlier_rejection_reject_using_reprojection_error
--ba_fix_ncamera_extrinsics_rotation                        --ba_outlier_rejection_reprojection_error_other_mission_px
--ba_fix_ncamera_extrinsics_translation                     --ba_outlier_rejection_reprojection_error_same_mission_px
--ba_fix_ncamera_intrinsics                                 --ba_use_cgnr_linear_solver
--ba_fix_velocity                                           --ba_use_jacobi_scaling
--ba_include_inertial                                       --ba_use_outlier_rejection_solver
--ba_include_visual                                         --ba_visualize_every_n_iterations
--ba_latitude                                               --bag_file
```

For more details about a specific parameter, search for the description in the full parameter list:

```bash
# Type:
rosrun maplab_console maplab_console --help | grep -e "-lc_" -A 1

# Result:
    -lc_projected_quantizer_filename (File where to read the projected
      quantizer from.) type: string default: ""
    -lc_projection_matrix_filename (The name of the file for the projection
      matrix.) type: string default: ""
    -lc_target_dimensionality (The target dimensionality of the projection.)
      type: int32 default: 10
--
    -lc_edge_covariance_scaler (Scaling the covariance of loopclosure edges. It
      is identity by default.) type: double default: 9.9999999999999995e-08
    -lc_edge_min_distance_meters (The minimum loop-closure gap distance such
      that a loop-closure edge is created.) type: double default: 1
    -lc_edge_min_inlier_count (The minimum loop-closure inlier count to add a
      loop-closure edge.) type: int32 default: 20
    -lc_edge_min_inlier_ratio (The minimum loop-closure inlier ratio to add a
      loop-closure edge.) type: double default: 0.5
    -lc_min_inlier_count (Minimum inlier count for loop closure.) type: int32
      default: 10
    -lc_min_inlier_ratio (Minimum inlier ratio for loop closure.) type: double
      default: 0.20000000000000001
    -lc_nonlinear_refinement_p3p (If nonlinear refinement on all ransac inliers
      should be run.) type: bool default: false
    -lc_num_ransac_iters (Maximum number of ransac iterations for absolute pose
      recovery.) type: int32 default: 100
    -lc_ransac_pixel_sigma (Pixel sigma for ransac.) type: double default: 2

--
    -lc_filter_underconstrained_landmarks (If underconstrained landmarks should
      be filtered for the loop-closure.) type: bool default: true
    -lc_use_random_pnp_seed (Use random seed for pnp RANSAC.) type: bool
      default: true
--
    -lc_visualize_outliers (If outlier matches should be published on the loop
      closure topic.) type: bool default: false
--
    -lc_knn_epsilon (Epsilon approximation value for the nearest neighbor
      search.) type: double default: 3
    -lc_knn_max_radius (Max radius for the nearest neighbor search.)
      type: double default: 20
--
    -lc_detector_engine (Which loop-closure engine to use) type: string
      default: "inverted_multi_index"
    -lc_fraction_best_scores (Fraction of best scoring keyframes/vertices that
      are considered for covisibility filtering.) type: double default: 0.25
    -lc_min_image_time_seconds (Minimum time between matching images to allow a
      loop closure.) type: double default: 10
    -lc_min_verify_matches_num (The minimum number of matches needed to verify
      geometry.) type: uint64 default: 10
    -lc_num_neighbors (Number of neighbors to retrieve for loop-closure. -1
      auto.) type: int32 default: -1
    -lc_num_words_for_nn_search (Number of nearest words to retrieve in the
      inverted index.) type: int32 default: 10
    -lc_scoring_function (Type of scoring function to be used for scoring
      keyframes.) type: string default: "accumulation"
--
    -lc_num_descriptors_to_train (Number of descriptors used for training.)
      type: int32 default: 100000
    -lc_number_of_vocabulary_words (Number of words in the vocabulary.)
      type: int32 default: 1000
    -lc_product_quantization_num_components (Number of components for product
      quantization.) type: int32 default: 1
    -lc_product_quantization_num_dim_per_component (Number of components for
      product quantization.) type: int32 default: 5
    -lc_product_quantization_num_words (Number of words in the product
      vocabulary.) type: int32 default: 256
--
    -lc_kdtree_accelerator_eps (Epsilon for the NN search inside the kd-tree
      based search accelerators.) type: double default: 0.10000000000000001
--
    -lc_kmeans_levels (Number of levels in the vocabulary tree.) type: int32
      default: 4
    -lc_kmeans_splits (Number of splits in the kmeans step per level.)
      type: int32 default: 10
    -lc_num_kmeans_restarts (Number of restarts for the kmeans.) type: int32
      default: 5
--
    -lc_only_against_other_missions (If true, no inter-mission loop-closures
      are sought.) type: bool default: false
--
    -lc_switch_variable_value (The value for the switch variable of the
      loop-closure edges, between 0.0 (edge is ignored) and 1.0 (edge is being
--
    -lc_switch_variable_variance (The variance for the switch variable of the
      loop-closure edges.) type: double default: 1e-08

```
