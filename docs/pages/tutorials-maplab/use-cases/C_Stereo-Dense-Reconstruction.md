## Stereo Dense Reconstruction: EuRoC, multi-session reconstruction use-case

This tutorial will show you how to compute a globally consistent multi-map reconstruction based on multiple EUROC datasets

![EuRoC consistent 3D reconstruction](images/dense/stereo_dense_reconstruction.png)

### Datasets:

In order to obtain the initial maps required for this task, you can either download the EUROC rosbags from the official website and use ROVIOLI to create the maps, or you can download the finished VI maps for reconstruction from the [[Sample Datasets]] page.

If you choose to create the maps yourselves, make sure you use to following flags, such that ROVIOLI attaches the images necessary for dense reconstruction to the map.
```
 --map_builder_save_image_as_resources=true
```

### Instructions:

In order to merge, optimize and reconstruct these three maps, we are suggesting the following procedure. Open your maplab console and execute the following commands in sequence:
```bash
# Load all maps
load --map_folder <dataset_folder>/euroc_ml1
load --map_folder <dataset_folder>/euroc_ml2
load --map_folder <dataset_folder>/euroc_ml3

# Loop-close and optimize each individual map
select_map --map_key euroc_ml1
# Re-triangulate the landmarks
rtl
# OPTIONAL STEP: Optimize the map to get a better initial
#                map state for loop-closure
optvi
# Run loop closure
lc
# Run bundle adjustment
optvi
# Save the optimized map
save --map_folder <dataset_folder>/euroc_ml1_opt

# Repeat the same for map 2 and 3
select_map --map_key euroc_ml2
rtl
optvi
lc
optvi
save --map_folder <dataset_folder>/euroc_ml2_opt
select_map --map_key euroc_ml3
rtl
optvi
lc
optvi
save --map_folder <dataset_folder>/euroc_ml3_opt

# Load all single-mission maps into the same map (which then contains 3 missions)
join_all_maps --target_map_key=euroc_ml1_2_3
# Set the baseframe of the first mission to known to anchor it.
sbk
# Align all missions with respect to one another by modifying
# their baseframe transformation.
aam
# Loop-close and optimize all missions
lc
optvi
save --map_folder <dataset_folder>/euroc_ml1_2_3_opt

# Save the map to a different folder such that you always have
# the optimized version of the map to go back to if something goes wrong
save --map_folder <dataset_folder>/euroc_ml1_2_3_dense
# Stereo dense reconstruction
sdr --flagfile=<path_to_stereo_dense_reconstruction_pkg>/parameter/stereo_params.gflags

# Compute a globally consistent dense reconstruction based on VOXBLOX.
create_tsdf_from_depth_resource --flagfile=<path_to_stereo_dense_reconstruction_pkg>/parameter/stereo_params.gflags

# Compute and export the surface reconstruction to a PLY file.
export_tsdf --dense_result_mesh_output_file=<dataset_folder>/euroc_surface_reconstruction.ply

# Save your map
save --map_folder <dataset_folder>/euroc_ml1_2_3_dense
```
