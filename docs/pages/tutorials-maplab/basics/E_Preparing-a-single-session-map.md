## Preparing a single session map: optimization, loop-closure

This tutorial will go through some commands to optimize and improve a map.

The commands in this tutorial assumes that you have loaded a VI map into the maplab console.
You can get a VI map from the [[Sample Datasets]] page.

### Landmark retriangulation
The command `rtl` retriangulates all landmarks in the VI map and reevaluates their quality.

Note: ROVIOLI already retriangulates all landmarks before the VI map is saved.

#### Note: output of commands
Most of the text output of the commands is hidden by default. You can see more detailed output of each command by using the standard Google Logging ``-v=<log_level>`` command. You can try to call:
```
rtl -v=3
```
to increase the verbosity and then standard:
```
rtl
```
and compare the output. The ``-v`` switch can be used with each maplab console command.

### Keyframing
The command `kfh` applies a heuristics-based keyframing on the VI map.
This will remove certain vertices from the map and merge the neighboring edges.

The following options are available:

Flag | Description
----|-----
`kf_distance_threshold_m` | Maximum distance in meters between two consecutive keyframes.
`kf_rotation_threshold_deg` | Maximum rotation in degrees between two consecutive keyframes.
`kf_every_nth_vertex` | Forces a keyframe after every n-th vertex.
`kf_min_shared_landmark_obs` | If two vertices share less landmarks than specified by this flag, both will be kept as a kayeframe.

### Loop closure
The command `lc` searches the map for loop closures. See the [loop closure tutorial](Understanding-loop-closure) for more details.

### Bundle adjustment
The main commands for bundle adjustment (BA) are:
- `optvi`: visual-inertial BA over the entire map.
- `optv`: visual only BA.

The BA iteratively tries to optimize the map.
During this process, a status output is printed after each step:

```
I1102 14:05:34.031795 10394 vi-map-optimizer.cc:85] Loading data for optimization, num of baseframes: 3
I1102 14:05:34.032006 10394 vi-map-optimizer.cc:157] Fixing 1 vertex poses.
I1102 14:05:34.117878 10394 graph-ba-optimizer.cc:1735] Removing landmarks that are still located behind the camera...
I1102 14:05:34.153887 10394 graph-ba-optimizer.cc:1741] Landmark count before: 345282
I1102 14:05:37.985208 10394 graph-ba-optimizer.cc:1779] Landmarks count after: 345282
I1102 14:05:37.985256 10394 graph-ba-optimizer.cc:2019] Adding visual term residual blocks...
I1102 14:05:47.078414 10394 graph-ba-optimizer.cc:2118] Added 2820048 visual residuals.
I1102 14:05:47.078451 10394 graph-ba-optimizer.cc:1253] Adding inertial term residual blocks...
I1102 14:05:47.137904 10394 graph-ba-optimizer.cc:1349] Added 9405 inertial residuals.
I1102 14:05:53.300902 10394 compressed_row_sparse_matrix.cc:85] # of rows: 5781171 # of columns: 407034 max_num_nonzeros: 87019350. Allocating 1067356888
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  1.672713e+09    0.00e+00    7.84e+08   0.00e+00   0.00e+00  1.00e+05        0    3.07e+00    7.65e+00
I1102 14:05:57.486155 10394 compressed_row_sparse_matrix.cc:85] # of rows: 407034 # of columns: 407034 max_num_nonzeros: 1390428. Allocating 18313276
   1  4.152947e+06    1.67e+09    1.66e+06   6.47e+01   1.00e+00  3.00e+05        1    3.07e+01    3.84e+01
I1102 14:06:28.178833 10394 compressed_row_sparse_matrix.cc:85] # of rows: 407034 # of columns: 407034 max_num_nonzeros: 1390428. Allocating 18313276
   2  3.115350e+06    1.04e+06    1.20e+06   3.86e+01   1.32e+00  9.00e+05        1    2.54e+01    6.38e+01
I1102 14:06:53.547498 10394 compressed_row_sparse_matrix.cc:85] # of rows: 407034 # of columns: 407034 max_num_nonzeros: 1390428. Allocating 18313276
   3  2.817663e+06    2.98e+05    4.83e+05   3.35e+01   1.39e+00  2.70e+06        1    2.44e+01    8.82e+01
I1102 14:07:17.925948 10394 compressed_row_sparse_matrix.cc:85] # of rows: 407034 # of columns: 407034 max_num_nonzeros: 1390428. Allocating 18313276
   4  2.699968e+06    1.18e+05    3.13e+05   3.53e+01   1.37e+00  8.10e+06        1    2.46e+01    1.13e+02
I1102 14:07:42.562119 10394 compressed_row_sparse_matrix.cc:85] # of rows: 407034 # of columns: 407034 max_num_nonzeros: 1390428. Allocating 18313276
   5  2.650624e+06    4.93e+04    1.22e+05   3.82e+01   1.31e+00  2.43e+07        1    2.49e+01    1.38e+02
I1102 14:08:07.494915 10394 compressed_row_sparse_matrix.cc:85] # of rows: 407034 # of columns: 407034 max_num_nonzeros: 1390428. Allocating 18313276
   6  2.628611e+06    2.20e+04    3.61e+04   5.47e+01   1.22e+00  7.29e+07        1    2.48e+01    1.63e+02
I1102 14:08:32.348891 10394 compressed_row_sparse_matrix.cc:85] # of rows: 407034 # of columns: 407034 max_num_nonzeros: 1390428. Allocating 18313276
   7  2.617790e+06    1.08e+04    4.79e+04   1.38e+02   1.07e+00  2.19e+08        1    2.54e+01    1.88e+02
I1102 14:08:57.799083 10394 compressed_row_sparse_matrix.cc:85] # of rows: 407034 # of columns: 407034 max_num_nonzeros: 1390428. Allocating 18313276
```

By pressing `Ctrl+C`, the BA will terminate after the current step is finished.

### Localization map
To use a VI map as a [localization map for ROVIOLI](Running-ROVIOLI-in-Localization-mode), two paths exist from the console:
- The optimized VI map can be saved and loaded as a localization map into ROVIOLI.
- The optimized VI map can be saved as a *localization summary map*. A localization summary map only contains the information necessary to perform localization (i.e., vertex poses and landmark descriptors and positions). The following command saves a localization summary map:
```
generate_summary_map_and_save_to_disk --summary_map_save_path path/to/save/localization/summary/map
```
