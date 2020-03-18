## Understanding loop-closure

![lc_flow](images/diagrams/lc_dataflow.png)

The loopclosure system of maplab is based on projected BRISK/FREAK descriptors. It is using the 2D-3D matching principle, i.e. features in the query frame (2D) are matched to the 3D structure of the prior map. The loop closure process can be divided into three main stages, as presented in the diagram above:
1. Nearest neighbor search in the projected descriptor space. A very fast and practically constant-time retrieval is possible using an inverted multi-index.
2. Covisibility filtering of the matches using the covisibility graph of the prior map. This step filters out matches from non-relevant parts of the prior map and only keeps the matches pointing to the most widely-matched places.
3. Pnp+RANSAC geometric verification that verifies if the established 2D-3D matches are geometrically consistent with the prior map structure.

The loopclosure engine of maplab is used by ROVIOLI and multiple algorithms within the maplab console:
* ROVIOLI localization
* ``aam`` finds links between missions and anchor them, i.e. aligns them rigidly.
* ``relax`` adds loopclosure edges (relative 6DOF pose constraints between two frames) and perform robust posegraph optimization. The loop closure edges are removed again after the optimization.
* ``lc`` find loopclosures and merge the landmarks.

### Useful parameters

Depending on your specific needs, it might be worth to tune the parameters. You can use the flags when calling any of the aforementioned commands.

- `lc_num_ransac_iters`: (reasonable range: 100-1000). The maximum number of RANSAC iterations for the geometric verification. Increasing this number increases the chance of finding a good PnP solution, but affects performance. Worth a try for offline loopclosing.
- `lc_ransac_pixel_sigma`: (default: 2.0, reasonable range: 2.0-8.0). The pixel sigma used for the PnP RANSAC pose estimation. Increasing this value increases recall, but increases the chance of accepting geometrically bogus loop-closures. This parameter often also depends on the number, configuration and calibration quality of the camera(s). In general, the better the calibration (both intrinsics and extrinsics), the lower this value can be chosen.
- `lc_min_image_time_seconds`: (default: 10). Whenever a vertex is queried against the database for a loop-closure, all descriptor matches associated with vertices that are less than `ld_min_image_time_seconds` seconds before or after the time of the query vertex are ignored. This prevents loop closures from being found between a frame and a temporally close neighbor frame on the same trajectory. If this threshold is decreased a lot, what happens is that loop closure behaves similar to feature tracking (which is highly inefficient).
- `lc_min_inlier_count`: (reasonable range: 10-40) The minimum number of PnP RANSAC inliers to treat the solution as valid. Making this number smaller increases the chance of loopclosing, but also has a good chance to introduce bogus loopclosures.
- `lc_num_neighbors`: (reasonable range: 3-10). Determines the number of nearest-neighbor descriptors retrieved for each query descriptor. This number per default depends on the number of landmarks/observations in the map. Setting it to a larger value increases recall but may decrease precision.

### Workflow to close the loops

#### Mission alignment
If you are dealing with multiple missions, the first step is to perform a rigid alignment using the `aam` command. For details, follow the [multisession tutorial](Preparing-a-multi-session-map).

#### Posegraph relaxation
Robust posegraph relaxation adds loopclosure edges and then performs an optimization that aims to find a balance between the loopclosure edges and the odometry edges (visual+inertial). During this optimization, we keep landmark positions, IMU biases and velocities fixed. 

The loopclosure edges use switchable constraints or a robust cost function to reject outliers. The switchable constraint variance can be used to adjust the stiffness of the switch variables:
- `lc_switch_variable_variance`: (reasonable range 1e-7 to 1e-9). The smaller the value, the harder it is for a solver to switch a loopclosure edge off.

Additionally, it might be worth to tweak the loopclosure edge addition parameters:
- `lc_edge_min_inlier_ratio`: (default: 0.5). A minimum inlier ratio within PnP RANSAC to add a loopclosure edge. Lowering this value increases the number of loopclosure edges, but might introduce outliers.
- `lc_edge_min_inlier_count`: (default: 20). A minimum inlier count within PnP RANSAC to add a loopclosure edge. Lowering this value increases the number of loopclosure edges, but might introduce outliers.

**Tip:** Perform the relaxation **before** running the `lc` command. After `lc`, some landmarks are already merged and the edges will not be created for them. This will impair the performance.

#### Full batch optimization

The relaxation will correct for cases where large drift occurred, but some smaller discrepancies may not be addressed. We therefore need to perform a full batch optimization after merging the duplicated landmarks. 

Start with the `lc` command. This searches for loop-closures across all missions of the loaded map. Whenever a loop is found, duplicate landmarks in that region are merged into one.
Note that at this stage, the geometry of the map is left unchanged and the found loops are not actually being closed yet. You need to `optvi` the map to make use of the new visual constraints.

After refining the geometry, you might consider repeating the `lc`-`optvi` iteration again.
