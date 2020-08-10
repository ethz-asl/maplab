## Optimizing VI-Maps

Commands:
```bash
optv     # Optimization based on the visual data only (landmark observations)
optvi    # Optimization based on the visual and inertial terms (landmarks and IMU)
```

Hints:

 * The optimization can be aborted at any time using `Ctrl-C`, it will then abort after at the end of the current iteration.
 * Important flags:
   ```bash
   --ba_num_iterations                # Set the maximum number of iterations of the optimization.
   --ba_visualize_every_n_iterations  # Visualize the result of the optimization at every Nth step.
   ```
 * Only landmarks flagged as 'good' will be part of the optimization. The following flags can be used to set the parameters of the quality metrics:
   ```bash
   --vi_map_landmark_quality_min_observation_angle_deg
        # Minimum angle disparity of observers for a landmark to be well constrained.
   --vi_map_landmark_quality_min_observers
        # Minimum number of observers for a landmark to be well constrained.
   --vi_map_landmark_quality_min_distance_from_closest_observer
        # A landmark needs to be at least as far away from the observer to be well constrained [m].
   --vi_map_landmark_quality_max_distance_from_closest_observer
        # A landmark cannot be further away from the observer than this to be well constrained [m].
   ```
   The landmark quality can be (re-)evaluated using the `rtl` or '`evaluate_landmark_quality` command (before the optimization):
   ```
   evaluate_landmark_quality  --vi_map_landmark_quality_min_observers=2 ...
   ```


