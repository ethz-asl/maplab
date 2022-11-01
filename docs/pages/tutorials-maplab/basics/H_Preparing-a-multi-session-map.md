## Preparing a multi-session map: map anchoring, loop-closure, pose-graph relaxation

This tutorial will guide you through the process of merging different maps of the same environment together.
As an example use case, we will merge the maps generated from the [Euroc machine hall 1, 2 and 3 datasets](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).
You can download the unoptimized VI maps from the [[Sample Datasets]] page.
Then, optionally, to improve the results, [optimize and loop-close](Preparing-a-single-session-map) each of these maps individually and save the map again.

### Merging together different maps
Start a maplab console:
```
roscore&
rosrun maplab_console maplab_console
```

If there are no maps loaded into the console, you can use the following command to merge different VI maps from the file system directly together:
```
load_merge_all_maps --maps_folder path/to/optimized/euroc_maps
```
This creates a map in the console that is the product of merging all maps within `path/to/optimized/euroc_maps` together.
You can then confirm that there is a mission for each in the individual maps by typing `ms`:
```
maplab <euroc_maps>:/$ ms
I1102 13:44:08.304085  7219 vi-map-basic-plugin.cc:973] VI-Map:

Mission 0:          		e4a9a7f9ffffffff0e00000000000000	Viwls
	Vertices:           	3677
	Landmarks:          	132954
	Landmarks by first observer backlink:
	Camera 0:           	132954 (g:31097 b:101857 u:0)
	Observations:       	2319656
	Num edges by type:
	IMU:                	3676
	Wheel-odometry:     	0
	Loop-closure:       	0
	Distance travelled [m]:	84.611516
	Start to end time:  	2017_11_02_13_44_08 to 2017_11_02_13_44_08
	T_G_M               	unknown
	Num GPS measurements: 	0/0 (UTM/WGS)

Mission 1:          		0cfe1a24000000000e00000000000000	Viwls
	Vertices:           	3036
	Landmarks:          	116748
	Landmarks by first observer backlink:
	Camera 0:           	116748 (g:28581 b:88167 u:0)
	Observations:       	1883207
	Num edges by type:
	IMU:                	3035
	Wheel-odometry:     	0
	Loop-closure:       	0
	Distance travelled [m]:	78.673416
	Start to end time:  	2017_11_02_13_44_08 to 2017_11_02_13_44_08
	T_G_M               	unknown
	Num GPS measurements: 	0/0 (UTM/WGS)

Mission 2:          		ec0479dfffffffff0e00000000000000	Viwls
	Vertices:           	2695
	Landmarks:          	95580
	Landmarks by first observer backlink:
	Camera 0:           	95580 (g:29244 b:66336 u:0)
	Observations:       	1467939
	Num edges by type:
	IMU:                	2694
	Wheel-odometry:     	0
	Loop-closure:       	0
	Distance travelled [m]:	134.781190
	Start to end time:  	2017_11_02_13_44_08 to 2017_11_02_13_44_08
	T_G_M               	unknown
	Num GPS measurements: 	0/0 (UTM/WGS)
```

Visualizing the map gives the following result:
```
spatially_distribute_missions
v --vis_color_by_mission 
```
![Initially merged maps](images/multi-map-initial.png)

### Alternative ways to merge maps
This section briefly lists alternative ways to merge multiple maps.

#### join_all_maps

```
load_all --maps_folder path/to/optimized/euroc_maps
join_all_maps --target_map_key euroc_maps
```
`join_all_maps` merges all loaded maps together into a map given by `target_map_key`.

#### load_merge
```
load --map_folder path/to/optimized/euroc_maps/MH_01_easy_optimized/
load_merge_map --map_folder path/to/optimized/euroc_maps/MH_02_easy_optimized/
load_merge_map --map_folder path/to/optimized/euroc_maps/MH_03_medium_optimized/
```
`load_merge_map` loads the map under the path given by `map_folder` and merges it with the selected map.

#### merge_map
```
load --map_folder path/to/optimized/euroc_maps/MH_01_easy_optimized/
load --map_folder path/to/optimized/euroc_maps/MH_02_easy_optimized/
load --map_folder path/to/optimized/euroc_maps/MH_03_medium_optimized/
merge_map --map_key MH_01_easy_optimized
merge_map --map_key MH_02_easy_optimized
```
`merge_map` merges the map given by `map_key` into the selected map.

### Mission anchoring
Each mission defines a baseframe transformation T\_G\_M.
Initially, this transformation is unknown as we don't have any information about where the missions should be anchored and what are the transformations between them.
The map statistics command `ms` shows this information:
```
Mission 0:          		e4a9a7f9ffffffff0e00000000000000	Viwls
	...
	T_G_M               	unknown

Mission 1:          		0cfe1a24000000000e00000000000000	Viwls
	...
	T_G_M               	unknown

Mission 2:          		ec0479dfffffffff0e00000000000000	Viwls
	...
	T_G_M               	unknown
```

In the process of mission anchoring, this T\_G\_M will be estimated.
This is done by finding loop closures between a mission with an unknown baseframe and the missions with known baseframes.
At least one of the missions needs to be known.
We therefore mark an arbitrary mission of the dataset as known:
```
sbk
```
`sbk` (short for *set mission baseframe to known*) operates on the first mission by default and sets its baseframe to known.

This changes the output of `ms` to the following:
```
Mission 0:          		e4a9a7f9ffffffff0e00000000000000	Viwls
	...
	T_G_M               	unknown

Mission 1:          		0cfe1a24000000000e00000000000000	Viwls
	...
	T_G_M               	known

Mission 2:          		ec0479dfffffffff0e00000000000000	Viwls
	...
	T_G_M               	unknown
```

Now we can anchor the other missionsn:
```
aam
```
This command (short for *anchor all missions*) will try to estimate all the unknown baseframe transformation.
The output of `ms` should look like this now:
```
Mission 0:          		e4a9a7f9ffffffff0e00000000000000	Viwls
	...
	T_G_M               	known

Mission 1:          		0cfe1a24000000000e00000000000000	Viwls
	...
	T_G_M               	known

Mission 2:          		ec0479dfffffffff0e00000000000000	Viwls
	...
	T_G_M               	known
```
Additionally, the command `print_baseframes` will also provide the values of the estimated transformations:
```
maplab <euroc_maps>:/$ print_baseframes
I1102 14:04:32.419026 10394 vi-map-basic-plugin.cc:1031] Baseframe transformations:
Mission 0 (e4a9a7f9ffffffff0e00000000000000):
  status: known
  T_G_M:
      0.999388  0.0349801          0   0.250711
    -0.0349801   0.999388         -0   0.710199
            -0          0          1   0.558163
             0          0          0          1

Mission 1 (0cfe1a24000000000e00000000000000):
  status: known
  T_G_M:
    1 0 0 0
    0 1 0 0
    0 0 1 0
    0 0 0 1

Mission 2 (ec0479dfffffffff0e00000000000000):
  status: known
  T_G_M:
       0.999838  -0.0179926           0 -0.00545835
      0.0179926    0.999838           0    0.349317
              0           0           1   0.0958929
              0           0           0           1
```

The resulting map will look like this:
![Anchoring result](images/multi-map-after-aam.png)

### Posegraph relaxation
Once the missions are anchored in a single coordinate frame, it is worth to relax the map using robust posegraph relaxation. This algorithm looks for matching keyframes between pairs of missions, creates relative 6DoF edges and performs an optimization operating on the odometry and loopclosure edges. The output of the relaxation is a globally consistent map that is a perfect starting point for further refinement (e.g. using visual-inertial least squares).
```
# Pose graph relaxation to close large loops.
relax
```

### Optimization and loop closure
Optimization and loop closure can be done like in the [single-mission case](Preparing-a-single-session-map). Keyframing (``kfh``) reduces the number of vertices/landmarks (so complexity of the optimization) and loopclosure (``lc``) merges the landmarks and creates visual constraints between missions. Finally, ``optvi`` performs a batch visual-inertial least squares optimization to refine the map geometry.
```
# (Optional) Key-frame the map.
kfh
# Loop close the map.
lc
# Bundle adjustment.
optvi
```

![Optimized map](images/multi-map-after-optvi.png)
