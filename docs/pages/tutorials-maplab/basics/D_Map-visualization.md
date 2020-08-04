## Map visualization: see your map in RViz!

### Adding topics in RViz

First, make sure you have an instance of roscore running. If not, you can start it in a terminal
```
roscore &
```
Then, start the maplab console:
```
rosrun maplab_console maplab_console
```
Now, load a map, and visualize it:
```
load --map_folder MH_01_easy  # or any other VI map you have.
v
```

The command ``v`` publishes the map via the ROS messages (and registers the corresponding ROS topics). Start RViz in a new terminal:
```
rviz
```

On the Displays pane on the left side, click on Add. (If you don't have the Displays pane, you can get it via Panels -> Displays)

![](images/visualization/rviz_add.png)

Click on the tab *By topic* and select a topic to visualize and click OK on the bottom right:

![](images/visualization/rviz_by_topic.png)

### Understanding visualization
Start the maplab console:
```
rosrun maplab_console maplab_console
```

Load a map, and visualize it:
```
load --map_folder MH_01_easy  # or any other VI map you have.
v
```

This will publish the following topics to RViz:
- `vi_map_baseframe`: Baseframes of the individual missions of the maps.
- `vi_map_edges/viwls`: Edges of the VI map. Visualization of edges:
![](images/visualization/vimap_edges.png)
- `vi_map_vertices`: Vertex transformations. Visualzation of edges and vertices:
![](images/visualization/vimap_edges_vertices.png)
- `vi_map_landmarks`: Point cloud of all landmarks. Visualization of edges, vertices and landmarks:
![](images/visualization/vimap_edges_vertices_landmarks.png)

![](images/visualization/vimap.png)

Loop closing a map (with the `lc` command) adds the visualization topic `loop_closures`, which will show merged landmark pairs:
![](images/visualization/lc.png)

Running `relax` adds the topic `vi_map_edges/loop_closure` to the list and displays loop closure edges between vertices that can be loop-closed:
![](images/visualization/lc_edges.png)
![](images/visualization/lc_edges_after.png)

### Additional flags for visualization
- `vis_scale`: Sets the size of the edges.
  ![](images/visualization/vis_scale.png)
- `vis_color_by_mission`: colors the landmarks and edges of each mission differently. This is enabled by default.
- `vis_color_landmarks_by_height`: Colors the landmarks by height. Use with these additional flags:
    - `vis_color_by_height_period_m`
    - `vis_color_by_height_offset_m`

  ![](images/visualization/vis_color_by_height.png)

- `vis_color_salt`: Changes the rotation of the colors, e.g.

Salt 1 (default value):
```
v --vis_color_by_mission --vis_color_salt 1
```
![](images/visualization/salt_1.png)

Salt 70:
```
v --vis_color_by_mission --vis_color_salt 70
```
![](images/visualization/salt_70.png)


### Visualizing the inertial states
You can use
```bash
plot_vi_states_of_mission
```
to plot the VI states of a mission.

This will open a plot with the position, velocity, and accelerometer/gyroscope biases.
This may be helpful for tuning the IMU parameters or better understanding what happens during the batch optimization.

![](images/visualization/plot_vi_states.png)
