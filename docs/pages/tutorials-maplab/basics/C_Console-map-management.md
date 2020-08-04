## Console map management: load, save, basic visualization

maplab can have multiple maps loaded in its database at the same time.
Each map has a unique name, called the map key.
Map keys can only contain alphanumeric characters as well as hyphen (`-`) and underscore (`_`) characters. Other characters are not supported.

The command prompt indicates the currently selected map inside the `<>` brackets (if there's no content in the `<>` brackets, no map is currently active).
All commands operate on the selected map.

![Console Prompt](images/maplab-console-prompt.png)

### Creating a new map
You can create a new, empty map with the key `my_empty_key` using
```
create_new_map --map_key my_empty_key
```

### Loading a map from a file system
You can also load a map from the file system.
You can use a VI map from one of the provided [[Sample Datasets]]:
```
load --map_folder <path/to/the/downloaded/map>
```
Note: the folder `vi_map`, in which the actual map data is stored, doesn't need to be specified.

By default, the map key is automatically generated from the path, e.g. if the path is `path/to/the/downloaded/map`, the map key will be `map`.
You can manually define the key under which the map should be stored in maplab by using the `map_key` flag when using `load`:
```
load --map_key my_map_key --map_folder <path/to/the/downloaded/map>
```

After loading or creating a new map, maplab automatically changes the selected map to the new map.

After loading a map, your console output should look like this:

![Creating and loading a map](images/maplab-new-map.png)

Notice, how after the load operation, the loaded map has automatically been selected.

#### Loading multiple maps at once
You can use the `load_all` command to load all maps from a given folder at once:
```
load_all --maps_folder path/to/maps/to/load
```
(The flag `maps_folder` is optional. If it's not specified, the current path (`.`) will be taken.)

This command loads all maps from the given folder into maplab.
The keys are generated from the folder name the individual map is located in.

### Saving a map
To save a map back to the file system, you can use the following command:
```
save
```
This will try to save the map to its map folder (by default the folder where the map was loaded from).
Alternatively, the path to save the map in can be specified with the `--map_folder` flag.
By default, if a map already exists in the given path, nothing will be saved.
If you want to overwrite existing files, add the `--overwrite` flag to the `save` command.
Your `save` call can look like this:
```
save --map_folder path/to/save/the/map --overwrite
```

#### Saving all maps
Similar to the `load` command, saving can also be applied for multiple maps with the `save_all` command.
If no argument is given, the maps will be saved into their map folder.
Alternatively, the flag `--maps_folder` can be specified.
This indicates a path where all maps will be saved into.
E.g., if `--maps_folder` is set to `maps_path`, the map with the key `my_map_key` will be stored into `maps_path/my_map_key`.
Similar to the normal `save` command, the flag `--overwrite` needs to be specified if pre-existing maps should be overwritten.

Example call:
```
save_all --maps_folder path/to/save/maps/into --overwrite
```

### Listing all loaded maps and changing the selected map
Use `ls` to list all maps that are loaded in the current maplab session.
The selected map will be highlighted in green.

![Listing Maps](images/maplab-ls.png)

`ls` lists all map keys and shows the map's respective map folder next to here.
Check out TODO to learn more about the map folder.

To change the selected map, use the `select` command. E.g. you can do
```
select --map_key my_empty_map
```
Note that the map key in the orange brackets should change to show the newly selected map:

![Select Map](images/maplab-select.png)

# Inspecting and visualizing a map
You can use the `ms` (**m**ap **s**tatistics) command to get information about the selected map.
In the case of the `vi_app_test` map, the output of `ms` should look similar to this:
```
VI-Map:

Mission 0:              c7957efb9cf29d4874676106d6beb551  Viwls
  Vertices:             405
  Landmarks:            8359
  Landmarks by first observer backlink:
  Camera 0:             8359 (g:0 b:0 u:8359)
  Observations:         174464
  Distance travelled [m]:  23.406851
  Start to end time:    1970_01_01_03_13_13 to 1970_01_01_03_13_53
  T_G_M                 unknown
```

To visualize a map, use the `v` (**v**isualize) command.
You can then look at the map in `rviz`.
Make sure you have a `roscore` running and start `rviz` in a new terminal with
```sh
rosrun rviz rviz
```

On the left side, click the Add button and select the data you want to visualize.
In the tab `By topic`, select `Marker` from `/vi_map_edges/viwls` and `PointCloud2` from `vi_map_landmarks`, should give you a view similar to this:

![RViz Visualization](images/maplab-v-rviz.png)

### Map management in the console
#### Renaming a map
```
rename --map_key my_new_map_key
```
This will rename the currently selected map to `my_new_map_key`.

#### Copying a map
```
copy_map --target_map_key copied_map_key
```
This will copy the currently selected map into `copied_map_key`.

#### Deleting a map
```
delete
```
This will delete the currently selected map from the console.
The map on the file system will not be touched and will remain.
