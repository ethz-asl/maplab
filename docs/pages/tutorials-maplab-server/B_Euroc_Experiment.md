## Maplab Server Euroc Experiment

In this tutorial we will use the maplab server to online build and optimize a map with five robots running the mapping node. For this purpose we will use five recordings from the EuRoC dataset that happened in the same environment.

The necessary launch files are already provided [here](https://github.com/ethz-asl/maplab/tree/master/applications/maplab-server-node/launch/euroc).

## Setting up
Download the five rosbags from the Machine Hall section in the [EuRoC dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) and place them in a folder. Afterwards change this [line](https://github.com/ethz-asl/maplab/blob/d00bc1ba35f9ed05713cdaaf2fa125d375cd2e70/applications/maplab-server-node/launch/euroc/euroc-maplab-server-robots.launch#L5) of the launch file to point to the folder where the bag files are stored.

For visualization purposes you can open RViz and load the provided [config file](https://github.com/ethz-asl/maplab/blob/master/applications/maplab-server-node/launch/euroc/ros/euroc-maplab-server-minimal.rviz).

## Starting the experiment
In one terminal first start the server by running `roslaunch maplab_server euroc-maplab-server-base-station.launch`. The server will periodically output a status update on the current situation regarding received maps.

In a separate terminal start up all five robots by running `roslaunch maplab_server euroc-maplab-server-robots.launch`. After the first set of submaps is transmitted (approx. 20 seconds) you should start seeing output in RViz. As the robots further explore their environment the maps will get merged into one global map.

When the robots are done the server will continue optimizing the global map. To force it to save the map you can run a service call in a separate terminal `rosservice call /maplab_server/save_map`. The merged map can then be loaded from the console from the default directory which is `/tmp/maplab_server/merged_map`.
