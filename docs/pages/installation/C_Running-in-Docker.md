## Running in Docker

This is my setup for running maplab in Docker, with the afferent setup files. Other setups are definitely possible as well. In this configuration the source code files and build files are kept outside the docker environment, to facilitate easy development and rebuilding of the docker environment.

### Install docker
Install Docker following the official [install instructions](https://docs.docker.com/engine/install/ubuntu/).

Additionally install docker compose:
```
sudo apt install -y docker-compose
```

### Create the docker image
We set up a few folders on the main machine which will then later be mounted in docker.
```
mkdir -p maplab/maplab_ws/src
mkdir maplab/data                # Datasets and maps can be placed here
mkdir maplab/maplab_ws/.ccache   # Preserve ccache files across containers
```

Clone the maplab repository:
```
cd maplab/maplab_ws/src/
git clone git@github.com:ethz-asl/maplab.git --recursive

```

Move up the docker setup files and build the image:
```
cd ../../
cp maplab_ws/src/maplab/deploy/* ./
docker-compose build
```

### Using the docker container
Running a single command using docker-compose can then be done from the `maplab` folder where the docker files are. For example to start a roscore:
```
docker-compose run --rm maplab roscore
```

For more complex operations where continued work in a single terminal is desirable:
```
docker-compose run --rm maplab /bin/bash

```
This will start a terminal in the maplab container. The advantages are for example auto-completion in the terminal. However, changes made to the container are not persistent.

**Warning:** Maps created in the docker container will be deleted if they are not placed in one of the mounted persistent directories, which by default are `/data` or `/maplab`.


### Compile maplab in the container
To compile maplab in the container one can then simply run
```
docker-compose run --rm maplab catkin build maplab
```
Afterwards, the rest of the tutorials can be run from within the container.


### Visualizing using docker
Running GUI applications from docker can sometimes be problematic. In the current setup a quick (but for security not the safest) hack is to enable authentification to the xserver from within the container:
```
xhost +local:root
```

Access will then reset after a reboot of the machine, or by running:
```
xhost -local:root
```

Alternatively, I've had greater success running rviz from the based machine and not from inside the docker container. For this in Ubuntu 22.04:
```
sudo apt install rviz
```
