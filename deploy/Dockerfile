FROM ubuntu:18.04

ENV TZ=Europe/Zurich
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install the base packages.
COPY install_base.sh ./install_base.sh
RUN sh ./install_base.sh && rm ./install_base.sh

# Install the ROS environment.
COPY install_ros.sh ./install_ros.sh
RUN sh ./install_ros.sh && rm ./install_ros.sh

# Configure the catkin workspace.
WORKDIR /usr/home
COPY ./ws ./ws
WORKDIR /usr/home/ws
RUN catkin init --workspace .
RUN catkin config --merge-devel
RUN catkin config -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-fdiagnostics-color=always

# Install the custom packages.
WORKDIR /usr/home/ws/src
COPY install_packages.sh ./install_packages.sh
RUN sh ./install_packages.sh && rm ./install_packages.sh

# Build the project.
WORKDIR /usr/home/ws
SHELL ["/bin/bash", "-c"]
COPY compile.sh ../
RUN ../compile.sh
COPY set_env.sh ../
ENTRYPOINT ["../set_env.sh"]
