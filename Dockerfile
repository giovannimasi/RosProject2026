FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Base tools
RUN apt-get update && apt-get install -y \
    git tmux neovim python3-pip python3-catkin-tools python3-rosdep wget curl nano \
    && rm -rf /var/lib/apt/lists/*

# TurtleBot3 and Gazebo dependencies
RUN apt-get update && apt-get install -y \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-tf ros-noetic-tf2 \
    ros-noetic-ompl \
    ros-noetic-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# Pyhton deps
RUN pip3 install --no-cache-dir --upgrade pip
RUN pip3 --default-timeout=1000 install --no-cache-dir \
    numpy scipy nashpy axelrod gym pettingzoo stable-baselines3

# Actor collisions plugin
WORKDIR /opt/gazebo_plugins
RUN git clone https://github.com/JiangweiNEU/actor_collisions.git && \
    cd actor_collisions && mkdir build && cd build && \
    cmake .. && make

# Gazebo plugin directory env 
ENV GAZEBO_PLUGIN_PATH=/opt/gazebo_plugins/actor_collisions/build:${GAZEBO_PLUGIN_PATH}

# Rosdep init
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update

WORKDIR /root/ros1_ws
CMD ["bash"]
