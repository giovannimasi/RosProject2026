# ROS Project 2026 - Project 7

## Requirements

```
ros-noetic-turtlebot3
ros-noetic-turtlebot3-msgs
ros-noetic-turtlebot3-simulations
ros-noetic-gazebo-ros-pkgs
ros-noetic-gazebo-ros-control
ros-noetic-teleop-twist-keyboard
ros-noetic-tf ros-noetic-tf2
```

## Setup instructions

### Base docker setup

`Dockerfile`

```
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# 1. Tool base
RUN apt-get update && apt-get install -y \
    git tmux neovim python3-pip python3-catkin-tools python3-rosdep wget curl nano \
    && rm -rf /var/lib/apt/lists/*

# 2. Pacchetti TurtleBot3 e Gazebo
RUN apt-get update && apt-get install -y \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-tf ros-noetic-tf2 \
    && rm -rf /var/lib/apt/lists/*

# 3. Pacchetti Python
RUN pip3 install --no-cache-dir --upgrade pip
RUN pip3 --default-timeout=1000 install --no-cache-dir \
    numpy scipy nashpy axelrod gym pettingzoo stable-baselines3

# 4. Installa il plugin per collisioni degli Actor
WORKDIR /opt/gazebo_plugins
RUN git clone https://github.com/JiangweiNEU/actor_collisions.git && \
    cd actor_collisions && mkdir build && cd build && \
    cmake .. && make

# Espone il plugin a Gazebo tramite variabile d'ambiente fissa (non serve il .bashrc)
ENV GAZEBO_PLUGIN_PATH=/opt/gazebo_plugins/actor_collisions/build:${GAZEBO_PLUGIN_PATH}

# 5. Inizializza rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update

WORKDIR /root/ros1_ws
CMD ["bash"]
```

`docker-compose.yml`

```
services:
  ros1_noetic:
    build: 
      context: .
      dockerfile: Dockerfile
    image: ros-noetic-game-theory:latest
    container_name: ros1_dev
    network_mode: "host"
    environment:
      - DISPLAY=${DISPLAY}
      - QT_QPA_PLATFORM=xcb
      - XDG_RUNTIME_DIR=/run/user/1000
      - GZ_PARTITION=test
      - ROS_DOMAIN_ID=0
      - QT_X11_NO_MITSHM=1
      - ROS_MASTER_URI=http://127.0.0.1:11311
      - ROS_HOSTNAME=127.0.0.1
      - LIBGL_ALWAYS_SOFTWARE=1
      - QT_AUTO_SCREEN_SCALE_FACTOR=1
      - QT_ENABLE_HIGHDPI_SCALING=1
    volumes:
      - /etc/hosts:/etc/hosts:ro
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./ros1_ws:/root/ros1_ws
      - /home/giovannimasi/MATLAB/Projects/Robotics:/root/Robotics
      - ./.bashrc:/root/.bashrc
      - /dev/dri:/dev/dri
    # Mette il source nel .bashrc e poi tiene il container acceso
    command: sleep infinity
    restart: "no"
```
### Run simulation

```
cd ~/ros1_ws
catkin build
source devel/setup.bash
roslaunch game_theory_sim game_theory.launch
```

## Contributors

Giovanni Masi

Pierfilippo Orsini

Federico Melizza

Riccardo Landi

Alessandro Salomone
