# ROS Project 2026 - Project 7

## Setup instructions

Docker setup is already present in the repo. Use the following instruction to build and start.

### Build image and start the container

```bash
docker compose -f docker-compose.yml -f docker-compose.wsl.yml up -d --build
```

### Enter in container bash

```bash
./ros1.sh
```

The script will enable x11 permission for docker user, so that he can use GUI environment. Then, you enter bash inside the container.

### Run simulation

```bash
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
