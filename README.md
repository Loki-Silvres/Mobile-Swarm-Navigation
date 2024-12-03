## Installation

1. Clone the repository:
```bash
cd 
git clone https://github.com/Loki-Silvres/Mobile-Swarm-Navigation.git
cd Mobile-Swarm-Navigation/
```
2. Build workspace:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/Mobile-Swarm-Navigation/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=waffle_pi 
export GAZEBO_MODEL_PATH=$HOME/Mobile-Swarm-Navigation/src/aws-robomaker-hospital-world/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$HOME/Mobile-Swarm-Navigation/src/aws-robomaker-hospital-world/fuel_models:$GAZEBO_MODEL_PATH
# export GAZEBO_MODEL_PATH=$HOME/Mobile-Swarm-Navigation/src/my_world/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$HOME/Mobile-Swarm-Navigation/src/aws-robomaker-small-warehouse-world/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$HOME/Mobile-Swarm-Navigation/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/plugins:$GAZEBO_PLUGIN_PATH

cd src/aws-robomaker-hospital-world/
rosdep install --from-paths . --ignore-src -r -y
chmod +x setup.sh
./setup.sh

cd ../../
colcon build
source install/setup.bash
```

## Deployment:

1. Launch world and robots
```bash
cd ~/Mobile-Swarm-Navigation/
source install/setup.bash

python3 src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/launch/map_maker_launch.py
```

2. Teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/bot_0/cmd_vel
```