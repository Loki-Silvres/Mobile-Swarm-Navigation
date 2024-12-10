# Turtlebot3_Multi_Robot

## Overview
The Turtlebot3_Multi_Robot is a package designed to launch a comprehensive robotic simulation environment with multiple robots.

## Execution
To launch the simulation with 4 robots at specified poses:
```bash
python3 launch/spawn_bots.py --num_bots=4 --x_pose 0.0 0.0 0.0 0.5 --y_pose 0.0 -0.5 0.5 0.0
```

## Key Features
- Launches entire simulation environment
- Launches the entire navigation stack for each robot
- Supports configurable number of robots
- Robots are launched in separate namespaces for isolation and independent control

### Command-Line Parameters
- `--num_bots`: Number of robots to spawn (default: 4)
- `--x_pose`: Initial x-coordinates for robots
- `--y_pose`: Initial y-coordinates for robots

## Deployment Characteristics
- Enables parallel robot deployment
- Provides flexible initial positioning
- Supports scalable multi-robot simulation setup

