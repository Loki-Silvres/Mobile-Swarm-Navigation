# Scheduler Component

## Overview
The Scheduler is a Python script consisting of two main nodes: Primary Scheduler and Sub-Scheduler, responsible for task allocation and goal distribution among robots.

## Execution
To run the Scheduler:
```bash
python3 launch/scheduler_launch.py --num_bots=4 --x_pose 0.0 0.0 0.0 0.5 --y_pose 0.0 -0.5 0.5 0.0
```

## Primary Scheduler Node

### Node Parameters
- `num_bots`
  - Type: Integer
  - Default: 4
  - Description: Number of spawned robots

### Subscribed Topics
- `/schedule_goal`: `geometry_msgs.msg.PoseStamped`
- `/bot_0/odom` to `/bot_n/odom`: `nav_msgs.msg.Odometry`

### Published Topics
- `/scheduled_goals`: `std_msgs.msg.String()`
  - Uses custom encoding for goal scheduling

### Key Features
- Dynamic subscriber and publisher creation at runtime
- Robot task allocation based on:
  - Proximity to goal
  - Robot state (idle/occupied/exploration)
- Unique task ID and bot ID management

### Robot States
- 0: Idle
- 1: Occupied
- 2: Exploration

### Processing
- Processes goals queue 10 times per second
- Maintains a dictionary mapping tasks to robots
- Generates a goal string with format: `(bot_id,x,y,z,r,p,y,w)`

## Sub-Scheduler Node

### Node Parameters
- `num_bots`
  - Type: Integer
  - Default: 4
  - Description: Number of spawned robots

### Subscribed Topics
- `/scheduled_goals`: `std_msgs.msg.String()`

### Published Topics
- `/bot_0/goal_pose` to `/bot_n/goal_pose`: `geometry_msgs.msg.PoseStamped()`

### Key Features
- Dynamic publisher creation based on number of bots
- Publishes goals to respective bot topics
- Facilitates dynamic plan restructuring in complex environments

### Goal Distribution
- Parses incoming goal string (separated by ';')
- Publishes goals to corresponding bot topics
- Enables flexible replanning in dynamic scenarios
