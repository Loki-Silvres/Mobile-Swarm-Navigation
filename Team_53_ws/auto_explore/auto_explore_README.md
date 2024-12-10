# Auto Explore Package

## Overview

This package is used to autonomously create environment maps for multiple robots. It uses a frontier-based search algorithm for mapping, taking the following approach:

- Retrieves initial bot pose from `/initialpose`
- Searches for nearby empty space using cost map from `/map` topic
- Publishes a nav2 goal pose to `/goal_pose`
- Navigates the bot to the identified space
- Continues until all map contours are closed

## Node Details

### Subscription Details

#### Costmap
- **Topic**: `costmap`
- **Type**: `nav_msgs/OccupancyGrid`
- **Description**: Map used for exploration planning
  - Can be a costmap from `move_base` or SLAM-created map
  - Requires properly marked unknown space

#### Costmap Updates
- **Topic**: `costmap_updates`
- **Type**: `map_msgs/OccupancyGridUpdate`
- **Description**: Incremental costmap updates (optional)

### Publication Details

#### Frontiers
- **Topic**: `~frontiers`
- **Type**: `visualization_msgs/MarkerArray`
- **Description**: Visualization of frontiers
  - Frontier points shown in blue
  - Small spheres indicate frontier cost

## Parameters

| Parameter | Default | Type | Description |
|-----------|---------|------|-------------|
| `~robot_base_frame` | `base_link` | string | Base frame of the robot |
| `~costmap_topic` | `costmap` | string | Source OccupancyGrid topic |
| `~costmap_updates_topic` | `costmap_updates` | string | Incremental updates topic |
| `~visualize` | `false` | bool | Publish visualized frontiers |
| `~planner_frequency` | `1.0` | double | Frontier computation rate (Hz) |
| `~progress_timeout` | `30.0` | double | Time before abandoning current goal |
| `~potential_scale` | `1e-3` | double | Frontier potential weight |
| `~orientation_scale` | `0` | double | Frontier orientation weight |
| `~gain_scale` | `1.0` | double | Frontier gain weight |
| `~transform_tolerance` | `0.3` | double | Transform tolerance |
| `~min_frontier_size` | `0.5` | double | Minimum frontier size to explore (meters) |

## Transform Requirements

- **From**: Global frame
- **To**: Robot base frame
- **Description**: Typically provided by mapping algorithm

## Action API

### Move Base
- **Action**: `move_base`
- **Type**: `move_base_msgs/MoveBaseAction`
- **Description**: Actionlib API for posting goals

## Resources

### Package Repository
- [GitHub Repository](https://github.com/robo-friends/m-explore-ros2?tab=readme-ov-file#Autonomous-exploration)

