# Interpreter Component

## Overview
The Interpreter is a ROS2 node that acts as an interface between the user and the ROS system, managing requests and interactions with a database.

## Execution
To run the Interpreter:
```bash
ros2 run interpreter interpreter
```

## ROS Topics

### Subscribed Topics
- `/get_object`
  - Interface Type: `std_msgs.msg.String()`
  - Publisher: Chatbot

### Published Topics
- `/schedule_goal`
  - Interface Type: `geometry_msgs.msg.PoseStamped`

## Key Characteristics

### Database Interaction
- Reads data from a CSV file (database)
- Filters rows based on the incoming object name from the `/get_object` topic
- Processes requests using a queue data structure

### Request Processing
- Uses `Node().create_timer()` to process requests at a specified frequency
- Selects the top row from filtered database entries
- Publishes goal coordinates with standard orientation:
  - Quaternion (w, x, y, z): (1, 0, 0, 0)

## Implementation Details
- Implemented as a ROS2 node
- Utilizes queue data structure for request management
- Dynamically processes and publishes goals based on database entries
