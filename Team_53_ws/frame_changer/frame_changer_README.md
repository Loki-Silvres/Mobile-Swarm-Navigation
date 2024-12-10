# Frame Changer

## Overview

The Frame Changer Node is designed to republish transformation (tf) messages from a namespaced topic to the global `/tf` and `/tf_static` topics.

## Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ns_name` | String | `bot_0` | Namespace for the robot's transform topics |

## Topic Subscriptions

### Namespaced Transform Topics
- **Topic**: `/ns_name1/tf`
- **Message Type**: `TFMessage`
- **Description**: Subscribes to the namespaced transform topic of a specific robot

## Topic Publications

### Global Transform Topics
1. **Topic**: `/tf`
   - **Message Type**: `TFMessage`
   - **Description**: Republishes transforms to the global tf topic

2. **Topic**: `/tf_static`
   - **Message Type**: `TFMessage`
   - **Description**: Republishes static transforms to the global tf_static topic

## Functionality

The node performs the following key functions:
- Listens to transformation messages within a specific namespace
- Republishes these transforms to the global transform topics
- Enables cross-namespace transform sharing
- Facilitates multi-robot transform coordination

