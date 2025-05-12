# Haptic Safety Envelope System

A ROS2 node that ingests tool pose, checks against a virtual boundary, and publishes a stop flag in less than 10 ms.

## Overview

This project implements a safety envelope system for haptic devices or robotic tools. The system monitors the position of a tool and checks if it stays within a predefined safety boundary. If the tool moves outside the boundary, the system immediately publishes a stop signal to prevent potential hazards.

Key features:
- Real-time performance with processing time under 10 ms
- Watchdog mechanism to detect communication failures
- Comprehensive safety considerations following medical device standards
- Microcontroller-compatible implementation

## Components

- **SafetyEnvelope Class**: A ROS2 node that subscribes to tool pose messages, checks against safety boundaries, and publishes stop flags
- **Microcontroller Stub**: A simplified implementation suitable for deployment on microcontrollers
- **Safety Notes**: Documentation on deterministic scheduling, watchdogs, and IEC 62304 artifacts

## Building the Project

The project uses Docker to provide a consistent build environment. To build the project:

```bash
# Build the Docker image
./ros2_docker.ps1 build

# Start the Docker container
./ros2_docker.ps1 start

# Inside the Docker container, build the project
cd /ros2_ws
colcon build
source install/setup.bash
```

## Running the Safety Envelope Node

To run the safety envelope node:

```bash
# Inside the Docker container
ros2 run safety_envelope safety_envelope_node
```

## Testing

You can test the safety envelope by publishing tool pose messages:

```bash
# Publish a pose within the boundary
ros2 topic pub /tool/pose geometry_msgs/msg/PoseStamped '{header: {frame_id: "base_link"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}'

# Publish a pose outside the boundary
ros2 topic pub /tool/pose geometry_msgs/msg/PoseStamped '{header: {frame_id: "base_link"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}}}'
```

Check the stop flag:

```bash
ros2 topic echo /safety/stop_flag
```

## Performance Monitoring

The node logs performance metrics including processing time. You can monitor these metrics using:

```bash
ros2 run rqt_console rqt_console
```

## Safety Considerations

For detailed information on safety considerations, including deterministic scheduling, watchdogs, and IEC 62304 artifacts, see the [safety_notes.md](src/safety_notes.md) document.

## License

Apache License 2.0
