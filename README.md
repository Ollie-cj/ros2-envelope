# Haptic Safety Envelope System

A ROS2 node that ingests tool pose, checks against a virtual boundary, and publishes a stop flag in less than 10 ms.

## Overview

This project implements a safety envelope system for haptic devices or robotic tools. The system monitors the position of a tool and checks if it stays within a predefined safety boundary. If the tool moves outside the boundary, the system immediately publishes a stop signal to prevent potential hazards.

Key features:
- Real-time performance with processing time under 10 ms
- Watchdog mechanism to detect communication failures
- Comprehensive safety considerations following medical device standards
- Microcontroller-compatible implementation

## Project Structure

```
project-geonosis/
├── .gitignore
├── docker-compose.yml
├── Dockerfile
├── README.md
├── ros2_docker.ps1
├── ros2_docker.sh
└── src/
    └── safety_envelope/         # Main ROS2 package
        ├── CMakeLists.txt       # Build configuration
        ├── package.xml          # Package metadata
        ├── README.md            # Package-specific documentation
        ├── include/             # Header files
        │   └── safety_envelope/
        │       └── safety_envelope.hpp
        ├── src/                 # Source files
        │   ├── safety_envelope.cpp       # Main implementation
        │   ├── safety_envelope_node.cpp  # Node executable
        │   └── microcontroller_stub.cpp  # Microcontroller implementation
        ├── launch/              # Launch files
        │   └── safety_envelope.launch.py
        ├── config/              # Configuration files
        │   └── default_boundaries.yaml
        ├── doc/                 # Documentation
        │   └── safety_envelope_notes.md
        └── test/                # Unit tests
            └── test_safety_envelope.cpp
```

## Components

- **SafetyEnvelope Class**: A ROS2 node that subscribes to tool pose messages, checks against safety boundaries, and publishes stop flags
- **Microcontroller Stub**: A simplified implementation suitable for deployment on microcontrollers
- **Safety Notes**: Documentation on deterministic scheduling, watchdogs, and IEC 62304 artifacts
- **Launch Files**: Easy-to-use launch configuration for the safety envelope node
- **Configuration Files**: YAML-based configuration for boundary parameters
- **Unit Tests**: Comprehensive tests for the safety envelope functionality

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
# Method 1: Using the executable directly
ros2 run safety_envelope safety_envelope_node

# Method 2: Using the launch file (recommended)
ros2 launch safety_envelope safety_envelope.launch.py
```

The launch file method is recommended as it automatically loads the configuration from the default_boundaries.yaml file.

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

For detailed information on safety considerations, including deterministic scheduling, watchdogs, and IEC 62304 artifacts, see the [safety_envelope_notes.md](src/safety_envelope/doc/safety_envelope_notes.md) document.

## License

Apache License 2.0
