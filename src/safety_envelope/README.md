# Safety Envelope Package

A ROS2 package that implements a safety envelope system for haptic devices or robotic tools. The system monitors the position of a tool and checks if it stays within a predefined safety boundary. If the tool moves outside the boundary, the system immediately publishes a stop signal to prevent potential hazards.

## Features

- Real-time performance with processing time under 10 ms
- Watchdog mechanism to detect communication failures
- Comprehensive safety considerations following medical device standards
- Microcontroller-compatible implementation

## Package Structure

```
safety_envelope/
├── CMakeLists.txt           # Build configuration
├── package.xml              # Package metadata
├── README.md                # This file
├── include/                 # Header files
│   └── safety_envelope/
│       └── safety_envelope.hpp
├── src/                     # Source files
│   ├── safety_envelope.cpp  # Main implementation
│   ├── safety_envelope_node.cpp  # Node executable
│   └── microcontroller_stub.cpp  # Microcontroller implementation
├── launch/                  # Launch files
│   └── safety_envelope.launch.py
├── config/                  # Configuration files
│   └── default_boundaries.yaml
├── doc/                     # Documentation
│   └── safety_envelope_notes.md
└── test/                    # Unit tests
    └── test_safety_envelope.cpp
```

## Building the Package

```bash
# Clone the repository
git clone <repository-url>
cd <repository-directory>

# Build with colcon
colcon build --packages-select safety_envelope
source install/setup.bash
```

## Running the Node

```bash
# Run using the launch file
ros2 launch safety_envelope safety_envelope.launch.py
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

## Running Tests

```bash
colcon test --packages-select safety_envelope
colcon test-result --verbose
```

## Configuration

The safety envelope can be configured using the parameters in `config/default_boundaries.yaml`. You can modify the boundary dimensions, topic names, and watchdog settings.

## License

Apache License 2.0
