# Use ROS2 Jazzy as base image
FROM osrf/ros:jazzy-desktop

# Install basic development tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /ros2_ws

# Source ROS2 in the bashrc (for interactive shells)
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Create a test script that sources ROS2 setup for non-interactive shells
RUN echo '#!/bin/bash\n\
source /opt/ros/jazzy/setup.bash\n\
echo "Starting ROS2 demo nodes test..."\n\
\n\
# Start both nodes in background\n\
echo "Starting listener..."\n\
ros2 run demo_nodes_cpp listener &\n\
LISTENER_PID=$!\n\
\n\
echo "Starting talker..."\n\
ros2 run demo_nodes_cpp talker &\n\
TALKER_PID=$!\n\
\n\
# Wait for 10 seconds\n\
echo "Running for 10 seconds..."\n\
sleep 10\n\
\n\
# Cleanup both processes\n\
echo "Cleaning up..."\n\
kill $LISTENER_PID $TALKER_PID\n\
wait $LISTENER_PID $TALKER_PID 2>/dev/null\n\
\n\
echo "Test complete!"\n\
' > /ros2_ws/test_ros2.sh && chmod +x /ros2_ws/test_ros2.sh

# Set the entrypoint
ENTRYPOINT ["/bin/bash"] 