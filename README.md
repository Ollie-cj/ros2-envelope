# ROS2 Docker Development Environment

This project provides a Docker-based development environment for ROS2 (Robot Operating System 2) using the Jazzy distribution.

## Prerequisites

- [Docker](https://www.docker.com/products/docker-desktop) installed and running
- [Docker Compose](https://docs.docker.com/compose/install/) installed (included with Docker Desktop for Windows)
- For Windows users: PowerShell

## Project Structure

- `Dockerfile`: Defines the ROS2 development environment
- `docker-compose.yml`: Configures the Docker services for ROS2 development
- `ros2_docker.ps1`: PowerShell script for Windows users to manage the Docker environment
- `ros2_docker.sh`: Bash script for Linux/macOS users (optional on Windows)
- `src/`: Directory for your ROS2 packages (will be mounted into the container)

## Getting Started on Windows

1. Make sure Docker Desktop is running
2. Open PowerShell in the project directory
3. Build the Docker image:
   ```
   .\ros2_docker.ps1 build
   ```
4. Start an interactive ROS2 development session:
   ```
   .\ros2_docker.ps1 start
   ```
5. Inside the container, you can create and build ROS2 packages:
   ```bash
   # Create a new ROS2 package (inside the container)
   cd /ros2_ws/src
   ros2 pkg create --build-type ament_cmake my_package
   
   # Build the workspace
   cd /ros2_ws
   colcon build
   
   # Source the workspace
   source /ros2_ws/install/setup.bash
   
   # Run your ROS2 nodes
   ros2 run my_package my_node
   ```

## Helper Script Commands

The `ros2_docker.ps1` script provides several commands to help manage your ROS2 development environment:

- `.\ros2_docker.ps1 build`: Build the Docker image
- `.\ros2_docker.ps1 start`: Start an interactive ROS2 development session
- `.\ros2_docker.ps1 run 'command'`: Run a specific command in the container
- `.\ros2_docker.ps1 test`: Run the ROS2 demo test script
- `.\ros2_docker.ps1 clean`: Clean up Docker resources

## GUI Applications on Windows

Running GUI applications from ROS2 (like RViz or Gazebo) on Windows requires additional setup:

1. Install an X Server for Windows, such as [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
2. Start the X Server with the following settings:
   - Multiple windows
   - Display number: 0
   - Start no client
   - Disable access control (check this option)
3. Find your IP address using PowerShell:
   ```
   ipconfig
   ```
4. Create a `.env` file in the project directory with:
   ```
   DISPLAY=YOUR_IP_ADDRESS:0.0
   ```
5. Update the docker-compose.yml file to include:
   ```yaml
   environment:
     - DISPLAY=${DISPLAY}
     - QT_X11_NO_MITSHM=1
   ```

## Development Workflow

1. Edit your ROS2 code in the `src/` directory using your favorite editor on Windows
2. Build and test your code inside the Docker container
3. Any changes you make to files in the `src/` directory will be immediately available inside the container
4. Build artifacts are stored in persistent volumes, so rebuilds are faster

## Troubleshooting

- If you encounter permission issues, make sure Docker Desktop has access to the drive where your project is located
- For networking issues, check that the required ports (11311, 9090) are not blocked by your firewall
- If GUI applications don't display, check that your X Server is running and properly configured
