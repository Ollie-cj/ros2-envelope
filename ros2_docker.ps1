# ROS2 Docker helper script for Windows
# PowerShell equivalent of ros2_docker.sh

# Display help message
function Show-Help {
    Write-Host "ROS2 Docker Development Environment Helper for Windows"
    Write-Host ""
    Write-Host "Usage: .\ros2_docker.ps1 [command]"
    Write-Host ""
    Write-Host "Commands:"
    Write-Host "  build       Build the ROS2 development environment"
    Write-Host "  start       Start an interactive ROS2 development session"
    Write-Host "  run [cmd]   Run a command in the ROS2 container"
    Write-Host "  test        Run the ROS2 demo test script"
    Write-Host "  clean       Remove built volumes and containers"
    Write-Host "  help        Show this help message"
    Write-Host ""
    Write-Host "Examples:"
    Write-Host "  .\ros2_docker.ps1 build"
    Write-Host "  .\ros2_docker.ps1 start"
    Write-Host "  .\ros2_docker.ps1 run 'ros2 topic list'"
    Write-Host "  .\ros2_docker.ps1 test"
}

# Build the Docker image
function Build-Image {
    Write-Host "Building ROS2 development environment..."
    docker-compose build
}

# Start an interactive session
function Start-Session {
    Write-Host "Starting ROS2 development environment..."
    Write-Host "Use Ctrl+D or type 'exit' to exit the container."
    docker-compose up -d
    docker-compose exec ros2_dev bash
}

# Run a command in the container
function Run-Command {
    param (
        [string]$Command
    )
    
    if ([string]::IsNullOrEmpty($Command)) {
        Write-Host "Error: No command specified" -ForegroundColor Red
        Write-Host "Usage: .\ros2_docker.ps1 run [command]"
        exit 1
    }
    
    # Check if container is running, if not start it
    $containerRunning = docker-compose ps | Select-String -Pattern "ros2_dev.*Up" -Quiet
    if (-not $containerRunning) {
        docker-compose up -d
    }
    
    Write-Host "Running command in ROS2 container: $Command"
    docker-compose exec ros2_dev bash -c "source /opt/ros/jazzy/setup.bash && $Command"
}

# Run the test script
function Run-Test {
    Write-Host "Running ROS2 demo test..."
    # Check if container is running, if not start it
    $containerRunning = docker-compose ps | Select-String -Pattern "ros2_dev.*Up" -Quiet
    if (-not $containerRunning) {
        docker-compose up -d
    }
    docker-compose exec ros2_dev /ros2_ws/test_ros2.sh
}

# Clean up resources
function Clean-Resources {
    Write-Host "Cleaning up ROS2 Docker resources..."
    docker-compose down -v
}

# Main script logic
$command = $args[0]
switch ($command) {
    "build" {
        Build-Image
    }
    "start" {
        Start-Session
    }
    "run" {
        $runCommand = $args[1..($args.Length-1)] -join " "
        Run-Command -Command $runCommand
    }
    "test" {
        Run-Test
    }
    "clean" {
        Clean-Resources
    }
    default {
        Show-Help
    }
}
