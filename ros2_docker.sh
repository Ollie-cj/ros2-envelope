#!/bin/bash
# ROS2 Docker helper script

# Display help message
function show_help {
    echo "ROS2 Docker Development Environment Helper"
    echo ""
    echo "Usage: ./ros2_docker.sh [command]"
    echo ""
    echo "Commands:"
    echo "  build       Build the ROS2 development environment"
    echo "  start       Start an interactive ROS2 development session"
    echo "  run [cmd]   Run a command in the ROS2 container"
    echo "  test        Run the ROS2 demo test script"
    echo "  clean       Remove built volumes and containers"
    echo "  help        Show this help message"
    echo ""
    echo "Examples:"
    echo "  ./ros2_docker.sh build"
    echo "  ./ros2_docker.sh start"
    echo "  ./ros2_docker.sh run 'ros2 topic list'"
    echo "  ./ros2_docker.sh test"
}

# Build the Docker image
function build_image {
    echo "Building ROS2 development environment..."
    docker-compose build
}

# Start an interactive session
function start_session {
    echo "Starting ROS2 development environment..."
    echo "Use Ctrl+D or type 'exit' to exit the container."
    docker-compose up -d
    docker-compose exec ros2_dev bash
}

# Run a command in the container
function run_command {
    if [ -z "$1" ]; then
        echo "Error: No command specified"
        echo "Usage: ./ros2_docker.sh run [command]"
        exit 1
    fi
    
    # Check if container is running, if not start it
    if ! docker-compose ps | grep -q "ros2_dev.*Up"; then
        docker-compose up -d
    fi
    
    echo "Running command in ROS2 container: $1"
    docker-compose exec ros2_dev bash -c "source /opt/ros/jazzy/setup.bash && $1"
}

# Run the test script
function run_test {
    echo "Running ROS2 demo test..."
    # Check if container is running, if not start it
    if ! docker-compose ps | grep -q "ros2_dev.*Up"; then
        docker-compose up -d
    fi
    docker-compose exec ros2_dev /ros2_ws/test_ros2.sh
}

# Clean up resources
function clean_resources {
    echo "Cleaning up ROS2 Docker resources..."
    docker-compose down -v
}

# Main script logic
case "$1" in
    build)
        build_image
        ;;
    start)
        start_session
        ;;
    run)
        shift
        run_command "$*"
        ;;
    test)
        run_test
        ;;
    clean)
        clean_resources
        ;;
    help|*)
        show_help
        ;;
esac
