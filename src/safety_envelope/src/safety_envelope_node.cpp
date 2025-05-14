/**
 * @file safety_envelope_node.cpp
 * @brief Main entry point for the safety envelope node
 * 
 * This file contains the main function that initializes and runs the
 * safety envelope node. It creates an instance of the SafetyEnvelope class
 * with default parameters and spins the ROS2 node.
 */

#include "../include/safety_envelope/safety_envelope.hpp"

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create a default safety boundary
    safety::BoundaryBox default_boundary{
        safety::Point3D{-0.5, -0.5, -0.5},  // min corner
        safety::Point3D{0.5, 0.5, 0.5}      // max corner
    };
    
    // Create the safety envelope node
    auto node = std::make_shared<safety::SafetyEnvelope>(default_boundary);
    
    // Spin the node
    rclcpp::spin(node);
    
    // Shutdown ROS2
    rclcpp::shutdown();
    
    return 0;
}
