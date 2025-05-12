/**
 * @file safety_envelope.cpp
 * @brief Implementation file for the safety envelope point inclusion testing system
 * 
 * This file implements the functionality for testing whether a point
 * (such as a robot tooltip position) is inside a defined safety envelope.
 * 
 * The implementation includes:
 * - Geometric algorithms for point-in-boundary testing
 * - Distance calculations from points to envelope boundaries
 * - Safety margin enforcement
 * - Real-time performance optimizations
 * - Watchdog mechanism for system safety
 * 
 * This system is designed to work with ROS2 to integrate with robotic systems
 * and prevent the robot tooltip from leaving the designated safe operational area.
 */

#include "headers/safety_envelope.hpp"
#include <algorithm>
#include <cmath>

namespace safety {

SafetyEnvelope::SafetyEnvelope(
    const BoundaryBox& boundary,
    const rclcpp::NodeOptions& node_options
) : Node("safety_envelope", node_options),
    boundary_(boundary),
    last_processing_time_ms_(0.0),
    total_processing_time_ms_(0.0),
    max_processing_time_ms_(0.0),
    pose_count_(0),
    received_pose_since_last_watchdog_(false)
{
    // Declare and get parameters
    this->declare_parameter("topic.tool_pose", "/tool/pose");
    this->declare_parameter("topic.stop_flag", "/safety/stop_flag");
    this->declare_parameter("watchdog.period_ms", 100);
    
    const auto tool_pose_topic = this->get_parameter("topic.tool_pose").as_string();
    const auto stop_flag_topic = this->get_parameter("topic.stop_flag").as_string();
    const auto watchdog_period_ms = this->get_parameter("watchdog.period_ms").as_int();
    
    // Create a subscription to the tool pose topic with a callback
    // Use a QoS profile suitable for real-time control
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::Volatile)
        .deadline(std::chrono::milliseconds(20));
    
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        tool_pose_topic,
        qos,
        std::bind(&SafetyEnvelope::poseCallback, this, std::placeholders::_1)
    );
    
    // Create a publisher for the stop flag
    stop_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        stop_flag_topic,
        qos
    );
    
    // Create a watchdog timer to detect if pose messages stop arriving
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(watchdog_period_ms),
        std::bind(&SafetyEnvelope::watchdogCallback, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Safety envelope node initialized");
    RCLCPP_INFO(this->get_logger(), "Boundary min: (%f, %f, %f)", 
                boundary_.min.x, boundary_.min.y, boundary_.min.z);
    RCLCPP_INFO(this->get_logger(), "Boundary max: (%f, %f, %f)", 
                boundary_.max.x, boundary_.max.y, boundary_.max.z);
}

void SafetyEnvelope::setBoundary(const BoundaryBox& boundary) {
    std::lock_guard<std::mutex> lock(boundary_mutex_);
    boundary_ = boundary;
    
    RCLCPP_INFO(this->get_logger(), "Safety boundary updated");
    RCLCPP_INFO(this->get_logger(), "New boundary min: (%f, %f, %f)", 
                boundary_.min.x, boundary_.min.y, boundary_.min.z);
    RCLCPP_INFO(this->get_logger(), "New boundary max: (%f, %f, %f)", 
                boundary_.max.x, boundary_.max.y, boundary_.max.z);
}

BoundaryBox SafetyEnvelope::getBoundary() const {
    std::lock_guard<std::mutex> lock(boundary_mutex_);
    return boundary_;
}

bool SafetyEnvelope::isPointWithinBoundary(const Point3D& point) const {
    std::lock_guard<std::mutex> lock(boundary_mutex_);
    
    // Check if the point is within the boundary box
    return (point.x >= boundary_.min.x && point.x <= boundary_.max.x &&
            point.y >= boundary_.min.y && point.y <= boundary_.max.y &&
            point.z >= boundary_.min.z && point.z <= boundary_.max.z);
}

double SafetyEnvelope::distanceToBoundary(const Point3D& point) const {
    std::lock_guard<std::mutex> lock(boundary_mutex_);
    
    // Calculate the distance to each boundary plane
    double dx_min = point.x - boundary_.min.x;
    double dx_max = boundary_.max.x - point.x;
    double dy_min = point.y - boundary_.min.y;
    double dy_max = boundary_.max.y - point.y;
    double dz_min = point.z - boundary_.min.z;
    double dz_max = boundary_.max.z - point.z;
    
    // Find the minimum distance (negative if outside)
    double min_dist = std::min({dx_min, dx_max, dy_min, dy_max, dz_min, dz_max});
    
    return min_dist;
}

double SafetyEnvelope::getLastProcessingTimeMs() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return last_processing_time_ms_;
}

double SafetyEnvelope::getAverageProcessingTimeMs() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    if (pose_count_ == 0) {
        return 0.0;
    }
    return total_processing_time_ms_ / static_cast<double>(pose_count_);
}

double SafetyEnvelope::getMaxProcessingTimeMs() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return max_processing_time_ms_;
}

void SafetyEnvelope::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Start timing the processing
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Mark that we received a pose for the watchdog
    {
        std::lock_guard<std::mutex> lock(watchdog_mutex_);
        received_pose_since_last_watchdog_ = true;
    }
    
    // Convert the pose to our internal format
    Point3D point = convertToPoint3D(msg->pose.position);
    
    // Check if the point is within the boundary
    bool is_within = isPointWithinBoundary(point);
    
    // Calculate distance to boundary (for logging/debugging)
    double distance = distanceToBoundary(point);
    
    // Prepare the stop flag message (true = stop, false = safe to continue)
    auto stop_msg = std::make_unique<std_msgs::msg::Bool>();
    stop_msg->data = !is_within;  // Stop if outside boundary
    
    // Publish the stop flag
    stop_flag_pub_->publish(std::move(stop_msg));
    
    // Log the result (at debug level to avoid flooding logs)
    if (!is_within) {
        RCLCPP_WARN(this->get_logger(), 
                   "Tool position (%f, %f, %f) is outside safety boundary by %f units",
                   point.x, point.y, point.z, -distance);
    } else {
        RCLCPP_DEBUG(this->get_logger(), 
                    "Tool position (%f, %f, %f) is within safety boundary, distance to boundary: %f",
                    point.x, point.y, point.z, distance);
    }
    
    // Calculate processing time
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double processing_time_ms = duration.count() / 1000.0;
    
    // Update performance metrics
    {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        last_processing_time_ms_ = processing_time_ms;
        total_processing_time_ms_ += processing_time_ms;
        max_processing_time_ms_ = std::max(max_processing_time_ms_, processing_time_ms);
        pose_count_++;
        
        // Log a warning if processing time exceeds 10ms
        if (processing_time_ms > 10.0) {
            RCLCPP_WARN(this->get_logger(), 
                       "Processing time exceeded 10ms threshold: %f ms",
                       processing_time_ms);
        }
    }
}

void SafetyEnvelope::watchdogCallback() {
    bool received_pose = false;
    
    {
        std::lock_guard<std::mutex> lock(watchdog_mutex_);
        received_pose = received_pose_since_last_watchdog_;
        received_pose_since_last_watchdog_ = false;
    }
    
    if (!received_pose) {
        // No pose messages received since last watchdog check
        // This could indicate a communication failure
        RCLCPP_ERROR(this->get_logger(), "Watchdog: No pose messages received recently!");
        
        // Publish a stop flag as a safety measure
        auto stop_msg = std::make_unique<std_msgs::msg::Bool>();
        stop_msg->data = true;  // Stop the system
        stop_flag_pub_->publish(std::move(stop_msg));
    }
}

Point3D SafetyEnvelope::convertToPoint3D(const geometry_msgs::msg::Point& point) const {
    return Point3D{point.x, point.y, point.z};
}

} // namespace safety

// Main function to demonstrate how to use the SafetyEnvelope class
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
