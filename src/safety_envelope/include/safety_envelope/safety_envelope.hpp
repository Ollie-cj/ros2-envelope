/**
 * @file safety_envelope.hpp
 * @brief Header file for the safety envelope point inclusion testing system
 * 
 * This file contains the declarations for a system that tests whether a point
 * (such as a robot tooltip position) is inside a defined safety envelope.
 * The safety envelope represents a boundary that the robot should not cross
 * during operation to ensure safe interaction with its environment.
 * 
 * Features implemented:
 * - Definition of safety envelope boundaries
 * - Point inclusion testing algorithms
 * - Safety violation detection and handling
 * - Real-time performance optimizations
 * 
 * This system will help prevent the robot tooltip from leaving the designated
 * safe operational area, enhancing operational safety.
 */

#ifndef SAFETY_ENVELOPE_HPP
#define SAFETY_ENVELOPE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <array>
#include <chrono>
#include <mutex>

namespace safety {

/**
 * @brief A 3D point in space
 */
struct Point3D {
    double x;
    double y;
    double z;
};

/**
 * @brief A 3D boundary represented as an axis-aligned bounding box (AABB)
 */
struct BoundaryBox {
    Point3D min;  // Minimum corner point
    Point3D max;  // Maximum corner point
};

/**
 * @class SafetyEnvelope
 * @brief ROS2 node that monitors tool pose and checks against safety boundaries
 * 
 * This class implements a ROS2 node that subscribes to tool pose messages,
 * checks if the pose is within defined safety boundaries, and publishes
 * a stop flag if the boundaries are violated. The implementation is optimized
 * for real-time performance with a target processing time of less than 10ms.
 */
class SafetyEnvelope : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the SafetyEnvelope node
     * @param boundary The safety boundary box
     * @param node_options Additional node options
     */
    explicit SafetyEnvelope(
        const BoundaryBox& boundary,
        const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()
    );

    /**
     * @brief Default destructor
     */
    ~SafetyEnvelope() = default;

    /**
     * @brief Set a new safety boundary
     * @param boundary The new boundary box
     */
    void setBoundary(const BoundaryBox& boundary);

    /**
     * @brief Get the current safety boundary
     * @return The current boundary box
     */
    BoundaryBox getBoundary() const;

    /**
     * @brief Check if a point is within the safety boundary
     * @param point The point to check
     * @return True if the point is within the boundary, false otherwise
     */
    bool isPointWithinBoundary(const Point3D& point) const;

    /**
     * @brief Calculate the distance from a point to the nearest boundary
     * @param point The point to check
     * @return The distance to the nearest boundary (negative if outside)
     */
    double distanceToBoundary(const Point3D& point) const;

    /**
     * @brief Get the last processing time in milliseconds
     * @return The processing time in milliseconds
     */
    double getLastProcessingTimeMs() const;

    /**
     * @brief Get the average processing time in milliseconds
     * @return The average processing time in milliseconds
     */
    double getAverageProcessingTimeMs() const;

    /**
     * @brief Get the maximum processing time observed in milliseconds
     * @return The maximum processing time in milliseconds
     */
    double getMaxProcessingTimeMs() const;

private:
    /**
     * @brief Callback for tool pose messages
     * @param msg The received pose message
     */
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /**
     * @brief Watchdog timer callback
     */
    void watchdogCallback();

    /**
     * @brief Convert a geometry_msgs::msg::Point to a Point3D
     * @param point The geometry_msgs point
     * @return The converted Point3D
     */
    Point3D convertToPoint3D(const geometry_msgs::msg::Point& point) const;

    // ROS2 subscribers and publishers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_flag_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // Safety boundary
    BoundaryBox boundary_;
    mutable std::mutex boundary_mutex_;

    // Performance metrics
    std::chrono::high_resolution_clock::time_point last_pose_time_;
    double last_processing_time_ms_;
    double total_processing_time_ms_;
    double max_processing_time_ms_;
    int64_t pose_count_;
    mutable std::mutex metrics_mutex_;

    // Watchdog state
    bool received_pose_since_last_watchdog_;
    std::mutex watchdog_mutex_;
};

} // namespace safety

#endif // SAFETY_ENVELOPE_HPP
