/**
 * @file test_safety_envelope.cpp
 * @brief Unit tests for the safety envelope functionality
 * 
 * This file contains unit tests for the safety envelope functionality,
 * including boundary checking and distance calculations.
 */

#include <gtest/gtest.h>
#include "../include/safety_envelope/safety_envelope.hpp"

// Test fixture for safety envelope tests
class SafetyEnvelopeTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a default boundary for testing
        boundary_ = {
            {-1.0, -1.0, -1.0},  // min corner
            {1.0, 1.0, 1.0}      // max corner
        };
    }

    safety::BoundaryBox boundary_;
};

// Test point inclusion within boundary
TEST_F(SafetyEnvelopeTest, PointInclusion) {
    // Create a node for testing
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<safety::SafetyEnvelope>(boundary_);
    
    // Test points inside the boundary
    EXPECT_TRUE(node->isPointWithinBoundary({0.0, 0.0, 0.0}));
    EXPECT_TRUE(node->isPointWithinBoundary({0.5, 0.5, 0.5}));
    EXPECT_TRUE(node->isPointWithinBoundary({-0.5, -0.5, -0.5}));
    
    // Test points on the boundary
    EXPECT_TRUE(node->isPointWithinBoundary({1.0, 0.0, 0.0}));
    EXPECT_TRUE(node->isPointWithinBoundary({0.0, 1.0, 0.0}));
    EXPECT_TRUE(node->isPointWithinBoundary({0.0, 0.0, 1.0}));
    EXPECT_TRUE(node->isPointWithinBoundary({-1.0, 0.0, 0.0}));
    
    // Test points outside the boundary
    EXPECT_FALSE(node->isPointWithinBoundary({1.1, 0.0, 0.0}));
    EXPECT_FALSE(node->isPointWithinBoundary({0.0, -1.1, 0.0}));
    EXPECT_FALSE(node->isPointWithinBoundary({0.0, 0.0, 2.0}));
    
    rclcpp::shutdown();
}

// Test distance calculation to boundary
TEST_F(SafetyEnvelopeTest, DistanceToBoundary) {
    // Create a node for testing
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<safety::SafetyEnvelope>(boundary_);
    
    // Test points inside the boundary
    EXPECT_NEAR(node->distanceToBoundary({0.0, 0.0, 0.0}), 1.0, 0.001);
    EXPECT_NEAR(node->distanceToBoundary({0.5, 0.0, 0.0}), 0.5, 0.001);
    
    // Test points on the boundary
    EXPECT_NEAR(node->distanceToBoundary({1.0, 0.0, 0.0}), 0.0, 0.001);
    EXPECT_NEAR(node->distanceToBoundary({0.0, -1.0, 0.0}), 0.0, 0.001);
    
    // Test points outside the boundary
    EXPECT_NEAR(node->distanceToBoundary({2.0, 0.0, 0.0}), -1.0, 0.001);
    EXPECT_NEAR(node->distanceToBoundary({0.0, 0.0, -1.5}), -0.5, 0.001);
    
    rclcpp::shutdown();
}

// Test boundary update
TEST_F(SafetyEnvelopeTest, BoundaryUpdate) {
    // Create a node for testing
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<safety::SafetyEnvelope>(boundary_);
    
    // Test initial boundary
    EXPECT_TRUE(node->isPointWithinBoundary({0.5, 0.5, 0.5}));
    EXPECT_FALSE(node->isPointWithinBoundary({1.5, 0.5, 0.5}));
    
    // Update boundary
    safety::BoundaryBox new_boundary = {
        {-2.0, -2.0, -2.0},  // min corner
        {2.0, 2.0, 2.0}      // max corner
    };
    node->setBoundary(new_boundary);
    
    // Test updated boundary
    EXPECT_TRUE(node->isPointWithinBoundary({0.5, 0.5, 0.5}));
    EXPECT_TRUE(node->isPointWithinBoundary({1.5, 0.5, 0.5}));
    EXPECT_FALSE(node->isPointWithinBoundary({2.5, 0.5, 0.5}));
    
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
