#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <task_msgs/action/navigation_task.hpp>

#include "jps_global_navigator/jps_global_navigator.hpp"

class JPSGlobalNavigatorIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        // Create the navigator node
        navigator_node_ = std::make_shared<JPSGlobalNavigator>(rclcpp::NodeOptions());
        
        // Create a test client node
        client_node_ = rclcpp::Node::make_shared("test_client");
        
        // Create action client
        action_client_ = rclcpp_action::create_client<task_msgs::action::NavigationTask>(
            client_node_, "navigation_task");
    }

    void TearDown() override {
        navigator_node_.reset();
        client_node_.reset();
        action_client_.reset();
        rclcpp::shutdown();
    }

    sensor_msgs::msg::PointCloud2::SharedPtr createSimpleCostMap() {
        auto cost_map = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cost_map->header.frame_id = "map";
        cost_map->header.stamp = navigator_node_->now();
        cost_map->height = 1;
        cost_map->width = 27; // 3x3x3 grid
        cost_map->is_dense = true;
        
        // Set up fields
        sensor_msgs::PointCloud2Modifier modifier(*cost_map);
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        
        // Add test points - create a 3x3x3 grid with low cost
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cost_map, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cost_map, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cost_map, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(*cost_map, "intensity");
        
        for (int x = 0; x < 3; ++x) {
            for (int y = 0; y < 3; ++y) {
                for (int z = 0; z < 3; ++z) {
                    *iter_x = x * 1.0;
                    *iter_y = y * 1.0;
                    *iter_z = z * 1.0;
                    *iter_intensity = 10.0; // Low cost - traversable
                    ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
                }
            }
        }
        
        return cost_map;
    }

    std::shared_ptr<JPSGlobalNavigator> navigator_node_;
    rclcpp::Node::SharedPtr client_node_;
    rclcpp_action::Client<task_msgs::action::NavigationTask>::SharedPtr action_client_;
};

// Test basic node creation and action client setup
TEST_F(JPSGlobalNavigatorIntegrationTest, NodeCreation) {
    EXPECT_NE(navigator_node_, nullptr);
    EXPECT_NE(action_client_, nullptr);
}

// Test cost map processing
TEST_F(JPSGlobalNavigatorIntegrationTest, CostMapProcessing) {
    // Create and publish a simple cost map
    auto cost_map = createSimpleCostMap();
    
    // Create a publisher to send the cost map
    auto cost_map_pub = navigator_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "cost_map", rclcpp::QoS(1).transient_local());
    
    // Publish the cost map
    cost_map_pub->publish(*cost_map);
    
    // Give some time for the cost map to be processed
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(navigator_node_);
    
    // Test passes if no exceptions are thrown during cost map processing
    SUCCEED();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}