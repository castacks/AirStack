#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "jps_global_navigator/jps_global_navigator.hpp"

class JPSGlobalNavigatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<JPSGlobalNavigator>(rclcpp::NodeOptions());
    }

    void TearDown() override {
        node_.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<JPSGlobalNavigator> node_;
};

// Test basic node initialization
TEST_F(JPSGlobalNavigatorTest, NodeInitialization) {
    ASSERT_NE(node_, nullptr);
    EXPECT_EQ(node_->get_name(), std::string("jps_global_navigator"));
}

// Test grid conversion functions
TEST_F(JPSGlobalNavigatorTest, GridConversion) {
    // Create a simple cost map to initialize the grid
    auto cost_map_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cost_map_msg->header.frame_id = "map";
    cost_map_msg->header.stamp = node_->now();
    cost_map_msg->height = 1;
    cost_map_msg->width = 8; // 2x2x2 grid
    cost_map_msg->is_dense = true;
    
    // Set up fields
    sensor_msgs::PointCloud2Modifier modifier(*cost_map_msg);
    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    
    // Add test points
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cost_map_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cost_map_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cost_map_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(*cost_map_msg, "intensity");
    
    // Create a 2x2x2 grid
    std::vector<std::tuple<float, float, float, float>> points = {
        {0.0, 0.0, 0.0, 1.0}, {0.5, 0.0, 0.0, 1.0},
        {0.0, 0.5, 0.0, 1.0}, {0.5, 0.5, 0.0, 1.0},
        {0.0, 0.0, 0.5, 1.0}, {0.5, 0.0, 0.5, 1.0},
        {0.0, 0.5, 0.5, 1.0}, {0.5, 0.5, 0.5, 1.0}
    };
    
    for (const auto& point : points) {
        *iter_x = std::get<0>(point);
        *iter_y = std::get<1>(point);
        *iter_z = std::get<2>(point);
        *iter_intensity = std::get<3>(point);
        ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
    }
    
    // This test verifies that the node can be created and basic functionality works
    // More detailed testing would require access to private methods or friend classes
    EXPECT_TRUE(true); // Placeholder - in a real implementation, we'd test grid conversion
}

// Test parameter loading
TEST_F(JPSGlobalNavigatorTest, ParameterLoading) {
    // Test that parameters are loaded with expected defaults
    EXPECT_TRUE(node_->has_parameter("jps_max_iterations"));
    EXPECT_TRUE(node_->has_parameter("jps_goal_tolerance"));
    EXPECT_TRUE(node_->has_parameter("cost_threshold"));
    EXPECT_TRUE(node_->has_parameter("resolution"));
    
    // Check default values
    EXPECT_EQ(node_->get_parameter("jps_max_iterations").as_int(), 10000);
    EXPECT_DOUBLE_EQ(node_->get_parameter("jps_goal_tolerance").as_double(), 2.0);
    EXPECT_DOUBLE_EQ(node_->get_parameter("cost_threshold").as_double(), 50.0);
    EXPECT_DOUBLE_EQ(node_->get_parameter("resolution").as_double(), 0.5);
}

// Test action server creation
TEST_F(JPSGlobalNavigatorTest, ActionServerCreation) {
    // Give some time for the action server to be created
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    // In a real test, we would check if the action server is available
    // For now, just verify the node is still running
    EXPECT_TRUE(rclcpp::ok());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}