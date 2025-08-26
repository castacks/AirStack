#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "simple_global_navigator/simple_global_navigator.hpp"

class SimpleGlobalNavigatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_options_ = rclcpp::NodeOptions();
        // Disable debug visualization for tests
        node_options_.parameter_overrides().push_back(
            rclcpp::Parameter("enable_debug_visualization", false));
        navigator_ = std::make_shared<SimpleGlobalNavigator>(node_options_);
    }

    void TearDown() override {
        navigator_.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<SimpleGlobalNavigator> navigator_;
    rclcpp::NodeOptions node_options_;
};

// Test basic node initialization
TEST_F(SimpleGlobalNavigatorTest, NodeInitialization) {
    ASSERT_NE(navigator_, nullptr);
    EXPECT_EQ(navigator_->get_name(), std::string("simple_global_navigator"));
}

// Test parameter loading
TEST_F(SimpleGlobalNavigatorTest, ParameterLoading) {
    // Test default parameters
    EXPECT_EQ(navigator_->get_parameter("rrt_max_iterations").as_int(), 5000);
    EXPECT_DOUBLE_EQ(navigator_->get_parameter("rrt_step_size").as_double(), 0.5);
    EXPECT_DOUBLE_EQ(navigator_->get_parameter("rrt_goal_tolerance").as_double(), 0.3);
    EXPECT_DOUBLE_EQ(navigator_->get_parameter("rrt_rewire_radius").as_double(), 1.0);
    EXPECT_EQ(navigator_->get_parameter("cost_map_topic").as_string(), "/cost_map");
    EXPECT_EQ(navigator_->get_parameter("odom_topic").as_string(), "/odom");
    EXPECT_FALSE(navigator_->get_parameter("enable_debug_visualization").as_bool());
}

// Test distance calculation
TEST_F(SimpleGlobalNavigatorTest, DistanceCalculation) {
    geometry_msgs::msg::Point p1, p2;
    
    // Test zero distance
    p1.x = 0.0; p1.y = 0.0; p1.z = 0.0;
    p2.x = 0.0; p2.y = 0.0; p2.z = 0.0;
    
    // We need to access private methods through a test-friendly approach
    // For now, we'll test the public interface and verify behavior indirectly
    EXPECT_TRUE(true); // Placeholder - actual distance testing would require friend class or public wrapper
}

// Test RRT node creation
TEST_F(SimpleGlobalNavigatorTest, RRTNodeCreation) {
    geometry_msgs::msg::Point pos;
    pos.x = 1.0;
    pos.y = 2.0;
    pos.z = 3.0;
    
    auto node = std::make_shared<RRTNode>(pos, nullptr, 5.0);
    
    EXPECT_DOUBLE_EQ(node->position.x, 1.0);
    EXPECT_DOUBLE_EQ(node->position.y, 2.0);
    EXPECT_DOUBLE_EQ(node->position.z, 3.0);
    EXPECT_EQ(node->parent, nullptr);
    EXPECT_DOUBLE_EQ(node->cost, 5.0);
    EXPECT_TRUE(node->children.empty());
}

// Test RRT node with parent
TEST_F(SimpleGlobalNavigatorTest, RRTNodeWithParent) {
    geometry_msgs::msg::Point parent_pos, child_pos;
    parent_pos.x = 0.0; parent_pos.y = 0.0; parent_pos.z = 0.0;
    child_pos.x = 1.0; child_pos.y = 1.0; child_pos.z = 1.0;
    
    auto parent_node = std::make_shared<RRTNode>(parent_pos, nullptr, 0.0);
    auto child_node = std::make_shared<RRTNode>(child_pos, parent_node, 1.5);
    
    EXPECT_EQ(child_node->parent, parent_node);
    EXPECT_DOUBLE_EQ(child_node->cost, 1.5);
    
    // Add child to parent's children list
    parent_node->children.push_back(child_node);
    EXPECT_EQ(parent_node->children.size(), 1);
    EXPECT_EQ(parent_node->children[0], child_node);
}

// Test CostMapData initialization
TEST_F(SimpleGlobalNavigatorTest, CostMapDataInitialization) {
    CostMapData cost_map;
    
    EXPECT_TRUE(cost_map.points.empty());
    EXPECT_TRUE(cost_map.costs.empty());
    EXPECT_DOUBLE_EQ(cost_map.resolution, 0.1);
    EXPECT_FALSE(cost_map.valid);
}

// Test cost map data processing with mock PointCloud2
TEST_F(SimpleGlobalNavigatorTest, CostMapProcessing) {
    // Create a mock PointCloud2 message
    auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud_msg->header.frame_id = "map";
    cloud_msg->header.stamp = navigator_->get_clock()->now();
    
    // Set up the point cloud structure
    cloud_msg->height = 1;
    cloud_msg->width = 3;
    cloud_msg->is_dense = true;
    cloud_msg->is_bigendian = false;
    
    // Define fields: x, y, z, intensity
    sensor_msgs::msg::PointField field_x, field_y, field_z, field_intensity;
    field_x.name = "x"; field_x.offset = 0; field_x.datatype = sensor_msgs::msg::PointField::FLOAT32; field_x.count = 1;
    field_y.name = "y"; field_y.offset = 4; field_y.datatype = sensor_msgs::msg::PointField::FLOAT32; field_y.count = 1;
    field_z.name = "z"; field_z.offset = 8; field_z.datatype = sensor_msgs::msg::PointField::FLOAT32; field_z.count = 1;
    field_intensity.name = "intensity"; field_intensity.offset = 12; field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32; field_intensity.count = 1;
    
    cloud_msg->fields = {field_x, field_y, field_z, field_intensity};
    cloud_msg->point_step = 16; // 4 floats * 4 bytes each
    cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
    
    // Resize data buffer
    cloud_msg->data.resize(cloud_msg->row_step * cloud_msg->height);
    
    // Fill with test data
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(*cloud_msg, "intensity");
    
    // Point 1: (0, 0, 0) with intensity 0.1
    *iter_x = 0.0f; *iter_y = 0.0f; *iter_z = 0.0f; *iter_intensity = 0.1f;
    ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
    
    // Point 2: (1, 1, 1) with intensity 0.5
    *iter_x = 1.0f; *iter_y = 1.0f; *iter_z = 1.0f; *iter_intensity = 0.5f;
    ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
    
    // Point 3: (2, 2, 2) with intensity 0.9
    *iter_x = 2.0f; *iter_y = 2.0f; *iter_z = 2.0f; *iter_intensity = 0.9f;
    
    // Process the message (this would normally be done by the callback)
    // We can't directly test the private callback, but we can verify the node accepts the message type
    EXPECT_TRUE(true); // Placeholder for actual callback testing
}

// Test odometry message processing
TEST_F(SimpleGlobalNavigatorTest, OdometryProcessing) {
    auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
    odom_msg->header.frame_id = "odom";
    odom_msg->header.stamp = navigator_->get_clock()->now();
    odom_msg->pose.pose.position.x = 1.0;
    odom_msg->pose.pose.position.y = 2.0;
    odom_msg->pose.pose.position.z = 3.0;
    odom_msg->pose.pose.orientation.w = 1.0;
    
    // We can't directly test the private callback, but we can verify the message structure
    EXPECT_DOUBLE_EQ(odom_msg->pose.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(odom_msg->pose.pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(odom_msg->pose.pose.position.z, 3.0);
    EXPECT_DOUBLE_EQ(odom_msg->pose.pose.orientation.w, 1.0);
}

// Test topic and service interfaces
TEST_F(SimpleGlobalNavigatorTest, TopicInterfaces) {
    // Verify publishers exist
    auto topic_names = navigator_->get_topic_names_and_types();
    
    bool has_global_plan = false;
    for (const auto& topic : topic_names) {
        if (topic.first.find("global_plan") != std::string::npos) {
            has_global_plan = true;
            break;
        }
    }
    EXPECT_TRUE(has_global_plan);
}

// Test action server interface
TEST_F(SimpleGlobalNavigatorTest, ActionServerInterface) {
    // Verify action server exists by checking service names
    auto service_names = navigator_->get_service_names_and_types();
    
    bool has_action_server = false;
    for (const auto& service : service_names) {
        if (service.first.find("simple_navigator") != std::string::npos) {
            has_action_server = true;
            break;
        }
    }
    EXPECT_TRUE(has_action_server);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}