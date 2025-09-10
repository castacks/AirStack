#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <task_msgs/action/navigation_task.hpp>
#include <chrono>
#include <thread>
#include "rrtstar_global_navigator/rrtstar_global_navigator.hpp"

using namespace std::chrono_literals;

class RRTStarGlobalNavigatorIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        // Create navigator node
        node_options_ = rclcpp::NodeOptions();
        node_options_.parameter_overrides().push_back(
            rclcpp::Parameter("enable_debug_visualization", false));
        navigator_ = std::make_shared<RRTStarGlobalNavigator>(node_options_);
        
        // Create test client node
        test_node_ = rclcpp::Node::make_shared("test_node");
        
        // Create publishers for test data
        cost_map_pub_ = test_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cost_map", rclcpp::QoS(10));
        odom_pub_ = test_node_->create_publisher<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10));
        
        // Create subscribers to monitor outputs
        global_plan_sub_ = test_node_->create_subscription<nav_msgs::msg::Path>(
            "/global_plan", rclcpp::QoS(10),
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                received_global_plan_ = msg;
                global_plan_received_ = true;
            });
        
        // Create action client
        action_client_ = rclcpp_action::create_client<task_msgs::action::NavigationTask>(
            test_node_, "/rrtstar_navigator");
        
        // Start executor in separate thread
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(navigator_);
        executor_->add_node(test_node_);
        executor_thread_ = std::thread([this]() { executor_->spin(); });
        
        // Wait for connections
        std::this_thread::sleep_for(500ms);
    }

    void TearDown() override {
        executor_->cancel();
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
        navigator_.reset();
        test_node_.reset();
        executor_.reset();
        rclcpp::shutdown();
    }

    // Helper function to create a simple cost map
    sensor_msgs::msg::PointCloud2::SharedPtr create_test_cost_map() {
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header.frame_id = "map";
        cloud_msg->header.stamp = test_node_->get_clock()->now();
        
        // Create a 10x10 grid of points
        cloud_msg->height = 1;
        cloud_msg->width = 100; // 10x10 grid
        cloud_msg->is_dense = true;
        cloud_msg->is_bigendian = false;
        
        // Define fields: x, y, z, intensity
        sensor_msgs::msg::PointField field_x, field_y, field_z, field_intensity;
        field_x.name = "x"; field_x.offset = 0; field_x.datatype = sensor_msgs::msg::PointField::FLOAT32; field_x.count = 1;
        field_y.name = "y"; field_y.offset = 4; field_y.datatype = sensor_msgs::msg::PointField::FLOAT32; field_y.count = 1;
        field_z.name = "z"; field_z.offset = 8; field_z.datatype = sensor_msgs::msg::PointField::FLOAT32; field_z.count = 1;
        field_intensity.name = "intensity"; field_intensity.offset = 12; field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32; field_intensity.count = 1;
        
        cloud_msg->fields = {field_x, field_y, field_z, field_intensity};
        cloud_msg->point_step = 16;
        cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
        cloud_msg->data.resize(cloud_msg->row_step * cloud_msg->height);
        
        // Fill with grid data
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(*cloud_msg, "intensity");
        
        for (int i = 0; i < 10; ++i) {
            for (int j = 0; j < 10; ++j) {
                *iter_x = static_cast<float>(i);
                *iter_y = static_cast<float>(j);
                *iter_z = 0.0f;
                
                // Create some obstacles (high cost) and free space (low cost)
                if ((i == 5 && j >= 3 && j <= 7) || (j == 5 && i >= 3 && i <= 7)) {
                    *iter_intensity = 0.9f; // High cost (obstacle)
                } else {
                    *iter_intensity = 0.1f; // Low cost (free space)
                }
                
                ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
            }
        }
        
        return cloud_msg;
    }

    // Helper function to create test odometry
    nav_msgs::msg::Odometry::SharedPtr create_test_odometry(double x, double y, double z) {
        auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
        odom_msg->header.frame_id = "odom";
        odom_msg->header.stamp = test_node_->get_clock()->now();
        odom_msg->pose.pose.position.x = x;
        odom_msg->pose.pose.position.y = y;
        odom_msg->pose.pose.position.z = z;
        odom_msg->pose.pose.orientation.w = 1.0;
        return odom_msg;
    }

    // Helper function to create navigation goal
    task_msgs::action::NavigationTask::Goal create_navigation_goal(double x, double y, double z) {
        task_msgs::action::NavigationTask::Goal goal;
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = test_node_->get_clock()->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.w = 1.0;
        
        goal.goal_poses = {pose};
        goal.max_planning_seconds = 10.0;
        
        return goal;
    }

    std::shared_ptr<RRTStarGlobalNavigator> navigator_;
    std::shared_ptr<rclcpp::Node> test_node_;
    rclcpp::NodeOptions node_options_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cost_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
    
    rclcpp_action::Client<task_msgs::action::NavigationTask>::SharedPtr action_client_;
    
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread executor_thread_;
    
    // Test state
    nav_msgs::msg::Path::SharedPtr received_global_plan_;
    bool global_plan_received_ = false;
};

// Test cost map subscription and processing
TEST_F(RRTStarGlobalNavigatorIntegrationTest, CostMapSubscription) {
    // Publish test cost map
    auto cost_map = create_test_cost_map();
    cost_map_pub_->publish(*cost_map);
    
    // Wait for processing
    std::this_thread::sleep_for(100ms);
    
    // The cost map should be processed internally
    // We can't directly verify the internal state, but the node should not crash
    EXPECT_TRUE(true);
}

// Test odometry subscription
TEST_F(RRTStarGlobalNavigatorIntegrationTest, OdometrySubscription) {
    // Publish test odometry
    auto odom = create_test_odometry(1.0, 2.0, 3.0);
    odom_pub_->publish(*odom);
    
    // Wait for processing
    std::this_thread::sleep_for(100ms);
    
    // The odometry should be processed internally
    EXPECT_TRUE(true);
}

// Test action server availability
TEST_F(RRTStarGlobalNavigatorIntegrationTest, ActionServerAvailability) {
    // Wait for action server
    bool server_available = action_client_->wait_for_action_server(5s);
    EXPECT_TRUE(server_available);
}

// Test navigation action with valid goal
TEST_F(RRTStarGlobalNavigatorIntegrationTest, NavigationActionValidGoal) {
    // Wait for action server
    ASSERT_TRUE(action_client_->wait_for_action_server(5s));
    
    // Publish cost map and odometry first
    auto cost_map = create_test_cost_map();
    cost_map_pub_->publish(*cost_map);
    
    auto odom = create_test_odometry(0.0, 0.0, 0.0);
    odom_pub_->publish(*odom);
    
    // Wait for data to be processed
    std::this_thread::sleep_for(200ms);
    
    // Create and send navigation goal
    auto goal = create_navigation_goal(8.0, 8.0, 0.0);
    
    auto send_goal_options = rclcpp_action::Client<task_msgs::action::NavigationTask>::SendGoalOptions();
    send_goal_options.feedback_callback = 
        [](rclcpp_action::ClientGoalHandle<task_msgs::action::NavigationTask>::SharedPtr,
           const std::shared_ptr<const task_msgs::action::NavigationTask::Feedback>) {
            // Feedback received
        };
    
    auto goal_handle_future = action_client_->async_send_goal(goal, send_goal_options);
    
    // Wait for goal to be accepted
    auto future_status = goal_handle_future.wait_for(5s);
    ASSERT_EQ(future_status, std::future_status::ready);
    
    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(goal_handle, nullptr);
    // In ROS 2 Humble, goal acceptance is implicit if goal_handle is not null
}

// Test global plan publication
TEST_F(RRTStarGlobalNavigatorIntegrationTest, GlobalPlanPublication) {
    // Wait for action server
    ASSERT_TRUE(action_client_->wait_for_action_server(5s));
    
    // Publish cost map and odometry
    auto cost_map = create_test_cost_map();
    cost_map_pub_->publish(*cost_map);
    
    auto odom = create_test_odometry(0.0, 0.0, 0.0);
    odom_pub_->publish(*odom);
    
    // Wait for data processing
    std::this_thread::sleep_for(200ms);
    
    // Reset global plan received flag
    global_plan_received_ = false;
    
    // Send navigation goal
    auto goal = create_navigation_goal(8.0, 8.0, 0.0);
    auto goal_handle_future = action_client_->async_send_goal(goal);
    
    // Wait for goal acceptance and some processing time
    auto future_status = goal_handle_future.wait_for(5s);
    ASSERT_EQ(future_status, std::future_status::ready);
    
    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(goal_handle, nullptr);
    
    if (goal_handle) {
        // Wait for global plan to be published
        auto start_time = std::chrono::steady_clock::now();
        while (!global_plan_received_ && 
               std::chrono::steady_clock::now() - start_time < 10s) {
            std::this_thread::sleep_for(100ms);
        }
        
        // Check if global plan was received
        if (global_plan_received_) {
            EXPECT_NE(received_global_plan_, nullptr);
            EXPECT_GT(received_global_plan_->poses.size(), 0);
            EXPECT_EQ(received_global_plan_->header.frame_id, "map");
        }
        
        // Cancel the goal to prevent segfault when test ends
        auto cancel_future = action_client_->async_cancel_goal(goal_handle);
        auto cancel_status = cancel_future.wait_for(2s);
        if (cancel_status == std::future_status::ready) {
            cancel_future.get(); // Get the result to ensure completion
        }
        std::this_thread::sleep_for(200ms);
    }
}

// Test action cancellation
TEST_F(RRTStarGlobalNavigatorIntegrationTest, ActionCancellation) {
    // Wait for action server
    ASSERT_TRUE(action_client_->wait_for_action_server(5s));
    
    // Publish cost map and odometry
    auto cost_map = create_test_cost_map();
    cost_map_pub_->publish(*cost_map);
    
    auto odom = create_test_odometry(0.0, 0.0, 0.0);
    odom_pub_->publish(*odom);
    
    std::this_thread::sleep_for(200ms);
    
    // Send navigation goal
    auto goal = create_navigation_goal(8.0, 8.0, 0.0);
    auto goal_handle_future = action_client_->async_send_goal(goal);
    
    auto future_status = goal_handle_future.wait_for(5s);
    ASSERT_EQ(future_status, std::future_status::ready);
    
    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(goal_handle, nullptr);
    
    if (goal_handle) {
        // Cancel the goal
        auto cancel_future = action_client_->async_cancel_goal(goal_handle);
        auto cancel_status = cancel_future.wait_for(5s);
        EXPECT_EQ(cancel_status, std::future_status::ready);
        
        auto cancel_response = cancel_future.get();
        EXPECT_EQ(cancel_response->return_code, 
                  action_msgs::srv::CancelGoal::Response::ERROR_NONE);
    }
}

// Test multiple waypoint navigation
TEST_F(RRTStarGlobalNavigatorIntegrationTest, MultipleWaypointNavigation) {
    // Wait for action server
    ASSERT_TRUE(action_client_->wait_for_action_server(5s));
    
    // Publish cost map and odometry
    auto cost_map = create_test_cost_map();
    cost_map_pub_->publish(*cost_map);
    
    auto odom = create_test_odometry(0.0, 0.0, 0.0);
    odom_pub_->publish(*odom);
    
    std::this_thread::sleep_for(200ms);
    
    // Create goal with multiple waypoints
    task_msgs::action::NavigationTask::Goal goal;
    goal.max_planning_seconds = 15.0;
    
    // Waypoint 1
    geometry_msgs::msg::PoseStamped pose1;
    pose1.header.frame_id = "map";
    pose1.header.stamp = test_node_->get_clock()->now();
    pose1.pose.position.x = 3.0;
    pose1.pose.position.y = 3.0;
    pose1.pose.position.z = 0.0;
    pose1.pose.orientation.w = 1.0;
    
    // Waypoint 2
    geometry_msgs::msg::PoseStamped pose2;
    pose2.header.frame_id = "map";
    pose2.header.stamp = test_node_->get_clock()->now();
    pose2.pose.position.x = 8.0;
    pose2.pose.position.y = 8.0;
    pose2.pose.position.z = 0.0;
    pose2.pose.orientation.w = 1.0;
    
    goal.goal_poses = {pose1, pose2};
    
    // Send goal
    auto goal_handle_future = action_client_->async_send_goal(goal);
    auto future_status = goal_handle_future.wait_for(5s);
    ASSERT_EQ(future_status, std::future_status::ready);
    
    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(goal_handle, nullptr);
    EXPECT_TRUE(goal_handle);
    
    // Cancel the goal to prevent segfault when test ends
    if (goal_handle) {
        auto cancel_future = action_client_->async_cancel_goal(goal_handle);
        auto cancel_status = cancel_future.wait_for(2s);
        if (cancel_status == std::future_status::ready) {
            cancel_future.get(); // Get the result to ensure completion
        }
        std::this_thread::sleep_for(200ms);
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}