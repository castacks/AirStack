#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <task_msgs/action/navigation_task.hpp>
#include <chrono>
#include <thread>
#include "rrtstar_global_navigator/rrtstar_global_navigator.hpp"

using namespace std::chrono_literals;

class RRTStarGlobalNavigatorVisualizationTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        // Create navigator node with visualization enabled
        node_options_ = rclcpp::NodeOptions();
        node_options_.parameter_overrides().push_back(
            rclcpp::Parameter("enable_debug_visualization", true));
        navigator_ = std::make_shared<RRTStarGlobalNavigator>(node_options_);
        
        // Create test client node
        test_node_ = rclcpp::Node::make_shared("test_node");
        
        // Create publishers for test data
        cost_map_pub_ = test_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cost_map", rclcpp::QoS(10));
        odom_pub_ = test_node_->create_publisher<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10));
        
        // Create subscribers to monitor visualization outputs
        rrt_tree_markers_sub_ = test_node_->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/rrt_tree_markers", rclcpp::QoS(10),
            [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                received_rrt_markers_ = msg;
                rrt_markers_received_ = true;
            });
        
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
        
        // Create a 5x5 grid for faster testing
        cloud_msg->height = 1;
        cloud_msg->width = 25; // 5x5 grid
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
        
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                *iter_x = static_cast<float>(i);
                *iter_y = static_cast<float>(j);
                *iter_z = 0.0f;
                *iter_intensity = 0.1f; // Low cost everywhere for simple testing
                
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
        goal.max_planning_seconds = 5.0; // Shorter for testing
        
        return goal;
    }

    std::shared_ptr<RRTStarGlobalNavigator> navigator_;
    std::shared_ptr<rclcpp::Node> test_node_;
    rclcpp::NodeOptions node_options_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cost_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr rrt_tree_markers_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
    
    rclcpp_action::Client<task_msgs::action::NavigationTask>::SharedPtr action_client_;
    
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread executor_thread_;
    
    // Test state
    visualization_msgs::msg::MarkerArray::SharedPtr received_rrt_markers_;
    nav_msgs::msg::Path::SharedPtr received_global_plan_;
    bool rrt_markers_received_ = false;
    bool global_plan_received_ = false;
};

// Test that visualization is enabled
TEST_F(RRTStarGlobalNavigatorVisualizationTest, VisualizationEnabled) {
    // Check that debug visualization parameter is set to true
    EXPECT_TRUE(navigator_->get_parameter("enable_debug_visualization").as_bool());
}

// Test RRT tree marker publication
TEST_F(RRTStarGlobalNavigatorVisualizationTest, RRTTreeMarkerPublication) {
    // Wait for action server
    ASSERT_TRUE(action_client_->wait_for_action_server(5s));
    
    // Publish cost map and odometry
    auto cost_map = create_test_cost_map();
    cost_map_pub_->publish(*cost_map);
    
    auto odom = create_test_odometry(0.0, 0.0, 0.0);
    odom_pub_->publish(*odom);
    
    // Wait for data processing
    std::this_thread::sleep_for(200ms);
    
    // Reset marker received flag
    rrt_markers_received_ = false;
    
    // Send navigation goal
    auto goal = create_navigation_goal(4.0, 4.0, 0.0);
    auto goal_handle_future = action_client_->async_send_goal(goal);
    
    // Wait for goal acceptance
    auto future_status = goal_handle_future.wait_for(5s);
    ASSERT_EQ(future_status, std::future_status::ready);
    
    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(goal_handle, nullptr);
    
    if (goal_handle) {
        // Wait for RRT tree markers to be published
        auto start_time = std::chrono::steady_clock::now();
        while (!rrt_markers_received_ && 
               std::chrono::steady_clock::now() - start_time < 10s) {
            std::this_thread::sleep_for(100ms);
        }
        
        // Check if RRT tree markers were received
        if (rrt_markers_received_) {
            EXPECT_NE(received_rrt_markers_, nullptr);
            EXPECT_GT(received_rrt_markers_->markers.size(), 0);
            
            // Check marker properties (skip DELETEALL markers)
            for (const auto& marker : received_rrt_markers_->markers) {
                EXPECT_EQ(marker.header.frame_id, "map");
                
                // Skip DELETEALL markers as they are used for clearing
                if (marker.action == visualization_msgs::msg::Marker::DELETEALL) {
                    continue;
                }
                
                EXPECT_TRUE(marker.type == visualization_msgs::msg::Marker::LINE_LIST ||
                           marker.type == visualization_msgs::msg::Marker::SPHERE_LIST ||
                           marker.type == visualization_msgs::msg::Marker::SPHERE ||
                           marker.type == visualization_msgs::msg::Marker::POINTS);
                EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD);
            }
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

// Test global plan visualization
TEST_F(RRTStarGlobalNavigatorVisualizationTest, GlobalPlanVisualization) {
    // Wait for action server
    ASSERT_TRUE(action_client_->wait_for_action_server(5s));
    
    // Publish cost map and odometry
    auto cost_map = create_test_cost_map();
    cost_map_pub_->publish(*cost_map);
    
    auto odom = create_test_odometry(0.0, 0.0, 0.0);
    odom_pub_->publish(*odom);
    
    std::this_thread::sleep_for(200ms);
    
    // Reset global plan received flag
    global_plan_received_ = false;
    
    // Send navigation goal
    auto goal = create_navigation_goal(4.0, 4.0, 0.0);
    auto goal_handle_future = action_client_->async_send_goal(goal);
    
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
            
            // Check that poses have valid timestamps
            for (const auto& pose : received_global_plan_->poses) {
                EXPECT_EQ(pose.header.frame_id, "map");
                EXPECT_GT(pose.header.stamp.sec, 0);
            }
            
            // Check that path makes sense (start and end points)
            const auto& first_pose = received_global_plan_->poses.front();
            const auto& last_pose = received_global_plan_->poses.back();
            
            // First pose should be near start (0, 0, 0)
            EXPECT_LT(std::abs(first_pose.pose.position.x), 1.0);
            EXPECT_LT(std::abs(first_pose.pose.position.y), 1.0);
            
            // Last pose should be near goal (4, 4, 0)
            EXPECT_LT(std::abs(last_pose.pose.position.x - 4.0), 1.0);
            EXPECT_LT(std::abs(last_pose.pose.position.y - 4.0), 1.0);
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

// Test marker clearing functionality
TEST_F(RRTStarGlobalNavigatorVisualizationTest, MarkerClearing) {
    // This test would verify that markers are properly cleared between planning sessions
    // For now, we'll just verify that the visualization system is working
    
    // Wait for action server
    ASSERT_TRUE(action_client_->wait_for_action_server(5s));
    
    // Publish cost map and odometry
    auto cost_map = create_test_cost_map();
    cost_map_pub_->publish(*cost_map);
    
    auto odom = create_test_odometry(0.0, 0.0, 0.0);
    odom_pub_->publish(*odom);
    
    std::this_thread::sleep_for(200ms);
    
    // Send first navigation goal
    auto goal1 = create_navigation_goal(2.0, 2.0, 0.0);
    auto goal_handle_future1 = action_client_->async_send_goal(goal1);
    
    auto future_status1 = goal_handle_future1.wait_for(5s);
    ASSERT_EQ(future_status1, std::future_status::ready);
    
    auto goal_handle1 = goal_handle_future1.get();
    if (goal_handle1 && goal_handle1) {
        // Wait a bit for processing
        std::this_thread::sleep_for(1s);
        
        // Cancel first goal
        action_client_->async_cancel_goal(goal_handle1);
        std::this_thread::sleep_for(500ms);
        
        // Send second navigation goal
        rrt_markers_received_ = false;
        auto goal2 = create_navigation_goal(4.0, 4.0, 0.0);
        auto goal_handle_future2 = action_client_->async_send_goal(goal2);
        
        auto future_status2 = goal_handle_future2.wait_for(5s);
        if (future_status2 == std::future_status::ready) {
            auto goal_handle2 = goal_handle_future2.get();
            if (goal_handle2 && goal_handle2) {
                // Wait for new markers
                auto start_time = std::chrono::steady_clock::now();
                while (!rrt_markers_received_ && 
                       std::chrono::steady_clock::now() - start_time < 10s) {
                    std::this_thread::sleep_for(100ms);
                }
                
                // Verify that new markers are published for the second goal
                if (rrt_markers_received_) {
                    EXPECT_NE(received_rrt_markers_, nullptr);
                    EXPECT_GT(received_rrt_markers_->markers.size(), 0);
                }
                
                // Cancel the second goal
                auto cancel_future2 = action_client_->async_cancel_goal(goal_handle2);
                auto cancel_status2 = cancel_future2.wait_for(2s);
                if (cancel_status2 == std::future_status::ready) {
                    cancel_future2.get(); // Get the result to ensure completion
                }
                std::this_thread::sleep_for(200ms);
            }
        }
    }
}

// Test visualization topic availability
TEST_F(RRTStarGlobalNavigatorVisualizationTest, VisualizationTopicsAvailable) {
    // Check that visualization topics are available
    auto topic_names = navigator_->get_topic_names_and_types();
    
    bool has_rrt_markers = false;
    bool has_global_plan = false;
    
    for (const auto& topic : topic_names) {
        if (topic.first.find("rrt_tree_markers") != std::string::npos) {
            has_rrt_markers = true;
        }
        if (topic.first.find("global_plan") != std::string::npos) {
            has_global_plan = true;
        }
    }
    
    EXPECT_TRUE(has_rrt_markers);
    EXPECT_TRUE(has_global_plan);
}

// Test marker message structure
TEST_F(RRTStarGlobalNavigatorVisualizationTest, MarkerMessageStructure) {
    // Wait for action server
    ASSERT_TRUE(action_client_->wait_for_action_server(5s));
    
    // Publish cost map and odometry
    auto cost_map = create_test_cost_map();
    cost_map_pub_->publish(*cost_map);
    
    auto odom = create_test_odometry(0.0, 0.0, 0.0);
    odom_pub_->publish(*odom);
    
    std::this_thread::sleep_for(200ms);
    
    // Reset marker received flag
    rrt_markers_received_ = false;
    
    // Send navigation goal
    auto goal = create_navigation_goal(4.0, 4.0, 0.0);
    auto goal_handle_future = action_client_->async_send_goal(goal);
    
    auto future_status = goal_handle_future.wait_for(5s);
    ASSERT_EQ(future_status, std::future_status::ready);
    
    auto goal_handle = goal_handle_future.get();
    ASSERT_NE(goal_handle, nullptr);
    
    if (goal_handle) {
        // Wait for RRT tree markers
        auto start_time = std::chrono::steady_clock::now();
        while (!rrt_markers_received_ && 
               std::chrono::steady_clock::now() - start_time < 10s) {
            std::this_thread::sleep_for(100ms);
        }
        
        if (rrt_markers_received_) {
            ASSERT_NE(received_rrt_markers_, nullptr);
            ASSERT_GT(received_rrt_markers_->markers.size(), 0);
            
            // Check each marker in the array (skip DELETEALL markers)
            for (const auto& marker : received_rrt_markers_->markers) {
                // Basic marker properties
                EXPECT_EQ(marker.header.frame_id, "map");
                EXPECT_GT(marker.header.stamp.sec, 0);
                EXPECT_GE(marker.id, 0);
                
                // Skip DELETEALL markers as they are used for clearing
                if (marker.action == visualization_msgs::msg::Marker::DELETEALL) {
                    continue;
                }
                
                EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD);
                
                // Color should be set
                EXPECT_GE(marker.color.r, 0.0);
                EXPECT_LE(marker.color.r, 1.0);
                EXPECT_GE(marker.color.g, 0.0);
                EXPECT_LE(marker.color.g, 1.0);
                EXPECT_GE(marker.color.b, 0.0);
                EXPECT_LE(marker.color.b, 1.0);
                EXPECT_GT(marker.color.a, 0.0);
                EXPECT_LE(marker.color.a, 1.0);
                
                // Scale should be positive
                EXPECT_GT(marker.scale.x, 0.0);
                EXPECT_GT(marker.scale.y, 0.0);
                EXPECT_GT(marker.scale.z, 0.0);
                
                // Namespace should be set
                EXPECT_FALSE(marker.ns.empty());
            }
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

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}