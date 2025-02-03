#include <behavior_tree/behavior_tree.hpp>  // Include the behavior tree library containing the Action and Condition classes.
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

class BehaviorTreeExample : public rclcpp::Node {
   private:
    bool got_robot_odom, got_home, got_destination, got_no_fly_zone, got_no_fly_zone_radius;
    nav_msgs::msg::Odometry robot_odom;
    float flight_z, ground_z;
    geometry_msgs::msg::PointStamped home, destination;
    float acceptance_radius;
    geometry_msgs::msg::PointStamped no_fly_zone;
    float no_fly_zone_radius;

    // Condition variables
    bt::Condition* at_flight_altitude_condition;
    bt::Condition* on_ground_condition;
    bt::Condition* visited_destination_condition;
    bt::Condition* at_home_condition;
    bt::Condition* in_fly_zone_condition;
    bt::Condition* in_no_fly_zone_condition;

    // Action variables
    bt::Action* land_action;
    bt::Action* takeoff_action;
    bt::Action* go_to_destination_action;
    bt::Action* go_home_action;

    // subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr home_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr destination_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr no_fly_zone_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr no_fly_zone_radius_sub;

    // publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_pub;

    // timers
    rclcpp::TimerBase::SharedPtr timer;

   public:
    BehaviorTreeExample() : Node("behavior_tree_example") {
        on_ground_condition = new bt::Condition("On Ground", this);
        at_flight_altitude_condition = new bt::Condition("At Flight Altitude", this);
        visited_destination_condition = new bt::Condition("Visited Destination", this);
        at_home_condition = new bt::Condition("At Home", this);
        in_fly_zone_condition = new bt::Condition("In Fly Zone", this);
        in_no_fly_zone_condition = new bt::Condition("In No Fly Zone", this);

        land_action = new bt::Action("Land", this);
        // Do the same for the "Takeoff" action.
        takeoff_action =
            new bt::Action("Takeoff", this, &BehaviorTreeExample::takeoff_active_callback, this,
                           &BehaviorTreeExample::takeoff_inactive_callback, this);
        go_to_destination_action = new bt::Action("Go To Destination", this);
        go_home_action = new bt::Action("Go Home", this);

        // init subscribers
        robot_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry", 10,
            std::bind(&BehaviorTreeExample::robot_odom_callback, this, std::placeholders::_1));
        home_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "home", 10,
            std::bind(&BehaviorTreeExample::home_callback, this, std::placeholders::_1));
        destination_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "destination", 10,
            std::bind(&BehaviorTreeExample::destination_callback, this, std::placeholders::_1));

        no_fly_zone_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "no_fly_zone", 10,
            std::bind(&BehaviorTreeExample::no_fly_zone_callback, this, std::placeholders::_1));
        no_fly_zone_radius_sub = this->create_subscription<std_msgs::msg::Float32>(
            "no_fly_zone_radius", 10,
            std::bind(&BehaviorTreeExample::no_fly_zone_radius_callback, this,
                      std::placeholders::_1));

        // init publishers
        vel_cmd_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/vel_cmd", 10);

        // initialization
        got_robot_odom = false;
        got_home = false;
        got_destination = false;
        got_no_fly_zone = false;
        got_no_fly_zone_radius = false;
        flight_z = 0.99;
        ground_z = 0.01;
        acceptance_radius = 0.01;

        // init timer
        timer = this->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&BehaviorTreeExample::timer_callback, this));
    }

    void timer_callback() {
        if (!got_robot_odom || !got_home || !got_destination) return;

        // The condition should be success if the robot is above the flying height's z level,
        // and failure otherwise.
        at_flight_altitude_condition->set(robot_odom.pose.pose.position.z > flight_z);
        at_flight_altitude_condition->publish();

        // Set the condition to success if the robot is below the ground's z level,
        // or to failure if the robot is above the ground's z level.
        on_ground_condition->set(robot_odom.pose.pose.position.z < ground_z);
        on_ground_condition->publish();

        // The at home condition should be set to success if the robot is within
        // a threshold distance of the home location and failure otherwise.
        at_home_condition->set(distance(robot_odom, home) < acceptance_radius);
        at_home_condition->publish();

        if (got_no_fly_zone && got_no_fly_zone_radius) {
            float distance = sqrt(pow(robot_odom.pose.pose.position.x - no_fly_zone.point.x, 2) +
                                  pow(robot_odom.pose.pose.position.y - no_fly_zone.point.y, 2));

            // Check if the robot is with the radius of the no fly zone point.
            bool in_no_fly_zone = distance <= no_fly_zone_radius;

            in_no_fly_zone_condition->set(in_no_fly_zone);  // Set the condition.
            in_no_fly_zone_condition->publish();            // Publish.

            in_fly_zone_condition->set(!in_no_fly_zone);  // Set the condition.
            in_fly_zone_condition->publish();             // Publish.
        }

        // The visited destination condition should be set to success (true) when we first
        // get close enough to the destination point and remain true forever.
        float destination_distance = distance(robot_odom, destination);
        if (!visited_destination_condition
                 ->get())  // Only changed the condition value if it hasn't been set to true yet.
            visited_destination_condition->set(destination_distance < acceptance_radius);
        visited_destination_condition->publish();  // Publish the condition.

        if (land_action
                ->is_active()) {  // Make sure the robot only lands if the land action is active.
            geometry_msgs::msg::TwistStamped vel_cmd;
            vel_cmd.header.stamp = this->get_clock()->now();
            vel_cmd.header.frame_id = "world";

            if (on_ground_condition->get()) {
                // Set the status to success if the robot has landed.
                land_action->set_success();
                // Command zero velocity when the robot is done landing.
                vel_cmd.twist.linear.x = vel_cmd.twist.linear.y = vel_cmd.twist.linear.z = 0;
            } else {
                // Set the status to running while the robot is landing.
                land_action->set_running();
                // Command downward velocity to land.
                vel_cmd.twist.linear.z = -0.3;
            }

            vel_cmd_pub->publish(vel_cmd);

            // Publish the land action's status.
            land_action->publish();
        }

        if (go_home_action->is_active()) {  // Make sure the robot tries to go home when the go home
                                            // action is active.
            geometry_msgs::msg::TwistStamped vel_cmd;
            vel_cmd.header.stamp = this->get_clock()->now();
            vel_cmd.header.frame_id = "world";

            if (at_home_condition->get()) {
                // Set the status to "SUCCESS" if the robot has reached the home location
                go_home_action->set_success();
                // Command zero velocity when the robot is done traveling home.
                vel_cmd.twist.linear.x = vel_cmd.twist.linear.y = vel_cmd.twist.linear.z = 0;
            } else {
                // Set the status to "RUNNING" while the robot is moving toward the home location.
                go_home_action->set_running();
                // Command a velocity towards the home location.
                tf2::Vector3 vel(home.point.x - robot_odom.pose.pose.position.x,
                                 home.point.y - robot_odom.pose.pose.position.y, 0);
                vel = 0.3 * vel.normalized();
                vel_cmd.twist.linear.x = vel.x();
                vel_cmd.twist.linear.y = vel.y();
                vel_cmd.twist.linear.z = vel.z();
            }

            vel_cmd_pub->publish(vel_cmd);

            // Publish the go home action's status
            go_home_action->publish();
        }

        // Make sure the robot only tries to go to the destination while the go to destination
        // action is active
        if (go_to_destination_action->is_active()) {
            geometry_msgs::msg::TwistStamped vel_cmd;
            vel_cmd.header.stamp = this->get_clock()->now();
            vel_cmd.header.frame_id = "world";

            if (destination_distance < acceptance_radius) {
                // Set the status to success if the robot has reached the destination location.
                go_to_destination_action->set_success();
                // Command zero velocity when the robot is done traveling to the destination.
                vel_cmd.twist.linear.x = vel_cmd.twist.linear.y = vel_cmd.twist.linear.z = 0;
            } else {
                // Set the status to running while the robot is moving towards the destination
                // location.
                go_to_destination_action->set_running();
                // Command a velocity towards the destination location.
                tf2::Vector3 vel(destination.point.x - robot_odom.pose.pose.position.x,
                                 destination.point.y - robot_odom.pose.pose.position.y, 0);
                vel = 0.3 * vel.normalized();
                vel_cmd.twist.linear.x = vel.x();
                vel_cmd.twist.linear.y = vel.y();
                vel_cmd.twist.linear.z = vel.z();
            }

            vel_cmd_pub->publish(vel_cmd);

            // Publish the go to destination action's status.
            go_to_destination_action->publish();
        }
    }

    void takeoff_active_callback() {
        geometry_msgs::msg::TwistStamped vel_cmd;
        vel_cmd.header.stamp = this->get_clock()->now();
        vel_cmd.header.frame_id = "world";
        // Set the status to "RUNNING" while the take off is in progress.
        takeoff_action->set_running();
        // Command upward velocity to take off.
        vel_cmd.twist.linear.z = 0.3;

        vel_cmd_pub->publish(vel_cmd);
    }

    void takeoff_inactive_callback() {
        geometry_msgs::msg::TwistStamped vel_cmd;
        vel_cmd.header.stamp = this->get_clock()->now();
        vel_cmd.header.frame_id = "world";
        // Set the status to "RUNNING" while the take off is in progress.
        takeoff_action->set_running();
        // Command upward velocity to take off.
        vel_cmd.twist.linear.z = 0.0;

        vel_cmd_pub->publish(vel_cmd);
    }

    void robot_odom_callback(const nav_msgs::msg::Odometry::SharedPtr robot_odom) {
        got_robot_odom = true;
        this->robot_odom = *robot_odom;
    }

    void home_callback(const geometry_msgs::msg::PointStamped::SharedPtr home) {
        got_home = true;
        this->home = *home;
    }

    void destination_callback(const geometry_msgs::msg::PointStamped::SharedPtr destination) {
        got_destination = true;
        this->destination = *destination;
    }

    void no_fly_zone_callback(const geometry_msgs::msg::PointStamped::SharedPtr no_fly_zone) {
        got_no_fly_zone = true;
        this->no_fly_zone = *no_fly_zone;
    }

    void no_fly_zone_radius_callback(const std_msgs::msg::Float32::SharedPtr no_fly_zone_radius) {
        got_no_fly_zone_radius = true;
        this->no_fly_zone_radius = no_fly_zone_radius->data;
    }

    float distance(nav_msgs::msg::Odometry odom, geometry_msgs::msg::PointStamped point) {
        float distance = sqrt(pow(odom.pose.pose.position.x - point.point.x, 2) +
                              pow(odom.pose.pose.position.y - point.point.y, 2));
        return distance;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviorTreeExample>());
    rclcpp::shutdown();
    return 0;
}
