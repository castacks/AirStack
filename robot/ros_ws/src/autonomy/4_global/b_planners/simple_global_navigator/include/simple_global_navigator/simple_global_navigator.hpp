#include <functional>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "task_msgs/action/navigation_task.hpp"

class SimpleGlobalNavigator : public rclcpp::Node {
   public:
    using NavigationTask = task_msgs::action::NavigationTask;
    using GoalHandleNavigationTask = rclcpp_action::ServerGoalHandle<NavigationTask>;

    SimpleGlobalNavigator(const rclcpp::NodeOptions& options)
        : Node("simple_global_navigator", options) {
        // Initialize the navigator
        RCLCPP_INFO(this->get_logger(), "Simple Global Navigator initialized.");

        global_plan_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
            "global_plan", rclcpp::QoS(10).transient_local());

        using namespace std::placeholders;
        this->action_server_ = rclcpp_action::create_server<NavigationTask>(
            this, 
            "simple_navigator", 
            std::bind(&SimpleGlobalNavigator::handle_goal, this, _1, _2),
            std::bind(&SimpleGlobalNavigator::handle_cancel, this, _1),
            std::bind(&SimpleGlobalNavigator::handle_accepted, this, _1));
    }

   private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_publisher_;
    rclcpp_action::Server<NavigationTask>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const NavigationTask::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request.");
        // Here you can validate the goal and return ACCEPT or REJECT
        (void) uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigationTask> action_handle) {
        RCLCPP_INFO(this->get_logger(), "Received cancel request.");
        // Here you can handle the cancel request
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNavigationTask> action_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal accepted.");
        // Here you can start processing the goal

        using namespace std::placeholders;
        std::thread{std::bind(&SimpleGlobalNavigator::execute, this, _1), action_handle}.detach();
    }

    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigationTask>>& action_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal.");
        // Here you can implement the goal execution logic
        rclcpp::Rate loop_rate(1);

        /* --------------- */
        const auto action_goal = action_handle->get_goal();


        const std::vector<geometry_msgs::msg::PoseStamped> &goal_poses = action_goal->goal_poses;


        /* --------------- */
        auto action_feedback = std::make_shared<NavigationTask::Feedback>();





        /* --------------- */
        auto action_result = std::make_shared<NavigationTask::Result>();

    }
};