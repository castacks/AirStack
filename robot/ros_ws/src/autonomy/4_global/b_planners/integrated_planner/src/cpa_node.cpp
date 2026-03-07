// Copyright (c) 2026 Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "../include/cpa_node.hpp"
#include "../include/vis_tools.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/create_timer_ros.h>
#include <vdb_edt/frontier_cluster.h>

CpaNode::CpaNode()
{
    node_handle_ = std::make_shared<rclcpp::Node>("integrated_plan_node");

    initialize();
}

void CpaNode::initialize()
{
    // TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_handle_->get_clock());
    tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(node_handle_->get_node_base_interface(),
                                                                                  node_handle_->get_node_timers_interface()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_handle_);

    // Initialize vdb-edt. It contains openvdb::initialize()
    map_manager_ = std::make_shared<VDBMap>(node_handle_, tf_buffer_, tf_listener_);
    RCLCPP_INFO(node_handle_->get_logger(), "Initialized VDB grids");

    setup_parameters();

    // Initialize astar planner.
    kino_astar_planner_.setParam(node_handle_);
    kino_astar_planner_.init(map_manager_, safe_robot_r_);

    cbg_planner_ = node_handle_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions opt;
    opt.callback_group = cbg_planner_;
    sub_tracking_point_ = node_handle_->create_subscription<geometry_msgs::msg::PoseStamped>("/tracking_point", rclcpp::QoS(10),
                                                                                             std::bind(&CpaNode::tracking_point_callback, this, std::placeholders::_1), opt);
    sub_goal_pose_ = node_handle_->create_subscription<geometry_msgs::msg::Pose>("/goal_point", 10,
                                                                                 std::bind(&CpaNode::goal_pose_callback, this, std::placeholders::_1), opt);

    pub_global_plan_ =
        node_handle_->create_publisher<visualization_msgs::msg::Marker>(pub_global_plan_topic_, 10);
    pub_trajectory_ =
        node_handle_->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/cmd_trajectory", 10);
    pub_trajectory_lines_ =
        node_handle_->create_publisher<visualization_msgs::msg::Marker>(pub_trajectory_viz_topic_, 10);
    planning_astar_vis_ =
        node_handle_->create_publisher<visualization_msgs::msg::MarkerArray>("/astar_plan_vis", 10);
    planning_smoothed_vis_ =
        node_handle_->create_publisher<visualization_msgs::msg::MarkerArray>("/smoothed_plan_vis", 10);
    pub_coll_point_vis_ =
        node_handle_->create_publisher<visualization_msgs::msg::Marker>("/collision_point", 10);

    pub_valid_viewpoints_ =
        node_handle_->create_publisher<geometry_msgs::msg::PoseArray>("/valid_viewpoints", 10);

    // Set up the timer
    // plan_timer_ = node_handle_->create_wall_timer(std::chrono::seconds(1),
    //                                               std::bind(&CpaNode::timerCallback, this), cbg_planner_);
    plan_timer_ = rclcpp::create_timer(node_handle_,
                                       node_handle_->get_clock(),
                                       rclcpp::Duration(std::chrono::seconds(1)),
                                       std::bind(&CpaNode::timerCallback, this),
                                       cbg_planner_);

    has_tracking_point_ = false;

    status_ = PlannerStatus::WAIT_GOAL;
}

void CpaNode::setup_parameters()
{
    auto log = node_handle_->get_logger();

    auto to_str = [](const auto &v) -> std::string
    {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, std::string>)
            return v;
        else if constexpr (std::is_same_v<T, bool>)
            return v ? "true" : "false";
        else
            return std::to_string(v);
    };

    auto param_set = [&](std::string_view name, const auto &def, auto &out)
    {
        using T = std::decay_t<decltype(out)>;
        const std::string key{name};
        T val{};

        if (!node_handle_->has_parameter(key))
        {
            val = node_handle_->template declare_parameter<T>(key, static_cast<T>(def));
        }
        else
        {
            node_handle_->get_parameter(key, val);
        }
        out = val;

        if (val == static_cast<T>(def))
        {
            RCLCPP_INFO(log, "Parameter %s uses default: %s", key.c_str(), to_str(val).c_str());
        }
        else
        {
            RCLCPP_INFO(log, "Parameter %s loaded from config: %s", key.c_str(), to_str(val).c_str());
        }
    };

    // ROS2: declare parameters with defaults
    param_set("world_frame_id", "map", world_frame_id_);
    param_set("robot_frame_id", "base_link", robot_frame_id_);

    param_set("sub_target_path_topic", "/perspective_goals", sub_goal_viewpoints_);

    param_set("pub_global_plan_topic", "/global_plan_viz", pub_global_plan_topic_);
    param_set("pub_trajectory_viz_topic", "/traj_viz", pub_trajectory_viz_topic_);

    param_set("srv_exploration_toggle_topic", "/robot_1/behavior/global_plan_toggle", srv_exploration_toggle_topic_);

    param_set("safe_robot_r", 0.5, safe_robot_r_);
    param_set("vox_size", 0.2, voxel_size_);

    safe_index_dist_ = safe_robot_r_ / voxel_size_;
    safe_sq_idx_dist_ = safe_index_dist_ * safe_index_dist_;
}

rclcpp::Node::SharedPtr CpaNode::get_node_handle() const
{
    return node_handle_;
}

void CpaNode::tracking_point_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    {
        std::unique_lock<std::shared_mutex> lk(tp_mtx_);
        has_tracking_point_ = true;
        current_tracking_point_ = *msg;
    }

    // Eigen::Vector3d p_track(current_tracking_point_.pose.position.x,
    //                         current_tracking_point_.pose.position.y,
    //                         current_tracking_point_.pose.position.z);
    // RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Got new tracking point \n"
    //                                                    << p_track);
}

void CpaNode::goal_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    RCLCPP_WARN(node_handle_->get_logger(), "Receiving a goal");

    curr_goal_xyz_.x() = msg->position.x;
    curr_goal_xyz_.y() = msg->position.y;
    curr_goal_xyz_.z() = msg->position.z;

    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    q.normalize();

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    curr_goal_yaw_ = yaw;

    status_ = PlannerStatus::NEW_GOAL;
}

void CpaNode::generate_plan()
{
    RCLCPP_INFO(node_handle_->get_logger(), "Starting to generate plan...");

    Eigen::Vector3d start_point;
    if (current_path_with_yaw_.empty())
    {
        start_point.x() = current_location_.translation.x;
        start_point.y() = current_location_.translation.y;
        start_point.z() = current_location_.translation.z;

        tf2::Quaternion q;
        tf2::fromMsg(current_location_.rotation, q);
        q.normalize();

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        new_start_yaw_ = yaw;
    }
    else
    {
        start_point = current_path_with_yaw_.back().state.head(3);
        new_start_yaw_ = current_path_with_yaw_.back().yaw;
    }

    kino_astar_planner_.reset();
    RCLCPP_INFO(node_handle_->get_logger(), "Kino Astar Reset");
    int plan_result;
    plan_result = kino_astar_planner_.search(start_point, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                             curr_goal_xyz_, Eigen::Vector3d::Zero(), true);
    RCLCPP_INFO(node_handle_->get_logger(), "Kino Astar finish");
    if (plan_result == KinodynamicAstar::NO_PATH)
    {
        RCLCPP_WARN(node_handle_->get_logger(), "Kino search fail");
        kino_astar_planner_.reset();
        plan_result = kino_astar_planner_.search(start_point, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                                 curr_goal_xyz_, Eigen::Vector3d::Zero(), false);
        if (plan_result == KinodynamicAstar::NO_PATH)
        {
            RCLCPP_WARN(node_handle_->get_logger(), "No path found to the goal");
            return;
        }
        RCLCPP_WARN(node_handle_->get_logger(), "Retry search success");
    }
    generated_path_ = kino_astar_planner_.getKinoTraj(delta_t_);
    if (generated_path_.empty())
    {
        RCLCPP_WARN(node_handle_->get_logger(), "Replan warning: Traj is empty.");
        return;
    }

    kino_astar_planner_.publishDebugVis();

    RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Received path goal is " << curr_goal_xyz_.transpose());
    RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Generate path goal is " << generated_path_.back().head(3).transpose());

    planYaw(delta_t_);
    // visualize
}

void CpaNode::planYaw(double dt)
{
    generated_path_with_yaw_.clear();

    if (generated_path_.empty())
        return;

    generated_path_with_yaw_.reserve(generated_path_.size());

    double last_yaw = new_start_yaw_;

    double lookahead_time = 0.5;
    int lookahead_idx_step = std::max(1, (int)(lookahead_time / dt));

    for (size_t i = 0; i < generated_path_.size(); ++i)
    {
        FullPathNode node;
        node.state = generated_path_[i];

        // --- Lookahead Logic ---
        size_t next_idx = std::min(i + lookahead_idx_step, generated_path_.size() - 1);

        Eigen::Vector3d curr_pos = node.state.head(3);
        Eigen::Vector3d next_pos = generated_path_[next_idx].head(3);
        Eigen::Vector3d dir_vec = next_pos - curr_pos;

        // --- Yaw Calculation ---
        if (dir_vec.head(2).norm() > 0.0001)
        {
            double target_yaw = std::atan2(dir_vec.y(), dir_vec.x());
            node.yaw = unwrappingYaw(last_yaw, target_yaw);
        }
        else
        {
            node.yaw = last_yaw;
        }

        last_yaw = node.yaw;
        generated_path_with_yaw_.push_back(node);
    }
}

double CpaNode::unwrappingYaw(double last_yaw, double target_yaw)
{
    double diff = target_yaw - last_yaw;

    while (diff > M_PI)
        diff -= 2 * M_PI;
    while (diff < -M_PI)
        diff += 2 * M_PI;

    return last_yaw + diff;
}

bool CpaNode::generate_replan(const FullPathNode &start_node)
{
    RCLCPP_INFO(node_handle_->get_logger(), "Generating replan...");

    Eigen::Vector3d start_pos = start_node.state.head(3);
    Eigen::Vector3d start_vel = start_node.state.segment(3, 3);
    Eigen::Vector3d start_acc = start_node.state.tail(3);

    kino_astar_planner_.reset();

    int plan_result;
    plan_result = kino_astar_planner_.search(start_pos, start_vel, start_acc,
                                             curr_goal_xyz_, Eigen::Vector3d::Zero(), true);

    if (plan_result == KinodynamicAstar::NO_PATH)
    {
        RCLCPP_WARN(node_handle_->get_logger(), "Replan: Kino search fail");
        kino_astar_planner_.reset();
        plan_result = kino_astar_planner_.search(start_pos, start_vel, start_acc,
                                                 curr_goal_xyz_, Eigen::Vector3d::Zero(), false);
        if (plan_result == KinodynamicAstar::NO_PATH)
        {
            RCLCPP_WARN(node_handle_->get_logger(), "Replan: No path found to the goal");
            return false;
        }
        RCLCPP_WARN(node_handle_->get_logger(), "Replan: Retry search success");
    }

    generated_path_ = kino_astar_planner_.getKinoTraj(delta_t_);
    if (generated_path_.empty())
    {
        RCLCPP_WARN(node_handle_->get_logger(), "Replan warning: Traj is empty.");
        return false;
    }

    kino_astar_planner_.publishDebugVis();

    new_start_yaw_ = start_node.yaw;
    planYaw(delta_t_);

    return true;
}

void CpaNode::publish_plan()
{
    if (current_path_with_yaw_.empty())
    {
        RCLCPP_WARN(node_handle_->get_logger(), "Current path is empty after trim, no traj to publish.");
        return;
    }

    trajectory_msgs::msg::MultiDOFJointTrajectory traj_msg;
    traj_msg.header.stamp = node_handle_->now();
    traj_msg.header.frame_id = world_frame_id_;

    traj_msg.joint_names.clear();
    traj_msg.joint_names.push_back(robot_frame_id_);

    traj_msg.points.clear();
    traj_msg.points.reserve(current_path_with_yaw_.size());

    for (size_t i = 0; i < current_path_with_yaw_.size(); ++i)
    {
        const auto &p = current_path_with_yaw_[i];

        double time_from_start = i * delta_t_;

        // Horizon Check
        if (time_from_start > traj_horizon_ && !traj_msg.points.empty())
            break;

        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint pt_msg;

        // 1. Transform (Pos + Yaw)
        geometry_msgs::msg::Transform tf_msg;
        tf_msg.translation.x = p.state(0);
        tf_msg.translation.y = p.state(1);
        tf_msg.translation.z = p.state(2);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, p.yaw);
        tf_msg.rotation = tf2::toMsg(q);
        pt_msg.transforms.push_back(tf_msg);

        // 2. Velocity
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = p.state(3);
        vel_msg.linear.y = p.state(4);
        vel_msg.linear.z = p.state(5);
        pt_msg.velocities.push_back(vel_msg);

        // 3. Acceleration
        geometry_msgs::msg::Twist acc_msg;
        acc_msg.linear.x = p.state(6);
        acc_msg.linear.y = p.state(7);
        acc_msg.linear.z = p.state(8);
        pt_msg.accelerations.push_back(acc_msg);

        // 4. Time (填入计算出的时间)
        pt_msg.time_from_start = rclcpp::Duration::from_seconds(time_from_start);

        traj_msg.points.push_back(pt_msg);
    }

    pub_trajectory_->publish(traj_msg);
}

void CpaNode::trim_covered_path()
{
    bool has_tp = false;
    geometry_msgs::msg::PoseStamped ctp;
    {
        std::shared_lock<std::shared_mutex> lk(tp_mtx_);
        has_tp = has_tracking_point_;
        if (has_tp)
        {
            ctp = current_tracking_point_;
        }
    }

    if (!has_tp)
    {
        // RCLCPP_DEBUG(node_handle_->get_logger(), "No tracking point to trim.");
        return;
    }

    Eigen::Vector3d p_track(ctp.pose.position.x,
                            ctp.pose.position.y,
                            ctp.pose.position.z);

    RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Trimming path based on: " << p_track.transpose());

    if (current_path_with_yaw_.empty())
        return;

    double best_d2 = std::numeric_limits<double>::infinity();
    size_t best_i = 0;

    size_t search_limit = std::min(current_path_with_yaw_.size(), (size_t)50);

    for (size_t i = 0; i < search_limit; ++i)
    {
        Eigen::Vector3d pi = current_path_with_yaw_[i].state.head(3);

        double d2 = (p_track - pi).squaredNorm();

        if (d2 < best_d2)
        {
            best_d2 = d2;
            best_i = i;

            if (d2 < 1e-4)
                break;
        }
    }

    if (best_i > 0)
    {
        current_path_with_yaw_.erase(current_path_with_yaw_.begin(),
                                     current_path_with_yaw_.begin() + best_i);

        // RCLCPP_INFO(node_handle_->get_logger(), "Trimmed %ld points. Remaining: %ld", best_i, current_path_with_yaw_.size());
    }
}

bool CpaNode::check_local_path_free()
{
    double check_horizon = traj_horizon_;

    for (size_t i = 0; i < current_path_with_yaw_.size(); ++i)
    {
        double time_from_start = i * delta_t_;

        if (time_from_start > check_horizon)
        {
            break;
        }

        const auto &pt = current_path_with_yaw_[i];
        double sq_dist;
        bool is_observed = map_manager_->query_sqdist_at_world(pt.state.head(3), sq_dist);

        if (is_observed && sq_dist <= safe_sq_idx_dist_)
        {
            RCLCPP_WARN(node_handle_->get_logger(), "Collision detected at index %ld (t=%.2f)", i, time_from_start);
            RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Point: " << pt.state.head(3).transpose());
            RCLCPP_WARN_STREAM(node_handle_->get_logger(), "DistSq: " << sq_dist << " <= SafeSq: " << safe_sq_idx_dist_);

            return false;
        }
    }

    return true;
}

void CpaNode::timerCallback()
{
    if (!planning_mtx_.try_lock())
    {
        RCLCPP_WARN(node_handle_->get_logger(), "Planner is busy (A* taking too long), skipping this loop.");
        return;
    }
    std::lock_guard<std::mutex> lg(planning_mtx_, std::adopt_lock);
    // get current TF to world
    try
    {
        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(world_frame_id_,
                                                                                             robot_frame_id_,
                                                                                             rclcpp::Time(0));
        current_location_ = transform_stamped.transform;
        if (!received_first_robot_tf_)
        {
            received_first_robot_tf_ = true;
            possible_stuck_location_ = current_location_;
            last_position_change_ = node_handle_->now();
            RCLCPP_WARN(node_handle_->get_logger(), "Received first robot_tf");
        }
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(node_handle_->get_logger(), "Robot tf not received: %s", ex.what());
    }

    switch (status_)
    {
    case PlannerStatus::NEW_GOAL:
    {
        // RCLCPP_WARN(node_handle_->get_logger(), "Generating plan...");
        // First plan: no generated path yet
        generate_plan();
        current_path_with_yaw_ = generated_path_with_yaw_;

        if (current_path_with_yaw_.size() < 2)
        {
            RCLCPP_WARN(node_handle_->get_logger(), "Planned path too short.");
            status_ = PlannerStatus::WAIT_GOAL;
            return;
        }

        status_ = PlannerStatus::PATH_EXEC;
        publish_plan();
        break;
    }
    case PlannerStatus::WAIT_GOAL:
    {
        RCLCPP_WARN(node_handle_->get_logger(), "Waiting for goal.");
        break;
    }
    case PlannerStatus::PATH_EXEC:
    {
        trim_covered_path();
        Eigen::Vector3d curr_path_end = current_path_with_yaw_.back().state.head(3);
        RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Curr Path end at " << curr_path_end.transpose());
        size_t lookahead_steps = 15;
        if (current_path_with_yaw_.size() < lookahead_steps + 1 && (curr_path_end - curr_goal_xyz_).norm() < 0.01)
        {
            RCLCPP_WARN(node_handle_->get_logger(), "Path ended after trim.");
            status_ = PlannerStatus::WAIT_GOAL;
        }
        // Need extension or
        // Check Collision
        else if ((curr_path_end - curr_goal_xyz_).norm() >= 0.01 || !check_local_path_free())
        {
            if (!generate_replan(current_path_with_yaw_[lookahead_steps]))
            {
                RCLCPP_WARN(node_handle_->get_logger(), "Replan failed, stay at the last track point.");
                current_path_with_yaw_.resize(1);
                current_path_with_yaw_[0].state.segment(3, 3) = Eigen::Vector3d::Zero(); // Vel = 0
                current_path_with_yaw_[0].state.tail(3) = Eigen::Vector3d::Zero();       // Acc = 0
                status_ = PlannerStatus::WAIT_GOAL;
            }
            else
            {
                RCLCPP_INFO(node_handle_->get_logger(), "Replan success.");
                double t_start = 0;
                current_path_with_yaw_.resize(lookahead_steps);
                current_path_with_yaw_.insert(current_path_with_yaw_.end(),
                                              generated_path_with_yaw_.begin(),
                                              generated_path_with_yaw_.end());
            }
        }

        trim_covered_path();
        publish_plan();
        break;
    }
    }
}

#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<CpaNode>();
    auto node = planner->get_node_handle();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}