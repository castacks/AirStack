// =============================================================================
//  tigris_search_planner_node — single-agent TIGRIS as an AirStack global planner.
//
//  A drop-in for mtl_search_planner on the SAME interface, so the unchanged
//  mtl_trajectory_follower flies it and the unchanged mtl_metrics_logger scores
//  it (identical telemetry / detection / residual-belief / report outputs):
//
//  * serves   search_mission               mtl_msgs/action/SearchMission
//  * publishes (transient local)
//      search/plan                 mtl_msgs/SearchPlan  (the whole sortie, one plan_id,
//                                  re-published with a newer stamp at every replan)
//      search/planned_trajectory   airstack_msgs/TrajectoryXYZVYaw
//      search/planned_boresight    nav_msgs/Path
//      search/planned_path         nav_msgs/Path
//      search/markers              visualization_msgs/MarkerArray
//      search/tigris_status        std_msgs/String (JSON per solve)
//  * listens  search/follower_status, odometry, gimbal/state, gimbal/cmd_pitch_yaw
//  * writes   runs/<run_id>/<agent>/{plan.json, track.json, scenario.json, tigris_replans.json}
//
//  Receding horizon (receding.hpp): the looks actually flown (measured pose +
//  measured gimbal, like the logger) are folded into the belief before every
//  replan; the track up to the commit point never changes, so the follower
//  keeps its arc-length progress across revisions (it must run with
//  accept_plan_revisions: true, which the tigris_search stack sets).
//
//  No gimbal actuation: the plan tells the follower a single-axis mount with
//  the cross-track travel and pitch nudge locked (1e-6 rad), so the camera is
//  body-fixed at the configured forward tilt and turns with the airframe.
// =============================================================================
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <airstack_msgs/msg/trajectory_xyzv_yaw.hpp>
#include <airstack_msgs/msg/waypoint_xyzv_yaw.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <mtl_msgs/action/search_mission.hpp>
#include <mtl_msgs/msg/follower_status.hpp>
#include <mtl_msgs/msg/search_plan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "tigris_search_planner/receding.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;
namespace ts = tigris_search;

namespace {

using SearchMission  = mtl_msgs::action::SearchMission;
using GoalHandle     = rclcpp_action::ServerGoalHandle<SearchMission>;
using FollowerStatus = mtl_msgs::msg::FollowerStatus;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kLockedRad = 1e-6;  // "zero" travel: the follower treats 0 as unset

geometry_msgs::msg::Quaternion yawQuat(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(0.5 * yaw);
    q.w = std::cos(0.5 * yaw);
    return q;
}

geometry_msgs::msg::Point point(double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

std::string utcStamp() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&t, &tm);
    std::ostringstream os;
    os << std::put_time(&tm, "%Y%m%d-%H%M%S");
    return os.str();
}

/// Write via a temp file + rename so a reader never sees half a file.
bool writeText(const fs::path& path, const std::string& text) {
    const fs::path tmp = path.string() + ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary);
        if (!f) return false;
        f << text << "\n";
        if (!f) return false;
    }
    std::error_code ec;
    fs::rename(tmp, path, ec);
    return !ec;
}

double yawOf(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class TigrisSearchPlannerNode : public rclcpp::Node {
public:
    TigrisSearchPlannerNode() : Node("tigris_search_planner") {
        const char* robot = std::getenv("ROBOT_NAME");
        scenario_file_ = declare_parameter<std::string>(
            "scenario_file", "/root/AirStack/stacks/tigris_search/config/scenario.json");
        agent_name_  = declare_parameter<std::string>("agent_name", robot ? robot : "robot_1");
        frame_id_    = declare_parameter<std::string>("frame_id", "map");
        home_up_m_   = declare_parameter<double>("home_up_m", 0.0);
        runs_root_   = declare_parameter<std::string>("runs_root", "/root/AirStack/runs");
        plan_on_startup_ = declare_parameter<bool>("plan_on_startup", true);
        status_timeout_s_ = declare_parameter<double>("follower_status_timeout_s", 10.0);
        mission_timeout_pad_s_ = declare_parameter<double>("mission_timeout_pad_s", 240.0);
        mission_timeout_factor_ = declare_parameter<double>("mission_timeout_factor", 3.0);
        path_step_m_ = declare_parameter<double>("viz_path_step_m", 2.0);
        start_from_odometry_ = declare_parameter<bool>("start_from_odometry", true);
        start_yaw_deg_ = declare_parameter<double>("start_yaw_deg", -999.0);
        lock_gimbal_ = declare_parameter<bool>("lock_gimbal", true);
        look_period_s_ = declare_parameter<double>("look_period_s", 0.1);

        // TIGRIS (see config/tigris_search_planner.yaml for the meaning and the 1/12.5 scaling)
        reward_mode_ = declare_parameter<std::string>("reward_mode", "original");
        sampler_ = declare_parameter<std::string>("sampler", "informed");
        receding_ = declare_parameter<bool>("receding", true);
        initial_planning_time_s_ = declare_parameter<double>("initial_planning_time_s", 5.0);
        planning_time_s_ = declare_parameter<double>("planning_time_s", 5.0);
        replan_period_s_ = declare_parameter<double>("replan_period_s", 5.0);
        commit_margin_s_ = declare_parameter<double>("commit_margin_s", 1.0);
        lookahead_turn_radii_ = declare_parameter<double>("lookahead_turn_radii", 1.2);
        min_replan_budget_m_ = declare_parameter<double>("min_replan_budget_m", 10.0);
        extend_dist_m_ = declare_parameter<double>("extend_dist_m", 60.0);
        extend_radius_m_ = declare_parameter<double>("extend_radius_m", 20.0);
        prune_radius_m_ = declare_parameter<double>("prune_radius_m", 60.0);
        reward_step_m_ = declare_parameter<double>("reward_step_m", 2.0);
        grid_res_m_ = declare_parameter<double>("grid_res_m", 4.0);
        view_point_goal_ = declare_parameter<double>("view_point_goal", 0.6);
        bounds_margin_m_ = declare_parameter<double>("bounds_margin_m", -1.0);
        use_entropy_ = declare_parameter<bool>("use_entropy", true);
        rs_ = declare_parameter<double>("rs", 2.0);
        rf_ = declare_parameter<double>("rf", 1.0);
        initial_confidence_ = declare_parameter<double>("initial_confidence", 0.01);
        max_iterations_ = declare_parameter<int>("max_iterations", 0);
        seed_ = declare_parameter<int>("seed", -1);
        budget_m_ = declare_parameter<double>("budget_m", -1.0);
        camera_fov_deg_ = declare_parameter<double>("camera_fov_deg", -1.0);
        camera_tilt_deg_ = declare_parameter<double>("camera_tilt_deg", -1.0);

        const auto latched = rclcpp::QoS(1).reliable().transient_local();
        plan_pub_      = create_publisher<mtl_msgs::msg::SearchPlan>("search/plan", latched);
        traj_pub_      = create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("search/planned_trajectory", latched);
        boresight_pub_ = create_publisher<nav_msgs::msg::Path>("search/planned_boresight", latched);
        path_pub_      = create_publisher<nav_msgs::msg::Path>("search/planned_path", latched);
        markers_pub_   = create_publisher<visualization_msgs::msg::MarkerArray>("search/markers", latched);
        status_pub_    = create_publisher<std_msgs::msg::String>("search/tigris_status", latched);
        abort_pub_     = create_publisher<std_msgs::msg::Empty>("search/abort", 10);

        status_sub_ = create_subscription<FollowerStatus>(
            "search/follower_status", 10, [this](FollowerStatus::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lk(status_mutex_);
                last_status_ = *msg;
                last_status_time_ = now();
                have_status_ = true;
                recording_ = flying_ && msg->plan_id == active_plan_id_ &&
                             (msg->state == FollowerStatus::INGRESS || msg->state == FollowerStatus::SEARCH);
            });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odometry", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                onOdometry(*msg);
            });
        gimbal_state_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
            "gimbal/state", rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::Vector3::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lk(status_mutex_);
                gimbal_meas_ = *msg;
                gimbal_meas_time_ = now();
                have_gimbal_meas_ = true;
            });
        gimbal_cmd_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
            "gimbal/cmd_pitch_yaw", 10, [this](geometry_msgs::msg::Vector3::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lk(status_mutex_);
                gimbal_cmd_ = *msg;
                have_gimbal_cmd_ = true;
            });

        action_server_ = rclcpp_action::create_server<SearchMission>(
            this, "search_mission",
            [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const SearchMission::Goal> goal) {
                RCLCPP_INFO(get_logger(), "search_mission goal received (run_id '%s', start_mission %s)",
                            goal->run_id.c_str(), goal->start_mission ? "true" : "false");
                if (busy_.load()) {
                    RCLCPP_WARN(get_logger(), "search_mission goal rejected: a sortie is already active");
                    return rclcpp_action::GoalResponse::REJECT;
                }
                goal_seen_.store(true);  // a pending startup preview yields to this goal
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](const std::shared_ptr<GoalHandle>) {
                RCLCPP_INFO(get_logger(), "search_mission cancel requested");
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandle> gh) {
                busy_.store(true);
                std::thread([this, gh]() {
                    try {
                        execute(gh);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(get_logger(), "search_mission execution failed: %s", e.what());
                        auto r = std::make_shared<SearchMission::Result>();
                        r->success = false;
                        r->message = std::string("internal error: ") + e.what();
                        try { gh->abort(r); } catch (...) {}
                        {
                            std::lock_guard<std::mutex> lk(status_mutex_);
                            flying_ = false;
                            recording_ = false;
                        }
                        busy_.store(false);
                    }
                }).detach();
            });

        RCLCPP_INFO(get_logger(), "tigris_search_planner: agent '%s', scenario %s, reward %s, %s",
                    agent_name_.c_str(), scenario_file_.c_str(), reward_mode_.c_str(),
                    receding_ ? "receding horizon" : "one-shot");
        if (plan_on_startup_) {
            // Deferred one tick so the sim clock can arrive; solved off the executor thread
            // (a TIGRIS solve takes initial_planning_time_s). A goal arriving meanwhile
            // simply waits for plan_mutex_.
            startup_timer_ = create_wall_timer(1s, [this]() {
                startup_timer_->cancel();
                std::thread([this]() {
                    std::lock_guard<std::mutex> lk(plan_mutex_);
                    if (goal_seen_.load()) return;  // a sortie already owns the plan
                    std::string err;
                    if (!planInitialLocked(scenario_file_, "", false, &err)) {
                        RCLCPP_ERROR(get_logger(), "startup preview plan failed: %s", err.c_str());
                    }
                }).detach();
            });
        }
    }

private:
    // ------------------------------------------------------------------ inputs
    void onOdometry(const nav_msgs::msg::Odometry& msg) {
        std::lock_guard<std::mutex> lk(status_mutex_);
        const auto& p = msg.pose.pose.position;
        if (have_odom_ && flying_) {
            flown_m_ += std::hypot(std::hypot(p.x - last_odom_.x, p.y - last_odom_.y), p.z - last_odom_.z);
        }
        last_odom_ = p;
        last_odom_yaw_ = yawOf(msg.pose.pose.orientation);
        have_odom_ = true;
        if (!recording_) return;
        // one look per look_period_s from the measured pose + gimbal (the logger's model)
        const rclcpp::Time t = now();
        const double dt = have_look_time_ ? (t - last_look_time_).seconds() : look_period_s_;
        if (have_look_time_ && dt < look_period_s_) return;
        last_look_time_ = t;
        have_look_time_ = true;
        const bool measured = have_gimbal_meas_ && (t - gimbal_meas_time_).seconds() < 0.5;
        if (!measured && !have_gimbal_cmd_) return;
        const geometry_msgs::msg::Vector3& g = measured ? gimbal_meas_ : gimbal_cmd_;
        const double xw = p.x + origin_[0], yw = p.y + origin_[1], zw = p.z + origin_[2];
        const double w = std::min(std::max(dt, 0.0), 0.5) / std::max(look_det_.dtRef, 1e-6);
        const ts::Look l = ts::lookFromGimbal(xw, yw, zw, g.y, g.z, look_fov_, look_det_, w);
        if (l.valid) flown_looks_.push_back(l);
    }

    // ------------------------------------------------------------------ planning
    ts::TigrisParams tigrisParams(const ts::Scenario& sc) const {
        if (sampler_ != "informed" && sampler_ != "uniform" && sampler_ != "random") {
            RCLCPP_WARN(get_logger(), "unknown sampler '%s': using 'informed'", sampler_.c_str());
        }
        ts::TigrisParams tp;
        tp.extendDist = extend_dist_m_;
        tp.extendRadius = extend_radius_m_;
        tp.pruneRadius = prune_radius_m_;
        tp.rewardStep = reward_step_m_;
        tp.viewPointGoal = view_point_goal_;
        tp.boundsMargin = bounds_margin_m_;
        tp.informedSampler = !(sampler_ == "uniform" || sampler_ == "random");
        tp.maxIterations = max_iterations_;
        tp.seed = seed_ >= 0 ? static_cast<std::uint64_t>(seed_) : sc.seed;
        tp.reward.mode = ts::rewardModeFromString(reward_mode_);
        tp.reward.useEntropy = use_entropy_;
        tp.reward.rs = rs_;
        tp.reward.rf = rf_;
        tp.reward.initialConfidence = initial_confidence_;
        return tp;
    }

    ts::HorizonParams horizonParams(const ts::Scenario& sc) const {
        ts::HorizonParams hp;
        hp.receding = receding_;
        hp.initialPlanningTime = initial_planning_time_s_;
        hp.replanPlanningTime = planning_time_s_;
        hp.replanPeriod = replan_period_s_;
        hp.commitMarginTime = commit_margin_s_;
        hp.lookahead = lookahead_turn_radii_ * sc.minTurnRadius;
        hp.minReplanBudget = min_replan_budget_m_;
        hp.gridRes = grid_res_m_;
        hp.budgetOverride = budget_m_;
        if (start_yaw_deg_ > -900.0) hp.startYaw = start_yaw_deg_ * kDeg;
        return hp;
    }

    ts::Camera camera(const ts::Scenario& sc) {
        ts::Camera cam;
        cam.fov = camera_fov_deg_ > 0.0 ? camera_fov_deg_ * kDeg : sc.fovRad;
        cam.tilt = camera_tilt_deg_ >= 0.0 ? camera_tilt_deg_ * kDeg : sc.tiltRad;
        if (std::fabs(cam.fov - sc.fovRad) > 1e-9) {
            RCLCPP_WARN(get_logger(), "camera_fov_deg %.1f differs from the scenario sensor.fov_deg %.1f: the "
                        "metrics logger scores the SCENARIO fov - change it in mission.yaml instead",
                        cam.fov / kDeg, sc.fovRad / kDeg);
        }
        return cam;
    }

    /// Load the scenario, build the horizon and solve the first plan. Caller holds plan_mutex_.
    bool planInitialLocked(const std::string& scenarioPath, const std::string& runId, bool start,
                           std::string* err) {
        try {
            scenario_ = std::make_unique<ts::Scenario>(ts::loadScenario(scenarioPath));
            agent_idx_ = scenario_->agentIndex(agent_name_);
            if (agent_idx_ < 0) {
                std::ostringstream os;
                os << "agent '" << agent_name_ << "' is not in the scenario team (";
                for (const auto& a : scenario_->agents) os << a.name << " ";
                os << ") - regenerate the scenario for this fleet";
                throw std::runtime_error(os.str());
            }
            const ts::AgentSpec& ag = scenario_->agents[static_cast<std::size_t>(agent_idx_)];
            {
                std::lock_guard<std::mutex> sl(status_mutex_);
                origin_[0] = ag.homeX();
                origin_[1] = ag.homeY();
                origin_[2] = home_up_m_;
                look_det_ = scenario_->det;
                look_fov_ = scenario_->fovRad;  // the logger's FOV
            }
            const ts::HorizonParams hp = horizonParams(*scenario_);
            horizon_ = std::make_unique<ts::RecedingHorizon>(*scenario_, agent_idx_, camera(*scenario_),
                                                             tigrisParams(*scenario_), hp);
            ts::Pose2 s0 = ts::defaultStartPose(*scenario_, agent_idx_, hp);
            {
                std::lock_guard<std::mutex> sl(status_mutex_);
                if (start && start_from_odometry_ && have_odom_) {
                    s0.x = last_odom_.x + origin_[0];
                    s0.y = last_odom_.y + origin_[1];
                    if (hp.startYaw >= 1e8) {
                        s0.yaw = std::atan2(scenario_->centerY() - s0.y, scenario_->centerX() - s0.x);
                    }
                }
            }
            const auto t0 = std::chrono::steady_clock::now();
            if (!horizon_->start(s0)) throw std::runtime_error("TIGRIS found no path from the start pose");
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            plan_id_ = scenario_->name + "/" + agent_name_ + "/" + (runId.empty() ? "preview" : runId);
            run_id_ = runId;
            start_mission_ = start;
            if (start) {
                // arm the flown-look recorder BEFORE the plan goes out, so the first
                // INGRESS / SEARCH status already records
                std::lock_guard<std::mutex> sl(status_mutex_);
                flown_m_ = 0.0;
                flying_ = true;
                recording_ = false;
                active_plan_id_ = plan_id_;
                flown_looks_.clear();
                have_look_time_ = false;
            }
            const auto& rec = horizon_->records().back();
            RCLCPP_INFO(get_logger(),
                        "planned %s in %.0f ms: %zu samples, %.0f m of %.0f m budget, %d iterations, tree %d, "
                        "reward original %.3f / matched %.4f",
                        plan_id_.c_str(), ms, horizon_->track().size(), horizon_->totalArc(), horizon_->budget(),
                        rec.iterations, rec.treeSize, rec.segmentReward.original, rec.segmentReward.matched);
            publishPlan();
            return true;
        } catch (const std::exception& e) {
            if (err) *err = e.what();
            return false;
        }
    }

    ts::TrackMeta meta() const {
        ts::TrackMeta m;
        m.planId = plan_id_;
        m.rewardMode = reward_mode_;
        m.sampler = sampler_;
        m.budget = horizon_->budget();
        m.revision = horizon_->revision();
        m.gimbalLocked = lock_gimbal_;
        return m;
    }

    void publishPlan() {
        const rclcpp::Time stamp = now();
        const ts::Scenario& sc = *scenario_;
        const ts::AgentSpec& ag = sc.agents[static_cast<std::size_t>(agent_idx_)];
        const double hx = ag.homeX(), hy = ag.homeY(), hz = home_up_m_;
        const ts::PlannerSetup& su = horizon_->setup();
        mtl_msgs::msg::SearchPlan msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id_;
        msg.plan_id = plan_id_;
        msg.run_id = run_id_;
        msg.scenario_name = sc.name;
        msg.agent_name = agent_name_;
        msg.agent_index = agent_idx_;
        msg.start_mission = start_mission_;
        msg.single_axis_gimbal = true;
        msg.mount_tilt_rad = su.camera.tilt;
        msg.fov_rad = su.camera.fov;
        msg.speed_mps = sc.speed;
        msg.min_turn_radius_m = sc.minTurnRadius;
        msg.altitude_m = su.altitude;
        msg.dt_s = sc.dt;
        msg.gimbal_max_rad = lock_gimbal_ ? kLockedRad : 80.0 * kDeg;
        msg.gimbal_rate_rad_s = 120.0 * kDeg;
        msg.pitch_nudge_max_rad = lock_gimbal_ ? kLockedRad : 5.0 * kDeg;
        msg.map_origin_in_world.x = hx;
        msg.map_origin_in_world.y = hy;
        msg.map_origin_in_world.z = hz;
        msg.trajectory.header = msg.header;
        msg.planned_length_m = horizon_->totalArc();
        msg.budget_m = horizon_->budget();

        nav_msgs::msg::Path path, bore;
        path.header = msg.header;
        bore.header = msg.header;
        const auto& tr = horizon_->track();
        double lastArc = -1e9;
        for (std::size_t k = 0; k < tr.size(); ++k) {
            const ts::TrackSample& s = tr[k];
            airstack_msgs::msg::WaypointXYZVYaw wp;
            wp.position = point(s.x - hx, s.y - hy, s.z - hz);
            wp.velocity = sc.speed;
            wp.yaw = s.yaw;
            msg.trajectory.waypoints.push_back(wp);
            msg.boresight.push_back(point(s.bx - hx, s.by - hy, s.bz - hz));
            msg.arc_length_m.push_back(s.arc);
            msg.time_s.push_back(s.t);
            msg.planned_gimbal_phi_rad.push_back(0.0);
            msg.planned_pitch_rad.push_back(0.0);
            if (s.arc - lastArc >= path_step_m_ || k + 1 == tr.size()) {
                lastArc = s.arc;
                geometry_msgs::msg::PoseStamped ps;
                ps.header = msg.header;
                ps.pose.position = wp.position;
                ps.pose.orientation = yawQuat(s.yaw);
                path.poses.push_back(ps);
                geometry_msgs::msg::PoseStamped pb = ps;
                pb.pose.position = point(s.bx - hx, s.by - hy, s.bz - hz);
                bore.poses.push_back(pb);
            }
        }
        const std::vector<int> cells = ts::servicedCells(sc, *horizon_);
        for (const int c : cells) msg.serviced_cells.push_back(c);

        plan_pub_->publish(msg);
        traj_pub_->publish(msg.trajectory);
        path_pub_->publish(path);
        boresight_pub_->publish(bore);
        markers_pub_->publish(buildMarkers(stamp, cells));
        publishTigrisStatus();
    }

    void publishTigrisStatus() {
        if (!horizon_ || horizon_->records().empty()) return;
        const ts::ReplanRecord& r = horizon_->records().back();
        std::ostringstream os;
        os << std::setprecision(6) << "{\"plan_id\":\"" << plan_id_ << "\",\"revision\":" << horizon_->revision()
           << ",\"solve\":" << r.index << ",\"trigger\":\"" << r.trigger << "\",\"improved\":"
           << (r.improved ? "true" : "false") << ",\"progress_m\":" << r.progressArc << ",\"commit_m\":"
           << r.commitArc << ",\"budget_left_m\":" << r.budgetLeft << ",\"planning_s\":" << r.seconds
           << ",\"iterations\":" << r.iterations << ",\"tree\":" << r.treeSize << ",\"segment_m\":"
           << r.segmentLength << ",\"reward_original\":" << r.segmentReward.original << ",\"reward_matched\":"
           << r.segmentReward.matched << ",\"residual_after_flown\":" << r.residualMassFlown
           << ",\"track_m\":" << horizon_->totalArc() << "}";
        std_msgs::msg::String s;
        s.data = os.str();
        status_pub_->publish(s);
    }

    visualization_msgs::msg::MarkerArray buildMarkers(const rclcpp::Time& stamp, const std::vector<int>& cells) const {
        visualization_msgs::msg::MarkerArray arr;
        const ts::Scenario& sc = *scenario_;
        const ts::AgentSpec& ag = sc.agents[static_cast<std::size_t>(agent_idx_)];
        const double hx = ag.homeX(), hy = ag.homeY(), hz = home_up_m_;
        auto base = [&](const std::string& ns, int id, int type) {
            visualization_msgs::msg::Marker m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            m.ns = ns;
            m.id = id;
            m.type = type;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            return m;
        };
        {
            auto m = base("tigris_area", 0, visualization_msgs::msg::Marker::LINE_STRIP);
            m.scale.x = 0.8;
            m.color = rgba(1.0f, 0.85f, 0.1f, 0.9f);
            const double xs[5] = {sc.xMin, sc.xMax, sc.xMax, sc.xMin, sc.xMin};
            const double ys[5] = {sc.yMin, sc.yMin, sc.yMax, sc.yMax, sc.yMin};
            for (int i = 0; i < 5; ++i) m.points.push_back(point(xs[i] - hx, ys[i] - hy, 0.2 - hz));
            arr.markers.push_back(m);
        }
        {
            auto all = base("tigris_cells", 0, visualization_msgs::msg::Marker::CUBE_LIST);
            auto mine = base("tigris_cells_seen", 0, visualization_msgs::msg::Marker::CUBE_LIST);
            all.scale.x = all.scale.y = sc.cellSize * 0.9;
            all.scale.z = 0.1;
            mine.scale = all.scale;
            mine.scale.z = 0.2;
            all.color = rgba(0.7f, 0.7f, 0.7f, 0.25f);
            mine.color = rgba(1.0f, 0.4f, 0.8f, 0.45f);
            std::vector<char> seen(sc.cellX.size(), 0);
            for (const int c : cells) seen[static_cast<std::size_t>(c)] = 1;
            for (std::size_t i = 0; i < sc.cellX.size(); ++i) {
                (seen[i] ? mine : all).points.push_back(point(sc.cellX[i] - hx, sc.cellY[i] - hy, 0.05 - hz));
            }
            arr.markers.push_back(all);
            arr.markers.push_back(mine);
        }
        return arr;
    }

    std::string runDir() const { return (fs::path(runs_root_) / run_id_ / agent_name_).string(); }

    void writeRunFiles(bool withScenario) {
        const fs::path dir(runDir());
        std::error_code ec;
        fs::create_directories(dir, ec);
        if (ec) {
            RCLCPP_WARN(get_logger(), "cannot create run dir %s: %s (is runs/ mounted?)", dir.c_str(),
                        ec.message().c_str());
            return;
        }
        const ts::TrackMeta m = meta();
        bool ok = writeText(dir / "track.json", ts::agentTrackJson(*scenario_, agent_idx_, *horizon_, m)) &&
                  writeText(dir / "plan.json", ts::planJson(*scenario_, agent_idx_, *horizon_, m)) &&
                  writeText(dir / "tigris_replans.json", ts::replansJson(*scenario_, agent_idx_, *horizon_, m));
        if (withScenario) ok = writeText(dir / "scenario.json", scenario_->raw.dump()) && ok;
        if (!ok) RCLCPP_WARN(get_logger(), "could not write the plan files into %s", dir.c_str());
        if (withScenario) {
            const fs::path latest = fs::path(runs_root_) / "latest";
            fs::remove(latest, ec);
            fs::create_directory_symlink(fs::path(run_id_), latest, ec);
        }
    }

    // ------------------------------------------------------------------ sortie
    void execute(const std::shared_ptr<GoalHandle> gh) {
        const auto goal = gh->get_goal();
        auto result = std::make_shared<SearchMission::Result>();
        auto feedback = std::make_shared<SearchMission::Feedback>();
        struct Release { std::atomic<bool>& b; ~Release() { b.store(false); } } release{busy_};

        feedback->phase = "PLANNING";
        gh->publish_feedback(feedback);
        std::string runId = goal->run_id.empty() ? utcStamp() : goal->run_id;
        if (goal->start_mission) {
            // a finished plan_id is never re-flown by the follower (nor re-scored by the
            // logger): make a reused run_id unique
            const std::string base = runId;
            for (int k = 2; flown_run_ids_.count(runId) > 0; ++k) runId = base + "_" + std::to_string(k);
            if (runId != base) {
                RCLCPP_WARN(get_logger(), "run_id '%s' was already flown by this node; using '%s'", base.c_str(),
                            runId.c_str());
            }
        }
        const std::string scenario = goal->scenario_file.empty() ? scenario_file_ : goal->scenario_file;
        std::lock_guard<std::mutex> planLock(plan_mutex_);  // the whole sortie owns the plan
        {
            std::lock_guard<std::mutex> lk(status_mutex_);
            have_status_ = false;
        }
        std::string err;
        if (!planInitialLocked(scenario, runId, goal->start_mission, &err)) {
            {
                std::lock_guard<std::mutex> lk(status_mutex_);
                flying_ = false;
                recording_ = false;
            }
            result->success = false;
            result->message = "planning failed: " + err;
            RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
            gh->abort(result);
            return;
        }
        if (goal->start_mission) flown_run_ids_.insert(runId);
        writeRunFiles(true);
        result->run_id = runId;
        result->plan_id = plan_id_;
        result->run_dir = runDir();
        result->planned_length_m = horizon_->totalArc();
        result->cells_planned = static_cast<int32_t>(ts::servicedCells(*scenario_, *horizon_).size());

        if (!goal->start_mission) {
            result->success = true;
            result->message = "plan published (start_mission=false: dry run, nothing flown)";
            busy_.store(false);
            gh->succeed(result);
            return;
        }
        struct StopFlying { TigrisSearchPlannerNode* n; ~StopFlying() {
            std::lock_guard<std::mutex> lk(n->status_mutex_); n->flying_ = false; n->recording_ = false; } }
            stop_flying{this};
        const rclcpp::Time t0 = now();
        rclcpp::Time lastReplan = t0;
        const double timeout = mission_timeout_pad_s_ +
                               mission_timeout_factor_ * horizon_->budget() / std::max(scenario_->speed, 0.1);
        RCLCPP_INFO(get_logger(), "sortie %s started (run dir %s, timeout %.0f s, %s)", plan_id_.c_str(),
                    result->run_dir.c_str(), timeout, receding_ ? "receding horizon" : "one-shot");
        bool finished = false;
        while (rclcpp::ok()) {
            std::this_thread::sleep_for(200ms);
            if (gh->is_canceling()) {
                abort_pub_->publish(std_msgs::msg::Empty());
                result->success = false;
                result->message = "canceled";
                writeRunFiles(false);
                gh->canceled(result);
                return;
            }
            FollowerStatus st;
            bool have = false;
            double age = 0.0;
            {
                std::lock_guard<std::mutex> lk(status_mutex_);
                feedback->current_position = last_odom_;
                result->flown_length_m = flown_m_;
                have = have_status_ && last_status_.plan_id == plan_id_;
                if (have) {
                    st = last_status_;
                    age = (now() - last_status_time_).seconds();
                }
            }
            const double elapsed = (now() - t0).seconds();
            if (!have) {
                if (elapsed > status_timeout_s_) {
                    result->success = false;
                    result->message = "mtl_trajectory_follower never acknowledged the plan (no "
                                      "search/follower_status for " + plan_id_ + ")";
                    abort_pub_->publish(std_msgs::msg::Empty());
                    gh->abort(result);
                    return;
                }
                continue;
            }
            feedback->phase = st.state_name;
            feedback->progress_m = st.progress_m;
            feedback->remaining_m = st.remaining_m;
            feedback->progress = st.total_m > 0.0 ? static_cast<float>(st.progress_m / st.total_m) : 0.0f;
            feedback->cross_track_error_m = st.cross_track_error_m;
            gh->publish_feedback(feedback);
            result->duration_s = elapsed;

            if (st.state == FollowerStatus::COMPLETE) {
                finished = true;
                result->success = true;
                result->message = "sortie complete";
                break;
            }
            if (st.state == FollowerStatus::ABORTED) {
                result->success = false;
                result->message = "follower aborted the sortie";
                writeRunFiles(false);
                gh->abort(result);
                return;
            }
            if (age > status_timeout_s_ || elapsed > timeout) {
                abort_pub_->publish(std_msgs::msg::Empty());
                result->success = false;
                result->message = age > status_timeout_s_ ? "follower status went silent" : "sortie timed out";
                writeRunFiles(false);
                gh->abort(result);
                return;
            }
            // ---- receding horizon: replan while the follower is on the track
            const bool onTrack = st.state == FollowerStatus::SEARCH || st.state == FollowerStatus::INGRESS;
            const double since = (now() - lastReplan).seconds();
            if (onTrack && horizon_->due(st.progress_m, since)) {
                std::vector<ts::Look> looks;
                {
                    std::lock_guard<std::mutex> lk(status_mutex_);
                    looks.swap(flown_looks_);
                }
                const std::string trigger = since >= replan_period_s_ ? "period" : "horizon";
                const bool changed = horizon_->replan(st.progress_m, looks, trigger);
                lastReplan = now();
                const auto& r = horizon_->records().back();
                RCLCPP_INFO(get_logger(),
                            "replan %d [%s] at %.0f m (commit %.0f m, %.0f m left): %s %.0f m in %.2f s, "
                            "%d it, tree %d, %zu flown looks, residual %.4f",
                            r.index, trigger.c_str(), r.progressArc, r.commitArc, r.budgetLeft,
                            changed ? "new segment" : "kept track", r.segmentLength, r.seconds, r.iterations,
                            r.treeSize, looks.size(), r.residualMassFlown);
                if (changed) {
                    publishPlan();
                    result->planned_length_m = horizon_->totalArc();
                } else {
                    publishTigrisStatus();
                }
                writeRunFiles(false);
            }
        }
        if (finished) {
            // fold the last flown looks in so tigris_replans.json ends with the full belief
            std::vector<ts::Look> looks;
            {
                std::lock_guard<std::mutex> lk(status_mutex_);
                looks.swap(flown_looks_);
            }
            horizon_->absorb(looks);
            writeRunFiles(false);
            result->planned_length_m = horizon_->totalArc();
            result->cells_planned = static_cast<int32_t>(ts::servicedCells(*scenario_, *horizon_).size());
            busy_.store(false);  // a client may send the next goal as soon as it has the result
            gh->succeed(result);
            RCLCPP_INFO(get_logger(), "sortie %s complete in %.0f s: track %.0f m, %zu solves, revision %d",
                        plan_id_.c_str(), result->duration_s, horizon_->totalArc(), horizon_->records().size(),
                        horizon_->revision());
        }
    }

    // parameters
    std::string scenario_file_, agent_name_, frame_id_, runs_root_, reward_mode_, sampler_;
    double home_up_m_ = 0.0, status_timeout_s_ = 10.0, mission_timeout_pad_s_ = 240.0;
    double mission_timeout_factor_ = 3.0, path_step_m_ = 2.0, start_yaw_deg_ = -999.0, look_period_s_ = 0.1;
    bool plan_on_startup_ = true, start_from_odometry_ = true, lock_gimbal_ = true, receding_ = true;
    double initial_planning_time_s_ = 5.0, planning_time_s_ = 5.0, replan_period_s_ = 5.0, commit_margin_s_ = 1.0;
    double lookahead_turn_radii_ = 1.2, min_replan_budget_m_ = 10.0;
    double extend_dist_m_ = 60.0, extend_radius_m_ = 20.0, prune_radius_m_ = 60.0, reward_step_m_ = 2.0;
    double grid_res_m_ = 4.0, view_point_goal_ = 0.6, bounds_margin_m_ = -1.0;
    bool use_entropy_ = true;
    double rs_ = 2.0, rf_ = 1.0, initial_confidence_ = 0.01, budget_m_ = -1.0;
    double camera_fov_deg_ = -1.0, camera_tilt_deg_ = -1.0;
    int max_iterations_ = 0, seed_ = -1;

    // plan state (startup preview thread or the single active goal thread, under plan_mutex_)
    std::unique_ptr<ts::Scenario> scenario_;
    std::unique_ptr<ts::RecedingHorizon> horizon_;
    int agent_idx_ = -1;
    std::string plan_id_, run_id_;
    bool start_mission_ = false;
    std::atomic<bool> busy_{false};
    std::atomic<bool> goal_seen_{false};
    std::set<std::string> flown_run_ids_;
    std::mutex plan_mutex_;

    // follower / odometry / gimbal (status_mutex_)
    std::mutex status_mutex_;
    FollowerStatus last_status_;
    rclcpp::Time last_status_time_;
    bool have_status_ = false;
    geometry_msgs::msg::Point last_odom_;
    double last_odom_yaw_ = 0.0;
    bool have_odom_ = false, flying_ = false, recording_ = false;
    double flown_m_ = 0.0;
    std::string active_plan_id_;
    geometry_msgs::msg::Vector3 gimbal_meas_, gimbal_cmd_;
    rclcpp::Time gimbal_meas_time_, last_look_time_;
    bool have_gimbal_meas_ = false, have_gimbal_cmd_ = false, have_look_time_ = false;
    double origin_[3] = {0.0, 0.0, 0.0};
    ts::DetectionModel look_det_;
    double look_fov_ = 1.0471975511965976;
    std::vector<ts::Look> flown_looks_;

    rclcpp::Publisher<mtl_msgs::msg::SearchPlan>::SharedPtr plan_pub_;
    rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr boresight_pub_, path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr abort_pub_;
    rclcpp::Subscription<FollowerStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gimbal_state_sub_, gimbal_cmd_sub_;
    rclcpp_action::Server<SearchMission>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TigrisSearchPlannerNode>();
    // Single-threaded like mtl_search_planner: the sortie runs in its own thread and
    // rclcpp_action servers can lose goals on a MultiThreadedExecutor.
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
