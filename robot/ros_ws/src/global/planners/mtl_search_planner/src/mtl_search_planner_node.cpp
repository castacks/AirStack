// =============================================================================
//  mtl_search_planner_node — the MTL search planner as an AirStack global planner.
//
//  * Loads the scenario (stacks/mtl_search/config/scenario.json), solves the
//    WHOLE team problem with the vendored mtl::planner and keeps this robot's
//    agent row (matched by agent_name = $ROBOT_NAME). Every robot does the
//    same independently; the planner is deterministic, so the team agrees
//    without a cross-domain link.
//  * Serves the action  search_mission  (mtl_msgs/action/SearchMission).
//  * Publishes, latched (transient local):
//      search/plan                 mtl_msgs/SearchPlan (the atomic sortie)
//      search/planned_trajectory   airstack_msgs/TrajectoryXYZVYaw
//      search/planned_boresight    nav_msgs/Path  (scheduled ground points)
//      search/planned_path         nav_msgs/Path  (flight track)
//      search/markers              visualization_msgs/MarkerArray
//  * Relays search/follower_status as action feedback and finishes the goal
//    when the follower reports COMPLETE (or ABORTED).
//  * Writes runs/<run_id>/<agent>/{plan.json, track.json, scenario.json}.
//
//  All geometry is expressed in this robot's odometry frame ("map"), whose
//  origin is the robot's home: p_map = p_worldENU - home_ENU (see
//  mtl_search_planner/search_problem.hpp).
// =============================================================================
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <airstack_msgs/msg/trajectory_xyzv_yaw.hpp>
#include <airstack_msgs/msg/waypoint_xyzv_yaw.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mtl_msgs/action/search_mission.hpp>
#include <mtl_msgs/msg/follower_status.hpp>
#include <mtl_msgs/msg/search_plan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mtl_search_planner/search_problem.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace {

using SearchMission = mtl_msgs::action::SearchMission;
using GoalHandle    = rclcpp_action::ServerGoalHandle<SearchMission>;
using FollowerStatus = mtl_msgs::msg::FollowerStatus;

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

std::string utcStamp() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&t, &tm);
    std::ostringstream os;
    os << std::put_time(&tm, "%Y%m%d-%H%M%S");
    return os.str();
}

bool writeText(const fs::path& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f << text << "\n";
    return static_cast<bool>(f);
}

std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

}  // namespace

class MtlSearchPlannerNode : public rclcpp::Node {
public:
    MtlSearchPlannerNode() : Node("mtl_search_planner") {
        const char* robot = std::getenv("ROBOT_NAME");
        scenario_file_   = declare_parameter<std::string>(
            "scenario_file", "/root/AirStack/stacks/mtl_search/config/scenario.json");
        agent_name_      = declare_parameter<std::string>("agent_name", robot ? robot : "robot_1");
        frame_id_        = declare_parameter<std::string>("frame_id", "map");
        home_up_m_       = declare_parameter<double>("home_up_m", 0.0);
        runs_root_       = declare_parameter<std::string>("runs_root", "/root/AirStack/runs");
        plan_on_startup_ = declare_parameter<bool>("plan_on_startup", true);
        status_timeout_s_ = declare_parameter<double>("follower_status_timeout_s", 10.0);
        mission_timeout_pad_s_ = declare_parameter<double>("mission_timeout_pad_s", 240.0);
        mission_timeout_factor_ = declare_parameter<double>("mission_timeout_factor", 3.0);
        path_step_m_     = declare_parameter<double>("viz_path_step_m", 2.0);
        show_teammates_  = declare_parameter<bool>("viz_show_teammates", true);

        const auto latched = rclcpp::QoS(1).reliable().transient_local();
        plan_pub_      = create_publisher<mtl_msgs::msg::SearchPlan>("search/plan", latched);
        traj_pub_      = create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>("search/planned_trajectory", latched);
        boresight_pub_ = create_publisher<nav_msgs::msg::Path>("search/planned_boresight", latched);
        path_pub_      = create_publisher<nav_msgs::msg::Path>("search/planned_path", latched);
        markers_pub_   = create_publisher<visualization_msgs::msg::MarkerArray>("search/markers", latched);
        abort_pub_     = create_publisher<std_msgs::msg::Empty>("search/abort", 10);

        status_sub_ = create_subscription<FollowerStatus>(
            "search/follower_status", 10, [this](FollowerStatus::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lk(status_mutex_);
                last_status_ = *msg;
                last_status_time_ = now();
                have_status_ = true;
            });
        // Odometry: action feedback position + the measured flown length.
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odometry", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lk(status_mutex_);
                const auto& p = msg->pose.pose.position;
                if (have_odom_ && flying_) {
                    flown_m_ += std::hypot(std::hypot(p.x - last_odom_.x, p.y - last_odom_.y), p.z - last_odom_.z);
                }
                last_odom_ = p;
                have_odom_ = true;
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
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](const std::shared_ptr<GoalHandle>) {
                RCLCPP_INFO(get_logger(), "search_mission cancel requested");
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandle> gh) {
                busy_.store(true);
                std::thread([this, gh]() {
                    // Never let an exception escape a detached thread (std::terminate would
                    // take the whole node down and leave the client waiting forever).
                    try {
                        execute(gh);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(get_logger(), "search_mission execution failed: %s", e.what());
                        auto r = std::make_shared<SearchMission::Result>();
                        r->success = false;
                        r->message = std::string("internal error: ") + e.what();
                        try { gh->abort(r); } catch (...) {}
                        busy_.store(false);
                    }
                }).detach();
            });

        RCLCPP_INFO(get_logger(), "mtl_search_planner: agent '%s', scenario %s",
                    agent_name_.c_str(), scenario_file_.c_str());
        if (plan_on_startup_) {
            // Deferred one tick so the sim clock has a chance to arrive.
            startup_timer_ = create_wall_timer(1s, [this]() {
                startup_timer_->cancel();
                std::string err;
                if (!planAndPublish(scenario_file_, "", false, &err)) {
                    RCLCPP_ERROR(get_logger(), "startup preview plan failed: %s", err.c_str());
                }
            });
        }
    }

private:
    // --------------------------------------------------------------------- //
    bool planAndPublish(const std::string& scenarioPath, const std::string& runId, bool start,
                        std::string* err) {
        std::lock_guard<std::mutex> lk(plan_mutex_);
        try {
            const auto t0 = std::chrono::steady_clock::now();
            problem_ = mtl_search::loadScenario(scenarioPath);
            const int idx = problem_.agentIndex(agent_name_);
            if (idx < 0) {
                std::ostringstream os;
                os << "agent '" << agent_name_ << "' is not in the scenario team (";
                for (const auto& a : problem_.agents) os << a.name << " ";
                os << ") - regenerate the scenario for this fleet";
                throw std::runtime_error(os.str());
            }
            result_ = mtl_search::solve(problem_);
            track_  = mtl_search::buildAgentTrack(result_, problem_, idx, home_up_m_);
            const double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            if (track_.samples.size() < 2) {
                throw std::runtime_error("agent '" + agent_name_ + "' has a " +
                                         std::to_string(track_.samples.size()) +
                                         "-sample track: the budget grounded it");
            }
            plan_id_ = problem_.name + "/" + agent_name_ + "/" + (runId.empty() ? "preview" : runId);
            RCLCPP_INFO(get_logger(),
                        "planned %s in %.0f ms: %zu samples, %.0f m of %.0f m budget, %zu cells, "
                        "team info %.1f %%%s",
                        plan_id_.c_str(), ms, track_.samples.size(), track_.totalArc(), track_.budget,
                        track_.servicedCells.size(), 100.0 * result_.team.infoFraction,
                        track_.feasible ? "" : " [INFEASIBLE - best effort]");
            publishPlan(runId, start);
            return true;
        } catch (const std::exception& e) {
            if (err) *err = e.what();
            return false;
        }
    }

    void publishPlan(const std::string& runId, bool start) {
        const rclcpp::Time stamp = now();
        mtl_msgs::msg::SearchPlan msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id_;
        msg.plan_id = plan_id_;
        msg.run_id = runId;
        msg.scenario_name = problem_.name;
        msg.agent_name = track_.name;
        msg.agent_index = track_.index;
        msg.start_mission = start;
        msg.single_axis_gimbal = track_.singleAxis;
        msg.mount_tilt_rad = track_.tilt;
        msg.fov_rad = track_.fov;
        msg.speed_mps = track_.speedMps;
        msg.min_turn_radius_m = track_.minTurnRadius;
        msg.altitude_m = track_.altitude;
        msg.dt_s = track_.dt;
        msg.gimbal_max_rad = track_.gimbalMax;
        msg.gimbal_rate_rad_s = track_.gimbalRate;
        msg.pitch_nudge_max_rad = track_.pitchNudgeMax;
        msg.map_origin_in_world.x = track_.homeEnu.x();
        msg.map_origin_in_world.y = track_.homeEnu.y();
        msg.map_origin_in_world.z = track_.homeEnu.z();
        msg.trajectory.header = msg.header;
        msg.planned_length_m = track_.totalArc();
        msg.budget_m = track_.budget;

        nav_msgs::msg::Path path, bore;
        path.header = msg.header;
        bore.header = msg.header;
        double lastArc = -1e9;
        for (const auto& s : track_.samples) {
            airstack_msgs::msg::WaypointXYZVYaw wp;
            wp.position = point(s.x, s.y, s.z);
            wp.velocity = s.speed;
            wp.yaw = s.yaw;
            msg.trajectory.waypoints.push_back(wp);
            msg.boresight.push_back(point(s.bx, s.by, s.bz));
            msg.arc_length_m.push_back(s.arc);
            msg.time_s.push_back(s.t);
            msg.planned_gimbal_phi_rad.push_back(s.gimbalPhi);
            msg.planned_pitch_rad.push_back(s.pitch);
            if (s.arc - lastArc >= path_step_m_ || &s == &track_.samples.back()) {
                lastArc = s.arc;
                geometry_msgs::msg::PoseStamped ps;
                ps.header = msg.header;
                ps.pose.position = wp.position;
                ps.pose.orientation = yawQuat(s.yaw);
                path.poses.push_back(ps);
                geometry_msgs::msg::PoseStamped pb = ps;
                pb.pose.position = point(s.bx, s.by, s.bz);
                bore.poses.push_back(pb);
            }
        }
        for (const long c : track_.servicedCells) msg.serviced_cells.push_back(static_cast<int32_t>(c));

        plan_pub_->publish(msg);
        traj_pub_->publish(msg.trajectory);
        path_pub_->publish(path);
        boresight_pub_->publish(bore);
        markers_pub_->publish(buildMarkers(stamp));
    }

    visualization_msgs::msg::MarkerArray buildMarkers(const rclcpp::Time& stamp) const {
        visualization_msgs::msg::MarkerArray arr;
        const mtl::Vec3 home = track_.homeEnu;
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
        // search-area boundary
        {
            auto m = base("mtl_area", 0, visualization_msgs::msg::Marker::LINE_STRIP);
            m.scale.x = 0.8;
            m.color = rgba(1.0f, 0.85f, 0.1f, 0.9f);
            const double half = problem_.params.mapSize / 2.0;
            const double cn = problem_.frame.nMin + half, ce = problem_.frame.eMin + half;
            const double corners[5][2] = {{-1, -1}, {-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
            for (const auto& c : corners) {
                const mtl::Vec3 w = mtl_search::nedToEnu(cn + c[0] * half, ce + c[1] * half);
                m.points.push_back(point(w.x() - home.x(), w.y() - home.y(), 0.2 - home.z()));
            }
            arr.markers.push_back(m);
        }
        // every valid cell (grey) and the ones this agent is scheduled to observe (cyan)
        {
            auto all = base("mtl_cells", 0, visualization_msgs::msg::Marker::CUBE_LIST);
            auto mine = base("mtl_cells_scheduled", 0, visualization_msgs::msg::Marker::CUBE_LIST);
            const double cs = problem_.params.targetCellSize;
            all.scale.x = all.scale.y = cs * 0.9;
            all.scale.z = 0.1;
            mine.scale = all.scale;
            mine.scale.z = 0.2;
            all.color = rgba(0.7f, 0.7f, 0.7f, 0.25f);
            mine.color = rgba(0.2f, 0.85f, 1.0f, 0.45f);
            std::vector<bool> scheduled(static_cast<std::size_t>(result_.cells.size()), false);
            for (const long c : track_.servicedCells) {
                if (c >= 0 && static_cast<std::size_t>(c) < scheduled.size()) scheduled[static_cast<std::size_t>(c)] = true;
            }
            for (mtl::Index i = 0; i < result_.cells.size(); ++i) {
                double n = 0, e = 0;
                problem_.frame.fromMtl(result_.cells.centers(i, 0), result_.cells.centers(i, 1), n, e);
                const mtl::Vec3 w = mtl_search::nedToEnu(n, e);
                const auto p = point(w.x() - home.x(), w.y() - home.y(), 0.05 - home.z());
                (scheduled[static_cast<std::size_t>(i)] ? mine : all).points.push_back(p);
            }
            arr.markers.push_back(all);
            arr.markers.push_back(mine);
        }
        // teammates' planned tracks, expressed in THIS robot's map frame
        if (show_teammates_) {
            for (std::size_t a = 0; a < result_.trajectories.size(); ++a) {
                if (static_cast<int>(a) == track_.index) continue;
                auto m = base("mtl_teammates", static_cast<int>(a), visualization_msgs::msg::Marker::LINE_STRIP);
                m.scale.x = 0.5;
                m.color = rgba(0.9f, 0.9f, 0.9f, 0.5f);
                const auto& d = result_.trajectories[a].drone;
                for (mtl::Index k = 0; k < d.rows(); k += 20) {
                    double n = 0, e = 0;
                    problem_.frame.fromMtl(d(k, 0), d(k, 1), n, e);
                    const mtl::Vec3 w = mtl_search::nedToEnu(n, e, -d(k, 2));
                    m.points.push_back(point(w.x() - home.x(), w.y() - home.y(), w.z() - home.z()));
                }
                if (m.points.size() >= 2) arr.markers.push_back(m);
            }
        }
        return arr;
    }

    std::string prepareRunDir(const std::string& runId) {
        const fs::path root(runs_root_);
        const fs::path dir = root / runId / agent_name_;
        std::error_code ec;
        fs::create_directories(dir, ec);
        if (ec) {
            RCLCPP_WARN(get_logger(), "cannot create run dir %s: %s (is runs/ mounted?)",
                        dir.c_str(), ec.message().c_str());
            return dir.string();
        }
        if (!writeText(dir / "plan.json", mtl_search::teamPlanJson(result_, problem_)) ||
            !writeText(dir / "track.json", mtl_search::agentTrackJson(track_, problem_)) ||
            !writeText(dir / "scenario.json", problem_.raw.dump())) {
            RCLCPP_WARN(get_logger(), "could not write the plan files into %s", dir.c_str());
        }
        // runs/latest -> <run_id> (relative, so it resolves on the host too)
        const fs::path latest = root / "latest";
        fs::remove(latest, ec);
        fs::create_directory_symlink(fs::path(runId), latest, ec);
        return dir.string();
    }

    // --------------------------------------------------------------------- //
    void execute(const std::shared_ptr<GoalHandle> gh) {
        const auto goal = gh->get_goal();
        auto result = std::make_shared<SearchMission::Result>();
        auto feedback = std::make_shared<SearchMission::Feedback>();
        struct Release { std::atomic<bool>& b; ~Release() { b.store(false); } } release{busy_};

        feedback->phase = "PLANNING";
        gh->publish_feedback(feedback);

        const std::string runId = goal->run_id.empty() ? utcStamp() : goal->run_id;
        const std::string scenario = goal->scenario_file.empty() ? scenario_file_ : goal->scenario_file;
        {
            std::lock_guard<std::mutex> lk(status_mutex_);
            have_status_ = false;
        }
        std::string err;
        if (!planAndPublish(scenario, runId, goal->start_mission, &err)) {
            result->success = false;
            result->message = "planning failed: " + err;
            RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
            gh->abort(result);
            return;
        }
        result->run_id = runId;
        result->plan_id = plan_id_;
        result->run_dir = prepareRunDir(runId);
        result->planned_length_m = track_.totalArc();
        result->cells_planned = static_cast<int32_t>(track_.servicedCells.size());

        if (!goal->start_mission) {
            result->success = true;
            result->message = "plan published (start_mission=false: dry run, nothing flown)";
            gh->succeed(result);
            return;
        }

        {
            std::lock_guard<std::mutex> lk(status_mutex_);
            flown_m_ = 0.0;
            flying_ = true;
        }
        struct StopFlying { MtlSearchPlannerNode* n; ~StopFlying() {
            std::lock_guard<std::mutex> lk(n->status_mutex_); n->flying_ = false; } } stop_flying{this};
        const rclcpp::Time t0 = now();
        const double timeout = mission_timeout_pad_s_ + mission_timeout_factor_ * track_.flightTime;
        RCLCPP_INFO(get_logger(), "sortie %s started (run dir %s, timeout %.0f s)",
                    plan_id_.c_str(), result->run_dir.c_str(), timeout);
        while (rclcpp::ok()) {
            std::this_thread::sleep_for(500ms);
            if (gh->is_canceling()) {
                abort_pub_->publish(std_msgs::msg::Empty());
                result->success = false;
                result->message = "canceled";
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
                result->success = true;
                result->message = "sortie complete";
                gh->succeed(result);
                RCLCPP_INFO(get_logger(), "sortie %s complete in %.0f s", plan_id_.c_str(), elapsed);
                return;
            }
            if (st.state == FollowerStatus::ABORTED) {
                result->success = false;
                result->message = "follower aborted the sortie";
                gh->abort(result);
                return;
            }
            if (age > status_timeout_s_ || elapsed > timeout) {
                abort_pub_->publish(std_msgs::msg::Empty());
                result->success = false;
                result->message = age > status_timeout_s_ ? "follower status went silent"
                                                          : "sortie timed out";
                gh->abort(result);
                return;
            }
        }
    }

    // parameters
    std::string scenario_file_, agent_name_, frame_id_, runs_root_;
    double home_up_m_ = 0.0, status_timeout_s_ = 10.0, mission_timeout_pad_s_ = 240.0;
    double mission_timeout_factor_ = 3.0, path_step_m_ = 2.0;
    bool plan_on_startup_ = true, show_teammates_ = true;

    // plan state (written only by the startup timer or the single active goal thread)
    mtl_search::SearchProblem problem_;
    mtl::PlanningResult result_;
    mtl_search::AgentTrack track_;
    std::string plan_id_;
    std::atomic<bool> busy_{false};
    std::mutex plan_mutex_;

    // follower status
    std::mutex status_mutex_;
    FollowerStatus last_status_;
    rclcpp::Time last_status_time_;
    bool have_status_ = false;
    geometry_msgs::msg::Point last_odom_;
    bool have_odom_ = false, flying_ = false;
    double flown_m_ = 0.0;

    rclcpp::Publisher<mtl_msgs::msg::SearchPlan>::SharedPtr plan_pub_;
    rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr boresight_pub_, path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr abort_pub_;
    rclcpp::Subscription<FollowerStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Server<SearchMission>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MtlSearchPlannerNode>();
    // Single-threaded on purpose: the sortie runs in its own thread, every callback is
    // short, and rclcpp_action servers on a MultiThreadedExecutor can lose goal
    // requests (two threads race to take the same ready waitable) - which leaves the
    // client waiting on "Sending goal" forever.
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
