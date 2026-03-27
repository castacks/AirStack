#ifndef BPMP_TRACKER_H
#define BPMP_TRACKER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>  // (Yunwoo) Added for Odometry message
#include <nav_msgs/msg/path.hpp>  // (Yunwoo) Added for path
#include <geometry_msgs/msg/pose_stamped.hpp>  // (Yunwoo) Added for target info
#include <std_srvs/srv/trigger.hpp>  // (Yunwoo) Added for toggle service
#include <tf2/LinearMath/Quaternion.h>  // (Yunwoo) For tf2::Quaternion
#include <tf2/LinearMath/Matrix3x3.h>   // (Yunwoo) For tf2::Matrix3x3 and getRPY
#include "bpmp_visualizer/BPMPVisualizer.h"
#include "bpmp_utils/eigenmvn.h"
#include "bpmp_utils/bpmp_utils.h"
#include "decomp_util/ellipsoid_decomp.h"
#include <thread>
#include <airstack_msgs/msg/trajectory_xyzv_yaw.hpp>

namespace bpmp_tracker{
    // (Yunwoo) State machine modes
    enum class TrackerMode {
        WAITING_FOR_ODOMETRY = 0,  // Mode 0: robot_odometry not received
        TRACKING = 1,              // Mode 1: both target_info and robot_odometry received
        WAITING_FOR_TARGET = 2     // Mode 2: robot_odometry received, but target_info not received
    };

    struct PlanningParams{
        int replanning_time{1000}; // [ms]
    };
    class BPMPTracker : public rclcpp::Node{
        private:
        PlanningParams planning_params_;
        bpmp_tracker::VisualizationParams vis_param_;  // (Yunwoo) Added visualization params

        // (Yunwoo) State machine
        TrackerMode current_mode_{TrackerMode::WAITING_FOR_ODOMETRY};
        bool odometry_received_{false};
        bool target_info_received_{false};
        bool vdb_map_received_{false};
        bool tracking_enabled_{false};  // (Yunwoo) Toggle for tracking enable/disable
        void UpdateMode();  // Update state machine mode

        // PLANNING INGREDIENTS
        nav_msgs::msg::Odometry current_odometry_;  // (Yunwoo) Store current odometry
        geometry_msgs::msg::PoseStamped current_target_info_;  // (Yunwoo) Store current target info

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odometry_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_info_sub_;  // (Yunwoo) Target info subscriber
        rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr vdb_map_sub_;  // (Yunwoo) Pointcloud subscriber

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr raw_primitives_publisher_; // RAW PRIMITIVES PUBLISHER
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr feasible_primitives_publisher_; // FEASIBLE PRIMITIVES PUBLISHER
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr corridor_publisher_; // CORRIDOR PUBLISHER
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr best_primitive_publisher_; // BEST PRIMITIVE PUBLISHER
        rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr desired_traj_publisher_; // DESIRED TRAJECTORY PUBLISHER

        bpmp_tracker::BPMPVisualizer visualizer_; // VISUALIZER  
        rclcpp::TimerBase::SharedPtr timer_; // TIMER

        // (Yunwoo) Service server for target_tracking_toggle
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr target_tracking_toggle_srv_;
        void TargetTrackingToggleCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        vec_Vec3f point_cloud_3d_;
        vec_E<Polyhedron3D> polys_;
        vector<LinearConstraint3D> corridor_constraints_;
        int num_threads_{4};
        int num_samples_{1000};
        vector<bpmp_tracker::PrimitivePlanning> primitives_;
        void initialize();

        void UpdateVisualizationParams();
        void UpdatePlanningParams();
        void EgoOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);  // (Yunwoo) Odometry callback
        void TargetInfoCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);  // (Yunwoo) Target info callback
        void VDBMapCallback(const visualization_msgs::msg::Marker::SharedPtr msg);
        void GenerateCorridor();
        void Run();
        bool Replan();
        void Publish(const bool &success_flag);
        vector<bpmp_tracker::Point> end_points_;
        vector<bpmp_tracker::uint> safe_index_;
        bpmp_tracker::uint best_index_;
        void SampleEndPoint();
        void SampleEndPointThread(const int & start_idx, const int & end_idx, vector<bpmp_tracker::Point>& endpoint_list_sub);
        void GeneratePrimitive();
        void GeneratePrimitiveThread(const int &start_idx, const int &end_idx, vector<bpmp_tracker::PrimitivePlanning> & primitive_list_sub);
        void GetSafeIndex();
        void GetSafeIndexThread(const int &start_idx, const int &end_idx, vector<bpmp_tracker::uint> &safe_idx_sub);
        void GetBestIndex();
        void GetBestIndexThread(const int &start_idx, const int &end_idx, std::pair<uint,double> &score_pair);


        public:
        BPMPTracker();
        ~BPMPTracker();
        
    };
}
#endif  // BPMP_TRACKER_H