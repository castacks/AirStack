
#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include "utils/utils.hpp"
#include "utils/viewpoint.hpp"
#include "rclcpp/rclcpp.hpp"

#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_ros/transforms.h>

namespace collision_checker_ns
{
    // struct VirtualObstacle
    // {
    //     ViewPoint point;
    //     float radius;
    //     rclcpp::Time expiry_time;
    // };

    class CollisionChecker
    {
    private:
        // Constants
        float kVoxSize_;
        float kBboxLength_, kBboxBreadth_, kBboxHeight_;
        float kL_occ_, kUnknown_fraction_thresh_, kOccupied_fraction_thresh_;
        // bool kConsider_unknown_vox_occupied_;
        // float kVirtual_obstacle_radius_, kVirtual_obstacle_expiry_time_;
        // float kRobot_pos_clear_radius_;
        // bool kEnable_gate_obstacle_;
        // float kGate_cube_size_, kGate_wall_size_;
        // std::string shared_frame_tf_topic_;

        // Stats counters
        int count_hits_ = 0, count_misses_ = 0;

        openvdb::BoolGrid::Ptr collision_grid_ = NULL;
        openvdb::FloatGrid::Ptr grid_map_ = NULL;

        // ros::Publisher debug_map_pub_, debug_virtual_obstacles_pub_, debug_virtual_gate_inner_pub_, debug_virtual_gate_outer_pub_;
        // pcl::PointCloud<pcl::PointXYZI>::Ptr debug_map_;
        // pcl::PointCloud<pcl::PointXYZI>::Ptr debug_virtual_obstacles_;
        // pcl::PointCloud<pcl::PointXYZI>::Ptr debug_gate_obstacles_;

        // openvdb::math::Transform::Ptr darpa_vdb_tf_;
        // tf::Transform darpa_tf_;
        // bool darpa_tf_set_ = false;

        // openvdb::BoolGrid::Ptr gate_obstacle_inner_ = NULL;
        // openvdb::BoolGrid::Ptr gate_obstacle_outer_ = NULL;
        // bool inner_gate_obstacle_enabled_;

        // std::vector<VirtualObstacle> virtual_obstacles_;

        // ViewPoint robot_pos_;
        // bool robot_pos_set_ = false;

        void initGrids();
        openvdb::Vec3d point2vdb(const ViewPoint &p_in);
        void clearCache();
        // bool isNearOdometry(const ViewPoint &p);
        // bool isInGateObstacle(const ViewPoint &p);
        // void checkOdomInInnerGateObstacle();

        static inline ViewPoint subtract(const ViewPoint &p1, const ViewPoint &p2)
        {
            ViewPoint p_return;
            p_return.x = p1.x - p2.x;
            p_return.y = p1.y - p2.y;
            p_return.z = p1.z - p2.z;
            return p_return;
        }

        static inline ViewPoint addPoint(const ViewPoint &p1, const ViewPoint &p2)
        {
            ViewPoint p_return;
            p_return.x = p1.x + p2.x;
            p_return.y = p1.y + p2.y;
            p_return.z = p1.z + p2.z;
            return p_return;
        }

        static inline ViewPoint divide(const ViewPoint &p, double factor)
        {
            ViewPoint p_return;
            p_return.x = p.x / factor;
            p_return.y = p.y / factor;
            p_return.z = p.z / factor;
            return p_return;
        }

        static inline double norm(const ViewPoint &p)
        {
            return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        }

        static inline ViewPoint multiply(const ViewPoint &p, double factor)
        {
            ViewPoint p_return;
            p_return.x = p.x * factor;
            p_return.y = p.y * factor;
            p_return.z = p.z * factor;
            return p_return;
        }

    public:
        CollisionChecker();

        void init(const float kVoxSize,
                  const float length,
                  const float breadth,
                  const float height,
                  const float l_occ,
                  const float unknown_fraction_thresh,
                  const float occupied_fraction_thresh);

        // void callbackDarpaTF(const nav_msgs::Odometry::ConstPtr &odom_msg);
        // void markStartGate();

        void updateGridPtr(const openvdb::FloatGrid::Ptr &grid);
        void setOdometryForVirtualObstacles(const ViewPoint &robot_pos);
        
        bool estimateSurfaceNormalAtHit(const openvdb::Vec3d &hit_point_world, double inflation_factor, openvdb::Vec3d &normal_out);
        bool collisionCheckRayStrict(const ViewPoint &p_start, const ViewPoint &p_end, openvdb::Vec3d& hit_point);
        bool collisionCheckInFreeSpace(const ViewPoint &p);
        bool collisionCheckInFreeSpaceVector(const ViewPoint &p_start, const ViewPoint &p_end);
        bool collisionCheckInFreeSpaceVectorStepSize(const ViewPoint &p_start, const ViewPoint &p_end, const float step_size);

        // void publishCollisionCheckerViz();
        // void publishGateOuterObstacleViz();
        // void publishGateInnerObstacleViz();

        // void addVirtualObstacles(const PointSet &obstacle_locations);
        // void clearExpiredVirtualObstacles();
        // bool isInVirtualObstacle(const ViewPoint &p);
        // void publishVirtualObstacleViz();

        // bool gate_obstacle_created_ = false;

    }; // class CollisionChecker

} // namespace collision_checker_ns

#endif