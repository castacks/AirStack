
#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include <math.h>
#include <map>
#include <utility>
#include <stdlib.h>
#include <limits>
#include <random>
#include <chrono>
#include <vector>

#include "utils/utils.hpp"
#include "utils/viewpoint.hpp"
#include "utils/collision_checker.h"

/*
// GRAEME redefined points in custom_point.h
typedef std::vector<double> ViewPoint;
typedef std::vector<ViewPoint> PointSet;

#define POINTSIZE  6
#define    NUMDOF  3
#define   NUMDOFY  4
#define   INFOIDX  4
#define  AWARDIDX  5
*/

// Define a Tree
struct Tree
{
    PointSet vertices;
    std::map<ViewPoint, ViewPoint> edges; // point, parent
    ViewPoint goal;

    void clear()
    {
        vertices.clear();
        edges.clear();
    }
    void insertPoint(const ViewPoint &new_point, const ViewPoint &parent)
    {
        vertices.push_back(new_point);
        edges[new_point] = parent;
    }
};

class RRT_Planner
{
public:
    RRT_Planner();
    bool RRT_Init( // const float              robot_model_cube_dim,
                   //  const openvdb::BoolGrid::Ptr&   grid_map,
                   //  const float               l_occ,
                   //  const float                         voxel_dim,
        const float norm_limit,
        const float max_explore_dist,
        const int max_iter,
        const int max_connection_iter,
        const int smooth_horizon,
        // const bool                    consider_unknown_vox_occupied,
        const int rrt_region_nodes_count,
        const float rrt_region_nodes_radius,
        const int rrt_region_nodes_z_layers_count_up,
        const float rrt_region_nodes_z_layers_step,
        const float dense_step);

    bool build_RRT(const ViewPoint &start_point, const ViewPoint &end_point,
                   collision_checker_ns::CollisionChecker &collision_checker, const bool reset_trees = true);
    PointSet getPath();
    PointSet getCoarsePath();
    PointSet getExecutingPath();
    // void updateGridPtr(const openvdb::BoolGrid::Ptr& grid);
    bool visualizeTrees();
    // void setRobotModelCubeDim(const float robot_model_cube_dim){kRobot_model_cube_dim_ = robot_model_cube_dim;}

    bool executingPathCollision(collision_checker_ns::CollisionChecker &collision_checker);
    ViewPoint getEdgeColl(const ViewPoint &p_new, const ViewPoint &p_near, collision_checker_ns::CollisionChecker &collision_checker);
    ViewPoint getPathCollision(collision_checker_ns::CollisionChecker &collision_checker);
    // Tree Structure
    Tree start_tree_;
    Tree end_tree_;

    /* ---------------------------------------------------------------------------- */
private:
    // Debugging publishers
    // ros::Publisher viz_tree_start_pub_;
    // ros::Publisher viz_tree_end_pub_;

    // RRT constants
    float dense_step_;
    float kNorm_limit_, kNorm_limit_sq_;
    float kMax_explore_dist_;
    // float kRobot_model_cube_dim_;
    // float kVoxel_dim_;
    int kMax_iter_;
    int kMax_connection_iter_;
    int kSmooth_horizon_;
    bool kConsider_unknown_vox_occupied_;
    int kRRT_region_nodes_count_;
    float kRRT_region_nodes_radius_;
    int kRRT_region_nodes_z_layers_count_up_;
    float kRRT_region_nodes_z_layers_step_;

    // Random number generator
    std::uniform_real_distribution<double> distribution_uniform_;
    std::default_random_engine generator_;

    // RRT variables
    ViewPoint start_point_, end_point_;
    ViewPoint last_point_;
    bool last_point_found_ = false;

    // Solution
    PointSet path_;
    PointSet coarse_path_;
    PointSet executing_path_;

    // Mapping variables
    // openvdb::BoolGrid::Ptr grid_map_ = NULL;
    // float kL_occ_;

    // Main functions
    ViewPoint randomPoint(ViewPoint goal);
    void interpolate(const ViewPoint &p1, const ViewPoint &p2, PointSet &path);
    void pathSmooth(PointSet &path, collision_checker_ns::CollisionChecker &collision_checker);
    ViewPoint nearestPoint(const ViewPoint &p_rand, const PointSet &vertices);
    void extend(const ViewPoint &p_rand, const ViewPoint &p_near, ViewPoint &p_new);
    bool validEdge(const ViewPoint &p_new, const ViewPoint &p_near, collision_checker_ns::CollisionChecker &collision_checker);
    bool validEdgeLoose(const ViewPoint &p_new, const ViewPoint &p_near, collision_checker_ns::CollisionChecker &collision_checker);
    bool extractPathFromGraph(collision_checker_ns::CollisionChecker &collision_checker);
    bool nearGoal(const ViewPoint &p_new, const ViewPoint &goal);
    void addRegionVertices(Tree &tree);
    // bool collisionCheckInFreeSpace(const ViewPoint& p);
    void swapTreePointers(Tree **a, Tree **b);
    void reversePath(PointSet &path);

    // ViewPoint auxillary functions
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

    static inline double normSq(const ViewPoint &p)
    {
        // use instead of norm for efficiency and when you don't need the sqrt
        return (p.x * p.x + p.y * p.y + p.z * p.z);
    }

    static inline ViewPoint multiply(const ViewPoint &p, double factor)
    {
        ViewPoint p_return;
        p_return.x = p.x * factor;
        p_return.y = p.y * factor;
        p_return.z = p.z * factor;
        return p_return;
    }

    static inline bool equal(const ViewPoint &p1, const ViewPoint &p2)
    {
        if (p1.x != p2.x)
        {
            return false;
        }
        if (p1.y != p2.y)
        {
            return false;
        }
        if (p1.z != p2.z)
        {
            return false;
        }
        return true;
    }
};

#endif