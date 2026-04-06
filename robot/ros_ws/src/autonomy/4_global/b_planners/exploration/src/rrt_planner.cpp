/*
 * RRT Connection Function for Subt Voxel Planner
 * Copyright (C) 2020 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu
 */

#include "utils/rrt_planner.h"

// #define PI 3.14159265358979323846 // already in custom_point.h

/* ---------------------------------------------------------------------------- */
RRT_Planner::RRT_Planner() {}

bool RRT_Planner::RRT_Init( // const float        robot_model_cube_dim,
                            //  const openvdb::BoolGrid::Ptr&   grid_map,
                            //  const float               l_occ,
                            //  const float        voxel_dim,
    const float norm_limit,
    const float max_explore_dist,
    const int max_iter,
    const int max_connection_iter,
    const int smooth_horizon,
    // const bool         consider_unknown_vox_occupied,
    const int rrt_region_nodes_count,
    const float rrt_region_nodes_radius,
    const int rrt_region_nodes_z_layers_count_up,
    const float rrt_region_nodes_z_layers_step,
    const float dense_step)
{
    // Initialization
    kSmooth_horizon_ = smooth_horizon;
    // kRobot_model_cube_dim_ = robot_model_cube_dim;
    kNorm_limit_ = norm_limit;
    dense_step_ = dense_step;
    kNorm_limit_sq_ = kNorm_limit_ * kNorm_limit_;
    // kVoxel_dim_ = voxel_dim;
    kMax_explore_dist_ = max_explore_dist;
    kMax_iter_ = max_iter;
    kMax_connection_iter_ = max_connection_iter;
    // kConsider_unknown_vox_occupied_ = consider_unknown_vox_occupied;

    // Clear graph
    start_tree_.clear();
    end_tree_.clear();

    last_point_found_ = false;

    // grid_map_ = openvdb::BoolGrid::Ptr(grid_map);
    // kL_occ_ = l_occ;

    kRRT_region_nodes_count_ = rrt_region_nodes_count;
    kRRT_region_nodes_radius_ = rrt_region_nodes_radius;
    kRRT_region_nodes_z_layers_count_up_ = rrt_region_nodes_z_layers_count_up;
    kRRT_region_nodes_z_layers_step_ = rrt_region_nodes_z_layers_step;

    // Set seed
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_ = std::default_random_engine(seed);
    distribution_uniform_ = std::uniform_real_distribution<double>(0.0, 1.0);

    // Debugging publishers
    // ros::NodeHandle nh;
    // viz_tree_start_pub_ = nh.advertise<sensor_msgs::PointCloud2>("viz_rrt_tree_start", 1);
    // viz_tree_end_pub_ = nh.advertise<sensor_msgs::PointCloud2>("viz_rrt_tree_end", 1);

    return true;
}

void RRT_Planner::addRegionVertices(Tree &tree)
{
    // Assumes edges has been initialised with one node
    // Add new nodes, without collision checking, around this point
    // These node form a cylinder

    if (kRRT_region_nodes_count_ > 0)
    {

        // Get the existing point
        const ViewPoint &point_center = tree.vertices[0];

        float two_PI_over_count = 2.0 * PI / (float)kRRT_region_nodes_count_;

        // Add new points on a circle around the center
        for (int i = 0; i < kRRT_region_nodes_count_; i++)
        {

            float theta = (float)i * two_PI_over_count;

            float x = point_center.x + kRRT_region_nodes_radius_ * cos(theta);
            float y = point_center.y + kRRT_region_nodes_radius_ * sin(theta);

            for (int j = -kRRT_region_nodes_z_layers_count_up_; j <= kRRT_region_nodes_z_layers_count_up_; j++)
            {
                // Create a new point for each z layer
                ViewPoint point_new;
                point_new.x = x;
                point_new.y = y;
                point_new.z = point_center.z + j * kRRT_region_nodes_z_layers_step_;

                // Ship it
                tree.insertPoint(point_new, point_center);
            }
        }
    }
}

bool RRT_Planner::build_RRT(const ViewPoint &start_point,
                            const ViewPoint &end_point, collision_checker_ns::CollisionChecker &collision_checker, const bool reset_trees)
{
    if (collision_checker.collisionCheckInFreeSpace(start_point))
    {
        std::cout << "Starting Point Not Free Enough \n";
    }

    if (collision_checker.collisionCheckInFreeSpace(end_point))
    {
        std::cout << "End Point Not Free Enough \n";
    }

    // Initialize the trees
    if (reset_trees)
    {
        start_tree_.clear();
        end_tree_.clear();
    }

    // Initialize flags
    last_point_found_ = false;
    bool is_path = false;
    bool is_trace_path = true;

    // Save start and end points in tree
    start_point_ = start_point;
    end_point_ = end_point;
    start_tree_.goal = end_point_;
    end_tree_.goal = start_point_;

    if (reset_trees)
    {
        start_tree_.vertices.push_back(start_point_);
        end_tree_.vertices.push_back(end_point_);

        // Add region vertices
        addRegionVertices(start_tree_);
        addRegionVertices(end_tree_);
    }

    // Intermediate points
    ViewPoint p_random;
    ViewPoint p_nearest, p_new, p_nearest_goal, p_new_goal;

    // Create pointers to the two trees so they can be efficiently swapped
    Tree *treeA = &start_tree_;
    Tree *treeB = &end_tree_;

    // Iterate until goal found or max_iterations
    for (int iteration = 0; iteration < kMax_iter_; ++iteration)
    {

        // Pick a new random point
        p_random = randomPoint(treeA->goal);

        // Get the closest point in graph to random point
        p_nearest = nearestPoint(p_random, treeA->vertices);

        // Extend towards random point
        extend(p_random, p_nearest, p_new);

        if (validEdge(p_new, p_nearest, collision_checker))
        {

            // Save the new vertex
            treeA->insertPoint(p_new, p_nearest);

            // Extend from goal, too
            p_nearest_goal = nearestPoint(p_new, treeB->vertices);
            extend(p_new, p_nearest_goal, p_new_goal); // direction; start; new
            ViewPoint p_new2;
            if (validEdge(p_new_goal, p_nearest_goal, collision_checker))
            {

                // Save the new vertex
                treeB->insertPoint(p_new_goal, p_nearest_goal);

                // Try to connect to the goal
                int connection_iters = 0;
                while (!nearGoal(p_new_goal, p_new) && connection_iters <= kMax_connection_iter_)
                {

                    // Extend towards goal
                    extend(p_new, p_new_goal, p_new2);

                    if (validEdge(p_new2, p_new_goal, collision_checker))
                    {

                        // Save the new vertex
                        treeB->insertPoint(p_new2, p_new_goal);
                        p_new_goal = p_new2;
                    }
                    else
                    {
                        break;
                    }

                    // Increment loop counter
                    connection_iters++;
                }
                if (connection_iters > kMax_connection_iter_)
                {
                    printPoint(p_new);
                    printPoint(p_new_goal);
                }
            }

            // Check termination condition, add edge between trees (sort of)
            if (nearGoal(p_new_goal, p_new))
            {
                // treeB->edges[p_new] = p_new_goal;
                treeB->insertPoint(p_new, p_new_goal);
                last_point_ = p_new;
                last_point_found_ = true;
                is_path = true;
                break;
            }
        }

        // Use the goal tree if it was better
        if (treeA->vertices.size() > treeB->vertices.size())
        {
            swapTreePointers(&treeA, &treeB);
        }
    }

    // Construct the path from the graph
    if (is_path)
    {
        path_.clear();
        coarse_path_.clear();
        is_trace_path = extractPathFromGraph(collision_checker);
        if (!is_trace_path)
        {
            path_.clear();
        }
    }

    return (is_path && is_trace_path);
}

// bool RRT_Planner::visualizeTrees()
// {
//     // Convert both trees to point cloud msgs
//     // So they can be viewed in rviz
//     // (just the vertices, not the edges)

//     // First tree
//     PCLCloudType pcl_cloud_start = pointSet2PCL(start_tree_.vertices);
//     sensor_msgs::PointCloud2Ptr pcl_cloud_start_ros(new sensor_msgs::PointCloud2);
//     pcl::toROSMsg(pcl_cloud_start, *pcl_cloud_start_ros);
//     pcl_cloud_start_ros->header.frame_id = "map";
//     pcl_cloud_start_ros->header.stamp = ros::Time::now();
//     viz_tree_start_pub_.publish(pcl_cloud_start_ros);

//     // Second tree
//     PCLCloudType pcl_cloud_end = pointSet2PCL(end_tree_.vertices);
//     sensor_msgs::PointCloud2Ptr pcl_cloud_end_ros(new sensor_msgs::PointCloud2);
//     pcl::toROSMsg(pcl_cloud_end, *pcl_cloud_end_ros);
//     pcl_cloud_end_ros->header.frame_id = "map";
//     pcl_cloud_end_ros->header.stamp = ros::Time::now();
//     viz_tree_end_pub_.publish(pcl_cloud_end_ros);
// }

/*bool RRT_Planner::collisionCheckInFreeSpace(const ViewPoint& p) {
  bool is_in_collision;
  // TODO - this conversion is unnecessary -> see second comment below
  pcl::PointXYZI query_pt = point2PCL(p);
  // LEGACY DEBT - TODO convert method to new point type
  if(grid_map_ == NULL) {
    // std::cout << "[RRT_Planner::collisionCheckInFreeSpace] Grid ptr within class is NULL - not initialized." << "\n";
    return false;
  }

  if(!vdbmap::VDBUtil::checkPointInFreeSpace(grid_map_, query_pt,
                                              kRobot_model_cube_dim_, kL_occ_,
                                              kConsider_unknown_vox_occupied_)) {
    is_in_collision = true;
    return is_in_collision;
  } else {
    is_in_collision = false;
    return is_in_collision;
  }
}*/

void RRT_Planner::swapTreePointers(Tree **a, Tree **b)
{

    // Swap the tree pointers
    Tree *temp_tree = *a;
    *a = *b;
    *b = temp_tree;
}

ViewPoint RRT_Planner::randomPoint(ViewPoint goal)
{

    // With some probability, select goal instead of random point
    double r = distribution_uniform_(generator_);
    if (r < 0.90)
    {

        // New random point
        ViewPoint p;
        float range, range_z;

        float dist = norm(subtract(end_point_, start_point_));

        if (r < 0.50)
        {
            // Sample more at closer range
            range = (dist + 3) * 1.5;
            range_z = (end_point_.z - start_point_.z + 3) * 1.5;
        }
        else
        {
            // Wider sample range
            range = (dist + 5) * 2.0;
            range_z = (end_point_.z - start_point_.z + 3) * 2.0;
        }

        p.x = (distribution_uniform_(generator_) - 0.5) * range + (start_point_.x + end_point_.x) / 2.0;
        p.y = (distribution_uniform_(generator_) - 0.5) * range + (start_point_.y + end_point_.y) / 2.0;
        p.z = (distribution_uniform_(generator_) - 0.5) * range_z + (start_point_.z + end_point_.z) / 2.0;

        return p;
    }
    else
    {

        // Select the goal instead
        return goal;
    }
}

void RRT_Planner::reversePath(PointSet &path)
{
    // Reverse the order of the trajectory, save in place

    std::reverse(path.begin(), path.end());
}

void RRT_Planner::extend(const ViewPoint &p_rand, const ViewPoint &p_near, ViewPoint &p_new)
{
    // Extend from p_near towards p_rand
    // Return the extended point p_new

    // Get unit vector in direction
    ViewPoint direction = subtract(p_rand, p_near);
    double diff = norm(direction);
    direction = divide(direction, diff);

    // Extend along unit vector, without going past p_rand
    ViewPoint dist_step = multiply(direction, fmin(kNorm_limit_, diff));

    // Get the new point
    p_new = addPoint(p_near, dist_step);
}

// bool RRT_Planner::validEdge(const ViewPoint &p_new, const ViewPoint &p_near, collision_checker_ns::CollisionChecker &collision_checker)
// {
//     // Collision checking at discrete points between the two points?

//     ViewPoint check_point = p_near;
//     ViewPoint direction = subtract(p_new, p_near);
//     double diff = norm(direction);
//     direction = divide(direction, diff);
//     ViewPoint step = multiply(direction, double(kNorm_limit_)); // kVoxel_dim_

//     int num_steps = ceil(diff / kNorm_limit_); // kVoxel_dim_

//     // while(norm(subtract(check_point,p_near)) < diff) {
//     for (int i = 0; i < num_steps; i++)
//     {
//         check_point = addPoint(check_point, step);
//         if (collision_checker.collisionCheckInFreeSpace(check_point))
//         {
//             return false;
//         }
//     }
//     return true;
// }

bool RRT_Planner::validEdge(const ViewPoint &p_new, const ViewPoint &p_near, collision_checker_ns::CollisionChecker &collision_checker)
{
    openvdb::Vec3d hit_point;
    if (collision_checker.collisionCheckRayStrict(p_new, p_near, hit_point))
    {
        return false;
    }
    return true;
}

ViewPoint RRT_Planner::nearestPoint(const ViewPoint &p_rand, const PointSet &vertices)
{
    // Get the nearest vertex
    // TODO KD-tree?

    bool first = true;
    double min_norm_dist_sq = 0;
    ViewPoint p_near;

    // Iterate through the vertices
    for (const auto &vertex : vertices)
    {

        // Compute the distance
        double norm_dist_sq = normSq(subtract(vertex, p_rand));
        if (first || (norm_dist_sq < min_norm_dist_sq))
        {
            min_norm_dist_sq = norm_dist_sq;
            p_near = vertex;
            first = false;
        }
    }
    return p_near;
}

inline bool RRT_Planner::nearGoal(const ViewPoint &p_new, const ViewPoint &goal)
{
    // Check if p_new is sufficiently close to the goal

    ViewPoint diff = subtract(goal, p_new);
    double norm_dist_sq = normSq(diff);
    if (norm_dist_sq < kNorm_limit_sq_)
    {
        return true;
    }
    return false;
}

bool RRT_Planner::extractPathFromGraph(collision_checker_ns::CollisionChecker &collision_checker)
{
    // Extract path from graph

    bool is_trace_success = true;

    // Was the goal found?
    if (!last_point_found_)
    {

        // If not, stay stationary
        path_.push_back(start_point_);
    }
    else
    {

        // Found goal -- build path along graph

        // Initialize stuff
        ViewPoint current;
        PointSet path;
        current = last_point_;
        path.push_back(current);
        std::size_t n_start_edges = start_tree_.edges.size();

        // Construct a path
        if (start_tree_.edges.count(current))
        {

            bool solution_found = false;

            // Upper bound to avoid infinite loop
            for (int i = 0; i < 2 * n_start_edges; ++i)
            {

                // Find parent
                auto next_edge = start_tree_.edges.find(current);
                if (next_edge != start_tree_.edges.end())
                {

                    // Follow parent
                    current = next_edge->second;
                    path.push_back(current);
                }
                else
                {

                    // Reached root
                    solution_found = true;
                    break;
                }

                // This condition appears to check for infinite loops?
                // if (start_tree_.edges.find(parent) != start_tree_.edges.end() && equal(start_tree_.edges.find(parent)->second,current)) {
                //   is_trace_success = false;
                //   return false;
                // }else{
                // Follow parent
                // current = parent;
                // }
                // path.push_back(current);
            }
            if (!solution_found)
            {
                // Something has gone wrong
                is_trace_success = false;
                return false;
            }
        }
        else
        {
            // Something has gone wrong
            is_trace_success = false;
            return false;
        }

        // Do the same for end_edges

        // Initialize stuff
        ViewPoint current1;
        PointSet path1;
        current1 = last_point_;
        path1.push_back(current1);
        std::size_t n_end_edges = end_tree_.edges.size();

        // Construct a path
        if (end_tree_.edges.count(current1))
        {

            bool solution_found = false;

            // Upper bound to avoid infinite loop
            for (int i = 0; i < 2 * n_end_edges; ++i)
            {

                // Find parent
                auto next_edge = end_tree_.edges.find(current1);
                if (next_edge != end_tree_.edges.end())
                {

                    // Follow parent
                    current1 = next_edge->second;
                    path1.push_back(current1);
                }
                else
                {

                    // Reached root
                    solution_found = true;
                    break;
                }
            }
            if (!solution_found)
            {
                // Something has gone wrong
                is_trace_success = false;
                return false;
            }
        }
        else
        {
            // Something has gone wrong
            is_trace_success = false;
            return false;
        }

        // Stitch the two paths together
        if (equal(current, start_point_))
        {
            reversePath(path);
            path_.insert(path_.end(), path.begin(), path.end());
            path_.insert(path_.end(), path1.begin(), path1.end());
        }
        else
        {
            // This case should no longer happen (since trees don't get directly swapped any more)
            reversePath(path1);
            path_.insert(path_.end(), path1.begin(), path1.end());
            path_.insert(path_.end(), path.begin(), path.end());
        }
    }

    // Error checking
    int path_size = path_.size();
    if (!equal(path_[0], start_point_) || !equal(path_[path_size - 1], end_point_))
    {
        // Something has gone wrong
        is_trace_success = false;
        return false;
    }

    // copy coarse path
    coarse_path_ = path_;

    // Smooth the trajectory
    pathSmooth(path_, collision_checker);

    // Ship it!
    return is_trace_success;
}

PointSet RRT_Planner::getPath()
{
    return path_;
}

PointSet RRT_Planner::getCoarsePath()
{
    return coarse_path_;
}

void RRT_Planner::interpolate(const ViewPoint &p1, const ViewPoint &p2, PointSet &path)
{
    // Interpolate points between two point -- exclusive of p1 and p2

    ViewPoint check_point = p1;
    ViewPoint direction = subtract(p2, p1);
    double dist = norm(direction);
    direction = divide(direction, dist);
    ViewPoint step = multiply(direction, dense_step_);
    while (norm(subtract(check_point, p1)) < dist - dense_step_)
    {
        check_point = addPoint(check_point, step);
        path.push_back(check_point);
    }
}

// void RRT_Planner::pathSmooth(PointSet &path, collision_checker_ns::CollisionChecker &collision_checker)
// {
//     /* Interpolate points --> smooth path */
//     // DEBUG
//     // std::cout << "Smoothing Path ..."<<std::endl;
//     std::size_t num_path = path.size();
//     PointSet temp_path = path;
//     ViewPoint p_check, p_forward, cur_p, last_p;
//     std::size_t check_idx, forward_idx;
//     check_idx = 0;
//     path.clear();
//     while (check_idx < num_path - 1)
//     {
//         p_check = temp_path[check_idx];
//         forward_idx = check_idx + kSmooth_horizon_;
//         path.push_back(p_check);
//         while (forward_idx > check_idx)
//         {
//             forward_idx = int(fmin(forward_idx, num_path - 1));
//             p_forward = temp_path[forward_idx];
//             if (validEdge(p_forward, p_check, collision_checker))
//             {
//                 interpolate(p_check, p_forward, path);
//                 check_idx = forward_idx - 1;
//                 break;
//             }
//             forward_idx--;
//         }
//         last_p = temp_path[check_idx];
//         check_idx++;
//         cur_p = temp_path[check_idx];
//         if (normSq(subtract(cur_p, last_p)) > kNorm_limit_sq_)
//         {
//             interpolate(last_p, cur_p, path);
//         }
//     }
// }

void RRT_Planner::pathSmooth(PointSet &path,
                             collision_checker_ns::CollisionChecker &collision_checker)
{
    // straight line, no shortening
    if (path.size() < 2)
        return;

    // original discretized path (copy)
    PointSet Pr;
    Pr.push_back(path.front());
    for (size_t j = 0; j + 1 < path.size(); ++j)
    {
        if (equal(path[j], path[j+1])) continue;
        interpolate(path[j], path[j+1], Pr);
        Pr.push_back(path[j+1]);
    }

    path.clear();

    // Start with the first point
    ViewPoint anchor = Pr.front();
    path.push_back(anchor);

    // Helpers to bridge ViewPoint <-> VDB world coords
    auto fromVdb = [](const openvdb::Vec3d &v) -> ViewPoint
    {
        ViewPoint p;
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();
        return p;
    };

    // Tunables
    const double step_delta = std::max(1e-3, static_cast<double>(dense_step_)); // linear push step
    const int max_pivot_steps = 5; // guard for very thick obstacles
    const double normal_inflation = 1.0; // for estimateSurfaceNormalAtHit

    // Scan forward along Pr, only appending when blocked (or at the very end)

    for (size_t i = 1; i < Pr.size(); ++i)
    {
        // Try to see Pr[i] from current anchor
        openvdb::Vec3d hit_world;
        const bool blocked = collision_checker.collisionCheckRayStrict(anchor, Pr[i], hit_world);

        if (!blocked)
        {
            // Visible, connect only if it's the last point on path
            if (i == Pr.size() - 1)
            {
                interpolate(anchor, Pr[i], path);
                path.push_back(Pr[i]);
            }
            continue;
        }

        // Blocked on Pr[i]: try to insert a first-hit pivot
        bool pivot_inserted = false;

        // Estimate outward normal from occupancy around the hit
        openvdb::Vec3d n_world;
        bool have_normal = collision_checker.estimateSurfaceNormalAtHit(hit_world,
                                                                        normal_inflation,
                                                                        n_world);
        if (have_normal)
        {
            // Linearly push along the normal until the local footprint is free
            double delta = 0;
            ViewPoint pivot;

            for (int step_count=0; step_count < max_pivot_steps; step_count++)
            {
                delta += step_delta;
                pivot = fromVdb(hit_world + n_world * delta);
                if (!collision_checker.collisionCheckInFreeSpace(pivot))
                {
                    if (validEdge(anchor, pivot, collision_checker) and
                        validEdge(pivot, Pr[i], collision_checker))
                    {
                        pivot_inserted = true;
                        interpolate(anchor, pivot, path);
                        anchor = pivot;
                        // new anchor
                        path.push_back(anchor);
                        if (i == Pr.size() - 1)
                        {
                            interpolate(anchor, Pr[i], path);
                            path.push_back(Pr[i]);
                        }
                        break;
                    }
                }
            }
        }
        else
        {
            std::cout << "no normal at hit point " << hit_world.x() << ", " << hit_world.y() << ", " << hit_world.z() << '\n';
            pivot_inserted = false;
        }

        // No pivot added, then fast forward to the last visible point on path
        if (!pivot_inserted)
        {
            // Pivot failed (or became unnecessary) — append the last visible original waypoint
            interpolate(anchor, Pr[i-1], path);
            anchor = Pr[i-1];
            // new anchor
            path.push_back(anchor);
            if (i == Pr.size() - 1)
            {
                path.push_back(Pr[i]);
            }
        }
    }
}

// void RRT_Planner::updateGridPtr(const openvdb::BoolGrid::Ptr& grid) {
//   grid_map_ = grid;
// }
