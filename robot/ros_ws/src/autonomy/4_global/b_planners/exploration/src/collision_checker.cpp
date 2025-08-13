
#include "utils/collision_checker.h"

namespace collision_checker_ns
{

    void CollisionChecker::init(const float kVoxSize,
                                const float length,
                                const float breadth,
                                const float height,
                                const float l_occ,
                                const float unknown_fraction_thresh,
                                const float occupied_fraction_thresh)
                                // const bool consider_unknown_vox_occupied,
                                // const float virtual_obstacle_radius,
                                // const float virtual_obstacle_expiry_time,
                                // const float robot_pos_clear_radius,
                                // const bool enable_gate_obstacle,
                                // const float kVoxSizeGate,
                                // const float gate_cube_size,
                                // const float gate_wall_size)
    {
        kVoxSize_ = kVoxSize;
        kBboxLength_ = length;
        kBboxBreadth_ = breadth;
        kBboxHeight_ = height;
        kL_occ_ = l_occ;
        kUnknown_fraction_thresh_ = unknown_fraction_thresh;
        kOccupied_fraction_thresh_ = occupied_fraction_thresh;
        // kConsider_unknown_vox_occupied_ = consider_unknown_vox_occupied;
        // kVirtual_obstacle_radius_ = virtual_obstacle_radius;
        // kVirtual_obstacle_expiry_time_ = virtual_obstacle_expiry_time;
        // kRobot_pos_clear_radius_ = robot_pos_clear_radius;
        // kEnable_gate_obstacle_ = enable_gate_obstacle;
        // kVoxSizeGate_ = kVoxSizeGate;
        // kGate_cube_size_ = gate_cube_size;
        // kGate_wall_size_ = gate_wall_size;

        //   debug_map_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        //                                         new pcl::PointCloud<pcl::PointXYZI>()
        //                                         );
        //   debug_virtual_obstacles_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        //                                         new pcl::PointCloud<pcl::PointXYZI>()
        //                                         );
        //   debug_gate_obstacles_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        //                                         new pcl::PointCloud<pcl::PointXYZI>()
        //                                         );

        //   debug_map_pub_ = nh->advertise<sensor_msgs::PointCloud2>("collision_grid_debug", 5);
        //   debug_virtual_obstacles_pub_ = nh->advertise<sensor_msgs::PointCloud2>("virtual_obstacles_debug", 5);
        //   debug_virtual_gate_inner_pub_ = nh->advertise<sensor_msgs::PointCloud2>("gate_inner_obstacle_debug", 5);
        //   debug_virtual_gate_outer_pub_ = nh->advertise<sensor_msgs::PointCloud2>("gate_outer_obstacle_debug", 5);

        initGrids();
    }

    void CollisionChecker::initGrids()
    {
        // Grid used to cache collisions
        collision_grid_ = openvdb::BoolGrid::create(false);
        VDBUtil::setVoxelSize(*collision_grid_, kVoxSize_);

        // Grid used to clear out the start gate
        // if (kEnable_gate_obstacle_)
        // {
        //     gate_obstacle_inner_ = openvdb::BoolGrid::create(false);
        //     VDBUtil::setVoxelSize(*gate_obstacle_inner_, kVoxSizeGate_);

        //     gate_obstacle_outer_ = openvdb::BoolGrid::create(false);
        //     VDBUtil::setVoxelSize(*gate_obstacle_outer_, kVoxSizeGate_);
        // }
    }

    inline openvdb::Vec3d CollisionChecker::point2vdb(const ViewPoint &p_in)
    {
        openvdb::Vec3d p_out(p_in.x, p_in.y, p_in.z);
        return p_out;
    }

    // void CollisionChecker::markStartGate()
    // {

    //     // Extract the DARPA TF
    //     openvdb::math::Transform::Ptr map_tf = nullptr;

    //     // Use global darpa tf to create no-fly zone. Will create it at the DARPA origin.
    //     if (darpa_tf_set_)
    //     {
    //         ROS_INFO_ONCE("Using DARPA Transform for creating the no-fly zone at the DARPA origin. ");
    //         map_tf = darpa_vdb_tf_;
    //     }
    //     else
    //     {
    //         ROS_WARN_THROTTLE(1.0, "CollisionChecker::markStartGate(): ");
    //         ROS_WARN_THROTTLE(1.0, "Unable to create gate obstacle. DARPA Transform not available.");
    //         return;
    //     }

    //     // Create two temporary grids where cube will be placed
    //     openvdb::BoolGrid::Ptr tmp_grid_inner = openvdb::BoolGrid::create(false);
    //     openvdb::BoolGrid::Ptr tmp_grid_outer = openvdb::BoolGrid::create(false);

    //     // Set voxel size
    //     VDBUtil::setVoxelSize(*tmp_grid_inner, kVoxSizeGate_ / 2.0); // divide by 2 to ensure no gaps are left after TF
    //     VDBUtil::setVoxelSize(*tmp_grid_outer, kVoxSizeGate_ / 2.0); // divide by 2 to ensure no gaps are left after TF

    //     // Create bbox for Grid in robot frame
    //     double center_x_inner = -(kGate_cube_size_ / 2);
    //     openvdb::Vec3d bbox_center_inner{center_x_inner, 0.0, 0.0};
    //     openvdb::CoordBBox inner_bbox;
    //     VDBUtil::constructCoordBBox<openvdb::BoolGrid::Ptr>(tmp_grid_inner,
    //                                                                 inner_bbox,
    //                                                                 bbox_center_inner,
    //                                                                 kGate_cube_size_,
    //                                                                 kGate_cube_size_,
    //                                                                 kGate_cube_size_);

    //     double outer_dim = kGate_cube_size_ + 2.0 * kGate_wall_size_;
    //     double center_x_outer = -(outer_dim / 2);
    //     openvdb::Vec3d bbox_center_outer{center_x_outer, 0.0, 0.0};
    //     openvdb::CoordBBox outer_bbox;
    //     VDBUtil::constructCoordBBox<openvdb::BoolGrid::Ptr>(tmp_grid_outer,
    //                                                                 outer_bbox,
    //                                                                 bbox_center_outer,
    //                                                                 outer_dim,
    //                                                                 outer_dim,
    //                                                                 outer_dim);

    //     // Mark the obstacles

    //     // Mark outer obstacle
    //     // Cube with hollow center and missing side
    //     openvdb::BoolGrid::Accessor tmp_grid_acc_outer = tmp_grid_outer->getAccessor();
    //     for (auto iter = outer_bbox.begin(); iter; ++iter)
    //     {
    //         openvdb::Coord ijk = *iter;
    //         tmp_grid_acc_outer.setValueOn(ijk, true);
    //     }
    //     for (auto iter = inner_bbox.begin(); iter; ++iter)
    //     {
    //         openvdb::Coord ijk = *iter;
    //         tmp_grid_acc_outer.setValueOff(ijk, false);
    //     }

    //     // Mark inner obstacle
    //     // Smaller cube
    //     openvdb::BoolGrid::Accessor tmp_grid_acc_inner = tmp_grid_inner->getAccessor();
    //     for (auto iter = inner_bbox.begin(); iter; ++iter)
    //     {
    //         openvdb::Coord ijk = *iter;
    //         tmp_grid_acc_inner.setValueOn(ijk, true);
    //     }

    //     // Apply Darpa tf to grids
    //     auto tf_inner = tmp_grid_inner->transformPtr();
    //     tf_inner->postMult(map_tf->baseMap()->getAffineMap()->getMat4().inverse());

    //     auto tf_outer = tmp_grid_outer->transformPtr();
    //     tf_outer->postMult(map_tf->baseMap()->getAffineMap()->getMat4().inverse());

    //     // Copy the tmp grids into the actual grids without changing their transforms
    //     const openvdb::math::Transform &tmp_grid_inner_tf = tmp_grid_inner->transform();
    //     const openvdb::math::Transform &grid_inner_tf = gate_obstacle_inner_->transform();
    //     openvdb::BoolGrid::Accessor grid_inner_acc = gate_obstacle_inner_->getAccessor();
    //     for (auto iter = tmp_grid_inner->cbeginValueOn(); iter; ++iter)
    //     {
    //         openvdb::Coord ijk = iter.getCoord();
    //         openvdb::Vec3d xyz = tmp_grid_inner_tf.indexToWorld(ijk);
    //         openvdb::Coord ijk_copy = grid_inner_tf.worldToIndexCellCentered(xyz);
    //         grid_inner_acc.setValueOn(ijk_copy, iter.getValue());
    //         // ROS_WARN_STREAM("inner " << ijk_copy);
    //     }

    //     const openvdb::math::Transform &tmp_grid_outer_tf = tmp_grid_outer->transform();
    //     const openvdb::math::Transform &grid_outer_tf = gate_obstacle_outer_->transform();
    //     openvdb::BoolGrid::Accessor grid_outer_acc = gate_obstacle_outer_->getAccessor();
    //     for (auto iter = tmp_grid_outer->cbeginValueOn(); iter; ++iter)
    //     {
    //         openvdb::Coord ijk = iter.getCoord();
    //         openvdb::Vec3d xyz = tmp_grid_outer_tf.indexToWorld(ijk);
    //         openvdb::Coord ijk_copy = grid_outer_tf.worldToIndexCellCentered(xyz);
    //         grid_outer_acc.setValueOn(ijk_copy, iter.getValue());
    //         // ROS_WARN_STREAM("outer " << ijk_copy);
    //     }

    //     // Ship it, clear cache
    //     gate_obstacle_created_ = true;
    //     clearCache();
    // }

    // DARPA Transform to embed in the shared maps
    // void CollisionChecker::callbackDarpaTF(const nav_msgs::Odometry::ConstPtr &odom_msg)
    // {

    //     if (!darpa_tf_set_)
    //     {
    //         const openvdb::math::Vec3d offset(
    //             odom_msg->pose.pose.position.x,
    //             odom_msg->pose.pose.position.y,
    //             odom_msg->pose.pose.position.z);
    //         const openvdb::math::Quatd rot(
    //             odom_msg->pose.pose.orientation.x,
    //             odom_msg->pose.pose.orientation.y,
    //             odom_msg->pose.pose.orientation.z,
    //             odom_msg->pose.pose.orientation.w);
    //         const auto xyz = rot.eulerAngles(openvdb::math::XYZ_ROTATION);
    //         darpa_vdb_tf_ = openvdb::math::Transform::createLinearTransform(1.0);
    //         darpa_vdb_tf_->preRotate(xyz[0], openvdb::math::X_AXIS);
    //         darpa_vdb_tf_->preRotate(xyz[1], openvdb::math::Y_AXIS);
    //         darpa_vdb_tf_->preRotate(xyz[2], openvdb::math::Z_AXIS);
    //         darpa_vdb_tf_->postTranslate(offset);

    //         darpa_tf_.setOrigin(tf::Vector3(
    //             odom_msg->pose.pose.position.x,
    //             odom_msg->pose.pose.position.y,
    //             odom_msg->pose.pose.position.z));
    //         darpa_tf_.setRotation(tf::Quaternion(
    //             odom_msg->pose.pose.orientation.x,
    //             odom_msg->pose.pose.orientation.y,
    //             odom_msg->pose.pose.orientation.z,
    //             odom_msg->pose.pose.orientation.w));

    //         ROS_INFO_STREAM("Darpa transform received .. " << *darpa_vdb_tf_);
    //     }
    //     darpa_tf_set_ = true;
    // }

    // void CollisionChecker::publishCollisionCheckerViz() {

    //   debug_map_->clear();

    //   // Convert to PCL
    //   for (auto iter = collision_grid_->cbeginValueOn(); iter; ++iter) {
    //     Vec3d xyz = collision_grid_->indexToWorld(iter.getCoord());
    //     pcl::PointXYZI vox_in;
    //     vox_in.x = xyz.x();
    //     vox_in.y = xyz.y();
    //     vox_in.z = xyz.z();
    //     debug_map_->push_back(vox_in);
    //   }

    //   // Convert to msg
    //   sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
    //   pcl::toROSMsg(*debug_map_, *cloud_msg);
    //   cloud_msg->header.frame_id = "map";
    //   cloud_msg->header.stamp = ros::Time::now();

    //   // Ship it
    //   debug_map_pub_.publish(cloud_msg);
    // }

    // void CollisionChecker::publishGateOuterObstacleViz()
    // {

    //     debug_gate_obstacles_->clear();

    //     if (kEnable_gate_obstacle_ && gate_obstacle_created_)
    //     {

    //         // Convert to PCL
    //         for (auto iter = gate_obstacle_outer_->cbeginValueOn(); iter; ++iter)
    //         {
    //             Vec3d xyz = gate_obstacle_outer_->indexToWorld(iter.getCoord());
    //             pcl::PointXYZI vox_in;
    //             vox_in.x = xyz.x();
    //             vox_in.y = xyz.y();
    //             vox_in.z = xyz.z();
    //             debug_gate_obstacles_->push_back(vox_in);
    //         }

    //         // Convert to msg
    //         sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
    //         pcl::toROSMsg(*debug_gate_obstacles_, *cloud_msg);
    //         cloud_msg->header.frame_id = "map";
    //         cloud_msg->header.stamp = ros::Time::now();

    //         // Ship it
    //         debug_virtual_gate_outer_pub_.publish(cloud_msg);
    //     }
    //     else
    //     {

    //         // Empty msg
    //         sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
    //         cloud_msg->header.frame_id = "map";
    //         cloud_msg->header.stamp = ros::Time::now();

    //         // Ship it
    //         debug_virtual_gate_outer_pub_.publish(cloud_msg);
    //     }
    // }

    // void CollisionChecker::publishGateInnerObstacleViz()
    // {

    //     debug_gate_obstacles_->clear();

    //     if (kEnable_gate_obstacle_ && gate_obstacle_created_ && inner_gate_obstacle_enabled_)
    //     {

    //         // Convert to PCL
    //         for (auto iter = gate_obstacle_inner_->cbeginValueOn(); iter; ++iter)
    //         {
    //             Vec3d xyz = gate_obstacle_inner_->indexToWorld(iter.getCoord());
    //             pcl::PointXYZI vox_in;
    //             vox_in.x = xyz.x();
    //             vox_in.y = xyz.y();
    //             vox_in.z = xyz.z();
    //             debug_gate_obstacles_->push_back(vox_in);
    //         }

    //         // Convert to msg
    //         sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
    //         pcl::toROSMsg(*debug_gate_obstacles_, *cloud_msg);
    //         cloud_msg->header.frame_id = "map";
    //         cloud_msg->header.stamp = ros::Time::now();

    //         // Ship it
    //         debug_virtual_gate_inner_pub_.publish(cloud_msg);
    //     }
    //     else
    //     {

    //         // Empty msg
    //         sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
    //         cloud_msg->header.frame_id = "map";
    //         cloud_msg->header.stamp = ros::Time::now();

    //         // Ship it
    //         debug_virtual_gate_inner_pub_.publish(cloud_msg);
    //     }
    // }

    void CollisionChecker::updateGridPtr(const openvdb::FloatGrid::Ptr &grid)
    {
        grid_map_ = grid;
        // clearExpiredVirtualObstacles();
        clearCache();
    }

    void CollisionChecker::clearCache()
    {
        collision_grid_->clear();
        // ROS_INFO_STREAM("hits: " << count_hits_ << " misses: " << count_misses_);
    }

    bool CollisionChecker::collisionCheckInFreeSpace(const ViewPoint &p)
    {

        // Convert to PCL point
        Eigen::Vector3d query_pt = point2eigen(p);

        // Map does not exist
        if (grid_map_ == NULL)
        {
            return false;
        }

        // Setup cache accessor
        openvdb::BoolGrid::Accessor collision_grid_acc = collision_grid_->getAccessor();
        const openvdb::math::Transform &collision_grid_tf(collision_grid_->transform());
        
        openvdb::Vec3d cache_xyz = point2vdb(p);
        openvdb::Coord cache_ijk = collision_grid_tf.worldToIndexCellCentered(cache_xyz);

        // Check cache
        bool cache_hit = false;
        bool cache_value; // only valid if cache_hit=true
        if (!collision_grid_->empty())
        {
            cache_hit = collision_grid_acc.probeValue(cache_ijk, cache_value);
        }

        // Do collision checking using cache or explicit check
        bool is_in_collision;

        if (cache_hit)
        {
            // Cache hit -- use stored value
            count_hits_++;
            is_in_collision = cache_value;
        }
        else
        {

            // Cache miss -- do collision check
            count_misses_++;

            // First check start gate obstacle
            // bool is_in_start_gate = isInGateObstacle(p);

            // if (is_in_start_gate)
            // {

            //     is_in_collision = true;
            // }
            // else
            // {

                // Second check virtual obstacles
                // bool is_in_virtual_obstacle = isInVirtualObstacle(p);

                // if (is_in_virtual_obstacle)
                // {

                //     is_in_collision = true;
                // }
                // else
                // {

                    // Then check grid
                    is_in_collision = !VDBUtil::collCheckPointInPartialFreeSpace(grid_map_, query_pt,
                                                                                 kBboxLength_, kBboxBreadth_, kBboxHeight_,
                                                                                 kL_occ_, kUnknown_fraction_thresh_, kOccupied_fraction_thresh_);
                // }
            // }

            // Add result to cache for next time
            collision_grid_acc.setValueOn(cache_ijk, is_in_collision);
        }

        // Ship it!
        return is_in_collision;
    }

    bool CollisionChecker::collisionCheckInFreeSpaceVector(const ViewPoint &p_start, const ViewPoint &p_end)
    {

        // Map does not exist
        if (grid_map_ == NULL)
        {
            return false;
        }

        // Setup vector
        ViewPoint check_point = p_start;
        ViewPoint direction = subtract(p_end, p_start);
        double diff = norm(direction);
        direction = divide(direction, diff);
        ViewPoint step = multiply(direction, double(kVoxSize_));

        int num_steps = ceil(diff / kVoxSize_);

        // Step along the vector
        for (int i = 0; i < num_steps; i++)
        {
            // Get next point on vector
            check_point = addPoint(check_point, step);

            // Collision check
            if (collisionCheckInFreeSpace(check_point))
            {
                // Collision found
                return true;
            }
        }
        return false;
    }

    // same as collisionCheckInFreeSpaceVector but with a custom step size
    bool CollisionChecker::collisionCheckInFreeSpaceVectorStepSize(const ViewPoint &p_start, const ViewPoint &p_end, const float step_size)
    {

        // Map does not exist
        if (grid_map_ == NULL)
        {
            return false;
        }

        // Setup vector
        ViewPoint check_point = p_start;
        ViewPoint direction = subtract(p_end, p_start);
        double diff = norm(direction);
        direction = divide(direction, diff);
        ViewPoint step = multiply(direction, double(step_size));

        int num_steps = ceil(diff / step_size);

        // Step along the vector
        for (int i = 0; i < num_steps; i++)
        {

            // Get next point on vector
            check_point = addPoint(check_point, step);

            // Collision check
            if (collisionCheckInFreeSpace(check_point))
            {

                // Collision found
                return true;
            }
        }
        return false;
    }

    // void CollisionChecker::addVirtualObstacles(const PointSet &obstacle_locations)
    // {

    //     if (!obstacle_locations.empty())
    //     {

    //         ros::Duration duration(kVirtual_obstacle_expiry_time_);
    //         ros::Time expiry_time = ros::Time::now() + duration;

    //         // Append these obstacles
    //         for (const auto &loc : obstacle_locations)
    //         {

    //             // Create new obstacle
    //             VirtualObstacle tmp;
    //             virtual_obstacles_.push_back(tmp);
    //             VirtualObstacle &virtual_obstacle = virtual_obstacles_.back();

    //             // Add the data
    //             virtual_obstacle.point = loc;
    //             virtual_obstacle.radius = kVirtual_obstacle_radius_;
    //             virtual_obstacle.expiry_time = expiry_time;
    //         }

    //         // Clear cache since obstacles changed
    //         clearCache();
    //     }
    // }

    // void CollisionChecker::clearExpiredVirtualObstacles()
    // {

    //     ros::Time current_time = ros::Time::now();
    //     int count_removals = 0;

    //     // Iterate through the obstacles, remove any that have expired
    //     auto it = virtual_obstacles_.begin();
    //     while (it != virtual_obstacles_.end())
    //     {

    //         if (it->expiry_time <= current_time)
    //         {

    //             // Remove it
    //             it = virtual_obstacles_.erase(it);
    //             ++count_removals;
    //         }
    //         else
    //         {

    //             // Iterate
    //             ++it;
    //         }
    //     }

    //     // Clear cache if obstacles changed
    //     if (count_removals > 0)
    //     {
    //         clearCache();
    //     }
    // }

    // bool CollisionChecker::isNearOdometry(const ViewPoint &p)
    // {
    //     if (!robot_pos_set_)
    //     {
    //         return false;
    //     }

    //     double distance_to_odometry = distancePoint2Point(p, robot_pos_);
    //     if (distance_to_odometry <= kRobot_pos_clear_radius_)
    //     {
    //         return true;
    //     }
    // }

    // void CollisionChecker::setOdometryForVirtualObstacles(const ViewPoint &robot_pos)
    // {
    //     robot_pos_ = robot_pos;
    //     robot_pos_set_ = true;
    //     clearCache();
    //     checkOdomInInnerGateObstacle();
    // }

    // void CollisionChecker::checkOdomInInnerGateObstacle()
    // {
    //     if (kEnable_gate_obstacle_ && gate_obstacle_created_)
    //     {

    //         // Setup accessor
    //         openvdb::BoolGrid::Accessor grid_acc = gate_obstacle_inner_->getAccessor();
    //         const openvdb::math::Transform &grid_tf(gate_obstacle_inner_->transform());
    //         openvdb::Vec3d xyz = point2vdb(robot_pos_);
    //         openvdb::Coord ijk = grid_tf.worldToIndexCellCentered(xyz);

    //         // Check obstacle
    //         bool active = false;
    //         bool value;
    //         if (!gate_obstacle_inner_->empty())
    //         {
    //             active = grid_acc.probeValue(ijk, value);
    //         }
    //         if (active == true && value == true)
    //         {
    //             // Within obstacle
    //             inner_gate_obstacle_enabled_ = false;
    //         }
    //         else
    //         {
    //             // Not within obstacle
    //             inner_gate_obstacle_enabled_ = true;
    //         }
    //     }
    // }

    // bool CollisionChecker::isInVirtualObstacle(const ViewPoint &p)
    // {

    //     // Gate obstacle not used
    //     if (!kEnable_gate_obstacle_ || !robot_pos_set_)
    //     {
    //         return false;
    //     }

    //     // Ignore virtual obstacles if point is near current robot odometry
    //     if (isNearOdometry(p))
    //     {
    //         return false;
    //     }

    //     // Return true if the point is contained within any of the virtual obstacle spheres
    //     for (const auto &virtual_obstacle : virtual_obstacles_)
    //     {
    //         double distance = distancePoint2Point(p, virtual_obstacle.point);
    //         if (distance <= virtual_obstacle.radius)
    //         {
    //             // Is in sphere
    //             return true;
    //         }
    //     }

    //     return false;
    // }

    // bool CollisionChecker::isInGateObstacle(const ViewPoint &p)
    // {

    //     if (!gate_obstacle_created_)
    //     {
    //         return false;
    //     }

    //     // If odometry is not in gate obstacle
    //     if (inner_gate_obstacle_enabled_)
    //     {

    //         // Check if point is in inner gate obstacle

    //         // Setup accessor
    //         openvdb::BoolGrid::Accessor grid_acc = gate_obstacle_inner_->getAccessor();
    //         const openvdb::math::Transform &grid_tf(gate_obstacle_inner_->transform());
    //         openvdb::Vec3d xyz = point2vdb(p);
    //         openvdb::Coord ijk = grid_tf.worldToIndexCellCentered(xyz);

    //         // Check obstacle
    //         bool active = false;
    //         bool value;
    //         if (!gate_obstacle_inner_->empty())
    //         {
    //             active = grid_acc.probeValue(ijk, value);
    //         }
    //         if (active == true && value == true)
    //         {
    //             // Within obstacle
    //             return true;
    //         }
    //     }

    //     // Check if point is in outer gate obstacle

    //     // Setup accessor
    //     openvdb::BoolGrid::Accessor grid_acc = gate_obstacle_outer_->getAccessor();
    //     const openvdb::math::Transform &grid_tf(gate_obstacle_outer_->transform());
    //     openvdb::Vec3d xyz = point2vdb(p);
    //     openvdb::Coord ijk = grid_tf.worldToIndexCellCentered(xyz);

    //     // Check obstacle
    //     bool active = false;
    //     bool value;
    //     if (!gate_obstacle_outer_->empty())
    //     {
    //         active = grid_acc.probeValue(ijk, value);
    //     }
    //     if (active == true && value == true)
    //     {
    //         // Within obstacle
    //         return true;
    //     }

    //     // Not in the gate obstacle
    //     return false;
    // }

    // void CollisionChecker::publishVirtualObstacleViz()
    // {

    //     debug_virtual_obstacles_->clear();

    //     // Convert to PCL
    //     for (const auto &virtual_obstacle : virtual_obstacles_)
    //     {
    //         pcl::PointXYZI pcl_point;
    //         pcl_point.x = virtual_obstacle.point.x;
    //         pcl_point.y = virtual_obstacle.point.y;
    //         pcl_point.z = virtual_obstacle.point.z;
    //         debug_virtual_obstacles_->push_back(pcl_point);
    //     }

    //     // Convert to msg
    //     sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
    //     pcl::toROSMsg(*debug_virtual_obstacles_, *cloud_msg);
    //     cloud_msg->header.frame_id = "map";
    //     cloud_msg->header.stamp = ros::Time::now();

    //     // Ship it
    //     debug_virtual_obstacles_pub_.publish(cloud_msg);
    // }

    CollisionChecker::CollisionChecker()
    {
    }

} // namespace collision_checker_ns