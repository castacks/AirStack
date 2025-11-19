
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

    {
        kVoxSize_ = kVoxSize;
        kBboxLength_ = length;
        kBboxBreadth_ = breadth;
        kBboxHeight_ = height;
        kL_occ_ = l_occ;
        kUnknown_fraction_thresh_ = unknown_fraction_thresh;
        kOccupied_fraction_thresh_ = occupied_fraction_thresh;

        initGrids();
    }

    void CollisionChecker::initGrids()
    {
        // Grid used to cache collisions
        collision_grid_ = openvdb::BoolGrid::create(false);
        VDBUtil::setVoxelSize(*collision_grid_, kVoxSize_);
    }

    inline openvdb::Vec3d CollisionChecker::point2vdb(const ViewPoint &p_in)
    {
        openvdb::Vec3d p_out(p_in.x, p_in.y, p_in.z);
        return p_out;
    }

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

    bool CollisionChecker::collisionCheckRayStrict(const ViewPoint &p_start, const ViewPoint &p_end, openvdb::Vec3d &hit_point)
    {
        openvdb::Vec3d vec_start = point2vdb(p_start);
        openvdb::Vec3d vec_end = point2vdb(p_end);

        return VDBUtil::checkCollisionAlongRay(grid_map_, vec_start, vec_end,
                                               kBboxLength_, kBboxBreadth_, kBboxHeight_,
                                               kL_occ_, hit_point, 0, false);
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
            is_in_collision = !VDBUtil::collCheckPointInPartialFreeSpace(grid_map_, query_pt,
                                                                         kBboxLength_, kBboxBreadth_, kBboxHeight_,
                                                                         kL_occ_, kUnknown_fraction_thresh_, kOccupied_fraction_thresh_);

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

    bool CollisionChecker::estimateSurfaceNormalAtHit(const openvdb::Vec3d &hit_point_world,
                                                      double inflation_factor,
                                                      openvdb::Vec3d &normal_out)
    {
        if (!grid_map_ || grid_map_->empty())
        {
            return false;
        }

        // Clamp inflation to non-negative.
        const double infl = std::max(0.0, inflation_factor);

        // Build an inflated world-space AABB around the hit using your helper.
        // We inflate the robot bbox uniformly by (1 + infl).
        const float len = std::max<float>(kVoxSize_, kBboxLength_ * static_cast<float>(1.0 + infl));
        const float brd = std::max<float>(kVoxSize_, kBboxBreadth_ * static_cast<float>(1.0 + infl));
        const float hgt = std::max<float>(kVoxSize_, kBboxHeight_ * static_cast<float>(1.0 + infl));

        openvdb::math::CoordBBox search_bbox;
        VDBUtil::createCoordBBox(grid_map_, search_bbox, hit_point_world, len, brd, hgt);

        // Accessor & transform
        openvdb::FloatGrid::Accessor acc = grid_map_->getAccessor();
        const openvdb::math::Transform &tf = grid_map_->transform();

        // Accumulate centroid of OCCUPIED voxels (value > kL_occ_) inside the bbox.
        openvdb::Vec3d sum_world(0.0, 0.0, 0.0);
        std::size_t count = 0;

        for (auto it = search_bbox.beginXYZ(); it; ++it)
        {
            const openvdb::Coord ijk = *it;

            float v = 0.0f;
            const bool active = acc.probeValue(ijk, v);
            if (active && v > kL_occ_)
            {
                // Use voxel center in world space
                sum_world += tf.indexToWorld(ijk);
                ++count;
            }
        }

        if (count == 0)
        {
            // No occupied mass found in the (inflated) neighborhood — cannot infer a normal.
            return false;
        }

        const openvdb::Vec3d centroid_world = sum_world / static_cast<double>(count);

        // Outward direction: from obstacle mass centroid -> hit point.
        openvdb::Vec3d n = hit_point_world - centroid_world;
        const double nlen = n.length();
        if (nlen <= 1e-9)
        {
            // Degenerate (e.g., centroid == hit); no stable direction.
            return false;
        }

        normal_out = n / nlen; // unit vector, world coordinates
        return true;
    }

    CollisionChecker::CollisionChecker()
    {
    }

} // namespace collision_checker_ns