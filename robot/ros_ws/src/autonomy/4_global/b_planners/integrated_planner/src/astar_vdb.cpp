#include "../include/astar_vdb.hpp"

Astar::Astar()
{
}

Astar::~Astar()
{
    for (int i = 0; i < allocate_num_; i++)
    {
        delete path_node_pool_[i];
    }
}

void Astar::initialize(std::shared_ptr<VDBMap> &map_manager)
{
    // map for safety query
    map_manager_ = map_manager;

    allocate_num_ = 100000;
    tie_breaker_ = 1.0 + 1.0 / 1000;
    lambda_heu_ = 1.0;
    max_search_time_ = 0.05;
    margin_ = 0.0;

    safe_sq_index_dist_ = 16;
    safe_index_dist_ = 4;

    num_inflate_check_normal_ = 4;
    normal_step_delta_ = 0.2;
    L_THRESH = 0.0;

    path_node_pool_.resize(allocate_num_);

    for (int i = 0; i < allocate_num_; i++)
    {
        path_node_pool_[i] = new AstarNode;
    }
    use_node_num_ = 0;
    iter_num_ = 0;
    early_terminate_cost_ = 0.0;
}

void Astar::reset()
{
    open_set_map_.clear();
    close_set_.clear();
    path_nodes_.clear();

    std::priority_queue<AstarNode *, std::vector<AstarNode *>, NodeComparator0> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; ++i)
    {
        path_node_pool_[i]->parent = nullptr;
        path_node_pool_[i]->g_score = 0.0;
        path_node_pool_[i]->f_score = 0.0;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
    early_terminate_cost_ = 0.0;
}

std::vector<openvdb::Vec3d> Astar::getPathAstar()
{
    const openvdb::math::Transform::ConstPtr tf = map_manager_->get_grid_transform();
    std::vector<openvdb::Vec3d> path_points;
    for (auto &path_coord : path_nodes_)
    {
        path_points.push_back(tf->indexToWorld(path_coord));
    }
    return path_points;
}

double Astar::getDiagHeu(const openvdb::Coord &x1,
                         const openvdb::Coord &x2)
{
    double dx = std::abs(double(x1.x() - x2.x()));
    double dy = std::abs(double(x1.y() - x2.y()));
    double dz = std::abs(double(x1.z() - x2.z()));

    double h = 0.0;
    double diag = std::min(dx, std::min(dy, dz));

    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx < 1e-4)
    {
        double d2 = std::min(dy, dz);
        double d1 = std::abs(dy - dz);
        h = std::sqrt(3.0) * diag + std::sqrt(2.0) * d2 + d1;
    }
    else if (dy < 1e-4)
    {
        double d2 = std::min(dx, dz);
        double d1 = std::abs(dx - dz);
        h = std::sqrt(3.0) * diag + std::sqrt(2.0) * d2 + d1;
    }
    else if (dz < 1e-4)
    {
        double d2 = std::min(dx, dy);
        double d1 = std::abs(dx - dy);
        h = std::sqrt(3.0) * diag + std::sqrt(2.0) * d2 + d1;
    }

    return tie_breaker_ * h;
}

double Astar::getManhHeu(const openvdb::Coord &x1,
                         const openvdb::Coord &x2)
{
    double dx = std::abs(double(x1.x() - x2.x()));
    double dy = std::abs(double(x1.y() - x2.y()));
    double dz = std::abs(double(x1.z() - x2.z()));
    return tie_breaker_ * (dx + dy + dz);
}

double Astar::getEuclHeu(const openvdb::Coord &x1,
                         const openvdb::Coord &x2)
{
    double dx = double(x1.x() - x2.x());
    double dy = double(x1.y() - x2.y());
    double dz = double(x1.z() - x2.z());
    return tie_breaker_ * std::sqrt(dx * dx + dy * dy + dz * dz);
}

int Astar::search(const openvdb::Coord &start_ijk,
                  const openvdb::Coord &end_ijk)
{
    reset();

    if (!map_manager_)
    {
        std::cerr << "[AstarVDB] map_manager_ is null!" << std::endl;
        return NO_PATH;
    }

    if (allocate_num_ <= 0)
    {
        std::cerr << "[AstarVDB] allocate_num_ <= 0, did you call initialize()?" << std::endl;
        return NO_PATH;
    }

    AstarNode *start_node = path_node_pool_[0];
    start_node->ijk = start_ijk;
    start_node->g_score = 0.0;
    start_node->f_score = lambda_heu_ * getDiagHeu(start_ijk, end_ijk);
    start_node->parent = nullptr;

    open_set_.push(start_node);
    open_set_map_.insert(std::make_pair(start_ijk, start_node));
    use_node_num_ = 1;

    auto t0 = std::chrono::steady_clock::now();

    // Search loop
    while (!open_set_.empty())
    {
        AstarNode *cur = open_set_.top();

        bool reach_end = (cur->ijk == end_ijk);
        if (reach_end)
        {
            backtrack(cur);
            return REACH_END;
        }

        if (max_search_time_ > 0.0)
        {
            auto dt = std::chrono::steady_clock::now() - t0;
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(dt).count();
            if (elapsed > max_search_time_)
            {
                early_terminate_cost_ =
                    cur->g_score + lambda_heu_ * getDiagHeu(cur->ijk, end_ijk);
                return NO_PATH;
            }
        }

        open_set_.pop();
        open_set_map_.erase(cur->ijk);
        close_set_.insert(cur->ijk);
        iter_num_++;

        // 26 neighbor
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dz = -1; dz <= 1; ++dz)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                    {
                        continue;
                    }

                    openvdb::Coord nbr_ijk(cur->ijk.x() + dx,
                                           cur->ijk.y() + dy,
                                           cur->ijk.z() + dz);

                    if (close_set_.find(nbr_ijk) != close_set_.end())
                    {
                        continue;
                    }

                    double sqdist = 0.0;
                    if (!map_manager_->query_sqdist_at_index(nbr_ijk, sqdist))
                    {
                        close_set_.insert(nbr_ijk);
                        continue;
                    }

                    if (sqdist <= safe_sq_index_dist_)
                    {
                        close_set_.insert(nbr_ijk);
                        continue;
                    }

                    double step_cost = std::sqrt(double(dx * dx + dy * dy + dz * dz));
                    double tentative_g = cur->g_score + step_cost;
                    double tentative_f = tentative_g + lambda_heu_ * getDiagHeu(nbr_ijk, end_ijk);

                    auto it = open_set_map_.find(nbr_ijk);
                    AstarNode *nbr_node = nullptr;

                    if (it == open_set_map_.end())
                    {
                        // Not in OPEN, build a new node (grab from pool)
                        if (use_node_num_ >= allocate_num_)
                        {
                            std::cerr << "[AstarVDB] run out of node pool." << std::endl;
                            return NO_PATH;
                        }

                        nbr_node = path_node_pool_[use_node_num_];
                        use_node_num_++;

                        nbr_node->ijk = nbr_ijk;
                        nbr_node->g_score = tentative_g;
                        nbr_node->f_score = tentative_f;
                        nbr_node->parent = cur;

                        open_set_.push(nbr_node);
                        open_set_map_.insert(std::make_pair(nbr_ijk, nbr_node));
                    }
                    else
                    {
                        // Already in OPEN, check if we have a smaller g
                        nbr_node = it->second;
                        if (tentative_g < nbr_node->g_score)
                        {
                            nbr_node->g_score = tentative_g;
                            nbr_node->f_score = tentative_f;
                            nbr_node->parent = cur;

                            open_set_.push(nbr_node);
                        }
                    }
                }
            }
        }
    }

    std::cerr << "[AstarVDB] open set empty, no path." << std::endl;
    return NO_PATH;
}

void Astar::backtrack(const AstarNode *end_node)
{
    path_nodes_.clear();
    const AstarNode *cur = end_node;
    while (cur != nullptr)
    {
        path_nodes_.push_back(cur->ijk);
        cur = cur->parent;
    }
    std::reverse(path_nodes_.begin(), path_nodes_.end());
}

void Astar::pathSmooth(std::vector<openvdb::Vec3d> &sparse_path_out)
{
    // straight line, no shortening
    if (path_nodes_.size() < 2)
    {
        return;
    }

    std::vector<openvdb::Coord> sparse_coord_out;
    const openvdb::math::Transform::ConstPtr tf = map_manager_->get_grid_transform();

    // original discretized path (copy)
    std::vector<openvdb::Coord> astar_path = path_nodes_;

    // Start with the first point
    openvdb::Coord anchor = astar_path.front();
    sparse_coord_out.push_back(anchor);

    // std::cout << "init add waypoint " << sparse_coord_out.size() - 1 << " at " << 0 << " position " << anchor.x << ", " << anchor.y << ", " << anchor.z << '\n';

    // Tunables
    const int max_pivot_steps = 5; // guard for very thick obstacles

    // Scan forward along astar_path, only appending when blocked (or at the very end)

    for (size_t i = 1; i < astar_path.size(); ++i)
    {
        openvdb::Coord hit_coord;
        // Try to see astar_path[i] from current anchor
        bool unblocked = map_manager_->ray_esdf_clear_index(anchor, astar_path[i], safe_index_dist_, hit_coord);

        if (unblocked)
        {
            // Visible, connect only if it's the last point on path
            if (i == astar_path.size() - 1)
            {
                sparse_coord_out.push_back(astar_path[i]);
            }
            continue;
        }

        // Blocked on astar_path[i]: try to insert a first-hit pivot
        bool pivot_inserted = false;

        // Estimate outward normal from occupancy around the hit
        openvdb::Vec3d n_world;
        openvdb::Vec3d hit_world = tf->indexToWorld(hit_coord);

        bool have_normal = estimateSurfaceNormalAtHit(hit_coord, n_world);
        if (have_normal)
        {
            // Linearly push along the normal until the local footprint is free
            double delta = 0;
            openvdb::Coord pivot;

            for (int step_count = 0; step_count < max_pivot_steps; step_count++)
            {
                delta += normal_step_delta_;
                pivot = openvdb::Coord::round(tf->worldToIndex(hit_world + n_world * delta));

                double pivot_dist;
                // Pivot free, try adding the new anchor
                if (map_manager_->query_sqdist_at_index(pivot, pivot_dist) && pivot_dist >= safe_sq_index_dist_)
                {
                    openvdb::Coord hit_coord_dumb;
                    if (map_manager_->ray_esdf_clear_index(anchor, pivot, safe_index_dist_, hit_coord_dumb) &&
                        map_manager_->ray_esdf_clear_index(pivot, astar_path[i], safe_index_dist_, hit_coord_dumb))
                    {
                        pivot_inserted = true;
                        // new anchor
                        anchor = pivot;
                        sparse_coord_out.push_back(anchor);

                        // Reach the end
                        if (i == astar_path.size() - 1)
                        {
                            sparse_coord_out.push_back(astar_path[i]);
                        }
                        break;
                    }
                }
            }
            if (!pivot_inserted)
            {
                std::cout << "coll point push failed to get a pivot \n";
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
            anchor = astar_path[i - 1];
            // new anchor
            sparse_coord_out.push_back(anchor);
            if (i == astar_path.size() - 1)
            {
                sparse_coord_out.push_back(astar_path[i]);
            }
        }
    }

    sparse_path_out.clear();
    for (auto &path_coord : sparse_coord_out)
    {
        sparse_path_out.push_back(tf->indexToWorld(path_coord)); // 体素中心 → world
    }
}

bool Astar::estimateSurfaceNormalAtHit(const openvdb::Coord &hit_coord,
                                       openvdb::Vec3d &normal_out)
{
    openvdb::math::CoordBBox search_bbox(hit_coord, hit_coord);
    search_bbox.expand(num_inflate_check_normal_);

    // Accessor & transform
    const openvdb::math::Transform::ConstPtr tf = map_manager_->get_grid_transform();

    // Accumulate centroid of OCCUPIED voxels (value > kL_occ_) inside the bbox.
    openvdb::Vec3d sum_world(0.0, 0.0, 0.0);
    std::size_t count = 0;

    double occ_thresh = L_THRESH;

    for (auto it = search_bbox.beginXYZ(); it; ++it)
    {
        const openvdb::Coord ijk = *it;

        float v = 0.0f;
        const bool active = map_manager_->query_log_odds_at_index(ijk, v);
        if (active && v >= occ_thresh)
        {
            // Use voxel center in world space
            sum_world += tf->indexToWorld(ijk);
            ++count;
        }
    }

    if (count == 0)
    {
        // No occupied mass found in the (inflated) neighborhood — cannot infer a normal.
        return false;
    }

    const openvdb::Vec3d hit_point_world = tf->indexToWorld(hit_coord);
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