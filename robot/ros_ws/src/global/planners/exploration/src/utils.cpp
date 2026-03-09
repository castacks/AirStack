#include "utils/utils.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <rclcpp/rclcpp.hpp>

std_msgs::msg::ColorRGBA heightToColor(double norm_z)
{
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;

    double h = norm_z * 0.8;
    int i = static_cast<int>(h * 6.0);
    double f = (h * 6.0) - i;
    double q = 1.0 - f;

    switch (i % 6)
    {
    case 0:
        color.r = 1.0;
        color.g = f;
        color.b = 0.0;
        break;
    case 1:
        color.r = q;
        color.g = 1.0;
        color.b = 0.0;
        break;
    case 2:
        color.r = 0.0;
        color.g = 1.0;
        color.b = f;
        break;
    case 3:
        color.r = 0.0;
        color.g = q;
        color.b = 1.0;
        break;
    case 4:
        color.r = f;
        color.g = 0.0;
        color.b = 1.0;
        break;
    case 5:
        color.r = 1.0;
        color.g = 0.0;
        color.b = q;
        break;
    default:
        color.r = 1.0;
        color.g = 0.5;
        color.b = 0.5;
        break;
    }
    return color;
}

std_msgs::msg::ColorRGBA valueToRainbowColor(float value)
{
    std_msgs::msg::ColorRGBA color;
    value = std::fmod(value, 1.0f); // Wrap values > 1.0 around

    float r = std::fabs(value * 6 - 3) - 1;
    float g = 2 - std::fabs(value * 6 - 2);
    float b = 2 - std::fabs(value * 6 - 4);

    color.r = std::clamp(r, 0.0f, 1.0f);
    color.g = std::clamp(g, 0.0f, 1.0f);
    color.b = std::clamp(b, 0.0f, 1.0f);
    color.a = 1.0;
    return color;
}

// void generateVDBMarker(const GridT::Ptr& grid,
//                        const std::string& frame_id,
//                        visualization_msgs::msg::Marker& marker_msg)
// {
//     marker_msg.header.frame_id = frame_id;
//     marker_msg.header.stamp = rclcpp::Clock().now();
//     marker_msg.ns = "vdb_grid";
//     marker_msg.id = 0;
//     marker_msg.type = visualization_msgs::msg::Marker::CUBE_LIST;
//     marker_msg.action = visualization_msgs::msg::Marker::ADD;
//     marker_msg.pose.orientation.w = 1.0;
//     marker_msg.color.a = 1.0;
//     marker_msg.frame_locked = true;

//     double voxel_size = grid->voxelSize()[0];
//     marker_msg.scale.x = voxel_size;
//     marker_msg.scale.y = voxel_size;
//     marker_msg.scale.z = voxel_size;

//     openvdb::CoordBBox bbox = grid->evalActiveVoxelBoundingBox();
//     double z_min = grid->indexToWorld(bbox.min()).z();
//     double z_max = grid->indexToWorld(bbox.max()).z();
//     double dz = z_max - z_min;

//     for (auto iter = grid->cbeginValueOn(); iter; ++iter) {
//         openvdb::Vec3d world = grid->indexToWorld(iter.getCoord());

//         geometry_msgs::msg::Point pt;
//         pt.x = world.x();
//         pt.y = world.y();
//         pt.z = world.z();
//         marker_msg.points.push_back(pt);

//         double norm_z = (dz > 1e-3) ? (world.z() - z_min) / dz : 0.5;
//         marker_msg.colors.push_back(heightToColor(norm_z));
//     }

//     if (marker_msg.points.empty()) {
//         marker_msg.action = visualization_msgs::msg::Marker::DELETE;
//     }
// }

void generateVDBMarker(const openvdb::FloatGrid::Ptr &grid,
                       const std::string &frame_id,
                       visualization_msgs::msg::Marker &marker_msg)
{
    marker_msg.header.frame_id = frame_id;
    marker_msg.header.stamp = rclcpp::Clock().now();
    marker_msg.ns = "vdb_grid";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.color.a = 1.0;
    marker_msg.frame_locked = true;

    double voxel_size = grid->voxelSize()[0];
    marker_msg.scale.x = voxel_size;
    marker_msg.scale.y = voxel_size;
    marker_msg.scale.z = voxel_size;

    openvdb::CoordBBox bbox = grid->evalActiveVoxelBoundingBox();
    double z_min = grid->indexToWorld(bbox.min()).z();
    double z_max = grid->indexToWorld(bbox.max()).z();
    double dz = z_max - z_min;

    for (auto iter = grid->cbeginValueOn(); iter; ++iter)
    {
        if (iter.getValue() <= 0.0f)
        {
            continue;
        }

        openvdb::Vec3d world = grid->indexToWorld(iter.getCoord());

        geometry_msgs::msg::Point pt;
        pt.x = world.x();
        pt.y = world.y();
        pt.z = world.z();
        marker_msg.points.push_back(pt);

        double norm_z = (dz > 1e-3) ? (world.z() - z_min) / dz : 0.5;
        marker_msg.colors.push_back(heightToColor(norm_z));
    }

    if (marker_msg.points.empty())
    {
        marker_msg.action = visualization_msgs::msg::Marker::DELETE;
    }
}

void generateFrontierMarker(const openvdb::BoolGrid::Ptr &grid,
                            const std::string &frame_id,
                            visualization_msgs::msg::Marker &marker_msg)
{
    marker_msg.header.frame_id = frame_id;
    marker_msg.header.stamp = rclcpp::Clock().now();
    marker_msg.ns = "vdb_grid";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.color.a = 1.0;
    marker_msg.frame_locked = true;

    double voxel_size = grid->voxelSize()[0];
    marker_msg.scale.x = voxel_size;
    marker_msg.scale.y = voxel_size;
    marker_msg.scale.z = voxel_size;

    openvdb::CoordBBox bbox = grid->evalActiveVoxelBoundingBox();
    double z_min = grid->indexToWorld(bbox.min()).z();
    double z_max = grid->indexToWorld(bbox.max()).z();
    double dz = z_max - z_min;

    for (auto iter = grid->cbeginValueOn(); iter; ++iter)
    {
        if (!iter.getValue())
        {
            continue;
        }

        openvdb::Vec3d world = grid->indexToWorld(iter.getCoord());

        geometry_msgs::msg::Point pt;
        pt.x = world.x();
        pt.y = world.y();
        pt.z = world.z();

        std_msgs::msg::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;

        marker_msg.points.push_back(pt);
        marker_msg.colors.push_back(color);
    }

    if (marker_msg.points.empty())
    {
        marker_msg.action = visualization_msgs::msg::Marker::DELETE;
    }
}

void generateClusterMarker(const openvdb::FloatGrid::Ptr &grid,
                           const std::string &frame_id,
                           visualization_msgs::msg::Marker &marker_msg)
{
    marker_msg.header.frame_id = frame_id;
    marker_msg.header.stamp = rclcpp::Clock().now();
    marker_msg.ns = "cluster_grid";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.color.a = 1.0;
    marker_msg.frame_locked = true;

    double voxel_size = grid->voxelSize()[0];
    marker_msg.scale.x = voxel_size;
    marker_msg.scale.y = voxel_size;
    marker_msg.scale.z = voxel_size;

    for (auto iter = grid->cbeginValueOn(); iter; ++iter)
    {
        float cluster_val = iter.getValue();
        if (cluster_val <= 0.0f)
            continue; // skip unclustered voxels

        openvdb::Vec3d world = grid->indexToWorld(iter.getCoord());

        geometry_msgs::msg::Point pt;
        pt.x = world.x();
        pt.y = world.y();
        pt.z = world.z();
        marker_msg.points.push_back(pt);

        marker_msg.colors.push_back(valueToRainbowColor(cluster_val));
    }

    if (marker_msg.points.empty())
    {
        marker_msg.action = visualization_msgs::msg::Marker::DELETE;
    }
}

bool surface_edge_frontier(openvdb::FloatGrid::Accessor accessor,
                           openvdb::Coord ijk)
{
    openvdb::math::Coord ijk_n;
    float gridvalue;
    std::vector<Eigen::Vector3d> unknown_list;
    std::vector<Eigen::Vector3d> occupied_list;
    for (int di = -1; di <= 1; ++di)
    {
        ijk_n[0] = ijk[0] + di;
        for (int dj = -1; dj <= 1; ++dj)
        {
            ijk_n[1] = ijk[1] + dj;
            for (int dk = -1; dk <= 1; ++dk)
            {
                ijk_n[2] = ijk[2] + dk;
                bool surround_state = accessor.probeValue(ijk_n, gridvalue);
                // if there is unknown neighbor, surround_state will be inactive
                if (!surround_state)
                {
                    unknown_list.push_back(Eigen::Vector3d(di, dj, dk));
                    // std::cout << "pushing unknown" << '\n';
                }
                else if (gridvalue > 0)
                {
                    occupied_list.push_back(Eigen::Vector3d(di, dj, dk));
                    // std::cout << "pushing occ" << '\n';
                }
            }
        }
    }
    if (unknown_list.empty() || occupied_list.empty())
    {
        return false;
    }
    for (size_t i = 0; i < unknown_list.size(); ++i)
    {
        for (size_t j = 0; j < occupied_list.size(); ++j)
        {
            Eigen::Vector3d un_occ_diff_vec = unknown_list[i] - occupied_list[j];
            // std::cout << "diff is " << un_occ_diff_vec.dot(un_occ_diff_vec) << '\n';
            if (un_occ_diff_vec.dot(un_occ_diff_vec) <= 3)
            {
                return true;
            }
        }
    }
    return false;
}

void create_clustering_coordbbox(const openvdb::BoolGrid::Ptr grid,
                                 openvdb::math::CoordBBox &coord_bbox_out,
                                 const openvdb::Vec3d &center,
                                 const float frontier_count_cube_dim)
{
    openvdb::Vec3d bbox_min, bbox_max;
    openvdb::Coord ijk_bbox_min, ijk_bbox_max;
    openvdb::math::Transform &grid_tf(grid->transform());

    bbox_min.x() = center.x() - (frontier_count_cube_dim / 2);
    bbox_max.x() = center.x() + (frontier_count_cube_dim / 2);

    bbox_min.y() = center.y() - (frontier_count_cube_dim / 2);
    bbox_max.y() = center.y() + (frontier_count_cube_dim / 2);

    bbox_min.z() = center.z() - (frontier_count_cube_dim / 2);
    bbox_max.z() = center.z() + (frontier_count_cube_dim / 2);

    ijk_bbox_min = grid_tf.worldToIndexCellCentered(bbox_min);
    ijk_bbox_max = grid_tf.worldToIndexCellCentered(bbox_max);

    coord_bbox_out.reset(ijk_bbox_min, ijk_bbox_max);
}

void point_clustering_vis(openvdb::BoolGrid::Ptr grid,
                          const float cluster_cube_dim,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud,
                          const int voxel_cluster_count_thresh,
                          std::vector<std::vector<Eigen::Vector3d>> &clustered_points,
                          openvdb::FloatGrid::Ptr visualized_clusters)
{
    // for visualization only
    float cluster_num = 0;
    openvdb::FloatGrid::Accessor vis_acc = visualized_clusters->getAccessor();
    //

    openvdb::BoolGrid::Ptr grid_copy = grid->deepCopy();
    int voxel_count = 0;
    openvdb::BoolGrid::Accessor acc = grid_copy->getAccessor();
    const openvdb::math::Transform &grid_tf(grid_copy->transform());
    pcl::PointXYZI cluster_score_point;
    bool voxel_value = false;
    std::vector<Eigen::Vector3d> points_buffer;
    for (openvdb::BoolGrid::ValueOnCIter itr = grid_copy->cbeginValueOn(); itr; ++itr)
    {
        voxel_value = itr.getValue();
        // voxels that have not been "marked" before
        if (voxel_value)
        {
            // for visualization only
            std::vector<openvdb::Coord> vis_boxes;
            //

            openvdb::Coord ijk = itr.getCoord();
            openvdb::Vec3d xyz = grid_tf.indexToWorld(ijk);
            // create a bbox centered around xyz
            openvdb::math::CoordBBox bbox;
            create_clustering_coordbbox(grid_copy, bbox, xyz, cluster_cube_dim);
            // iterate through bbox and find intersecting grid voxels
            // save points to buffer, copy from buffer to output list
            // if cluster is big enough
            // count neighboring voxels (with voxel_value > voxel_mark_priority_medium) within bbox
            voxel_count = 0;
            points_buffer.clear();
            for (auto ijk = bbox.beginXYZ(); ijk; ++ijk)
            {
                openvdb::Coord idx = *ijk;
                openvdb::Vec3d pt_xyz = grid_tf.indexToWorld(idx);
                if (acc.getValue(idx))
                {
                    Eigen::Vector3d point{pt_xyz.x(), pt_xyz.y(), pt_xyz.z()};
                    points_buffer.push_back(point);
                    voxel_count += 1;

                    // for visualization only
                    openvdb::Coord vis_ijk = idx;
                    vis_boxes.push_back(vis_ijk);
                    //
                }
            }
            // if nn vox count > thresh
            // insert xyz and nn vox count as an XYZI point into cluster cloud
            // mark all nn voxels and cluster centroid vox as --> voxel_mark_priority_medium
            if (voxel_count >= voxel_cluster_count_thresh)
            {
                Eigen::Vector3d cluster_centroid = VDBUtil::computeCentroid(points_buffer);
                cluster_score_point.x = cluster_centroid.x();
                cluster_score_point.y = cluster_centroid.y();
                cluster_score_point.z = cluster_centroid.z();
                cluster_score_point.intensity = voxel_count;
                cluster_cloud->push_back(cluster_score_point);
                mark_voxels_bool(points_buffer, grid_copy, false);
                clustered_points.push_back(points_buffer);

                // for visualization only, one cluster one value, thus one color.
                cluster_num += 0.1;
                for (size_t pt = 0; pt < vis_boxes.size(); ++pt)
                {
                    vis_acc.setValue(vis_boxes[pt], cluster_num);
                }
                //
            }
        }
    }
}

void mark_voxels_bool(const std::vector<Eigen::Vector3d> &point_list,
                      openvdb::BoolGrid::Ptr grid,
                      const bool voxel_value)
{
    openvdb::BoolGrid::Accessor acc = grid->getAccessor();
    const openvdb::math::Transform &grid_tf(grid->transform());
    for (auto &point : point_list)
    {
        openvdb::Vec3d xyz{point.x(), point.y(), point.z()};
        openvdb::Coord ijk = grid_tf.worldToIndexCellCentered(xyz);
        acc.setValue(ijk, voxel_value);
    }
}

Eigen::Vector3d VDBUtil::computeCentroid(const std::vector<Eigen::Vector3d> &point_list)
{

    Eigen::Vector3d sum = std::accumulate(point_list.begin(),
                                          point_list.end(),
                                          Eigen::Vector3d::Zero().eval());
    // Eigen::Vector3d centroid {sum / point_list.size()};

    return sum / point_list.size();
}

void VDBUtil::updateOccMapFromNdArray(openvdb::FloatGrid::Ptr grid_logocc,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz,
                                      const tf2::Vector3 &origin,
                                      float l_free,
                                      float l_occ,
                                      float l_min,
                                      float l_max,
                                      int hit_thickness,
                                      double threshold,
                                      float visited_cleared_logocc_min_thresh)
{
    openvdb::FloatGrid::Accessor acc = grid_logocc->getAccessor();
    openvdb::Vec3d origin2(origin.x(), origin.y(), origin.z());
    openvdb::Vec3d origin_ijk = grid_logocc->worldToIndex(origin2);

    for (size_t i = 0; i < xyz->points.size(); ++i)
    {
        openvdb::Vec3d p_ijk = grid_logocc->worldToIndex(openvdb::Vec3d(xyz->points[i].x,
                                                                        xyz->points[i].y,
                                                                        xyz->points[i].z));

        openvdb::Vec3d dir(p_ijk - origin_ijk);
        double range = dir.length();
        dir.normalize();
        openvdb::math::Ray<double> ray(origin_ijk, dir);
        // ray, start_time, max_time

        if (range <= threshold)
        {
            openvdb::math::DDA<openvdb::math::Ray<double>, 0> dda(ray, 0., range);
            // want to stop before dda.time == dda.maxTime
            do
            {
                openvdb::Coord ijk(dda.voxel());
                float ll = acc.getValue(ijk);
                if (ll < visited_cleared_logocc_min_thresh)
                {
                    acc.setValue(ijk, std::max(l_min, ll + l_free));
                }
                dda.step();
            } while (dda.time() < dda.maxTime());

            // post condition, at hit voxel
            for (int i = 0; i < hit_thickness; ++i)
            {
                openvdb::Coord ijk(dda.voxel());
                float ll = acc.getValue(ijk);
                if (ll < visited_cleared_logocc_min_thresh)
                {
                    acc.setValue(ijk, std::min(l_max, ll + l_occ));
                }
                dda.step();
            }
        }
        else
        {
            openvdb::math::DDA<openvdb::math::Ray<double>, 0> dda(ray, 0., std::min(range, threshold));
            do
            {
                openvdb::Coord ijk(dda.voxel());
                float ll = acc.getValue(ijk);
                if (ll < visited_cleared_logocc_min_thresh)
                {
                    acc.setValue(ijk, std::max(l_min, ll + l_free));
                }
                dda.step();
            } while (dda.time() < dda.maxTime());
        }
    }
}

void VDBUtil::setVoxelSize(openvdb::GridBase &grid, double vs)
{
    // cell-centered
    const openvdb::math::Vec3d offset(vs / 2., vs / 2., vs / 2.);
    openvdb::math::Transform::Ptr tf = openvdb::math::Transform::createLinearTransform(vs);
    tf->postTranslate(offset);
    grid.setTransform(tf);
}

bool VDBUtil::collCheckPointInPartialFreeSpace(const openvdb::FloatGrid::Ptr grid,
                                               const Eigen::Vector3d &point_xyz,
                                               const float length,
                                               const float breadth,
                                               const float height,
                                               const float l_occ,
                                               const float unknown_fraction_thresh,
                                               const float occupied_fraction_thresh)
{

    bool point_is_in_free_space;
    if (grid->empty())
    {
        std::cerr << "[VDBUtil::checkPointInFreeSpace] Grid is empty. \
                  Cannot perform collision check. Exiting."
                  << "\n";
        return false;
    }
    const openvdb::math::Transform &grid_tf(grid->transform());
    openvdb::Vec3d xyz(point_xyz.x(), point_xyz.y(), point_xyz.z());
    openvdb::math::CoordBBox coord_bbox;

    VDBUtil::createCoordBBox(grid, coord_bbox, xyz,
                             length, breadth, height);

    long int bbox_total_vox_count = 0;
    long int unknown_vox_count = 0;
    long int occupied_vox_count = 0;
    openvdb::FloatGrid::Accessor acc = grid->getAccessor();
    for (auto ijk = coord_bbox.beginXYZ(); ijk; ++ijk)
    {
        ++bbox_total_vox_count;
        openvdb::Coord idx = *ijk;
        float vox_val = 0.0;
        bool vox_state = acc.probeValue(idx, vox_val);
        if (vox_state && vox_val > l_occ)
        {
            ++occupied_vox_count;
        }
        else if (!vox_state)
        {
            ++unknown_vox_count;
        }
    }
    double occupied_fraction = double(occupied_vox_count) / double(bbox_total_vox_count);
    double unknown_fraction = double(unknown_vox_count) / double(bbox_total_vox_count);
    // if ( occupied_fraction <= occupied_fraction_thresh && unknown_fraction <= unknown_fraction_thresh) {
    //   point_is_in_free_space = true;
    // } else {  // in collision
    //   point_is_in_free_space = false;
    // }

    if (occupied_fraction > occupied_fraction_thresh || unknown_fraction > unknown_fraction_thresh)
    {
        point_is_in_free_space = false;
    }
    else
    { // in free space
        point_is_in_free_space = true;
    }

    return point_is_in_free_space;
}

bool VDBUtil::checkCollisionAlongRay(const openvdb::FloatGrid::Ptr& grid,
                                     const openvdb::Vec3d &p1_xyz,
                                     const openvdb::Vec3d &p2_xyz,
                                     const float bbox_length,
                                     const float bbox_breadth,
                                     const float bbox_height,
                                     const float l_occ,
                                     openvdb::Vec3d &hit_point,
                                     float dist_offset_from_cluster,
                                     bool consider_unknown_occupied)
{

    bool in_collision = false;
    openvdb::Coord ray_element_ijk;
    openvdb::Vec3d ray_element_xyz;

    openvdb::FloatGrid::Accessor acc = grid->getAccessor();
    const openvdb::math::Transform &grid_tf(grid->transform());

    openvdb::Vec3d p1_ijk = grid_tf.worldToIndex(p1_xyz);
    openvdb::Vec3d p2_ijk = grid_tf.worldToIndex(p2_xyz);
    openvdb::Vec3d dir(p2_ijk - p1_ijk);
    double distance = dir.length();
    dir.normalize();
    openvdb::math::Ray<double> ray(p1_ijk, dir);
    openvdb::math::DDA<openvdb::math::Ray<double>, 0> dda(ray, 0., distance);
    // want to stop before dda.time == dda.maxTime
    // do {
    while (dda.next() < dda.maxTime())
    {

        ray_element_ijk = dda.voxel();
        // check for robot model cube occupancy
        ray_element_xyz = grid->indexToWorld(ray_element_ijk);
        openvdb::math::CoordBBox coord_bbox;
        VDBUtil::createCoordBBox(grid, coord_bbox, ray_element_xyz,
                                 bbox_length, bbox_breadth, bbox_height);
        if (VDBUtil::checkIntersectWithCoordBBox(coord_bbox, grid,
                                                 l_occ, consider_unknown_occupied))
        {
            in_collision = true;
            break;
        }
        dda.step();
    }
    // } while (dda.time() < dda.maxTime());
    // } while (dda.next() < dda.maxTime());

    if (in_collision)
    {
        // openvdb::Vec3d hit_ray(ray_element_ijk - p1_ijk);
        // double hit_distance = hit_ray.length();
        // double min_threshold_distance = distance - dist_offset_from_cluster;
        // if (hit_distance >= min_threshold_distance)
        // {
        //     return false;
        // }
        // post condition, at hit voxel
        hit_point = ray_element_xyz;
        return true;
    }
    else
    {
        return false;
    }
}

long int VDBUtil::countVoxelsInsideBbox(const openvdb::FloatGrid::Ptr &grid,
                                        const Eigen::Vector3d &bbox_center,
                                        const float length,
                                        const float breadth,
                                        const float height)
{

    openvdb::CoordBBox bbox;
    long int vox_count = 0;
    float log_occ_val = 0.0;
    const openvdb::math::Transform &grid_tf(grid->transform());
    openvdb::FloatTree::Ptr grid_tree = grid->treePtr();
    createCoordBBox(grid,
                    bbox,
                    convertEigenVecToVec3D(bbox_center),
                    length,
                    breadth,
                    height);

    for (auto bbox_iter = bbox.beginXYZ(); bbox_iter; ++bbox_iter)
    {
        openvdb::Coord ijk = *bbox_iter;
        if (grid_tree->probeValue(ijk, log_occ_val))
        {
            vox_count += 1;
        }
    }
    return vox_count;
}

// void VDBUtil::createCoordBBox(const openvdb::FloatGrid::Ptr grid,
//                               openvdb::math::CoordBBox &coord_bbox_out,
//                               const openvdb::Vec3d &center,
//                               const float bbox_length,
//                               const float bbox_breadth,
//                               const float bbox_height)
// {

//     openvdb::Vec3d bbox_min, bbox_max;
//     openvdb::Coord ijk_bbox_min, ijk_bbox_max;
//     openvdb::math::Transform &grid_tf(grid->transform());

//     bbox_min.x() = center.x() - (bbox_length / 2);
//     bbox_max.x() = center.x() + (bbox_length / 2);

//     bbox_min.y() = center.y() - (bbox_breadth / 2);
//     bbox_max.y() = center.y() + (bbox_breadth / 2);

//     bbox_min.z() = center.z() - (bbox_height / 2);
//     bbox_max.z() = center.z() + (bbox_height / 2);

//     ijk_bbox_min = grid_tf.worldToIndexCellCentered(bbox_min);
//     ijk_bbox_max = grid_tf.worldToIndexCellCentered(bbox_max);

//     coord_bbox_out.reset(ijk_bbox_min, ijk_bbox_max);
// }

// The above implementation rounds to the nearest cell center. 
// If bbox_max lands just past a cell center, rounding can pull it back and 
// you’ll silently drop the outermost layer of cells. 
// Use a continuous index bbox and floor/ceil:
void VDBUtil::createCoordBBox(const openvdb::FloatGrid::Ptr grid,
                              openvdb::math::CoordBBox &coord_bbox_out,
                              const openvdb::Vec3d &center,
                              const float bbox_length,
                              const float bbox_breadth,
                              const float bbox_height)
{
    const auto& tf = grid->transform();
    const openvdb::Vec3d half(bbox_length*0.5, bbox_breadth*0.5, bbox_height*0.5);

    // World-space box -> continuous index-space box
    const openvdb::BBoxd wbox(center - half, center + half);
    const openvdb::BBoxd ibox = tf.worldToIndex(wbox);

    // Integer (cell-centered) bounds: floor(min), ceil(max)
    const openvdb::Coord ijk_min(
        static_cast<int>(std::floor(ibox.min().x() + 0.5)),
        static_cast<int>(std::floor(ibox.min().y() + 0.5)),
        static_cast<int>(std::floor(ibox.min().z() + 0.5)));
    
    const openvdb::Coord ijk_max(
        static_cast<int>(std::ceil (ibox.max().x() - 0.5)),
        static_cast<int>(std::ceil (ibox.max().y() - 0.5)),
        static_cast<int>(std::ceil (ibox.max().z() - 0.5)));

    coord_bbox_out.reset(ijk_min, ijk_max);
}

bool VDBUtil::checkIntersectWithCoordBBox(const openvdb::math::CoordBBox &search_bbox,
                                          const openvdb::FloatGrid::Ptr grid,
                                          float occupancy_threshold,
                                          bool consider_unknown_occupied)
{

    openvdb::math::Transform &grid_tf(grid->transform());
    const size_t count = search_bbox.volume();
    openvdb::Vec3d xyz;
    openvdb::FloatGrid::Accessor acc = grid->getAccessor();
    for (auto iter = search_bbox.beginXYZ(); iter; ++iter)
    {
        openvdb::Coord ijk = *iter;
        float vox_val = 0.0;
        bool vox_state = acc.probeValue(ijk, vox_val);
        if (vox_state && vox_val > occupancy_threshold)
        {
            return true;
        }
        else if (consider_unknown_occupied && std::fabs(vox_val) <= FLT_EPSILON)
        {
            return true;
        }
    }
    return false;
}

openvdb::Vec3d VDBUtil::convertEigenVecToVec3D(const Eigen::Vector3d &point)
{

    return openvdb::Vec3d(point.x(), point.y(), point.z());
}