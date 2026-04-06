#ifndef _OLE_3D_UTIL_H_
#define _OLE_3D_UTIL_H_

#include <iostream>
#include <cmath>
#include <cfloat>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <time.h>
#include <memory>
#include <string>
#include <ctime>
#include <numeric>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <tf2/LinearMath/Vector3.h>

#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>
#include <openvdb/tools/Dense.h>
#include <openvdb/tools/Prune.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/Clip.h>
#include <openvdb/math/Ray.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Coord.h>
#include <openvdb/math/Transform.h>
#include <rclcpp/rclcpp.hpp>
#include <openvdb/tree/ValueAccessor.h>

#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

using GridT = openvdb::Grid<openvdb::tree::Tree4<float, 5, 4, 3>::Type>;

// void generateVDBMarker(const GridT::Ptr& grid,
//                        const std::string& frame_id,
//                        visualization_msgs::msg::Marker& marker_msg);

void generateVDBMarker(const openvdb::FloatGrid::Ptr &grid,
                       const std::string &frame_id,
                       visualization_msgs::msg::Marker &marker_msg);

void generateFrontierMarker(const openvdb::BoolGrid::Ptr &grid,
                            const std::string &frame_id,
                            visualization_msgs::msg::Marker &marker_msg);

void generateClusterMarker(const openvdb::FloatGrid::Ptr &grid,
                           const std::string &frame_id,
                           visualization_msgs::msg::Marker &marker_msg);

bool surface_edge_frontier(openvdb::FloatGrid::Accessor accessor, openvdb::Coord ijk);

void create_clustering_coordbbox(const openvdb::BoolGrid::Ptr grid,
                                 openvdb::math::CoordBBox &coord_bbox_out,
                                 const openvdb::Vec3d &center,
                                 const float frontier_count_cube_dim);

void point_clustering_vis(openvdb::BoolGrid::Ptr grid,
                          const float cluster_cube_dim,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud,
                          const int voxel_cluster_count_thresh,
                          std::vector<std::vector<Eigen::Vector3d>> &clustered_points,
                          openvdb::FloatGrid::Ptr visualized_clusters);

void mark_voxels_bool(const std::vector<Eigen::Vector3d> &point_list,
                      openvdb::BoolGrid::Ptr grid,
                      const bool voxel_value);

class VDBUtil
{
public:
    static void updateOccMapFromNdArray(openvdb::FloatGrid::Ptr grid_logocc,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz,
                                        const tf2::Vector3 &origin,
                                        float l_free = -0.5f, // - a value if not hit
                                        float l_occ = 1.0f,   // + a value if hit
                                        float l_min = -10.0f, // lower bound of free
                                        float l_max = 10.0f,  // upper bound of occ
                                        int hit_thickness = 1,
                                        double max_range = 50,
                                        float visited_cleared_logocc_min_thresh = 8.0f);

    static void setVoxelSize(openvdb::GridBase &grid, double vs);

    static Eigen::Vector3d computeCentroid(const std::vector<Eigen::Vector3d> &point_list);

    static bool collCheckPointInPartialFreeSpace(const openvdb::FloatGrid::Ptr grid,
                                                 const Eigen::Vector3d &point_xyz,
                                                 const float length,
                                                 const float breadth,
                                                 const float height,
                                                 const float l_occ,
                                                 const float unknown_fraction_thresh,
                                                 const float occupied_fraction_thresh);

    static bool checkCollisionAlongRay(const openvdb::FloatGrid::Ptr& grid,
                                       const openvdb::Vec3d &p1_xyz,
                                       const openvdb::Vec3d &p2_xyz,
                                       const float bbox_length,
                                       const float bbox_breadth,
                                       const float bbox_height,
                                       const float l_occ,
                                       openvdb::Vec3d &hit_point,
                                       float dist_offset_from_cluster,
                                       bool consider_unknown_occupied);

    static long int countVoxelsInsideBbox(const openvdb::FloatGrid::Ptr &grid,
                                          const Eigen::Vector3d &bbox_center,
                                          const float length,
                                          const float breadth,
                                          const float height);

    static void createCoordBBox(const openvdb::FloatGrid::Ptr grid,
                                openvdb::math::CoordBBox &coord_bbox_out,
                                const openvdb::Vec3d &center,
                                const float bbox_length,
                                const float bbox_breadth,
                                const float bbox_height);

    static bool checkIntersectWithCoordBBox(const openvdb::math::CoordBBox &search_bbox,
                                            const openvdb::FloatGrid::Ptr grid,
                                            float occupancy_threshold,
                                            bool consider_unknown_occupied);

    static openvdb::Vec3d convertEigenVecToVec3D(const Eigen::Vector3d &point);
    
}; // class VDBUtil
#endif