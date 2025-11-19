#ifndef _VIEWPOINT_SAMPLING_H_
#define _VIEWPOINT_SAMPLING_H_

#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "utils/utils.hpp"

namespace viewpoint_sampling_ns
{
    class ViewpointSampling;
} // namespace viewpoint_sampling_ns

class viewpoint_sampling_ns::ViewpointSampling
{

public:
    using ListOfClustersT = std::vector<std::vector<Eigen::Vector3d>>;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr viewpoint_list_;

private:
    int cluster_idx_;

    // params
    double viewing_distance_;
    float bbox_length_, bbox_breadth_, bbox_height_;
    float l_occ_, vox_size_;
    uint16_t num_viewp_to_sample_;
    double viewing_distance_inner_r_, viewing_distance_outer_r_, viewing_distance_z_offset_;

    // offset the endpoint for collision checking along the ray connecting the viewp to cluster centroid
    // fraction of the robot model bbox so that the collision checking above is done for a small bbox
    double coll_check_endpoint_offset_, bbox_fraction_viewp_centroid_coll_check_;

    // flags
    uint8_t solver_type_;

    // viewpoint attributes
    double current_cluster_score_, heading_;
    double viewp_bbox_unknown_frac_thresh_, viewp_bbox_occ_frac_thresh_;

    // lists
    std::shared_ptr<ListOfClustersT> cluster_list_;
    std::shared_ptr<std::vector<double>> cluster_score_list_;
    std::vector<Eigen::Vector2d> xy_points_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr centroid_list_;
    std::vector<double> line_y_points_, line_x_points_;
    std::vector<double> cluster_x_points_, cluster_y_points_;

    // current geometric entities
    Eigen::Vector3d current_cluster_centroid_;
    Eigen::Matrix<double, 2, 3> opposing_viewpoints_;
    Eigen::Vector3d direction_vector_;
    Eigen::Vector2d fitted_line_;
    Eigen::MatrixXd A_mat_;
    Eigen::VectorXd b_vec_;
    Eigen::Vector3d selected_viewpoint_;
    openvdb::FloatGrid::Ptr grid_map_;

public:
    ViewpointSampling(ListOfClustersT &cluster_list,
                      double viewing_distance,
                      float bbox_length,
                      float bbox_breadth,
                      float bbox_height,
                      float l_occ,
                      float vox_size,
                      uint16_t num_viewp_sample_per_cluster,
                      double viewing_distance_inner_r,
                      double viewing_distance_outer_r,
                      double viewing_distance_z_offset,
                      double coll_check_endpoint_offset,
                      double bbox_fraction_viewp_centroid_coll_check,
                      double viewp_bbox_unknown_frac_thresh,
                      double viewp_bbox_occ_frac_thresh);

    // data handling
    void updateGridPtr(const openvdb::FloatGrid::Ptr &grid);
    void updateCentroidListPtr(const pcl::PointCloud<pcl::PointXYZI>::Ptr &centroid_list);
    void reset(ListOfClustersT &cluster_list);

    // utility methods
    void sampleViewpoints();
    void projectToXYPlane();
    void computePointsPerpToLine();
    void selectOnePoint();
    float computeYawFromDirVector();
    bool fitLineToPoints(int solver_flag);
    void useSVDDecomposition();
    void useQRDecomposition();
    void useNormalEquations();
    void createAMatrix();
    void createBMatrix();
    Eigen::Vector3d computeMean();
    pcl::PointXYZHSV constructViewpoint();
    void sampleViewpointsAroundCentroid();
    int sampleViewpointsAroundCentroidParamaterized(const double viewing_distance,
                                                    const double z_offset,
                                                    const double angle_spacing);

    // logging and plotting
    std::string setupFilePath(int file_index, std::string file_name);
    void writeToCSVfile(std::string file_name, Eigen::MatrixXd &matrix);
    void writeVecToCSVfile(std::string file_name,
                           std::vector<Eigen::Vector2d> &points);
    void logDataToFile();
    void plotPointsAndLine();
    void drawPointsAlongLine(float resolution, float length);
    void toXVecYVec();

}; // class ViewpointSampling
#endif