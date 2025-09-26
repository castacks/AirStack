/*
 * Viewpoint Sampling Module
 * Copyright (C) 2020 Rohit Garg - All rights reserved
 * Original author: rg1@cmu.edu / rohitgarg617@gmail.com
 * Modified by junbiny@andrew.cmu.edu, removed matplotlib part and modified viewpoint sampling method
 */

#include "../include/viewpoint_sampling.hpp"

namespace viewpoint_sampling_ns
{
    // constructor
    ViewpointSampling::ViewpointSampling(ListOfClustersT &cluster_list,
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
                                         double viewp_bbox_occ_frac_thresh)
    {

        viewing_distance_ = viewing_distance;
        bbox_length_ = bbox_length;
        bbox_breadth_ = bbox_breadth;
        bbox_height_ = bbox_height;
        l_occ_ = l_occ;
        vox_size_ = vox_size;
        num_viewp_to_sample_ = num_viewp_sample_per_cluster;
        viewing_distance_inner_r_ = viewing_distance_inner_r;
        viewing_distance_outer_r_ = viewing_distance_outer_r;
        viewing_distance_z_offset_ = viewing_distance_z_offset;
        coll_check_endpoint_offset_ = coll_check_endpoint_offset;
        bbox_fraction_viewp_centroid_coll_check_ = bbox_fraction_viewp_centroid_coll_check;
        cluster_list_ = std::make_shared<ListOfClustersT>(cluster_list);
        viewpoint_list_ = pcl::PointCloud<pcl::PointXYZHSV>::Ptr(new pcl::PointCloud<pcl::PointXYZHSV>());
        viewp_bbox_unknown_frac_thresh_ = viewp_bbox_unknown_frac_thresh;
        viewp_bbox_occ_frac_thresh_ = viewp_bbox_occ_frac_thresh;
    }

    // reset data stored by the class
    void ViewpointSampling::reset(ListOfClustersT &cluster_list)
    {

        // TODO -> not sure if this make_shared is required
        cluster_list_ = std::make_shared<ListOfClustersT>(cluster_list);
        xy_points_.clear();
        viewpoint_list_->clear();
    }

    // update the global map grid pointer
    void ViewpointSampling::updateGridPtr(const openvdb::FloatGrid::Ptr &grid)
    {
        grid_map_ = grid;
    }

    // update the cluster centroid list pointer
    void ViewpointSampling::updateCentroidListPtr(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &centroid_list)
    {
        centroid_list_ = centroid_list;
    }

    // main method that samples viewpoints
    void ViewpointSampling::sampleViewpoints()
    {
        cluster_idx_ = -1;

        for (const auto &cluster : *cluster_list_)
        {

            cluster_idx_ += 1;

            // current_cluster_centroid_ = computeMean();
            pcl::PointXYZI centroid = centroid_list_->at(cluster_idx_);
            current_cluster_score_ = centroid.intensity;
            current_cluster_centroid_.x() = centroid.x;
            current_cluster_centroid_.y() = centroid.y;
            current_cluster_centroid_.z() = centroid.z;
            sampleViewpointsAroundCentroid();
        }
    }

    // remove z component from 3D points
    void ViewpointSampling::projectToXYPlane()
    {
        xy_points_.clear();
        for (const auto &point : cluster_list_->at(cluster_idx_))
        {
            Eigen::Vector2d point_in{point.x(), point.y()};
            xy_points_.push_back(point_in);
        }
    }

    // logging to file
    void ViewpointSampling::logDataToFile()
    {

        std::string file1 = "cluster";
        std::string file_path1 = setupFilePath(cluster_idx_, file1);
        writeVecToCSVfile(file_path1, xy_points_);

        std::string file2 = "fit_line_wsolver" + std::to_string(solver_type_);
        std::string file_path2 = setupFilePath(cluster_idx_, file2);
        Eigen::MatrixXd output_mat = fitted_line_;
        writeToCSVfile(file_path2, output_mat);
    }

    // generate file path
    std::string ViewpointSampling::setupFilePath(int file_index, std::string file_name)
    {
        std::string fname = file_name + "_";
        std::string path = "/home/rohit/rWorkspace/vdb_map_ws/src/map_processor/logs/";
        std::string idx = std::to_string(file_index);
        std::string fformat = ".txt";
        return path + fname + idx + fformat;
    }

    // write matrix to a CSV file
    void ViewpointSampling::writeToCSVfile(std::string file_name, Eigen::MatrixXd &matrix)
    {
        const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
        std::ofstream file(file_name.c_str());
        file << matrix.format(CSVFormat);
        file.close();
    }

    // write vector to a CSV file
    void ViewpointSampling::writeVecToCSVfile(std::string file_name,
                                              std::vector<Eigen::Vector2d> &points)
    {
        const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, 0, ", ", "\n");
        std::ofstream file(file_name.c_str());
        for (const auto &point : points)
        {
            file << point(0) << ", " << point(1) << "\n";
        }
        file.close();
    }

    // splitting into two separate vectors
    void ViewpointSampling::toXVecYVec()
    {
        for (auto point : xy_points_)
        {
            cluster_x_points_.push_back(point.x());
            cluster_y_points_.push_back(point.y());
        }
    }

    // discretize line into a list of points
    void ViewpointSampling::drawPointsAlongLine(float spacing, float length)
    {
        double start_x = current_cluster_centroid_.x();
        uint64_t n_points = std::floor(length / spacing);
        for (int i = 0; i < n_points; ++i)
        {
            double line_x_pt = start_x + (i * spacing);
            double line_y_pt = (fitted_line_(0) * line_x_pt) + fitted_line_(0);
            line_x_points_.push_back(line_x_pt);
            line_y_points_.push_back(line_y_pt);
        }
    }

    // Ax=b -> A matrix
    void ViewpointSampling::createAMatrix()
    {
        A_mat_ = Eigen::MatrixXd::Ones(xy_points_.size(), 2);
        uint32_t idx = 0;
        for (const auto &point : xy_points_)
        {
            Eigen::Vector2d point_in{point.x(), 1};
            A_mat_.row(idx) = point_in;
            idx += 1;
        }
    }

    // Ax=b -> b matrix
    void ViewpointSampling::createBMatrix()
    {
        b_vec_ = Eigen::VectorXd::Zero(xy_points_.size());
        uint32_t idx = 0;
        for (const auto &point : xy_points_)
        {
            b_vec_(idx) = point.y();
            idx += 1;
        }
    }

    // for getting the cluster centroid
    Eigen::Vector3d ViewpointSampling::computeMean()
    {

        double sum_x = 0, sum_y = 0, sum_z = 0;
        for (const auto &point : cluster_list_->at(cluster_idx_))
        {
            sum_x += point.x();
            sum_y += point.y();
            sum_z += point.z();
        }
        uint32_t n_points = cluster_list_->at(cluster_idx_).size();
        Eigen::Vector3d point_out{sum_x / n_points, sum_y / n_points, sum_z / n_points};
        return point_out;
    }

    // get two points at a perpendicular distance from a line and a point on the line
    void ViewpointSampling::computePointsPerpToLine()
    {

        opposing_viewpoints_ = Eigen::MatrixXd::Zero(2, 3);

        double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;

        double x_c = current_cluster_centroid_.x(); // cluster xy coords
        double y_c = current_cluster_centroid_.y();

        double m = fitted_line_(0);   // slope of the line
        double r = viewing_distance_; // distance from the line

        // ref: http://www.rasmus.is/uk/t/F/Su58k05.htm#:~:text=To%20find%20a%20direction%20vector,%2B%20by%20%2B%20c%20%3D%200.
        // ref: https://math.stackexchange.com/questions/2043054/find-a-point-on-a-perpendicular-line-a-given-distance-from-another-point
        opposing_viewpoints_(0, 0) = x_c +
                                     std::sqrt(std::pow(r, 2) / (1 + (1 / std::pow(m, 2))));

        opposing_viewpoints_(1, 0) = x_c -
                                     std::sqrt(std::pow(r, 2) / (1 + (1 / std::pow(m, 2))));

        opposing_viewpoints_(0, 1) = y_c -
                                     (1 / m) * std::sqrt(std::pow(r, 2) / (1 + (1 / std::pow(m, 2))));

        opposing_viewpoints_(1, 1) = y_c +
                                     (1 / m) * std::sqrt(std::pow(r, 2) / (1 + (1 / std::pow(m, 2))));

        // z -> same height as the cluster centroid since the
        // view plane is perpendicular to XY plane
        opposing_viewpoints_(0, 2) = current_cluster_centroid_.z();
        opposing_viewpoints_(1, 2) = current_cluster_centroid_.z();
    }

    // select one out of the two points on either side of the line
    void ViewpointSampling::selectOnePoint()
    {

        long int active_vox_count_side1 = 0, active_vox_count_side2 = 0;

        active_vox_count_side1 = VDBUtil::countVoxelsInsideBbox(grid_map_,
                                                                opposing_viewpoints_.row(0),
                                                                bbox_length_,
                                                                bbox_breadth_,
                                                                bbox_height_);

        active_vox_count_side2 = VDBUtil::countVoxelsInsideBbox(grid_map_,
                                                                opposing_viewpoints_.row(1),
                                                                bbox_length_,
                                                                bbox_breadth_,
                                                                bbox_height_);

        // viewpoint selection is happening using the number of active voxels
        // found in a bbox centered around the opposing points
        // whichever bbox has a higher count is chosen
        if (active_vox_count_side1 >= active_vox_count_side2)
        {
            selected_viewpoint_ = opposing_viewpoints_.row(0);
        }
        else
        {
            selected_viewpoint_ = opposing_viewpoints_.row(1);
        }
        direction_vector_ = current_cluster_centroid_ - selected_viewpoint_;
    }

    // assuming x-forward, z-up convention
    float ViewpointSampling::computeYawFromDirVector()
    {
        return std::atan2(direction_vector_(1), direction_vector_(0));
    }

    pcl::PointXYZHSV ViewpointSampling::constructViewpoint()
    {
        pcl::PointXYZHSV view_point;

        view_point.x = selected_viewpoint_.x();
        view_point.y = selected_viewpoint_.y();
        view_point.z = selected_viewpoint_.z();
        view_point.h = heading_;
        view_point.s = current_cluster_score_;
        view_point.v = 0.0;
        return view_point;
    }

    // line fit using linear least squares
    bool ViewpointSampling::fitLineToPoints(int solver_flag)
    {

        createAMatrix();
        createBMatrix();
        // ref: https://eigen.tuxfamily.org/dox/group__LeastSquares.html
        switch (solver_flag)
        {
        case 1:
            useSVDDecomposition();
            break;
        case 2:
            useQRDecomposition();
            break;
        case 3:
            useNormalEquations();
            break;
        }
        return true;
    }

    // SVD is the most accurate but slowest
    void ViewpointSampling::useSVDDecomposition()
    {
        fitted_line_ = Eigen::Vector2d::Zero();
        fitted_line_ = A_mat_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_vec_);
    }

    // QR decomposition sits between SVD and normal equations
    void ViewpointSampling::useQRDecomposition()
    {
        fitted_line_ = Eigen::Vector2d::Zero();
        fitted_line_ = A_mat_.colPivHouseholderQr().solve(b_vec_);
    }

    // solving A^T A x = A^T b (linear least squares using normal equations)
    // fastest but also least accurate
    void ViewpointSampling::useNormalEquations()
    {
        // Normal Equations method
        fitted_line_ = Eigen::Vector2d::Zero();
        fitted_line_ = (A_mat_.transpose() * A_mat_).ldlt().solve(A_mat_.transpose() * b_vec_);
    }

    // create viewpoints on a circle centered around the centroid
    void ViewpointSampling::sampleViewpointsAroundCentroid()
    {

        /* viewing_distance_inner_r_ = viewing_distance_inner_r;
        viewing_distance_outer_r_ = viewing_distance_outer_r;
        viewing_distance_z_offset_ */

        // successively try viewpoints on different horizontal circles until some
        // are successfully found

        double angle_spacing = num_viewp_to_sample_ / (2 * M_PI);
        int count_viewpoints_added = 0;

        // First, try outer circle with 0 z offset
        count_viewpoints_added += sampleViewpointsAroundCentroidParamaterized(
            viewing_distance_outer_r_,
            0.0,
            angle_spacing);

        if (count_viewpoints_added > 1)
        {
            // Don't add any more
            return;
        }

        // Next, try inner circle with 0 z offset
        count_viewpoints_added += sampleViewpointsAroundCentroidParamaterized(
            viewing_distance_inner_r_,
            0.0,
            angle_spacing);

        if (count_viewpoints_added > 0)
        {
            // Don't add any more
            return;
        }

        // Try outer circle with +/- z offset
        count_viewpoints_added += sampleViewpointsAroundCentroidParamaterized(
            viewing_distance_outer_r_,
            +viewing_distance_z_offset_,
            angle_spacing);
        count_viewpoints_added += sampleViewpointsAroundCentroidParamaterized(
            viewing_distance_outer_r_,
            -viewing_distance_z_offset_,
            angle_spacing);

        if (count_viewpoints_added > 0)
        {
            // Don't add any more
            return;
        }

        // Try inner circle with +/- z offset
        count_viewpoints_added += sampleViewpointsAroundCentroidParamaterized(
            viewing_distance_inner_r_,
            +viewing_distance_z_offset_,
            angle_spacing);
        count_viewpoints_added += sampleViewpointsAroundCentroidParamaterized(
            viewing_distance_inner_r_,
            -viewing_distance_z_offset_,
            angle_spacing);

        if (count_viewpoints_added > 0)
        {
            // Don't add any more
            return;
        }
    }

    // create viewpoints on a circle centered around the centroid
    // with preset parameters
    int ViewpointSampling::sampleViewpointsAroundCentroidParamaterized(
        const double viewing_distance,
        const double z_offset,
        const double angle_spacing)
    {
        // return the number of viewpoints added

        int count_viewpoints_added = 0;
        double psi = 0.0;

        for (int i = 0; i < num_viewp_to_sample_; ++i)
        {

            psi += angle_spacing * i;

            Eigen::Vector3d viewpoint;
            viewpoint.x() = current_cluster_centroid_.x() + (viewing_distance * std::cos(psi));
            viewpoint.y() = current_cluster_centroid_.y() + (viewing_distance * std::sin(psi));
            viewpoint.z() = current_cluster_centroid_.z() + z_offset;

            // bool viewpoint_is_in_free_space = VDBUtil::checkPointInFreeSpace(grid_map_,
            //                                                     viewpoint, bbox_length_,
            //                                                     bbox_breadth_, bbox_height_,
            //                                                     l_occ_, true);

            bool viewpoint_is_in_free_space =
                VDBUtil::collCheckPointInPartialFreeSpace(grid_map_,
                                                                  viewpoint, bbox_length_,
                                                                  bbox_breadth_, bbox_height_,
                                                                  l_occ_,
                                                                  viewp_bbox_unknown_frac_thresh_,
                                                                  viewp_bbox_occ_frac_thresh_);

            openvdb::Vec3d hp;
            openvdb::Vec3d viewpoint_vbd(viewpoint.x(), viewpoint.y(), viewpoint.z());
            openvdb::Vec3d centroid_vdb(current_cluster_centroid_.x(),
                                        current_cluster_centroid_.y(),
                                        current_cluster_centroid_.z());

            bool consider_unknown_occupied = false;

            double ray_coll_check_bbox_length =
                bbox_fraction_viewp_centroid_coll_check_ * bbox_length_;
            double ray_coll_check_bbox_breadth =
                bbox_fraction_viewp_centroid_coll_check_ * bbox_breadth_;
            double ray_coll_check_bbox_height =
                bbox_fraction_viewp_centroid_coll_check_ * bbox_height_;

            bool viewpoint_to_centroid_is_free_space = !VDBUtil::checkCollisionAlongRay(
                grid_map_,
                viewpoint_vbd,
                centroid_vdb,
                ray_coll_check_bbox_length,
                ray_coll_check_bbox_breadth,
                ray_coll_check_bbox_height,
                l_occ_,
                hp,
                coll_check_endpoint_offset_,
                consider_unknown_occupied);

            if (viewpoint_is_in_free_space && viewpoint_to_centroid_is_free_space)
            {
                selected_viewpoint_ = viewpoint;
                direction_vector_ = current_cluster_centroid_ - selected_viewpoint_;
                heading_ = computeYawFromDirVector();
                viewpoint_list_->push_back(constructViewpoint());
                count_viewpoints_added++;
            }
        }
        return count_viewpoints_added;
    }

} // namespace viewpoint_sampling_ns