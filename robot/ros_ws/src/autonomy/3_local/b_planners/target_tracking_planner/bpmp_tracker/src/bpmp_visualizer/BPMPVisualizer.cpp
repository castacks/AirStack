#include "bpmp_visualizer/BPMPVisualizer.h"

namespace bpmp_tracker{

    BPMPVisualizer::BPMPVisualizer(const bpmp_tracker::VisualizationParams& vis_param){
        vis_param_ = vis_param;
        // RAW PRIMITIVES
        raw_primitive_.header.frame_id = vis_param_.frame_id;
        raw_primitive_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        raw_primitive_.ns = "TRACKER_RAW_PRIMITIVES";
        raw_primitive_.action = visualization_msgs::msg::Marker::ADD;
        raw_primitive_.color.a = vis_param_.raw_primitive.color_a;
        raw_primitive_.color.r = vis_param_.raw_primitive.color_r;
        raw_primitive_.color.g = vis_param_.raw_primitive.color_g;
        raw_primitive_.color.b = vis_param_.raw_primitive.color_b;
        raw_primitive_.pose.orientation.w = 1.0;
        raw_primitive_.pose.orientation.x = 0.0;
        raw_primitive_.pose.orientation.y = 0.0;
        raw_primitive_.pose.orientation.z = 0.0;
        raw_primitive_.scale.x = vis_param_.raw_primitive.line_scale;
        
        // FEASIBLE PRIMITIVES
        feasible_primitive_.header.frame_id = vis_param_.frame_id;
        feasible_primitive_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        feasible_primitive_.ns = "TRACKER_FEASIBLE_PRIMITIVES";
        feasible_primitive_.action = visualization_msgs::msg::Marker::ADD;
        feasible_primitive_.color.a = vis_param_.feasible_primitive.color_a;
        feasible_primitive_.color.r = vis_param_.feasible_primitive.color_r;
        feasible_primitive_.color.g = vis_param_.feasible_primitive.color_g;
        feasible_primitive_.color.b = vis_param_.feasible_primitive.color_b;
        feasible_primitive_.pose.orientation.w = 1.0;
        feasible_primitive_.pose.orientation.x = 0.0;
        feasible_primitive_.pose.orientation.y = 0.0;
        feasible_primitive_.pose.orientation.z = 0.0;
        feasible_primitive_.scale.x = vis_param_.feasible_primitive.line_scale;
        vis_param_.feasible_primitive.publish = true;

        // BEST PRIMITIVE
        best_primitive_.header.frame_id = vis_param_.frame_id;
        best_primitive_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        best_primitive_.ns = "TRACKER_BEST_PRIMITIVE";
        best_primitive_.action = visualization_msgs::msg::Marker::ADD;
        best_primitive_.color.a = vis_param_.best_primitive.color_a;
        best_primitive_.color.r = vis_param_.best_primitive.color_r;
        best_primitive_.color.g = vis_param_.best_primitive.color_g;
        best_primitive_.color.b = vis_param_.best_primitive.color_b;
        best_primitive_.pose.orientation.w = 1.0;
        best_primitive_.pose.orientation.x = 0.0;
        best_primitive_.pose.orientation.y = 0.0;
        best_primitive_.pose.orientation.z = 0.0;
        best_primitive_.scale.x = vis_param_.best_primitive.line_scale;


        // CORRIDOR
        corridor_.header.frame_id = vis_param_.frame_id;
        
        corridor_.pose.position = GetDefaultPointMsg();
        corridor_.pose.orientation = GetDefaultQuaternionMsg();
        corridor_.scale = GetDefaultScaleMsg();
        corridor_.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        corridor_.ns = "SAFE_CORRIDOR";
        corridor_.action = visualization_msgs::msg::Marker::ADD;

        corridor_.color.a = vis_param_.corridor.color_a;
        corridor_.color.r = vis_param_.corridor.color_r;
        corridor_.color.g = vis_param_.corridor.color_g;
        corridor_.color.b = vis_param_.corridor.color_b;
        // CORRIDOR COLOR
        corridor_color_ = corridor_.color;
    }

    // (Yunwoo) Fixed: ROS2 types, param_ -> vis_param_
    void BPMPVisualizer::UpdateParams(const bpmp_tracker::VisualizationParams& vis_param){
        vis_param_ = vis_param;
        // RAW PRIMITIVE
        raw_primitive_.header.frame_id = vis_param_.frame_id;
        raw_primitive_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        raw_primitive_.scale.x = vis_param_.raw_primitive.line_scale;
        raw_primitive_.ns = "TRACKER_RAW_PRIMITIVES";
        raw_primitive_.color.a = (float) vis_param_.raw_primitive.color_a;
        raw_primitive_.color.r = (float) vis_param_.raw_primitive.color_r;
        raw_primitive_.color.g = (float) vis_param_.raw_primitive.color_g;
        raw_primitive_.color.b = (float) vis_param_.raw_primitive.color_b;
        raw_primitive_.pose.orientation.w = 1.0;
        raw_primitive_.pose.orientation.x = 0.0;
        raw_primitive_.pose.orientation.y = 0.0;
        raw_primitive_.pose.orientation.z = 0.0;
        // FEASIBLE PRIMITIVE
        feasible_primitive_.header.frame_id = vis_param_.frame_id;
        feasible_primitive_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        feasible_primitive_.scale.x = vis_param_.feasible_primitive.line_scale;
        feasible_primitive_.ns = "TRACKER_FEASIBLE_PRIMITIVES";
        feasible_primitive_.action = visualization_msgs::msg::Marker::ADD;
        feasible_primitive_.id = 0;
        feasible_primitive_.color.a = (float) vis_param_.feasible_primitive.color_a;
        feasible_primitive_.color.r = (float) vis_param_.feasible_primitive.color_r;
        feasible_primitive_.color.g = (float) vis_param_.feasible_primitive.color_g;
        feasible_primitive_.color.b = (float) vis_param_.feasible_primitive.color_b;
        feasible_primitive_.pose.orientation.w = 1.0;
        feasible_primitive_.pose.orientation.x = 0.0;
        feasible_primitive_.pose.orientation.y = 0.0;
        feasible_primitive_.pose.orientation.z = 0.0;
        feasible_primitive_.points.clear();
        // publish_feasible_primitive_ = true;
        // BEST PRIMITIVE
        best_primitive_.header.frame_id = vis_param_.frame_id;
        best_primitive_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        best_primitive_.scale.x = vis_param_.best_primitive.line_scale;
        best_primitive_.ns = "TRACKER_BEST_PRIMITIVE";
        best_primitive_.color.a = (float) vis_param_.best_primitive.color_a;
        best_primitive_.color.r = (float) vis_param_.best_primitive.color_r;
        best_primitive_.color.g = (float) vis_param_.best_primitive.color_g;
        best_primitive_.color.b = (float) vis_param_.best_primitive.color_b;
        best_primitive_.pose.orientation.w = 1.0;
        best_primitive_.pose.orientation.x = 0.0;
        best_primitive_.pose.orientation.y = 0.0;
        best_primitive_.pose.orientation.z = 0.0;   
    }

    // (Yunwoo) Fixed: Removed tracker_idx loop, fixed ROS2 types, fixed param_ -> vis_param_
    visualization_msgs::msg::MarkerArray BPMPVisualizer::VisualizeRawPrimitives(const vector<bpmp_tracker::PrimitivePlanning> & primitive){
        visualization_msgs::msg::MarkerArray visual_output;
        if (not vis_param_.raw_primitive.publish)
            return visual_output;
        if (primitive.empty())
            return visual_output;
        
        raw_primitive_.ns = "TRACKER_RAW_PRIMITIVES";
        std::vector<double> time_seq;
        double seg_t0 = primitive[0].t0;
        double seg_tf = primitive[0].tf;
        for (int i = 0; i < vis_param_.raw_primitive.num_time_sample; i++)
            time_seq.push_back(seg_t0 + (double) i * (seg_tf - seg_t0) /
                                        (double) (vis_param_.raw_primitive.num_time_sample - 1));
        geometry_msgs::msg::Point temp_point;
        int num_primitive_vis = (int) ((double) primitive.size() * vis_param_.raw_primitive.proportion);
        if (num_primitive_vis < 1) num_primitive_vis = 1;
        int sample_step = primitive.size() / num_primitive_vis;
        if (sample_step < 1) sample_step = 1;
        double ctrl_pts_x[4];
        double ctrl_pts_y[4];
        double ctrl_pts_z[4];
        for (int idx = 0; idx < num_primitive_vis - 1; idx++) {
            raw_primitive_.points.clear();
            raw_primitive_.id = idx;
            for (int i = 0; i < 4; i++) {
                ctrl_pts_x[i] = primitive[sample_step * idx].ctrl_x[i];
                ctrl_pts_y[i] = primitive[sample_step * idx].ctrl_y[i];
                ctrl_pts_z[i] = primitive[sample_step * idx].ctrl_z[i];
            }
            for (size_t i = 0; i < time_seq.size(); i++) {
                temp_point.x =
                        getBernsteinValue(ctrl_pts_x, time_seq[i], primitive[sample_step * idx].t0,
                                          primitive[sample_step * idx].tf, 3);
                temp_point.y =
                        getBernsteinValue(ctrl_pts_y, time_seq[i], primitive[sample_step * idx].t0,
                                          primitive[sample_step * idx].tf, 3);
                temp_point.z =
                        getBernsteinValue(ctrl_pts_z, time_seq[i], primitive[sample_step * idx].t0,
                                          primitive[sample_step * idx].tf, 3);
                raw_primitive_.points.push_back(temp_point);
            }
            visual_output.markers.push_back(raw_primitive_);
        }
    
        return visual_output;
    }
    // (Yunwoo) Fixed: Removed tracker_idx loop, fixed ROS2 types, fixed param_ -> vis_param_
    visualization_msgs::msg::MarkerArray BPMPVisualizer::VisualizeFeasiblePrimitives(const vector<bpmp_tracker::PrimitivePlanning> &primitive, const vector<bpmp_tracker::uint> &feasible_index){
        visualization_msgs::msg::MarkerArray visual_output;
        if (not vis_param_.feasible_primitive.publish)
            return visual_output;
        if (primitive.empty())
            return visual_output;
        if (feasible_index.empty())
            return visual_output;
        
        feasible_primitive_.ns = "TRACKER_FEASIBLE_PRIMITIVES";
        std::vector<double> time_seq;
        double seg_t0 = primitive[0].t0;
        double seg_tf = primitive[0].tf;
        for (int i = 0; i < vis_param_.feasible_primitive.num_time_sample; i++)
            time_seq.push_back(seg_t0 + (double) i * (seg_tf - seg_t0) /
                                        (double) (vis_param_.feasible_primitive.num_time_sample - 1));
        geometry_msgs::msg::Point temp_point;
        int num_primitive_vis =
                (int) ((double) feasible_index.size() * vis_param_.feasible_primitive.proportion);
        int sample_step = 1;
        if (num_primitive_vis > 0)
            sample_step = feasible_index.size() / num_primitive_vis;
        else
            num_primitive_vis = 1;
        if (sample_step < 1) sample_step = 1;

        double ctrl_pts_x[4];
        double ctrl_pts_y[4];
        double ctrl_pts_z[4];
        for (int idx = 0; idx < num_primitive_vis - 1; idx++) {
            feasible_primitive_.points.clear();
            feasible_primitive_.id = idx;
            int prim_idx = feasible_index[sample_step * idx];
            for (int i = 0; i < 4; i++) {
                ctrl_pts_x[i] = primitive[prim_idx].ctrl_x[i];
                ctrl_pts_y[i] = primitive[prim_idx].ctrl_y[i];
                ctrl_pts_z[i] = primitive[prim_idx].ctrl_z[i];
            }
            for (size_t i = 0; i < time_seq.size(); i++) {
                temp_point.x = getBernsteinValue(
                        ctrl_pts_x, time_seq[i],
                        primitive[prim_idx].t0,
                        primitive[prim_idx].tf, 3);
                temp_point.y = getBernsteinValue(
                        ctrl_pts_y, time_seq[i],
                        primitive[prim_idx].t0,
                        primitive[prim_idx].tf, 3);
                temp_point.z = getBernsteinValue(
                        ctrl_pts_z, time_seq[i],
                        primitive[prim_idx].t0,
                        primitive[prim_idx].tf, 3);
                feasible_primitive_.points.push_back(temp_point);
            }
            visual_output.markers.push_back(feasible_primitive_);
        }
        
        // ERASE old markers
        {
            static int last_num_id = 2000;
            visualization_msgs::msg::Marker erase_marker;
            erase_marker.action = visualization_msgs::msg::Marker::DELETE;
            erase_marker.ns = "TRACKER_FEASIBLE_PRIMITIVES";
            erase_marker.header.frame_id = vis_param_.frame_id;
            for (int i = num_primitive_vis - 1; i <= last_num_id; i++) {
                erase_marker.id = i;
                visual_output.markers.push_back(erase_marker);
            }
            last_num_id = num_primitive_vis - 1;
        }
        
        return visual_output;
    }
    
    // (Yunwoo) Fixed: ROS2 type
    visualization_msgs::msg::Marker BPMPVisualizer::VisualizeCorridor(const vector<AffineCoeff3D> &corridor_constraints){
        visualization_msgs::msg::Marker visual_output;
        if (corridor_constraints.empty())
            return visual_output;

        Vec3List vertices = GetFeasibleVertices(corridor_constraints);
        return VisualizeConvexHull(vertices);
    }

    // (Yunwoo) Fixed: int -> size_t, removed redundant double call, int -> Eigen::Index
    Vec3List BPMPVisualizer::GetFeasibleVertices(const vector<AffineCoeff3D> &constraint){
        Vec3List vertices;
        if (constraint.empty())
            return vertices;
        bpmp_tracker::AffineCoeff3D single_constraint;
        vector<bpmp_tracker::AffineCoeff3D> constraint_list;
    
        for (size_t idx = 0; idx < constraint.size(); idx++) {
            single_constraint[0] = constraint[idx][0];
            single_constraint[1] = constraint[idx][1];
            single_constraint[2] = constraint[idx][2];
            single_constraint[3] = constraint[idx][3];
            constraint_list.push_back(single_constraint);
        }
    
        Eigen::Matrix3Xd vertices_mtx;
        Eigen::MatrixX4d hs_mtx(constraint_list.size(), 4);
        int row = 0;
        for (const auto &h: constraint_list) {
            // h = {x | n \dot (x - p) - d >= 0}
            // h1*x + h2*y +h3*z+ h4 <=0
            hs_mtx(row, 0) = h[0];
            hs_mtx(row, 1) = h[1];
            hs_mtx(row, 2) = h[2];
            hs_mtx(row, 3) = h[3];
            row++;
        }
        if (not geo_utils::enumerateVs(hs_mtx, vertices_mtx)) {
            std::cout << "[Constraints]: Fail to find feasible vertices!" << std::endl;
            return vertices;
        }
        for (Eigen::Index i = 0; i < vertices_mtx.cols(); i++)
            vertices.emplace_back(vertices_mtx.col(i)(0), vertices_mtx.col(i)(1), vertices_mtx.col(i)(2));
    
        return vertices;
    }

    // (Yunwoo) Fixed: Removed tracker_idx and cell_type, simplified to use corridor_
    visualization_msgs::msg::Marker BPMPVisualizer::VisualizeConvexHull(const Vec3List &convex_hull){
        if (convex_hull.empty())
            return visualization_msgs::msg::Marker{};
        
        corridor_.colors.clear();
        corridor_.points.clear();
        corridor_.id = 0;
        corridor_.header.frame_id = "map";
        corridor_.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        corridor_.ns = "SAFE_CORRIDOR";
        corridor_.action = visualization_msgs::msg::Marker::ADD;
        corridor_.color.r= 0.2;
        corridor_.color.g= 0.8;
        corridor_.color.b= 1.0;
        corridor_.color.a= 0.5;
        corridor_.scale.x = 1.0;
        corridor_.scale.y = 1.0;
        corridor_.scale.z = 1.0;
        corridor_.pose.orientation.w = 1.0;
        corridor_.pose.orientation.x = 0.0;
        corridor_.pose.orientation.y = 0.0;
        corridor_.pose.orientation.z = 0.0;
        corridor_.pose.position.x = 0.0;
        corridor_.pose.position.y = 0.0;
        corridor_.pose.position.z = 0.0;
        corridor_color_.a = 0.5;
        corridor_color_.r = 0.2;
        corridor_color_.g = 0.8;
        corridor_color_.b = 1.0;

        size_t num_vertices = convex_hull.size();
        ch_vertex *vertices;
        vertices = (ch_vertex *) malloc(num_vertices * sizeof(ch_vertex));
        for (size_t i = 0; i < num_vertices; i++) {
            vertices[i].x = convex_hull[i].x();
            vertices[i].y = convex_hull[i].y();
            vertices[i].z = convex_hull[i].z();
        }
        int *face_indices = nullptr;
        int num_faces;
        convhull_3d_build(vertices, num_vertices, &face_indices, &num_faces);
        
        PointMsg vertex;
        for (int i = 0; i < num_faces; i++) {
            auto vertex1 = vertices[face_indices[i * 3]];
            auto vertex2 = vertices[face_indices[i * 3 + 1]];
            auto vertex3 = vertices[face_indices[i * 3 + 2]];
            vertex.x = vertex1.x, vertex.y = vertex1.y, vertex.z = vertex1.z;
            corridor_.points.emplace_back(vertex);
            vertex.x = vertex2.x, vertex.y = vertex2.y, vertex.z = vertex2.z;
            corridor_.points.emplace_back(vertex);
            vertex.x = vertex3.x, vertex.y = vertex3.y, vertex.z = vertex3.z;
            corridor_.points.emplace_back(vertex);
            corridor_.colors.push_back(corridor_color_);
        }

        free(vertices);
        free(face_indices);
        
        return corridor_;
    }
}

