#ifndef BPMP_VISUALIZER_H
#define BPMP_VISUALIZER_H

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "bpmp_visualizer/geo_utils.hpp"
#include "bpmp_visualizer/convhull_3d.h"

#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "bpmp_utils/bpmp_utils.h"

typedef std_msgs::msg::ColorRGBA ColorMsg;    // (Yunwoo) ROS2 style
typedef std::vector<ColorMsg> ColorMsgs;
typedef geometry_msgs::msg::Point PointMsg;   // (Yunwoo) ROS2 style
typedef Eigen::Vector3d Vec3;
typedef std::vector<Eigen::Vector3d> Vec3List;
typedef Eigen::Vector4d Vec4;
typedef std::vector<Eigen::Vector4d> Vec4List;


namespace bpmp_tracker{
struct VisualizationParams{
    std::string frame_id;
    struct{
        bool publish{false};
        int num_time_sample{10};
        double proportion{0.0};
        double line_scale{0.01};
        double color_a{0.0};
        double color_r{0.0};
        double color_g{0.0};
        double color_b{0.0};
    }raw_primitive;

    struct{
        bool publish{true};
        int num_time_sample{10};
        double proportion{0.2};
        double line_scale{0.01};
        double color_a{0.2};
        double color_r{0.0};
        double color_g{1.0};
        double color_b{0.0};
    }feasible_primitive;

    struct{
        bool publish{false};  // (Yunwoo) Added publish flag
        int num_time_sample{10};
        double line_scale{0.01};
        double color_a{0.0};
        double color_r{0.0};
        double color_g{0.0};
        double color_b{0.0};
    }best_primitive;

    struct{
        bool publish{true};  // (Yunwoo) Added publish flag
        double color_a{0.5};
        double color_r{1.0};
        double color_g{0.0};
        double color_b{1.0};
    }corridor;
};

class BPMPVisualizer{
    private:
    bpmp_tracker::VisualizationParams vis_param_;
    ColorMsg corridor_color_;
    visualization_msgs::msg::Marker corridor_;           // (Yunwoo) ROS2 style
    visualization_msgs::msg::Marker raw_primitive_;      // (Yunwoo) ROS2 style
    visualization_msgs::msg::Marker feasible_primitive_; // (Yunwoo) ROS2 style
    visualization_msgs::msg::Marker best_primitive_;     // (Yunwoo) Added missing declaration

    inline PointMsg GetDefaultPointMsg(){
        PointMsg p;
        p.x = 0.0, p.y = 0.0, p.z = 0.0;
        return p;
    }
    inline geometry_msgs::msg::Quaternion GetDefaultQuaternionMsg(){   // (Yunwoo) ROS2 style
        geometry_msgs::msg::Quaternion q;                               // (Yunwoo) ROS2 style
        q.w = 1.0, q.x = 0.0, q.y = 0.0, q.z =0.0;
        return q;
    }
    inline geometry_msgs::msg::Vector3 GetDefaultScaleMsg(){           // (Yunwoo) ROS2 style
        geometry_msgs::msg::Vector3 v;                                  // (Yunwoo) ROS2 style
        v.x = 1.0, v.y = 1.0, v.z = 1.0;
        return v;
    }

    public:
    BPMPVisualizer() = default;  // (Yunwoo) Default constructor    
    BPMPVisualizer(const bpmp_tracker::VisualizationParams& vis_param);
    void UpdateParams(const bpmp_tracker::VisualizationParams& vis_param);

    visualization_msgs::msg::MarkerArray VisualizeRawPrimitives(const vector<bpmp_tracker::PrimitivePlanning> & primitive); // (Yunwoo) ROS2 style
    visualization_msgs::msg::MarkerArray VisualizeFeasiblePrimitives(const vector<bpmp_tracker::PrimitivePlanning> &primitive, const vector<bpmp_tracker::uint> &feasible_index); // (Yunwoo) ROS2 style
    visualization_msgs::msg::Marker VisualizeCorridor(const vector<AffineCoeff3D> &corridor_constraints);
    Vec3List GetFeasibleVertices(const vector<AffineCoeff3D> &constraint);
    visualization_msgs::msg::Marker VisualizeConvexHull(const Vec3List &convex_hull);
};

}
#endif