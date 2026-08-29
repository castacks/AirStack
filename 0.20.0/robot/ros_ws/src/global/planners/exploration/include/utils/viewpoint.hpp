// Modified from cmusubt drone_traj_planner

#ifndef CUSTOM_POINT_H
#define CUSTOM_POINT_H

#include <vector>
#include <cmath>
#include <iostream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define PI 3.14159265359
#define TWO_PI 6.28318530718

// Define a Point
struct ViewPoint
{
    // Position
    double x;
    double y;
    double z;

    bool orientation_set = false;
    geometry_msgs::msg::Quaternion orientation;
    double orientation_yaw; // equivalent to the yaw within orientation

    // Scoring
    bool score_initialized = false;
    double score;
    bool cleared_by_other_robot = false;

    // Viewpoint score (from map processor)
    bool viewpoint_score_initialized = false;
    double viewpoint_score;
};

// Define a set of Points
typedef std::vector<ViewPoint> PointSet;

// Define operators (for map to work)
static inline bool operator>(const ViewPoint &i, const ViewPoint &j)
{
    if (i.x > j.x)
    {
        return true;
    }
    else if (i.x == j.x)
    {
        if (i.y > j.y)
        {
            return true;
        }
        else if (i.y == j.y)
        {
            if (i.z > j.z)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

static inline bool operator<(const ViewPoint &i, const ViewPoint &j)
{
    if (i.x < j.x)
    {
        return true;
    }
    else if (i.x == j.x)
    {
        if (i.y < j.y)
        {
            return true;
        }
        else if (i.y == j.y)
        {
            if (i.z < j.z)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

static inline void printPoint(const ViewPoint &p)
{
    std::cout << "(" << p.x << ", " << p.y << ", " << p.z << ") {" << p.orientation.x << ", " << p.orientation.y << ", " << p.orientation.z << ", " << p.orientation.w << "}" << std::endl;
}

static inline double distancePoint2Point(const ViewPoint &p1, const ViewPoint &p2)
{
    // Euclidean distance
    double x_diff = p1.x - p2.x;
    double y_diff = p1.y - p2.y;
    double z_diff = p1.z - p2.z;

    return sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}

static inline double angleDiffPoint2Point(const ViewPoint &p1, const ViewPoint &p2)
{
    if (!p1.orientation_set || !p2.orientation_set)
    {
        return 0.0;
    }
    else
    {
        double angle_diff = p2.orientation_yaw - p1.orientation_yaw;

        // wrap around
        if (angle_diff > PI)
        {
            angle_diff -= TWO_PI;
        }
        else if (angle_diff < -PI)
        {
            angle_diff += TWO_PI;
        }

        return angle_diff;
    }
}

/////////////////////
// PCL
typedef pcl::PointXYZI PCLPointType;
typedef pcl::PointCloud<PCLPointType> PCLCloudType;

static inline Eigen::Vector3d point2eigen(const ViewPoint& p)
{
    return Eigen::Vector3d(p.x, p.y, p.z);
}

static inline openvdb::Vec3d point2vdb(const ViewPoint& p)
{
    return openvdb::Vec3d(p.x, p.y, p.z);
}

static inline PCLPointType point2PCL(const ViewPoint &p)
{
    PCLPointType point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    // point.intensity = p.score;
    point.intensity = 0.0;
    return point;
}

static inline ViewPoint pcl2Point(const PCLPointType &pcl_point)
{
    ViewPoint point;
    point.x = pcl_point.x;
    point.y = pcl_point.y;
    point.z = pcl_point.z;
    return point;
}

static inline PCLCloudType pointSet2PCL(const PointSet &ps)
{
    PCLCloudType pcl_cloud;
    for (const auto &p : ps)
    {
        PCLPointType pcl_point = point2PCL(p);
        pcl_cloud.push_back(pcl_point);
    }
    return pcl_cloud;
}

/////////////////////
// Geometry msgs
static inline ViewPoint geometryMsgs2Point(const geometry_msgs::msg::Point &geom_point)
{
    ViewPoint point;
    point.x = geom_point.x;
    point.y = geom_point.y;
    point.z = geom_point.z;
    return point;
}

static inline ViewPoint geometryMsgs2Point(const geometry_msgs::msg::Pose& geom_pose)
{
    ViewPoint point;
    point.x = geom_pose.position.x;
    point.y = geom_pose.position.y;
    point.z = geom_pose.position.z;

    // tf2 (ROS 2-friendly)
    double roll, pitch, yaw;
    tf2::Quaternion quat_tf;
    tf2::fromMsg(geom_pose.orientation, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

    point.orientation = geom_pose.orientation;
    point.orientation_set = true;
    point.orientation_yaw = yaw;
    return point;
}

static inline geometry_msgs::msg::Point point2GeometryMsgs(const ViewPoint &point)
{
    geometry_msgs::msg::Point geom_point;
    geom_point.x = point.x;
    geom_point.y = point.y;
    geom_point.z = point.z;
    return geom_point;
}

static inline geometry_msgs::msg::Pose point2GeometryMsgsPose(const ViewPoint &point)
{
    geometry_msgs::msg::Pose geom_pose;
    geom_pose.position.x = point.x;
    geom_pose.position.y = point.y;
    geom_pose.position.z = point.z;
    if (point.orientation_set)
    {
        geom_pose.orientation = point.orientation;
    }
    else
    {
        geom_pose.orientation.x = 0.0;
        geom_pose.orientation.y = 0.0;
        geom_pose.orientation.z = 0.0;
        geom_pose.orientation.w = 1.0;
    }
    return geom_pose;
}

static inline geometry_msgs::msg::PoseArray pointSet2GeometryMsgsPoseArray(const PointSet &point_set)
{
    geometry_msgs::msg::PoseArray msg;
    for (const auto &point : point_set)
    {
        msg.poses.push_back(point2GeometryMsgsPose(point));
    }
    return msg;
}

static inline nav_msgs::msg::Path pointSet2NavMsgsPath(const PointSet &point_set, const std_msgs::msg::Header &header)
{
    nav_msgs::msg::Path msg;
    msg.header = header;
    for (const auto &point : point_set)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.pose = point2GeometryMsgsPose(point);
        msg.poses.push_back(pose_stamped);
    }
    return msg;
}

///////////////////
// Viewpoints
static inline ViewPoint pclHSVtoPoint(const pcl::PointXYZHSV &pcl_point)
{
    ViewPoint point;

    // Position
    point.x = pcl_point.x;
    point.y = pcl_point.y;
    point.z = pcl_point.z;

    // Score (from HSV 's' field)
    point.viewpoint_score = pcl_point.s;
    point.viewpoint_score_initialized = true;

    // Orientation from heading (HSV 'h' field)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pcl_point.h); // roll=0, pitch=0, yaw=heading
    point.orientation = tf2::toMsg(q);
    point.orientation_set = true;

    // Store yaw separately
    point.orientation_yaw = pcl_point.h;

    return point;
}

#endif