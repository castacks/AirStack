/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <deque>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <hgp/utils.hpp>
#include <mighty/mighty_type.hpp>

#include "rclcpp/rclcpp.hpp"

#include "dynus_interfaces/msg/coeff_poly3.hpp"
#include "dynus_interfaces/msg/dyn_traj.hpp"
#include "dynus_interfaces/msg/pwp_traj.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mighty_utils {

// Define colors
static constexpr int red_normal = 1;
static constexpr int red_trans = 2;
static constexpr int red_trans_trans = 3;
static constexpr int green_normal = 4;
static constexpr int blue_normal = 5;
static constexpr int blue_trans = 6;
static constexpr int blue_trans_trans = 7;
static constexpr int blue_light = 8;
static constexpr int yellow_normal = 9;
static constexpr int orange_trans = 10;
static constexpr int black_trans = 11;
static constexpr int teal_normal = 12;
static constexpr int green_trans_trans = 13;

/** @brief Convert a PWPTraj ROS message to a PieceWisePol struct.
 *  @param pwp_msg Input ROS message.
 *  @return Equivalent PieceWisePol.
 */
PieceWisePol convertPwpMsg2Pwp(const dynus_interfaces::msg::PWPTraj& pwp_msg);

/** @brief Convert a QuinticPWPTraj ROS message to a PieceWiseQuinticPol struct.
 *  @param pwp_msg Input ROS message.
 *  @return Equivalent PieceWiseQuinticPol.
 */
PieceWiseQuinticPol convertPwpMsg2Pwp(const dynus_interfaces::msg::QuinticPWPTraj& pwp_msg);

/** @brief Convert a PieceWisePol to a PWPTraj ROS message.
 *  @param pwp Input piecewise polynomial.
 *  @return Equivalent ROS message.
 */
dynus_interfaces::msg::PWPTraj convertPwp2PwpMsg(const PieceWisePol& pwp);

/** @brief Convert a PieceWiseQuinticPol to a QuinticPWPTraj ROS message.
 *  @param pwp Input piecewise quintic polynomial.
 *  @return Equivalent ROS message.
 */
dynus_interfaces::msg::QuinticPWPTraj convertPwp2PwpMsg(const PieceWiseQuinticPol& pwp);

/** @brief Convert a PieceWisePol to a colored marker array for RViz visualization.
 *  @param pwp Piecewise polynomial trajectory.
 *  @param samples Number of sample points per segment.
 *  @return MarkerArray for visualization.
 */
visualization_msgs::msg::MarkerArray convertPwp2ColoredMarkerArray(PieceWisePol& pwp, int samples);

/** @brief Convert an Eigen::Vector3d to a geometry_msgs::msg::Point.
 *  @param vector Input 3D vector.
 *  @return Equivalent Point message.
 */
geometry_msgs::msg::Point convertEigen2Point(Eigen::Vector3d vector);

/** @brief Convert a float vector covariance message to Eigen::Vector3d.
 *  @param msg_cov Input covariance as a float vector (size 3).
 *  @return Eigen::Vector3d covariance.
 */
Eigen::Vector3d convertCovMsg2Cov(const std::vector<float>& msg_cov);

/** @brief Convert an Eigen::Vector3d covariance to a float vector message.
 *  @param cov Input covariance.
 *  @return Float vector representation.
 */
std::vector<float> convertCov2CovMsg(const Eigen::Vector3d& cov);

/** @brief Convert a float vector coefficient message to an Eigen 6x1 vector.
 *  @param msg_coeff Input coefficients as a float vector (size 6).
 *  @return Eigen 6x1 coefficient vector.
 */
Eigen::Matrix<double, 6, 1> convertCoeffMsg2Coeff(const std::vector<float>& msg_coeff);

/** @brief Get a predefined RGBA color by ID.
 *  @param id Color identifier constant (e.g., red_normal, blue_trans).
 *  @return ColorRGBA message.
 */
std_msgs::msg::ColorRGBA getColor(int id);

/** @brief Convert polynomial coefficients to Bezier control points.
 *  @param pwp Piecewise polynomial trajectory.
 *  @param A_rest_pos_basis_inverse Inverse of the rest-position basis matrix.
 *  @return Vector of 3x4 control point matrices per segment.
 */
std::vector<Eigen::Matrix<double, 3, 4>> convertCoefficients2ControlPoints(
    const PieceWisePol& pwp, const Eigen::Matrix<double, 4, 4>& A_rest_pos_basis_inverse);

/** @brief Compute minimum time for a 1D double-integrator point-to-point transfer.
 *  @param p0 Start position.
 *  @param v0 Start velocity.
 *  @param pf Final position.
 *  @param vf Final velocity.
 *  @param v_max Maximum velocity.
 *  @param a_max Maximum acceleration.
 *  @return Minimum transfer time.
 */
double getMinTimeDoubleIntegrator1D(const double p0, const double v0, const double pf,
                                    const double vf, const double v_max, const double a_max);

/** @brief Compute minimum time for a 3D double-integrator point-to-point transfer.
 *  @param p0 Start position.
 *  @param v0 Start velocity.
 *  @param pf Final position.
 *  @param vf Final velocity.
 *  @param v_max Per-axis maximum velocity.
 *  @param a_max Per-axis maximum acceleration.
 *  @return Minimum transfer time (max over all axes).
 */
double getMinTimeDoubleIntegrator3D(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0,
                                    const Eigen::Vector3d& pf, const Eigen::Vector3d& vf,
                                    const Eigen::Vector3d& v_max, const Eigen::Vector3d& a_max);

/** @brief Convert a tf2 quaternion to Euler angles (roll, pitch, yaw).
 *  @param q Input quaternion.
 *  @param roll Output roll angle.
 *  @param pitch Output pitch angle.
 *  @param yaw Output yaw angle.
 */
void quaternion2Euler(tf2::Quaternion q, double& roll, double& pitch, double& yaw);

/** @brief Convert an Eigen quaternion to Euler angles (roll, pitch, yaw).
 *  @param q Input quaternion.
 *  @param roll Output roll angle.
 *  @param pitch Output pitch angle.
 *  @param yaw Output yaw angle.
 */
void quaternion2Euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw);

/** @brief Convert a geometry_msgs quaternion to Euler angles (roll, pitch, yaw).
 *  @param q Input quaternion message.
 *  @param roll Output roll angle.
 *  @param pitch Output pitch angle.
 *  @param yaw Output yaw angle.
 */
void quaternion2Euler(geometry_msgs::msg::Quaternion q, double& roll, double& pitch, double& yaw);

/** @brief Clamp a value to the range [min, max].
 *  @param var Value to saturate (modified in place).
 *  @param min Minimum bound.
 *  @param max Maximum bound.
 */
void saturate(double& var, double min, double max);

/** @brief Wrap an angle difference to the range [-pi, pi].
 *  @param diff Angle difference (modified in place).
 */
void angle_wrap(double& diff);

/** @brief Project point P2 onto the boundary of a box centered at P1.
 *  @param P1 Box center.
 *  @param P2 Point to project.
 *  @param wdx Box half-width in x.
 *  @param wdy Box half-width in y.
 *  @param wdz Box half-width in z.
 *  @return Projected point on the box boundary.
 */
Eigen::Vector3d projectPointToBox(Eigen::Vector3d& P1, Eigen::Vector3d& P2, double wdx, double wdy,
                                  double wdz);

/** @brief Project point P2 onto a sphere centered at P1.
 *  @param P1 Sphere center.
 *  @param P2 Point to project.
 *  @param radius Sphere radius.
 *  @return Projected point on the sphere surface.
 */
Eigen::Vector3d projectPointToSphere(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2,
                                     double radius);

/** @brief Convert a MarkerArray to a vector of vec_Vecf<3> paths with scales.
 *  @param marker_array Input marker array.
 *  @param vec Output vector of paths.
 *  @param scale_vec Output vector of marker scales.
 */
void convertMarkerArray2Vec_Vec_Vecf3(const visualization_msgs::msg::MarkerArray& marker_array,
                                      std::vector<vec_Vecf<3>>& vec,
                                      std::vector<double>& scale_vec);

/** @brief Subdivide path segments so no segment exceeds length d.
 *  @param path Path to refine (modified in place).
 *  @param d Maximum segment length.
 */
void createMoreVertexes(vec_Vecf<3>& path, double d);

/** @brief Remove consecutive points closer than min_spacing.
 *  @param path Path to filter (modified in place).
 *  @param min_spacing Minimum allowed distance between consecutive points.
 */
void enforceMinimumSpacing(vec_Vecf<3>& path, double min_spacing);

/** @brief Resample a path at uniform arc-length intervals.
 *  @param path Path to resample (modified in place).
 *  @param spacing Desired spacing between consecutive points.
 */
void resamplePathUniform(vec_Vecf<3>& path, double spacing);

/** @brief Compute Euclidean distance between two 3D points.
 *  @param p1 First point.
 *  @param p2 Second point.
 *  @return Distance.
 */
double euclideanDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);

/** @brief Convert a TransformStamped message to a 4x4 homogeneous transformation matrix.
 *  @param transform_stamped Input transform.
 *  @return 4x4 transformation matrix.
 */
Eigen::Matrix4d transformStampedToMatrix(
    const geometry_msgs::msg::TransformStamped& transform_stamped);

/** @brief Estimate velocities at each waypoint of a path using dynamics limits.
 *  @param path Input waypoint path.
 *  @param velocities Output velocity vectors at each waypoint.
 *  @param A Start state (provides initial velocity).
 *  @param v_max_3d Per-axis maximum velocity.
 *  @param verbose Enable debug output.
 */
void findVelocitiesInPath(const vec_Vecf<3>& path, vec_Vecf<3>& velocities, const state& A,
                          const Eigen::Vector3d& v_max_3d, bool verbose);

/** @brief Compute segment travel times for a path using double-integrator dynamics.
 *  @param path Waypoint path.
 *  @param A Start state (provides initial velocity).
 *  @param debug_verbose Enable debug output.
 *  @param v_max_3d Per-axis maximum velocity.
 *  @param a_max_3d Per-axis maximum acceleration.
 *  @return Vector of segment travel times.
 */
std::vector<double> getTravelTimes(const vec_Vecf<3>& path, const state& A, bool debug_verbose,
                                   const Eigen::Vector3d& v_max_3d,
                                   const Eigen::Vector3d& a_max_3d);

/** @brief Create an identity geometry_msgs::msg::Pose.
 *  @return Pose with zero position and identity quaternion.
 */
geometry_msgs::msg::Pose identityGeometryMsgsPose();

/** @brief Signum function.
 *  @param val Input value.
 *  @return -1, 0, or 1.
 */
template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

}  // namespace mighty_utils
