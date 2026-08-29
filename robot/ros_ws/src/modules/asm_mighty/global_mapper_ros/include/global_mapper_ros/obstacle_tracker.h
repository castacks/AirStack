#pragma once

#include <vector>
#include <utility>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <rclcpp/rclcpp.hpp>
#include <dynus_interfaces/msg/dyn_traj.hpp>
#include <dynus_interfaces/msg/pwp_traj.hpp>
#include <dynus_interfaces/msg/coeff_poly3.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace global_mapper_ros
{

// Local PieceWisePol (avoids dependency on dynus package)
struct PieceWisePol
{
  std::vector<double> times;
  std::vector<Eigen::Matrix<double, 4, 1>> coeff_x;
  std::vector<Eigen::Matrix<double, 4, 1>> coeff_y;
  std::vector<Eigen::Matrix<double, 4, 1>> coeff_z;
};

dynus_interfaces::msg::PWPTraj convertPwp2PwpMsg(const PieceWisePol& pwp);

// EKF state for 9-state constant-acceleration model
struct EKFState
{
  Eigen::VectorXd x;   // [x, y, z, vx, vy, vz, ax, ay, az]
  Eigen::MatrixXd P;   // state covariance
  Eigen::MatrixXd Q;   // process noise covariance
  Eigen::MatrixXd R;   // measurement noise covariance
  double time_updated = 0.0;
  Eigen::Vector3d bbox = Eigen::Vector3d::Zero();
  int id = 0;
  std_msgs::msg::ColorRGBA color;

  // Rolling history of observed positions (timestamp, position)
  std::vector<std::pair<double, Eigen::Vector3d>> position_history;

  EKFState() = default;
  EKFState(int state_size, const Eigen::MatrixXd& Q_init,
           const Eigen::MatrixXd& R_init, double time,
           const Eigen::Vector3d& bbox_init, int id_val);
};

// Cluster: centroid + associated EKF state
struct Cluster
{
  EKFState ekf_state;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
};

// Configurable parameters
struct ObstacleTrackerParams
{
  bool enabled = false;
  bool use_adaptive_kf = true;
  double adaptive_kf_alpha = 0.90;
  double adaptive_kf_dt = 0.1;
  double cluster_tolerance = 1.0;
  int min_cluster_size = 20;
  int max_cluster_size = 2000;
  double prediction_horizon = 1.0;
  double prediction_dt = 0.1;
  double time_to_delete_old_obstacles = 5.0;
  double cluster_bbox_cutoff_size = 2.0;
  double velocity_threshold = 0.8;
  double acceleration_threshold = 2.0;
  double cutoff_length_threshold = 0.1;
  int degree_for_pwp = 3;
  int degree_for_poly = 5;
  int max_history_size = 30;
  int min_observations_for_prediction = 5;
  double max_obstacle_velocity = 2.0;
};

// A tracked obstacle's position + bounding box (for feedback to dynamic cloud)
struct TrackedObstacle
{
  Eigen::Vector3d position;
  Eigen::Vector3d bbox;  // half-extents
  int id;
};

// Result of a single tracking update
struct TrackingResult
{
  std::vector<dynus_interfaces::msg::DynTraj> trajectories;
  visualization_msgs::msg::MarkerArray bbox_markers;
  visualization_msgs::msg::MarkerArray prediction_markers;
};

// Obstacle tracker: clustering + EKF tracking + trajectory prediction
class ObstacleTracker
{
public:
  explicit ObstacleTracker(const ObstacleTrackerParams& params,
                           rclcpp::Logger logger);

  TrackingResult update(const pcl::PointCloud<pcl::PointXYZ>::Ptr& dynamic_cloud,
                        double current_time_sec);

  // Return active tracked obstacles (position + bbox) for feedback to dynamic cloud
  std::vector<TrackedObstacle> getTrackedObstacles() const;

private:
  // EKF operations
  static void ekfPredict(EKFState& state, double dt);
  static void aekfUpdate(EKFState& state, const Eigen::VectorXd& z,
                         double alpha, double time_updated,
                         const Eigen::Vector3d& bbox, bool use_adaptive);

  // Data association
  static int associateCluster(const Eigen::Vector3d& centroid,
                              const std::vector<EKFState>& states,
                              double tolerance);

  // Polynomial fitting
  static Eigen::VectorXd polyfit(const std::vector<double>& t,
                                 const std::vector<double>& y, int degree);
  static double calculateVariance(const std::vector<double>& t,
                                  const std::vector<double>& y,
                                  const Eigen::VectorXd& beta, int degree);

  // Housekeeping
  void deleteOldStates(double current_time);
  void calculateAverageQandR(Eigen::MatrixXd& Q_avg, Eigen::MatrixXd& R_avg);

  // Clustering
  void getCentroidsAndSizes(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                            const std::vector<pcl::PointIndices>& indices,
                            std::vector<Eigen::Vector3d>& centroids,
                            std::vector<Eigen::Vector3d>& bboxes);

  // History management
  void appendToHistory(EKFState& state, double time, const Eigen::Vector3d& pos);

  // Prediction output generation (history-based)
  void generatePredictions(const std::vector<Cluster>& clusters,
                           double current_time_sec,
                           TrackingResult& result);
  void generateBoxMarkers(const std::vector<Cluster>& clusters,
                          TrackingResult& result);

  ObstacleTrackerParams params_;
  rclcpp::Logger logger_;
  std::vector<EKFState> ekf_states_;
  int next_ekf_id_ = 0;
  int marker_id_ = 0;
};

}  // namespace global_mapper_ros
