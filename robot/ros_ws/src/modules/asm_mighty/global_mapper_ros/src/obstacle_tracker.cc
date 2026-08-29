// Obstacle tracker: clustering + EKF tracking + history-based trajectory prediction
// Ported from dynus/obstacle_tracker_node.cpp into global_mapper_ros
// Improvements over original:
//   1. EKF predict uses actual elapsed dt (not fixed)
//   2. Per-obstacle position history stored for polynomial fitting
//   3. Polynomials fit to real observations (not forward-simulated points)
//   4. Minimum observation count required before publishing predictions

#include "global_mapper_ros/obstacle_tracker.h"

#include <limits>
#include <cmath>
#include <cstdlib>

#include <pcl/search/kdtree.h>

namespace global_mapper_ros
{

// ---------------------------------------------------------------------------
// convertPwp2PwpMsg  (local replica of dynus_utils::convertPwp2PwpMsg)
// ---------------------------------------------------------------------------
dynus_interfaces::msg::PWPTraj convertPwp2PwpMsg(const PieceWisePol& pwp)
{
  dynus_interfaces::msg::PWPTraj pwp_msg;
  for (size_t i = 0; i < pwp.times.size(); ++i)
    pwp_msg.times.push_back(pwp.times[i]);

  for (const auto& c : pwp.coeff_x)
  {
    dynus_interfaces::msg::CoeffPoly3 cp;
    cp.a = c(0); cp.b = c(1); cp.c = c(2); cp.d = c(3);
    pwp_msg.coeff_x.push_back(cp);
  }
  for (const auto& c : pwp.coeff_y)
  {
    dynus_interfaces::msg::CoeffPoly3 cp;
    cp.a = c(0); cp.b = c(1); cp.c = c(2); cp.d = c(3);
    pwp_msg.coeff_y.push_back(cp);
  }
  for (const auto& c : pwp.coeff_z)
  {
    dynus_interfaces::msg::CoeffPoly3 cp;
    cp.a = c(0); cp.b = c(1); cp.c = c(2); cp.d = c(3);
    pwp_msg.coeff_z.push_back(cp);
  }
  return pwp_msg;
}

// ---------------------------------------------------------------------------
// EKFState constructor
// ---------------------------------------------------------------------------
EKFState::EKFState(int state_size, const Eigen::MatrixXd& Q_init,
                   const Eigen::MatrixXd& R_init, double time,
                   const Eigen::Vector3d& bbox_init, int id_val)
{
  x = Eigen::VectorXd::Zero(state_size);
  P = Eigen::MatrixXd::Identity(state_size, state_size);
  Q = Q_init;
  R = R_init;
  time_updated = time;
  bbox = bbox_init;
  id = id_val;
  // Random color for visualization
  color.r = static_cast<float>(rand()) / RAND_MAX;
  color.g = static_cast<float>(rand()) / RAND_MAX;
  color.b = static_cast<float>(rand()) / RAND_MAX;
  color.a = 0.4f;
}

// ---------------------------------------------------------------------------
// ObstacleTracker
// ---------------------------------------------------------------------------
ObstacleTracker::ObstacleTracker(const ObstacleTrackerParams& params,
                                 rclcpp::Logger logger)
  : params_(params), logger_(logger)
{
}

TrackingResult ObstacleTracker::update(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& dynamic_cloud,
    double current_time_sec)
{
  TrackingResult result;

  if (!dynamic_cloud || dynamic_cloud->empty())
    return result;

  // 1) Euclidean Cluster Extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(dynamic_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(params_.cluster_tolerance);
  ec.setMinClusterSize(params_.min_cluster_size);
  ec.setMaxClusterSize(params_.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(dynamic_cloud);
  ec.extract(cluster_indices);

  // 2) Compute centroids & bounding boxes
  std::vector<Eigen::Vector3d> centroids;
  std::vector<Eigen::Vector3d> bboxes;
  getCentroidsAndSizes(dynamic_cloud, cluster_indices, centroids, bboxes);

  // 3) Delete old EKF states
  deleteOldStates(current_time_sec);

  // 4) Data association + EKF predict/update
  std::vector<Cluster> clusters;
  for (size_t i = 0; i < cluster_indices.size(); ++i)
  {
    const Eigen::Vector3d& centroid = centroids[i];
    const Eigen::Vector3d& bbox = bboxes[i];

    // Skip overly large clusters (likely static objects).
    // Compare the largest single dimension (bbox is already cubic-inflated).
    if (bbox.maxCoeff() > params_.cluster_bbox_cutoff_size)
      continue;

    int closest_idx = associateCluster(centroid, ekf_states_,
                                      params_.cluster_tolerance);

    Cluster cluster;
    if (closest_idx >= 0)
    {
      // FIX: Use actual elapsed time instead of fixed dt
      double actual_dt = current_time_sec - ekf_states_[closest_idx].time_updated;
      if (actual_dt < 1e-6) actual_dt = params_.adaptive_kf_dt;  // fallback

      ekfPredict(ekf_states_[closest_idx], actual_dt);
      aekfUpdate(ekf_states_[closest_idx], centroid,
                 params_.adaptive_kf_alpha, current_time_sec,
                 bbox, params_.use_adaptive_kf);

      // Store observed position in history
      appendToHistory(ekf_states_[closest_idx], current_time_sec, centroid);

      cluster.ekf_state = ekf_states_[closest_idx];
      cluster.centroid = centroid;
    }
    else
    {
      // Create new EKF state
      Eigen::MatrixXd Q_avg, R_avg;
      calculateAverageQandR(Q_avg, R_avg);
      EKFState new_state(9, Q_avg, R_avg, current_time_sec,
                         bbox, next_ekf_id_++);
      new_state.x.head(3) = centroid;

      // Initialize history with first observation
      appendToHistory(new_state, current_time_sec, centroid);

      ekf_states_.push_back(new_state);
      cluster.ekf_state = new_state;
      cluster.centroid = centroid;
    }

    clusters.push_back(cluster);
  }

  // 5) Generate outputs
  generateBoxMarkers(clusters, result);
  generatePredictions(clusters, current_time_sec, result);

  return result;
}

// ---------------------------------------------------------------------------
// History management
// ---------------------------------------------------------------------------
void ObstacleTracker::appendToHistory(EKFState& state, double time,
                                      const Eigen::Vector3d& pos)
{
  state.position_history.emplace_back(time, pos);

  // Trim to max size (keep most recent)
  if (static_cast<int>(state.position_history.size()) > params_.max_history_size)
  {
    state.position_history.erase(state.position_history.begin());
  }
}

// ---------------------------------------------------------------------------
// EKF predict (constant-acceleration model)
// ---------------------------------------------------------------------------
void ObstacleTracker::ekfPredict(EKFState& state, double dt)
{
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt;
  F(0, 6) = 0.5 * dt * dt;
  F(1, 7) = 0.5 * dt * dt;
  F(2, 8) = 0.5 * dt * dt;
  F(3, 6) = dt;
  F(4, 7) = dt;
  F(5, 8) = dt;

  state.x = F * state.x;
  state.P = F * state.P.selfadjointView<Eigen::Lower>() * F.transpose() + state.Q;
}

// ---------------------------------------------------------------------------
// Adaptive EKF update (position-only measurement)
// ---------------------------------------------------------------------------
void ObstacleTracker::aekfUpdate(EKFState& state, const Eigen::VectorXd& z,
                                 double alpha, double time_updated,
                                 const Eigen::Vector3d& bbox, bool use_adaptive)
{
  Eigen::MatrixXd H(3, 9);
  H << 1, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0, 0, 0, 0;

  Eigen::VectorXd d = z - H * state.x;
  Eigen::MatrixXd S = H * state.P * H.transpose() + state.R;
  Eigen::MatrixXd K = state.P * H.transpose() * S.inverse();

  state.x = state.x + K * d;

  Eigen::VectorXd epsilon = z - H * state.x;

  if (use_adaptive)
  {
    state.R = alpha * state.R +
              (1.0 - alpha) * (epsilon * epsilon.transpose() +
                               H * state.P * H.transpose());
    state.Q = alpha * state.Q +
              (1.0 - alpha) * (K * d * d.transpose() * K.transpose());
  }
  else
  {
    state.R = Eigen::MatrixXd::Identity(3, 3) * 0.1;
    state.Q = Eigen::MatrixXd::Identity(9, 9) * 0.01;
  }

  state.P = (Eigen::MatrixXd::Identity(9, 9) - K * H) * state.P;
  state.time_updated = time_updated;
  state.bbox = 0.5 * state.bbox + 0.5 * bbox;
}

// ---------------------------------------------------------------------------
// Nearest-neighbour data association
// ---------------------------------------------------------------------------
int ObstacleTracker::associateCluster(const Eigen::Vector3d& centroid,
                                      const std::vector<EKFState>& states,
                                      double tolerance)
{
  double min_distance = tolerance;
  int closest_idx = -1;

  for (size_t i = 0; i < states.size(); ++i)
  {
    Eigen::Vector3d ekf_pos(states[i].x[0], states[i].x[1], states[i].x[2]);
    double dist = (centroid - ekf_pos).norm();
    if (dist < min_distance)
    {
      min_distance = dist;
      closest_idx = static_cast<int>(i);
    }
  }
  return closest_idx;
}

// ---------------------------------------------------------------------------
// Polynomial fitting (Vandermonde / normal-equations)
// ---------------------------------------------------------------------------
Eigen::VectorXd ObstacleTracker::polyfit(const std::vector<double>& t,
                                         const std::vector<double>& y,
                                         int degree)
{
  int n = static_cast<int>(t.size());
  // Clamp degree to available data points
  if (degree >= n) degree = n - 1;

  Eigen::MatrixXd X(n, degree + 1);
  Eigen::VectorXd Y(n);

  for (int i = 0; i < n; ++i)
  {
    Y(i) = y[i];
    for (int j = 0; j <= degree; ++j)
      X(i, j) = std::pow(t[i], degree - j);
  }

  return (X.transpose() * X).ldlt().solve(X.transpose() * Y);
}

double ObstacleTracker::calculateVariance(const std::vector<double>& t,
                                          const std::vector<double>& y,
                                          const Eigen::VectorXd& beta,
                                          int degree)
{
  int n = static_cast<int>(t.size());
  double residual_sum = 0.0;
  for (int i = 0; i < n; ++i)
  {
    double fitted = 0.0;
    for (int j = 0; j <= degree; ++j)
      fitted += beta(j) * std::pow(t[i], j);
    double r = y[i] - fitted;
    residual_sum += r * r;
  }
  int denom = n - degree - 1;
  return (denom > 0) ? residual_sum / denom : 0.0;
}

// ---------------------------------------------------------------------------
// Housekeeping
// ---------------------------------------------------------------------------
void ObstacleTracker::deleteOldStates(double current_time)
{
  for (auto it = ekf_states_.begin(); it != ekf_states_.end();)
  {
    if (current_time - it->time_updated > params_.time_to_delete_old_obstacles)
      it = ekf_states_.erase(it);
    else
      ++it;
  }
}

void ObstacleTracker::calculateAverageQandR(Eigen::MatrixXd& Q_avg,
                                            Eigen::MatrixXd& R_avg)
{
  Q_avg = Eigen::MatrixXd::Zero(9, 9);
  R_avg = Eigen::MatrixXd::Zero(3, 3);

  if (!ekf_states_.empty())
  {
    for (const auto& s : ekf_states_)
    {
      Q_avg += s.Q;
      R_avg += s.R;
    }
    Q_avg /= static_cast<double>(ekf_states_.size());
    R_avg /= static_cast<double>(ekf_states_.size());
  }
  else
  {
    Q_avg = Eigen::MatrixXd::Identity(9, 9) * 0.01;
    R_avg = Eigen::MatrixXd::Identity(3, 3) * 0.1;
  }
}

// ---------------------------------------------------------------------------
// Clustering helpers
// ---------------------------------------------------------------------------
void ObstacleTracker::getCentroidsAndSizes(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<pcl::PointIndices>& indices,
    std::vector<Eigen::Vector3d>& centroids,
    std::vector<Eigen::Vector3d>& bboxes)
{
  centroids.reserve(indices.size());
  bboxes.reserve(indices.size());

  for (const auto& cluster : indices)
  {
    Eigen::Vector3d min_pt = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::max());
    Eigen::Vector3d max_pt = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::lowest());

    for (const auto& idx : cluster.indices)
    {
      const auto& pt = cloud->points[idx];
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      min_pt = min_pt.cwiseMin(p);
      max_pt = max_pt.cwiseMax(p);
    }

    centroids.emplace_back((min_pt + max_pt) * 0.5);

    // The sensor only sees one face of the obstacle, so the AABB is very
    // thin along the viewing direction.  Inflate the bbox so that each
    // dimension is at least as large as the maximum observed dimension,
    // producing a roughly cubic box.  This is conservative (safer for
    // collision avoidance) and matches the visual expectation better.
    Eigen::Vector3d raw_bbox = max_pt - min_pt;
    double max_dim = raw_bbox.maxCoeff();
    bboxes.emplace_back(Eigen::Vector3d::Constant(max_dim));
  }
}

// ---------------------------------------------------------------------------
// Constant-velocity prediction using EKF-filtered velocity
//
// Instead of fitting a polynomial to the position history (which diverges
// when extrapolated beyond the observation window), we use the EKF's
// filtered velocity estimate directly:
//
//   P(t) = current_position + velocity * t
//
// The velocity is clamped to max_obstacle_velocity so predictions stay
// physically reasonable at any horizon. This is stable, bounded, and
// works immediately once min_observations_for_prediction is met.
// ---------------------------------------------------------------------------
void ObstacleTracker::generatePredictions(const std::vector<Cluster>& clusters,
                                          double current_time_sec,
                                          TrackingResult& result)
{
  int marker_id = 0;

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    const auto& ekf = clusters[i].ekf_state;
    const auto& history = ekf.position_history;
    const Eigen::Vector3d& cur_pos = clusters[i].centroid;

    // Need enough observations for a meaningful velocity estimate
    if (static_cast<int>(history.size()) < params_.min_observations_for_prediction)
      continue;

    // Get EKF-filtered velocity
    Eigen::Vector3d vel(ekf.x[3], ekf.x[4], ekf.x[5]);
    double speed = vel.norm();

    // Skip stationary obstacles
    if (speed < params_.cutoff_length_threshold)
      continue;

    // Clamp velocity to max obstacle velocity
    if (speed > params_.max_obstacle_velocity)
      vel = vel * (params_.max_obstacle_velocity / speed);

    // Constant-velocity PWP: P(t) = pos + vel * (t - t_now)
    PieceWisePol pwp;
    pwp.times.push_back(current_time_sec);
    pwp.times.push_back(current_time_sec + params_.prediction_horizon);
    pwp.coeff_x.push_back({0.0, 0.0, vel.x(), cur_pos.x()});
    pwp.coeff_y.push_back({0.0, 0.0, vel.y(), cur_pos.y()});
    pwp.coeff_z.push_back({0.0, 0.0, vel.z(), cur_pos.z()});

    // DynTraj message
    dynus_interfaces::msg::DynTraj msg;
    msg.header.frame_id = "RR06/map";
    msg.id = ekf.id;
    msg.pos.x = cur_pos.x();
    msg.pos.y = cur_pos.y();
    msg.pos.z = cur_pos.z();
    msg.mode = "pwp";
    msg.bbox = {ekf.bbox.x(), ekf.bbox.y(), ekf.bbox.z()};
    msg.pwp = convertPwp2PwpMsg(pwp);
    msg.ekf_cov_p = {ekf.P(0,0), ekf.P(1,1), ekf.P(2,2)};
    msg.ekf_cov_q = {ekf.Q(0,0), ekf.Q(1,1), ekf.Q(2,2)};
    msg.ekf_cov_r = {ekf.R(0,0), ekf.R(1,1), ekf.R(2,2)};
    msg.poly_cov = {0.0, 0.0, 0.0};
    msg.poly_coeffs_x = {vel.x(), cur_pos.x()};
    msg.poly_coeffs_y = {vel.y(), cur_pos.y()};
    msg.poly_coeffs_z = {vel.z(), cur_pos.z()};
    msg.poly_start_time = current_time_sec;
    msg.poly_end_time = current_time_sec + params_.prediction_horizon;
    msg.is_agent = false;
    result.trajectories.push_back(msg);

    // LINE_STRIP visualization
    double dt_vis = params_.prediction_dt * 0.25;
    int num_vis_samples = static_cast<int>(params_.prediction_horizon / dt_vis) + 1;

    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "RR06/map";
    line_marker.id = marker_id++;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    line_marker.scale.x = 0.05;
    line_marker.color = ekf.color;
    line_marker.color.a = 1.0f;
    line_marker.points.reserve(num_vis_samples);

    for (int step = 0; step < num_vis_samples; ++step)
    {
      double t = step * dt_vis;
      geometry_msgs::msg::Point pt;
      pt.x = cur_pos.x() + vel.x() * t;
      pt.y = cur_pos.y() + vel.y() * t;
      pt.z = cur_pos.z() + vel.z() * t;
      line_marker.points.push_back(pt);
    }
    result.prediction_markers.markers.push_back(line_marker);
  }
}

// ---------------------------------------------------------------------------
// Bounding-box visualization: wireframe edges + centroid sphere
// ---------------------------------------------------------------------------
void ObstacleTracker::generateBoxMarkers(const std::vector<Cluster>& clusters,
                                         TrackingResult& result)
{
  for (size_t i = 0; i < clusters.size(); ++i)
  {
    const auto& cluster = clusters[i];
    const double cx = cluster.centroid[0];
    const double cy = cluster.centroid[1];
    const double cz = cluster.centroid[2];
    const double hx = std::max(cluster.ekf_state.bbox[0], 0.05) * 0.5;
    const double hy = std::max(cluster.ekf_state.bbox[1], 0.05) * 0.5;
    const double hz = std::max(cluster.ekf_state.bbox[2], 0.05) * 0.5;

    // --- Wireframe box (LINE_LIST: 12 edges = 24 points) ---
    visualization_msgs::msg::Marker wire;
    wire.header.frame_id = "RR06/map";
    wire.ns = "wireframe";
    wire.id = cluster.ekf_state.id;
    wire.type = visualization_msgs::msg::Marker::LINE_LIST;
    wire.action = visualization_msgs::msg::Marker::ADD;
    wire.lifetime = rclcpp::Duration::from_seconds(0.2);
    wire.scale.x = 0.06;  // edge thickness
    wire.color = cluster.ekf_state.color;
    wire.color.a = 1.0f;
    wire.pose.orientation.w = 1.0;
    wire.points.reserve(24);

    auto addEdge = [&](double x1, double y1, double z1,
                       double x2, double y2, double z2) {
      geometry_msgs::msg::Point p1, p2;
      p1.x = x1; p1.y = y1; p1.z = z1;
      p2.x = x2; p2.y = y2; p2.z = z2;
      wire.points.push_back(p1);
      wire.points.push_back(p2);
    };

    // Bottom face
    addEdge(cx-hx, cy-hy, cz-hz, cx+hx, cy-hy, cz-hz);
    addEdge(cx+hx, cy-hy, cz-hz, cx+hx, cy+hy, cz-hz);
    addEdge(cx+hx, cy+hy, cz-hz, cx-hx, cy+hy, cz-hz);
    addEdge(cx-hx, cy+hy, cz-hz, cx-hx, cy-hy, cz-hz);
    // Top face
    addEdge(cx-hx, cy-hy, cz+hz, cx+hx, cy-hy, cz+hz);
    addEdge(cx+hx, cy-hy, cz+hz, cx+hx, cy+hy, cz+hz);
    addEdge(cx+hx, cy+hy, cz+hz, cx-hx, cy+hy, cz+hz);
    addEdge(cx-hx, cy+hy, cz+hz, cx-hx, cy-hy, cz+hz);
    // Vertical edges
    addEdge(cx-hx, cy-hy, cz-hz, cx-hx, cy-hy, cz+hz);
    addEdge(cx+hx, cy-hy, cz-hz, cx+hx, cy-hy, cz+hz);
    addEdge(cx+hx, cy+hy, cz-hz, cx+hx, cy+hy, cz+hz);
    addEdge(cx-hx, cy+hy, cz-hz, cx-hx, cy+hy, cz+hz);

    result.bbox_markers.markers.push_back(wire);

    // --- Centroid sphere ---
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "RR06/map";
    sphere.ns = "centroid";
    sphere.id = cluster.ekf_state.id;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.lifetime = rclcpp::Duration::from_seconds(0.2);
    sphere.pose.position.x = cx;
    sphere.pose.position.y = cy;
    sphere.pose.position.z = cz;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = 0.2;
    sphere.scale.y = 0.2;
    sphere.scale.z = 0.2;
    sphere.color = cluster.ekf_state.color;
    sphere.color.a = 1.0f;

    result.bbox_markers.markers.push_back(sphere);
  }
}

std::vector<TrackedObstacle> ObstacleTracker::getTrackedObstacles() const
{
  std::vector<TrackedObstacle> result;
  result.reserve(ekf_states_.size());
  const double vel_thresh_sq = params_.velocity_threshold * params_.velocity_threshold;
  for (const auto& ekf : ekf_states_)
  {
    if (ekf.position_history.size() >= static_cast<size_t>(params_.min_observations_for_prediction))
    {
      // Skip stationary obstacles — EKF velocity below threshold means
      // this is likely static structure absorbed by the tracker.
      double vel_sq = ekf.x.segment<3>(3).squaredNorm();
      if (vel_sq < vel_thresh_sq)
        continue;

      TrackedObstacle obs;
      obs.position = ekf.x.head<3>();
      obs.bbox = ekf.bbox;
      obs.id = ekf.id;
      result.push_back(obs);
    }
  }
  return result;
}

}  // namespace global_mapper_ros
