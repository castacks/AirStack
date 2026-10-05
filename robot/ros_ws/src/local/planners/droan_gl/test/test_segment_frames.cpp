#include <gtest/gtest.h>
#include <droan_gl/gl_interface.hpp>
#include <droan_gl/global_plan.hpp>
#include <droan_gl/checked_prefix.hpp>

class SegmentFrames : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

namespace {
std::vector<TrajectoryPoint> samples(size_t count = 20) {
  std::vector<TrajectoryPoint> points(count);
  for (size_t i = 0; i < count; ++i) {
    points[i].v1 = {static_cast<float>(i * .2), 0.f, 1.f, 1.f};
    points[i].v2 = {0.f, 0.f, 2.f, 1.f};
  }
  return points;
}
}

TEST_F(SegmentFrames, DiagnosticRejectionSharesExactPublicationPredicate) {
  auto p = samples()[0];
  EXPECT_EQ(checked_point_rejection(p, 1., 3.), nullptr);
  p.v1.z = .99f;
  EXPECT_STREQ(checked_point_rejection(p, 1., 3.), "altitude");
  p = samples()[0]; p.v2.z = 1.f;
  EXPECT_STREQ(checked_point_rejection(p, 1., 3.), "unobserved");
  p.v2.z = 2.f; p.v2.x = 3.f;
  EXPECT_STREQ(checked_point_rejection(p, 1., 3.), "collision");
  p.v2.x = 2.f;
  EXPECT_EQ(checked_point_rejection(p, 1., 3.), nullptr);
  p.v2.w = -1.f;
  EXPECT_STREQ(checked_point_rejection(p, 1., 3.), "negative_speed");
  p.v1.x = NAN;
  EXPECT_STREQ(checked_point_rejection(p, 1., 3.), "nonfinite");
}

TEST_F(SegmentFrames, VerticalCandidatesAppendWithoutChangingExistingLibrary) {
  std::vector<TrajectoryParams> params;
  for (int i = 0; i < 120; ++i) {
    params.push_back({{static_cast<float>(i), .25f, -.25f}, 2.f});
  }
  const auto original = params;
  append_vertical_trajectory_params(params);
  ASSERT_EQ(params.size(), 122u);
  for (size_t i = 0; i < original.size(); ++i) {
    for (int axis = 0; axis < 3; ++axis) {
      EXPECT_FLOAT_EQ(params[i].vel_desired[axis], original[i].vel_desired[axis]);
    }
    EXPECT_FLOAT_EQ(params[i].vel_max, original[i].vel_max);
  }
  for (size_t i = 120; i < 122; ++i) {
    EXPECT_FLOAT_EQ(params[i].vel_desired[0], 0.f);
    EXPECT_FLOAT_EQ(params[i].vel_desired[1], 0.f);
    EXPECT_FLOAT_EQ(params[i].vel_max, 2.f);
  }
  EXPECT_FLOAT_EQ(params[120].vel_desired[2], .5f);
  EXPECT_FLOAT_EQ(params[121].vel_desired[2], -.5f);
}

TEST_F(SegmentFrames, CommandedAnchorDiagnosticRetainsFreshnessBoundaries) {
  EXPECT_EQ(commanded_anchor_rejection(true, true, 0., 0., true), nullptr);
  EXPECT_EQ(commanded_anchor_rejection(true, true, .5, .5, true), nullptr);
  EXPECT_STREQ(commanded_anchor_rejection(false, true, 0., 0., true), "missing");
  EXPECT_STREQ(commanded_anchor_rejection(true, false, 0., 0., true), "frame");
  EXPECT_STREQ(commanded_anchor_rejection(true, true, NAN, 0., true), "nonfinite_source_age");
  EXPECT_STREQ(commanded_anchor_rejection(true, true, -.000001, 0., true), "source_future");
  EXPECT_STREQ(commanded_anchor_rejection(true, true, .500001, 0., true), "source_stale");
  EXPECT_STREQ(commanded_anchor_rejection(true, true, 0., -.000001, true), "receipt_future");
  EXPECT_STREQ(commanded_anchor_rejection(true, true, 0., .500001, true), "receipt_stale");
  EXPECT_STREQ(commanded_anchor_rejection(true, true, 0., 0., false), "nonfinite_xy");
}

TEST_F(SegmentFrames, CheckedPrefixNeverReentersLaterFreeIslandAndReservesStopSpace) {
  auto points = samples();
  points[10].v2 = {0.f, 2.f, 0.f, 1.f};  // First unknown, later points free.
  auto prefix = checked_prefix(points, 0, points.size(), 1., 3., .5);
  ASSERT_GE(prefix.size(), 2u);
  EXPECT_LT(prefix.back().x(), points[10].x() - .5);
  EXPECT_FLOAT_EQ(prefix.back().get_vel(), 0.f);
  EXPECT_LT(prefix[prefix.size() - 2].get_vel(), 1.f);
  for (const auto& point : prefix) {
    EXPECT_LE(point.v1.x, 1.3f);
    EXPECT_GE(point.v1.z, 1.f); EXPECT_LE(point.v1.z, 3.f);
  }
}

TEST_F(SegmentFrames, ShortGoalStopsCheckedForwardHorizonWithoutOvershootPenalty) {
  const auto raw = samples();
  const auto long_prefix = checked_prefix(raw, 0, raw.size(), 1., 1., .5);
  const tf2::Vector3 goal(1., 0., 1.);
  EXPECT_GT(long_prefix.back().position().distance(goal), 2.);
  size_t before = 0;
  auto cropped = checked_prefix(raw, 0, raw.size(), 1., 1., .5, &goal, &before);
  EXPECT_EQ(before, long_prefix.size());
  ASSERT_EQ(cropped.size(), 6u);
  EXPECT_NEAR(cropped.back().position().distance(goal), 0., 1e-6);
  EXPECT_FLOAT_EQ(cropped.back().v2.w, 0.f);
  EXPECT_NEAR(cropped[4].v2.w, .4, 1e-6);  // One ramp from raw speed1.
  for (size_t i = 0; i < cropped.size(); ++i) {
    EXPECT_FLOAT_EQ(cropped[i].v1.x, raw[i].v1.x);
    EXPECT_FLOAT_EQ(cropped[i].v1.z, raw[i].v1.z);
    EXPECT_LE(cropped[i].v2.w, raw[i].v2.w);
    EXPECT_EQ(checked_point_rejection(cropped[i], 1., 1.), nullptr);
  }
}

TEST_F(SegmentFrames, GoalCaptureDoesNotExtendBlockedSpaceOrAlterDistantRoute) {
  auto raw = samples(); raw[5].v2 = {4.f, 0.f, 2.f, 1.f};
  const tf2::Vector3 blocked_goal(1., 0., 1.);
  auto admitted = checked_prefix(raw, 0, raw.size(), 1., 1., .5);
  ASSERT_GE(admitted.size(), 2u);
  auto cropped = checked_prefix(raw, 0, raw.size(), 1., 1., .5, &blocked_goal);
  EXPECT_EQ(cropped.size(), admitted.size());
  EXPECT_FLOAT_EQ(cropped.back().v1.x, admitted.back().v1.x);
  auto full = samples();
  const auto ordinary = checked_prefix(full, 0, full.size(), 1., 1., .5);
  for (const auto& goal : {tf2::Vector3(1., 2., 1.), tf2::Vector3(-1., 0., 1.),
                           tf2::Vector3(NAN, 0., 1.)}) {
    EXPECT_EQ(checked_prefix(full, 0, full.size(), 1., 1., .5, &goal).size(), ordinary.size());
  }
  EXPECT_TRUE(checked_prefix({}, 0, 0, 1., 1., .5, &blocked_goal).empty());
  EXPECT_EQ(single_goal_stop_count(full, blocked_goal, NAN), full.size());
}

TEST_F(SegmentFrames, ShortVerticalGoalPlaneKeepsCheckedEndpointAndBraking) {
  for (float direction : {1.f, -1.f}) {
    auto raw = samples(12);
    for (size_t i = 0; i < raw.size(); ++i) {
      raw[i].v1 = {0.f, 0.f, 1.f + direction * static_cast<float>(i) * .1f, 1.f};
      raw[i].v2.w = .5f;
    }
    const tf2::Vector3 goal(0., 0., 1. + direction * .5);
    const double low = std::min(1., goal.z()), high = std::max(1., goal.z());
    EXPECT_TRUE(checked_prefix(raw, 0, raw.size(), low, high, .5).empty());
    EXPECT_TRUE(checked_prefix(raw, 0, raw.size(), low, high, .5, &goal).empty());
    auto segment = checked_prefix(raw, 0, raw.size(), low, high, .5, &goal, nullptr, true);
    ASSERT_GE(segment.size(), 2u);
    EXPECT_LE(segment.back().position().distance(goal), .100001);
    EXPECT_FLOAT_EQ(segment.back().v2.w, 0.f);
    for (size_t i = 0; i < segment.size(); ++i) {
      EXPECT_EQ(checked_point_rejection(segment[i], low, high), nullptr);
      EXPECT_FLOAT_EQ(segment[i].v1.z, raw[i].v1.z);
      EXPECT_LE(segment[i].v2.w, raw[i].v2.w);
    }
  }
}

TEST_F(SegmentFrames, GoalPlaneNeverExemptsBlockedUnknownOrInvalidCrossing) {
  auto raw = samples(12);
  for (size_t i = 0; i < raw.size(); ++i) raw[i].v1 = {0.f, 0.f, 1.f + i * .1f, 1.f};
  const tf2::Vector3 goal(0., 0., 1.5);
  size_t crossing = 0;
  while (crossing < raw.size() && raw[crossing].v1.z <= 1.5) ++crossing;
  ASSERT_LT(crossing, raw.size());
  for (int kind = 0; kind < 4; ++kind) {
    auto blocked = raw;
    if (kind == 0) blocked[crossing].v2.x = 3.f;
    if (kind == 1) blocked[crossing].v2.z = 1.f;
    if (kind == 2) blocked[crossing].v1.x = NAN;
    if (kind == 3) blocked[crossing].v2.w = -.1f;
    EXPECT_TRUE(checked_prefix(blocked, 0, blocked.size(), 1., 1.5, .5, &goal, nullptr, true).empty());
  }
  raw[3].v2.z = 1.f;  // Earlier loss of checked space cannot be skipped.
  EXPECT_TRUE(checked_prefix(raw, 0, raw.size(), 1., 1.5, .5, &goal, nullptr, true).empty());
}

TEST_F(SegmentFrames, GoalPlaneExemptionRequiresMatchingNearbyGoalAndProgress) {
  auto raw = samples(12);
  for (size_t i = 0; i < raw.size(); ++i) raw[i].v1 = {0.f, 0.f, 1.f + i * .1f, 1.f};
  for (const auto& goal : {tf2::Vector3(0., 0., 1.), tf2::Vector3(2., 0., 1.5),
                           tf2::Vector3(0., 0., 1.4), tf2::Vector3(NAN, 0., 1.5)}) {
    EXPECT_TRUE(checked_prefix(raw, 0, raw.size(), 1., 1.5, .5, &goal, nullptr, true).empty());
  }
}

TEST_F(SegmentFrames, HeightTerminationIsBoundOnceToVerticalStartNotHorizontalApproach) {
  SingletonGoalStop capture;
  const tf2::Vector3 start(0., 0., 1.);
  capture.bind(1, {0., 0., 1.5}, &start);
  EXPECT_TRUE(capture.height_stop());
  capture.route_replaced();
  EXPECT_FALSE(capture.height_stop());
  capture.bind(1, {1., 0., .95}, &start);
  EXPECT_FALSE(capture.height_stop());
  capture.bind(1, {.06, 0., 1.5}, &start);
  EXPECT_FALSE(capture.height_stop());
  capture.bind(1, {.04, 0., 1.02}, &start);
  EXPECT_FALSE(capture.height_stop());
  capture.bind(2, {0., 0., 1.5}, &start);
  EXPECT_FALSE(capture.height_stop());
  capture.bind(1, {0., 0., 1.5});
  EXPECT_FALSE(capture.height_stop());
}

TEST_F(SegmentFrames, GoalCaptureIsOriginalSingletonAndInvalidatedByRouteReplacement) {
  SingletonGoalStop capture;
  EXPECT_FALSE(capture.goal());
  capture.bind(1, {5., -2., 1.});
  ASSERT_TRUE(capture.goal());
  EXPECT_DOUBLE_EQ(capture.goal()->x(), 5.);
  capture.route_replaced();
  EXPECT_FALSE(capture.goal());
  capture.bind(2, {5., -2., 1.});
  EXPECT_FALSE(capture.goal());  // Do not capture a multicorner route's final goal.
  capture.bind(1, {5., -2., 1.});
  capture.bind(3, {7., 0., 1.});
  EXPECT_FALSE(capture.goal());
}

TEST_F(SegmentFrames, CheckedPrefixRejectsUnsafeFirstInsufficientReserveAndDuplicatePoints) {
  auto points = samples(); points[0].v2 = {3.f, 0.f, 2.f, 1.f};
  EXPECT_TRUE(checked_prefix(points, 0, points.size(), 1., 3., .5).empty());
  points = samples(); points[2].v2 = {0.f, 1.f, 0.f, 1.f};
  EXPECT_TRUE(checked_prefix(points, 0, points.size(), 1., 3., .5).empty());
  points = samples(); for (auto& point : points) point.v1.x = 0.f;
  EXPECT_TRUE(checked_prefix(points, 0, points.size(), 1., 3., .5).empty());
}

TEST_F(SegmentFrames, CheckedPrefixStopsAtNonfiniteOrOutsideAltitude) {
  auto points = samples(); points[10].v1.z = 3.1f;
  auto prefix = checked_prefix(points, 0, points.size(), 1., 3., .5);
  ASSERT_FALSE(prefix.empty()); EXPECT_LE(prefix.back().x(), 1.3f);
  points[10].v1.z = NAN;
  auto nonfinite = checked_prefix(points, 0, points.size(), 1., 3., .5);
  EXPECT_EQ(nonfinite.size(), prefix.size());
  points[0].v2.z = NAN;
  EXPECT_TRUE(checked_prefix(points, 0, points.size(), 1., 3., .5).empty());
}

TEST_F(SegmentFrames, StartCorridorIncludesHeldCommandWithoutAdmittingDescent) {
  const auto bounds = navigation_start_corridor(1.007, 3., 1.011, 1.);
  EXPECT_DOUBLE_EQ(bounds.first, 1.); EXPECT_DOUBLE_EQ(bounds.second, 3.);
  auto points = samples();
  EXPECT_TRUE(checked_prefix(points, 0, points.size(), 1.007, 3., .5).empty());
  EXPECT_FALSE(checked_prefix(points, 0, points.size(), bounds.first, bounds.second, .5).empty());
  points[0].v1.z = .99f;
  EXPECT_TRUE(checked_prefix(points, 0, points.size(), bounds.first, bounds.second, .5).empty());
  const auto inward = navigation_start_corridor(1., 3., .97, 1.);
  EXPECT_DOUBLE_EQ(inward.first, .97);
  const auto upper = navigation_start_corridor(1., 2.9, 2.99, 3.);
  EXPECT_DOUBLE_EQ(upper.second, 3.);
  EXPECT_THROW(navigation_start_corridor(1., 3., 1., NAN), std::invalid_argument);
  EXPECT_THROW(navigation_start_corridor(3., 1., 1., 1.), std::invalid_argument);
}

TEST_F(SegmentFrames, CollisionReadbackMapPointIsNotTransformedAgain) {
  tf2::Quaternion yaw;
  yaw.setRPY(0., 0., M_PI / 2.);
  tf2::Transform local_to_map(yaw, tf2::Vector3(5., -2., 1.));
  TrajectoryPoint point{};
  const auto collision_output = local_to_map * tf2::Vector3(1., 0., .5);
  point.v1 = {static_cast<float>(collision_output.x()), static_cast<float>(collision_output.y()),
              static_cast<float>(collision_output.z()), 1.f};
  const auto mapped = point.position();
  EXPECT_NEAR(mapped.x(), 5., 1e-6);
  EXPECT_NEAR(mapped.y(), -1., 1e-6);
  EXPECT_NEAR(mapped.z(), 1.5, 1e-6);
  EXPECT_GT(mapped.distance(local_to_map * mapped), 1.);
}

TEST_F(SegmentFrames, ScoringUsesTheSameMapPointAsPublication) {
  auto node = std::make_shared<rclcpp::Node>("segment_frame_score_test");
  tf2_ros::Buffer buffer(node->get_clock());
  GlobalPlan plan(node.get(), &buffer);
  auto path = std::make_shared<nav_msgs::msg::Path>();
  path->header.frame_id = "map";
  for (double y : {-2., 0.}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 5.; pose.pose.position.y = y; pose.pose.position.z = 1.;
    path->poses.push_back(pose);
  }
  plan.set_global_plan(path);
  tf2::Quaternion yaw; yaw.setRPY(0., 0., M_PI / 2.);
  const auto collision_output = tf2::Transform(yaw, {5., -2., 1.}) * tf2::Vector3(1., 0., 0.);
  TrajectoryPoint sample{};
  sample.v1 = {static_cast<float>(collision_output.x()), static_cast<float>(collision_output.y()),
               static_cast<float>(collision_output.z()), 1.f};
  const auto published = sample.position();
  const auto [deviation, distance] = plan.get_distance(published.x(), published.y(), published.z());
  EXPECT_NEAR(deviation, 0., 1e-6);
  EXPECT_NEAR(distance, 1., 1e-6);
  const auto [wrong_deviation, wrong_distance] = plan.get_distance(1., 0., 0.);
  EXPECT_GT(wrong_deviation, 4.);
}

TEST_F(SegmentFrames, UninitializedGpuEvaluationDiscardsCachedPoints) {
  auto node = std::make_shared<rclcpp::Node>("segment_frame_failure_test");
  tf2_ros::Buffer buffer(node->get_clock());
  GLInterface interface(node.get(), &buffer);  // No camera/GL context: read-only.
  std::vector<TrajectoryPoint> stale(35);
  EXPECT_FALSE(interface.evaluate_trajectories(airstack_msgs::msg::Odometry{}, stale));
  EXPECT_TRUE(stale.empty());
}

TEST_F(SegmentFrames, ActualGpuReadbackIsMapFrameAndMissingTfClearsOutput) {
  if (!std::getenv("DROAN_TEST_GPU")) GTEST_SKIP() << "Opt-in GPU regression";
  auto node = std::make_shared<rclcpp::Node>("segment_gpu_contract_test");
  tf2_ros::Buffer buffer(node->get_clock());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "look_ahead_point_stabilized";
  transform.transform.translation.x = 5.; transform.transform.translation.y = -2.;
  transform.transform.translation.z = 1.;
  tf2::Quaternion yaw; yaw.setRPY(0., 0., M_PI / 2.);
  transform.transform.rotation = tf2::toMsg(yaw);
  ASSERT_TRUE(buffer.setTransform(transform, "test", true));
  GLInterface interface(node.get(), &buffer);
  auto camera = std::make_shared<sensor_msgs::msg::CameraInfo>();
  camera->width = 64; camera->height = 48;
  camera->k = {50., 0., 32., 0., 50., 24., 0., 0., 1.};
  camera->p = {50., 0., 32., -12.5, 0., 50., 24., 0., 0., 0., 1., 0.};
  interface.handle_camera_info(camera);
  airstack_msgs::msg::Odometry look;
  look.header.frame_id = "map"; look.child_frame_id = "map";
  look.pose.position.x = 5.; look.pose.position.y = -2.; look.pose.position.z = 1.;
  look.pose.orientation.w = 1.;
  std::vector<TrajectoryPoint> output;
  ASSERT_TRUE(interface.evaluate_trajectories(look, output));
  ASSERT_FALSE(output.empty());
  // First candidate desired local velocity is +Y; 90-degree yaw maps it to -X.
  EXPECT_LT(output.front().x(), 5.);
  EXPECT_NEAR(output.front().y(), -2., 1e-5);
  EXPECT_GT(output.front().x(), 4.9);
  EXPECT_NEAR(output.front().z(), .99, .02);
  look.header.frame_id = "missing_frame";
  EXPECT_FALSE(interface.evaluate_trajectories(look, output));
  EXPECT_TRUE(output.empty());
}
