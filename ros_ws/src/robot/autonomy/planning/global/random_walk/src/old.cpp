/**************************************************************************
random_walk_planner.cpp

Graeme Best (bestg@oregonstate.edu)
Jan 2020

Copyright Carnegie Mellon University (CMU) <2019>

This code is proprietary to the CMU SubT challenge. Do not share or distribute
without express permission of a project lead (Sebastian or Matt).
**************************************************************************/

#include <misc_utils/misc_utils.h>
// #include <graph_utils/graph_utils.h>

#include <tf/transform_datatypes.h>

#include <math.h>
#define PI 3.14159265
#include <stdlib.h>

namespace random_walk_planner_ns {

bool RandomWalkPlanner::readParameters() 
{
  // Read in parameters
  ros::NodeHandle *nh = get_private_node_handle();
  if (!nh->getParam("world_frame_id_", world_frame_id_)) {
    ROS_ERROR("Cannot read parameter: world_frame_id_");
    return false;
  }
  if (!nh->getParam("pub_waypoint_topic_", pub_waypoint_topic_)) {
    ROS_ERROR("Cannot read parameter: pub_waypoint_topic_");
    return false;
  }
  if (!nh->getParam("sub_odometry_topic_", sub_odometry_topic_)) {
    ROS_ERROR("Cannot read parameter: sub_odometry_topic_");
    return false;
  }
  if (!nh->getParam("sub_command_topic_", sub_command_topic_)) {
    ROS_ERROR("Cannot read parameter: sub_command_topic_");
    return false;
  }
  if (!nh->getParam("pub_status_topic_", pub_status_topic_)) {
    ROS_ERROR("Cannot read parameter: pub_status_topic_");
    return false;
  }
  if (!nh->getParam("sub_lp_found_path_topic_", sub_lp_found_path_topic_)) {
    ROS_ERROR("Cannot read parameter: sub_lp_found_path_topic_");
    return false;
  }  
  if (!nh->getParam("kLookaheadDistance", kLookaheadDistance)) {
    ROS_ERROR("Cannot read parameter: kLookaheadDistance");
    return false;
  }
  if (!nh->getParam("kMaxVerticalAngle", kMaxVerticalAngle)) {
    ROS_ERROR("Cannot read parameter: kMaxVerticalAngle");
    return false;
  }
  if (!nh->getParam("kStoppedConditionDistance", kStoppedConditionDistance)) {
    ROS_ERROR("Cannot read parameter: kStoppedConditionDistance");
    return false;
  }
  if (!nh->getParam("kStoppedConditionTime", kStoppedConditionTime)) {
    ROS_ERROR("Cannot read parameter: kStoppedConditionTime");
    return false;
  }
  if (!nh->getParam("kVectorResetTime", kVectorResetTime)) {
    ROS_ERROR("Cannot read parameter: kVectorResetTime");
    return false;
  }
  if (!nh->getParam("kRecoveryTime", kRecoveryTime)) {
    ROS_ERROR("Cannot read parameter: kRecoveryTime");
    return false;
  }
  if (!nh->getParam("kStartupTime", kStartupTime)) {
    ROS_ERROR("Cannot read parameter: kStartupTime");
    return false;
  }
  if (!nh->getParam("kZScalingDistance", kZScalingDistance)) {
    ROS_ERROR("Cannot read parameter: kZScalingDistance");
    return false;
  }
  if (!nh->getParam("kStoppedConditionDistanceStartup", kStoppedConditionDistanceStartup)) {
    ROS_ERROR("Cannot read parameter: kStoppedConditionDistanceStartup");
    return false;
  }
  if (!nh->getParam("rw_only_if_lp_no_path_found_", rw_only_if_lp_no_path_found_)) {
    ROS_ERROR("Cannot read parameter: rw_only_if_lp_no_path_found_");
    return false;
  }
  if (!nh->getParam("pub_path_topic_", pub_path_topic_)) {
    ROS_ERROR("Cannot read parameter: pub_path_topic_");
    return false;
  }
  
  return true;
}

void RandomWalkPlanner::odometryCallback(const nav_msgs::Odometry::ConstPtr &odometry_msg) 
{
  // Get the current robot position
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geo_quat = odometry_msg->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);

  robot_yaw_ = yaw;
  robot_pos_ = odometry_msg->pose.pose.position;


  // Append it to the trajectory
  PositionTimePair new_position_time_pair;
  trajectory_.push_back(new_position_time_pair);
  auto& position_time_pair = trajectory_.back();

  position_time_pair.time = ros::Time::now();
  position_time_pair.position = robot_pos_;

  // Initialise startup time
  if(!received_first_odometry_)
  {
    received_first_odometry_ = true;
    time_startup_ = ros::Time::now();
  }
}

void RandomWalkPlanner::lpFoundPathCallback(const std_msgs::Bool::ConstPtr& msg)
{
  lp_found_path_ = msg->data;

  // ROS_INFO_STREAM("lpFoundPathCallback()" << lp_found_path_);

  if(lp_found_path_)
  {
    time_lp_found_path_ = ros::Time::now();
    lp_found_at_least_one_path_ = true;
  }
}


void RandomWalkPlanner::commandCallback(const random_walk_planner::RandomWalkPlannerCommand::ConstPtr& msg)
{
  const auto previous_command = random_walk_planner_command_.command;

  // Receive command from behavior executive
  random_walk_planner_command_ = *msg;

  // Check if recently switched to random walk mode
  if(previous_command == random_walk_planner::RandomWalkPlannerCommand::COMMAND_DISABLE 
          && random_walk_planner_command_.command == random_walk_planner::RandomWalkPlannerCommand::COMMAND_RANDOM_WALK)
  {
    ROS_INFO("RandomWalkPlanner commanded!");

    // Pick a new random direction
    pickNewRandomVector();

    // Reset recovery timer
    resetRecoveryTime();
  }
}

void RandomWalkPlanner::resetRecoveryTime()
{
  time_recovery_start_ = ros::Time::now();
}


double randMToN(double M, double N)
{
  // see https://stackoverflow.com/questions/686353/random-float-number-generation
  return M+((double)rand()/((double)RAND_MAX/(N-M))) ;  
}

void RandomWalkPlanner::pickNewRandomVector()
{
  // Pick random angles
  float kMaxVerticalAngle_radians = kMaxVerticalAngle*PI/180.0;
  float theta = randMToN(PI/2.0 - kMaxVerticalAngle_radians, PI/2.0 + kMaxVerticalAngle_radians);
  float phi = randMToN(0, 2.0*PI);
  float r = kLookaheadDistance;

  // Convert to cartesian
  random_vector_.x = r * sin(theta) * cos(phi);
  random_vector_.y = r * sin(theta) * sin(phi);
  random_vector_.z = r * cos(theta);
}

void RandomWalkPlanner::timerRandomVectorResetCallback()
{
  pickNewRandomVector();
}

bool RandomWalkPlanner::hasStartupTimeElapsed()
{
  // If not odometry received, don't start RW
  if(!received_first_odometry_)
  {
    return false;
  }

  // If minimum time since first odometry received hasn't elapsed, don't start RW
  ros::Time current_time = ros::Time::now();
  ros::Duration duration(kStartupTime);

  if(time_startup_ + duration < current_time)
  {
    return true;
  }  
  else
  {
    return false;
  }
}

bool RandomWalkPlanner::isRecovering()
{
  // Only recovering if commanded to
  if(random_walk_planner_command_.command != random_walk_planner::RandomWalkPlannerCommand::COMMAND_RANDOM_WALK)
    return false;

  // And if the timer hasn't expired
  ros::Time current_time = ros::Time::now();
  ros::Duration duration(kRecoveryTime);

  if(time_recovery_start_ + duration < current_time)
  {
    return false;
  }  
  else
  {
    return true;
  }
}

double RandomWalkPlanner::PointXYZScaledDist(const geometry_msgs::Point& pnt1, const geometry_msgs::Point& pnt2)
{
  return sqrt(pow((pnt1.x - pnt2.x), 2) + pow((pnt1.y - pnt2.y), 2) + pow((pnt1.z - pnt2.z)/kZScalingDistance, 2));
}

bool RandomWalkPlanner::isStopped()
{
  if(trajectory_.empty())
    return true;

  // If in LP listening mode, first check that
  if(rw_only_if_lp_no_path_found_ && !isLPStuck())
  {
    // Assumed not stopped, since the LP recently says it's not stuck
    // ROS_INFO("local planner NOT stuck");
    return false;
  }
  // ROS_INFO("local planner stuck!");
  

  // Get the current time
  ros::Time current_time = ros::Time::now();
  ros::Duration duration(kStoppedConditionTime);

  // Iterate through the recorded trajectory
  auto it = trajectory_.begin();
  while(it != trajectory_.end())
  {
    auto position_time_pair = *it;

    // If data point is old, and have moved at least once 
    if(has_moved_at_least_once_ && position_time_pair.time + duration < current_time)
    {
      // Remove it
      it = trajectory_.erase(it);
    }
    else
    {
      // Is this outside the ball?
      // double distance = misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(position_time_pair.position, robot_pos_);
      double distance = PointXYZScaledDist(position_time_pair.position, robot_pos_);

      // Use a different distance until first movement is received
      if(!has_moved_at_least_once_)
      {
        if(distance >= kStoppedConditionDistanceStartup)
        {
          // Moved outside ball!
          return false;
        }
      }
      else
      {
        if(distance >= kStoppedConditionDistance)
        {
          // Moved outside ball!
          return false;
        }
      }

      // Iterate
      ++it;
    }
  }

  return true;
}

bool RandomWalkPlanner::isLPStuck()
{
  // If not found a path, assume LP is stuck
  if(!lp_found_at_least_one_path_)
  {
    return true;
  }

  // If time has elapsed since last lp_path_found, enable random walk
  ros::Time current_time = ros::Time::now();
  ros::Duration duration(kStoppedConditionTime);

  if(time_lp_found_path_ + duration < current_time)
  {
    return true;
  }  
  else
  {
    return false;
  }
}

void RandomWalkPlanner::publishStatus()
{
  // Send status to behavior executive
  random_walk_planner::RandomWalkPlannerStatus status;
  if(in_progress_)
  {
    status.status = random_walk_planner::RandomWalkPlannerStatus::STATUS_IN_PROGRESS;
  } 
  else
  {
    status.status = random_walk_planner::RandomWalkPlannerStatus::STATUS_DISABLED;
  }

  // Also signal any other relevant conditions
  // that help inform the BE select a planner

  if(received_first_odometry_)
  {
    bool is_stopped = isStopped();
    if(!is_stopped && !has_moved_at_least_once_) 
    {
      // First move detected, start startup timer
      has_moved_at_least_once_ = true;
      time_startup_ = ros::Time::now();
    }


    if(has_moved_at_least_once_ && hasStartupTimeElapsed())
    {
      // Get the statuses
      status.is_stopped = is_stopped;
      status.is_recovering = isRecovering();
    }
    else
    {
      // Too soon since startup, don't trigger random walk
      status.is_stopped = false;
      status.is_recovering = false;
    }
  }
  else
  {
    status.is_stopped = false;
    status.is_recovering = false;
  }

  // Ship it!
  pub_status_.publish(status);
}

void RandomWalkPlanner::publishWaypoint() 
{
  // Important: Don't publish a waypoint if this planner has been disabled!
  if(in_progress_)
  {
    ///////////////////////////////
    // Publish waypoint

    // Add the header
    waypoint_.header.frame_id = world_frame_id_;
    waypoint_.header.stamp = ros::Time::now();

    // Convert to PoseStamped
    geometry_msgs::PoseStamped waypoint_pose;
    waypoint_pose.header = waypoint_.header;
    waypoint_pose.pose.position = waypoint_.point;

    // Orientation not used by local planner if set to (0,0,0,0)
    waypoint_pose.pose.orientation.x = 0;
    waypoint_pose.pose.orientation.y = 0;
    waypoint_pose.pose.orientation.z = 0;
    waypoint_pose.pose.orientation.w = 0;

    // Ship it!
    pub_waypoint_.publish(waypoint_pose);

    ///////////////////////////////
    // Publish path (for updated local planner)

    core_trajectory_msgs::PlannerPath path_msg;
    path_msg.type = core_trajectory_msgs::PlannerPath::RANDOM_WALK;
    path_msg.path.header = waypoint_.header;

    path_msg.path.poses.push_back(waypoint_pose);

    path_pub_.publish(path_msg);
  }
}





bool RandomWalkPlanner::doPlannerRandomWalk()
{
  // Select waypoint
  // Just add the random_vector_ to current robot position
  waypoint_.point.x = robot_pos_.x + random_vector_.x;
  waypoint_.point.y = robot_pos_.y + random_vector_.y;
  waypoint_.point.z = robot_pos_.z + random_vector_.z;

  // ROS_INFO_STREAM("do RandomWalkPlanner, vector: " << random_vector_.x << ", " << random_vector_.y << ", " << random_vector_.z);

  bool successful = true;
  return successful;
}

bool RandomWalkPlanner::doPlanner()
{
  // Pick which variant of this planner to execute
  if (random_walk_planner_command_.command == random_walk_planner::RandomWalkPlannerCommand::COMMAND_RANDOM_WALK) 
  {
    if(isStopped())
    {
      resetRecoveryTime();
    }
    return doPlannerRandomWalk();
  }
  else
  {
    ROS_ERROR("RandomWalkPlanner received undefined command");
  }
}

RandomWalkPlanner::RandomWalkPlanner() {}

RandomWalkPlanner::RandomWalkPlanner(std::string node_name)
    : BaseNode(node_name), 
    time_recovery_start_(0), time_last_reset_(0), time_startup_(0), time_lp_found_path_(0) {}

bool RandomWalkPlanner::initialize() {
  
  if (!readParameters()) 
    return false;

  ros::NodeHandle *nh = get_node_handle();

  // Initialize target waypoint
  waypoint_ = geometry_msgs::PointStamped();
  waypoint_.point.x = 0.0;
  waypoint_.point.y = 0.0;
  waypoint_.point.z = 0.0;
  waypoint_.header.frame_id = world_frame_id_;

  // Initialize subscribers 
  sub_command_ = nh->subscribe(sub_command_topic_, 1, &RandomWalkPlanner::commandCallback, this);
  sub_odometry_ = nh->subscribe(sub_odometry_topic_, 1, &RandomWalkPlanner::odometryCallback, this);
  sub_lp_found_path_ = nh->subscribe(sub_lp_found_path_topic_, 1, &RandomWalkPlanner::lpFoundPathCallback, this);

  // Initialize publishers
  pub_waypoint_ = nh->advertise<geometry_msgs::PoseStamped>(pub_waypoint_topic_, 2);
  path_pub_ = nh->advertise<core_trajectory_msgs::PlannerPath>(pub_path_topic_, 2);
  pub_status_ = nh->advertise<random_walk_planner::RandomWalkPlannerStatus>(pub_status_topic_, 2);

  // Initial command
  random_walk_planner_command_.command = random_walk_planner::RandomWalkPlannerCommand::COMMAND_DISABLE;

  // Initialise startup time
  time_startup_ = ros::Time::now();

  ROS_INFO("Successfully launched random_walk_planner node");

  return true;
}

bool RandomWalkPlanner::execute() 
{
  // An iteration of the main loop

  // Check reset timer
  // Polling here instead of timer since was having issues with timer
  ros::Time current_time = ros::Time::now();
  ros::Duration duration(kVectorResetTime);
  if(current_time >= time_last_reset_ + duration)
  {
    time_last_reset_ = current_time;
    timerRandomVectorResetCallback();
  }

  // Planning
  if (random_walk_planner_command_.command == random_walk_planner::RandomWalkPlannerCommand::COMMAND_DISABLE) 
  {
    // Random walk planner is disabled -- do nothing
    in_progress_ = false;
  } 
  else  
  {
    // Random walk planner is enabled -- do something
    bool success = doPlanner();
    if(success)
    {
      in_progress_ = true;
    }
    else
    {
      in_progress_ = false;
    }
  }

  // Publish waypoint and status
  publishWaypoint();
  publishStatus();

  return true;
}
} // namespace random_walk_planner_ns

BaseNode *BaseNode::get() {
  random_walk_planner_ns::RandomWalkPlanner *node = new random_walk_planner_ns::RandomWalkPlanner("random_walk_planner_node");
  return node;
}