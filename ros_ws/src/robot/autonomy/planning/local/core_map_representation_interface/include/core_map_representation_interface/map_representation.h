#ifndef _CORE_MAP_REPRESENTATION_H_
#define _CORE_MAP_REPRESENTATION_H_

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PointStamped.h>
#include <core_trajectory_msgs/TrajectoryXYZVYaw.h>

class MapRepresentation {
private:

public:

  /**
     Takes in a list of trajectories and outputs a value for each waypoint in each trajectory.
     @return A vector of vectors containing values for each waypoint. There is a vector of vectors for each trajectory. There is a vector of doubles for each waypoint within a trajectory.
   */
  virtual std::vector<std::vector<double> > get_values(std::vector<std::vector<geometry_msgs::PointStamped> > trajectories){//std::vector<core_trajectory_msgs::TrajectoryXYZVYaw> trajectories){
    ROS_ERROR("get_values CALLED BUT NOT IMPLEMENTED");

    std::vector<std::vector<double> > values;
    return values;
  }

  /**
     Clears the map.
   */
  virtual void clear(){
    ROS_ERROR("clear CALLED BUT NOT IMPLEMENTED");
  }

  /**
     Use this function to publish visualizations of the map that might be helpful for debugging.
   */
  virtual void publish_debug(){}

  virtual ~MapRepresentation(){}

protected:
  MapRepresentation(){}
};

#endif
