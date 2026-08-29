#ifndef _TEMPLIB_H_
#define _TEMPLIB_H_

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <utility>
#include <list>
#include <tf2/LinearMath/Vector3.h>

namespace airstack {

  double vector3_distance(const tf2::Vector3& x, const tf2::Vector3& y);

  //========================================================================================================
  // --------------------------------------- TemporalThresholdMonitor --------------------------------------
  //========================================================================================================
  
  template <typename InputType, typename ThresholdType>
  class TemporalThresholdMonitor {
  private:
    ThresholdType threshold;
    float history_duration;
    float min_dt;
    std::function<ThresholdType(const InputType& x, const InputType& y)> function;
    std::list<std::pair<InputType, rclcpp::Time> > measurements;

    void remove_old_measurements(rclcpp::Time time){
      bool updated = false;
      std::pair<InputType, rclcpp::Time> front;
      while(measurements.size() > 0 && (time - measurements.front().second).seconds() > history_duration){
	updated = true;
	front = measurements.front();
	measurements.pop_front();
      }

      if(updated)
	measurements.push_front(front); // make sure to keep the oldest measurement around so the duration check doesn't fail
    }
    
  public:
    TemporalThresholdMonitor(ThresholdType threshold, float history_duration, float min_dt,
			     std::function<ThresholdType(const InputType& x, const InputType& y)> function)
      : threshold(threshold)
      , history_duration(history_duration)
      , min_dt(min_dt)
      , function(function){}
    
    void add_measurement(InputType measurement, rclcpp::Time time){
      if(measurements.empty() || (time - measurements.back().second).seconds() >= min_dt){
	remove_old_measurements(time);
	measurements.push_back(std::pair<InputType, rclcpp::Time>(measurement, time));
      }
    }
    
    void clear_history(){
      measurements.clear();
    }
    
    bool threshold_exceeded(){
      if(measurements.size() == 0 || !has_full_history())
	return false;
      
      const InputType& front = measurements.front().first;
      const InputType& back = measurements.back().first;
      for(auto it = measurements.begin(); it != measurements.end(); it++)
	if(function(front, it->first) >= threshold || function(back, it->first) >= threshold)
	  return true;
      
      return false;
    }

    bool has_full_history(){
      return measurements.size() > 0 && (measurements.back().second - measurements.front().second).seconds() >= history_duration;
    }
  
  };


  //========================================================================================================
  // --------------------------------------------- TimeOutMonitor ------------------------------------------
  //========================================================================================================
  
  class TimeOutMonitor {
  private:
    float time_out_duration;
    rclcpp::Time most_recent_time;
    bool initialized;
    
  public:
    TimeOutMonitor(float time_out_duration);
    void add_time(rclcpp::Time time);
    bool is_timed_out(rclcpp::Time time);
    void clear_history();
    float time_until_timed_out(rclcpp::Time time);
  };

}

#endif
