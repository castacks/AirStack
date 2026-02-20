#include <airstack_common/templib.hpp>

namespace airstack {


  double vector3_distance(const tf2::Vector3& x, const tf2::Vector3& y){
    return x.distance(y);
  }
  
  //========================================================================================================
  // --------------------------------------- TemporalThresholdMonitor --------------------------------------
  //========================================================================================================
  


  //========================================================================================================
  // --------------------------------------------- TimeOutMonitor ------------------------------------------
  //========================================================================================================
  
  TimeOutMonitor::TimeOutMonitor(float time_out_duration)
    : time_out_duration(time_out_duration)
    , initialized(false){

  }
  
  void TimeOutMonitor::add_time(rclcpp::Time time){
    initialized = true;
    most_recent_time = time;
  }

  bool TimeOutMonitor::is_timed_out(rclcpp::Time time){
    return !initialized || (time - most_recent_time).seconds() > time_out_duration;
  }

  void TimeOutMonitor::clear_history(){
    initialized = false;
  }

  float TimeOutMonitor::time_until_timed_out(rclcpp::Time time){
    if(!initialized)
      return -1.f;

    return (time - most_recent_time).seconds();
  }
  
};
