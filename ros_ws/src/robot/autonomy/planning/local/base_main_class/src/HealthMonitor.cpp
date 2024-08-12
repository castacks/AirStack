#include <ros/ros.h>
#include "base/HealthMonitor.h"
#include "base/Base.h"
#include <limits>

HealthMonitor::HealthMonitor(Base* base)
  : base_(base)
{
}

//=========================================================================================
// ---------------------------- Time Measurement Functions --------------------------------
//=========================================================================================

void HealthMonitor::tic(std::string id){
  // initialize time struct if tic is called for the first time on id
  if(times.count(id) == 0){
    times[id].elapsed = 0;
    times[id].calls = 0;
    times[id].min_us = std::numeric_limits<long int>::max();
    times[id].max_us = std::numeric_limits<long int>::min();

    base_->get_private_node_handle()->param<double>(id + "_target", times[id].target_hz, -1);
  }

  // log the current time
  times[id].start = boost::chrono::system_clock::now();
}

int64_t HealthMonitor::toc(std::string id){
  // get elapsed time if tic has been called.
  if(times.count(id) > 0){
    typedef boost::chrono::duration<int64_t, boost::micro> microseconds;
    microseconds  delta_us = boost::chrono::duration_cast<microseconds>(boost::chrono::system_clock::now() - times[id].start);
    int64_t count = delta_us.count();
    times[id].elapsed += count;
    times[id].calls++;
    times[id].min_us = std::min(count, times[id].min_us);
    times[id].max_us = std::max(count, times[id].max_us);

    double hz = 1000000./count;
    bool is_hitting_target_hz = hz >= times[id].target_hz;
    if(!is_hitting_target_hz)
      ROS_ERROR_STREAM(std::setprecision(2) << "execute function is not running fast enought."
		       << " Actual Hz: " << hz
		       << " Target Hz: " << times[id].target_hz);

    return count;
  }

  // otherwise return -1
  return -1;
}

double HealthMonitor::get_target_hz(std::string id){
  return times[id].target_hz;
}

void HealthMonitor::print_time_statistics(){
  ROS_INFO_STREAM( "Time Statistics for " << base_->get_node_name());//node_name_);
  int col_width = 15;
  ROS_INFO_STREAM(std::setw(col_width) << "Name"
		  << std::setw(col_width) << "Avg Hz"
		  << std::setw(col_width) << "Target Hz"
		  << std::setw(col_width) << "Avg ms"
		  << std::setw(col_width) << "Min ms"
		  << std::setw(col_width) << "Max ms"
		  << std::setw(col_width) << "Calls");

  for(std::map<std::string, time_info>::iterator it = times.begin(); it != times.end(); it++){
    if(it->second.calls == 0){
      ROS_INFO_STREAM(it->first << ": toc(\"" << it->first << "\"); was never called");
    }
    else{
      double average_elapsed = (double)it->second.elapsed/(double)it->second.calls;
      double average_hz = 1000000./average_elapsed;
      ROS_INFO_STREAM(std::setprecision(2) << std::setw(col_width) << it->first
		      << std::setw(col_width) << average_hz
		      << std::setw(col_width) << it->second.target_hz
		      << std::setw(col_width) << average_elapsed/1000.
		      << std::setw(col_width) << (double)it->second.min_us/1000.
		      << std::setw(col_width) << (double)it->second.max_us/1000.
		      << std::setw(col_width) << it->second.calls);
    }
  }
}
