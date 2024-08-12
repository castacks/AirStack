#ifndef _HEALTH_MONITOR_H_
#define _HEALTH_MONITOR_H_

#include <map>
#include <string>
#include <boost/chrono.hpp>
#include <stdint.h>

class Base; //forward declaration

// Struct for time logging.
struct time_info {
  boost::chrono::time_point<boost::chrono::system_clock> start;
  int64_t elapsed, min_us, max_us;
  uint64_t calls;
  double target_hz;
};

class HealthMonitor {
 private:
  std::map<std::string, time_info> times;
  Base* base_;

 public:
  HealthMonitor(Base* base);


  //=========================================================================================
  // ---------------------------- Time Measurement Functions --------------------------------
  //=========================================================================================

  void tic(std::string id);
  int64_t toc(std::string id);
  double get_target_hz(std::string id);
  void print_time_statistics();
};


#endif
