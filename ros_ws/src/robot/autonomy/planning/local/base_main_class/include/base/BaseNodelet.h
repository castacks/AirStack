#ifndef _BASE_NODELET_H_
#define _BASE_NODELET_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <string>
#include "Base.h"

class BaseNodelet : public Base, public nodelet::Nodelet {
 public:
  // This is inherited from nodelet::Nodelet
  virtual void onInit();

 protected:
  BaseNodelet(std::string node_name);
  ~BaseNodelet();
};


#endif
