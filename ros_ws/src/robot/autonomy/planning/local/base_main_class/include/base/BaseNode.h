#ifndef _BASE_NODE_H_
#define _BASE_NODE_H_

#include <ros/ros.h>
#include "Base.h"

class BaseNode : public Base {
 public:
  // This will be implemented by a specific task and will return an instance of itself, it will be called by the main function
  static BaseNode* get();

  virtual ~BaseNode();

  // Needed to call ros init
  static int argc;
  static char** argv;

 protected:
  BaseNode(std::string node_name, bool anonymous=false);

 private:
  bool anonymous_;
};



#endif
