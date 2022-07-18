// Header file for the class
#include "Homework2.hpp"

// Namespace matches ROS package name
namespace homework2 
{  
  // Constructor with global and private node handle arguments
  Homework2::Homework2(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    srv_.setCallback(boost::bind(&Homework2::reconfig, this, _1, _2));
  }

  void Homework2::reconfig(Homework2Config& config, uint32_t level)
  {
    cfg_ = config;
  }

}
