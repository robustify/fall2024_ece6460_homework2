// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <homework2/Homework2Config.h>

// Namespace matches ROS package name
namespace homework2 {

  class Homework2 {
    public:
      Homework2(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void reconfig(Homework2Config& config, uint32_t level);

      dynamic_reconfigure::Server<Homework2Config> srv_;
      Homework2Config cfg_;

  };

}
