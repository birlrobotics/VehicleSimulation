#ifndef LQR_FOLLOWING_HH_
#define LQR_FOLLOWING_HH_

#include <algorithm>
#include <cmath>
#include <string>
#include <fstream>

#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include <sstream>
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>


namespace mpc_control{

  class LQRControl{
    public:

    void amclCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);


  };
}
#endif
