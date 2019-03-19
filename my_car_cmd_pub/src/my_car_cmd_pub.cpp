
#include "ros/ros.h"
#include <sstream>
#include <Eigen/Dense>
#include "LQRFollowing.hh"
#include <geometry_msgs/Twist.h>

namespace mpc_control{

    using Matrix = Eigen::MatrixXd;

    // Control Para
    const int N_l_ = 1000;
    const int N_p_ = 10;
    const int N_x_ = 3;
    const int N_u_ = 1;
    const int G_test_ = 3;
    const float T_ = 0.1;
    //const int N_c_ = 30;
    //const int N_p_ = 60;

    // Vehicle Para
    const float L_ = 2.0;
    const float Cen_x_ = 0.0;
    const float Cen_y_ = 30.0;
    const float Vel_ = 5.0;
    float x_pose_ = 0.0;
    float y_pose_ = 0.0;
    float theta_ = 0.0;   
    float theta_init_ = 0.0;  
    const float r_ = 30.0;
    float omega_ = 0.0;
    float steer_fw_ = 0.0;
    float steer_fb_ = 0.0;
    float steer_cmd_ = 0.0;
    const float pi = 3.14159;

    // Vehicle State Matrix
    Eigen::MatrixXd X_real_;
    Eigen::MatrixXd u_real_;

    // Trajectory Ref
    Eigen::MatrixXd X_ref_;
    Eigen::MatrixXd x_ref_;
    Eigen::MatrixXd y_ref_;
    Eigen::MatrixXd theta_ref_;
    // Front Steering Angel
    Eigen::MatrixXd steer_ref_;

    // State Delta
    Eigen::MatrixXd X_delta_;
    Eigen::MatrixXd x_delta_;
    Eigen::MatrixXd y_delta_;
    Eigen::MatrixXd theta_delta_;

    Eigen::MatrixXd Q_;
    const float R_ = 10.0;
    Eigen::MatrixXd Pk_;
    Eigen::MatrixXd Vk_;
    Eigen::MatrixXd Pk_1_;
    Eigen::MatrixXd Vk_1_;
    Eigen::MatrixXd K_;
    Eigen::MatrixXd K_v_;
    float K_u_;
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;

  geometry_msgs::Twist cmd_vel;

}
using namespace mpc_control;

void CircularReferenceTrajectoryGenerate()
{
  X_ref_ = Matrix::Zero(N_p_, (N_x_ + 1));
  x_ref_ = Matrix::Zero(N_p_, 1);
  y_ref_ = Matrix::Zero(N_p_, 1);
  theta_ref_ = Matrix::Zero(N_p_, 1);
  steer_ref_ = Matrix::Zero(N_p_, 1);

  X_delta_ = Matrix::Zero(N_x_, N_p_);
  x_delta_ = Matrix::Zero(N_p_, 1);
  y_delta_ = Matrix::Zero(N_p_, 1);
  theta_delta_ = Matrix::Zero(N_p_, 1);

  theta_init_ = atan((y_pose_ - Cen_y_)/ (x_pose_ - Cen_x_));
  if (x_pose_ < Cen_x_)
  {
	theta_init_ = theta_init_ + pi;
  }
  omega_ = Vel_ /r_;
  steer_fw_ = atan(L_ /r_);
  
  for (int k = 0; k < N_p_; k++)
  {
	theta_ref_(k, 0) = theta_init_ + omega_ * T_ * (k - 1);
	x_ref_(k, 0) = r_ * cos(theta_ref_(k, 0)) + Cen_x_;
	X_ref_(k, 0) = x_ref_(k, 0);
	y_ref_(k, 0) = r_ * sin(theta_ref_(k, 0)) + Cen_y_;
	X_ref_(k, 1) = y_ref_(k, 0);
	theta_ref_(k, 0) = theta_ref_(k, 0) + pi/ 2;
	X_ref_(k, 2) = theta_ref_(k, 0);
	steer_ref_(k, 0) = steer_fw_;
	X_ref_(k, 3) = steer_ref_(k, 0);
  }

  X_delta_(0, 0) = X_real_(0, 0) - X_ref_(2, 0);
  X_delta_(1, 0) = X_real_(1, 0) - X_ref_(2, 1);
  X_delta_(2, 0) = X_real_(2, 0) - X_ref_(2, 2);
}
 
void ComputeSteeringFB()
{

  for(int j = N_p_; j >= 2; j--) 
  {
	Pk_1_ = Pk_;
	Vk_1_ = Vk_;	
	A_ = Matrix::Zero(3, 3);
	B_ = Matrix::Zero(3, 1);	
	A_ << 1, 0, -Vel_ * sin(X_ref_((j-2), 2)) * T_,
		0, 1, Vel_ * cos(X_ref_((j-2), 2)) * T_,
		0, 0, 1;
	B_ << 0,
		0,
		Vel_ * T_ / (L_ * cos(X_ref_((j-2), 2)) * cos(X_ref_((j-2), 2)));

	float a = (B_.transpose() * Pk_1_ * B_)(0, 0);
	K_ = (B_.transpose() * Pk_1_ * A_)/ (a + R_);
	K_u_ = R_ / (a + R_);
	K_v_ = B_.transpose()/ (a + R_);	
	Pk_ = A_.transpose() * Pk_1_ * (A_ - B_ * K_) + Q_;
	Vk_ = (A_ - B_ * K_).transpose() * Vk_1_ - K_.transpose() * R_ * X_ref_((j-2), 3);
  }
  
  steer_fb_ = (-K_ * X_delta_)(0, 0) - K_u_ * steer_fw_ - (K_v_ * Vk_1_)(0, 0); 
}
 
void amclCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  theta_ = atan((msg->pose[94].orientation.y * msg->pose[94].orientation.x - msg->pose[94].orientation.w * msg->pose[94].orientation.z) * 2 /( msg->pose[94].orientation.w * msg->pose[94].orientation.w - msg->pose[94].orientation.x * msg->pose[94].orientation.x + msg->pose[94].orientation.y * msg->pose[94].orientation.y - msg->pose[94].orientation.z * msg->pose[94].orientation.z ));

  x_pose_ = msg->pose[1].position.x;
  y_pose_ = msg->pose[1].position.y;
  ROS_INFO("I heard: theta,[%f]", theta_);
  ROS_INFO("I heard: x,[%f]", x_pose_);
  ROS_INFO("I heard: y,[%f]", y_pose_);

  X_real_ = Matrix::Zero(N_x_, 1);
  X_real_(0, 0) = x_pose_;
  X_real_(1, 0) = y_pose_;
  X_real_(2, 0) = theta_;

  CircularReferenceTrajectoryGenerate();
  std::cout << X_ref_ << std::endl;
  if (X_delta_(2, 0) > pi)
  {
	X_delta_(2, 0) = X_delta_(2, 0) - 2 * pi;
  }
  else if (X_real_(2, 0) < (-1 * pi))
  {
	X_delta_(2, 0) = X_delta_(2, 0) + 2 * pi;
  }

  ComputeSteeringFB();
  std::cout << "steer_fw_: " << steer_fw_ << std::endl;
  std::cout << "steer_fb_: " << steer_fb_ << std::endl;
  steer_cmd_ = steer_fw_ + steer_fb_; 

  cmd_vel.linear.x = 0.1;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = steer_cmd_;
  std::cout << "vel_cmd_:   " << cmd_vel.linear.x << std::endl;
  std::cout << "steer_cmd_: " << cmd_vel.angular.z << std::endl;
}


int main(int argc, char **argv)
{
  Pk_ = Matrix::Zero(3, 3);
  Pk_(0, 0) = 1;
  Pk_(1, 1) = 1;
  Pk_(2, 2) = 1;

  Vk_ = Matrix::Zero(3, 1);

  Q_ = Matrix::Zero(3, 3);
  Q_(0, 0) = 10;
  Q_(1, 1) = 10;
  Q_(2, 2) = 100;

  ros::init(argc, argv, "move_base");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  ros::Subscriber sub = n.subscribe("/gazebo/model_states", 10, amclCallback);

  ros::Rate loop_rate(10);
  // publish
  while (ros::ok())
  {
    pub.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


